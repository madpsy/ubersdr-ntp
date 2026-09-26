// ubersdr-ntp-allouistest: the ALS162 (Allouis) decoder against synthesised
// complex baseband, then against a real recording.
//
// Synthesised per ITU-R TF.2487 section 9.1 and the GNU Radio transmitter
// model in henningM1r/gr_ALS162_Receiver (GPLv3): every second but 59 opens
// with a phase excursion 0 -> +1 -> 0 -> -1 -> 0 rad in 25 ms ramps, twice for
// a 1; 200-900 ms carries that second's position code; the time code is
// DCF77's layout from second 20 with the ALS162 header before it, in French
// legal time naming the next minute. The position-code table is the model's,
// indexed here by the second each is sent in (its key is one ahead).
// Independent of the decoder's copy: this file writes its own from the model.
//
// Checked:
//   (a) every labelled second is the right UTC second, including across the
//       CET -> CEST change, and the decoder locks where it should;
//   (b) edges -- the second, 50 ms after the excursion starts -- past the
//       tracker's first twenty seconds, against limits that scale with the
//       noise, and unbiased;
//   (c) a Tuesday maintenance outage is ridden out and locked again after.
//
// Then, if tools/testdata/als162_m9psy1_*.wav is present with its
// .times.csv (IQ from M9PSY-1, 1057 km from Allouis, with radiod's GPS capture
// time of every packet), decodes it, checks every labelled second against the
// UTC second those stamps put it in, and reports where its edges fall
// against UTC.
//
// Exit status 0 when every check passes.

#include "CivilTime.h"
#include "clock/AllouisDecoder.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <limits>
#include <random>
#include <sstream>
#include <string>
#include <vector>

using namespace clockdec;
namespace civ = ubersdr_ntp;

namespace {

constexpr double kPi = 3.14159265358979323846;

constexpr std::array<std::array<int, 32>, 60> kCodes = {{
    {{ 0,+1,-1, 0, 0, 0,+1,-1, 0, 0, 0, 0, 0, 0, 0,-1, 0,+2, 0,-2, 0,+2, 0,-2, 0,+1, 0, 0, 0, 0, 0, 0}},   // s00 (key "01")
    {{+1, 0,-2, 0,+2,-1,-1,+1,+1,-1,-1,+1, 0,+1, 0,-2,+1, 0, 0,+1,-2, 0,+1, 0,+1,-1,-1,+1, 0, 0, 0, 0}},   // s01 (key "02")
    {{+1,-1, 0, 0,-1,+2, 0,-1,-1, 0,+2,-1,-1,+2, 0,-1,-1, 0,+2,-1,-1,+2, 0,-1,-1, 0,+1, 0, 0, 0, 0, 0}},   // s02 (key "03")
    {{+1,-1,-1,+1,+1, 0,-1,-1,+1, 0,-1,+1, 0, 0, 0,+1, 0,-2,+1,+1,-1,-1, 0,+1,+1,-1,-1,+1, 0, 0, 0, 0}},   // s03 (key "04")
    {{+1,-1, 0,+1,-2, 0,+1, 0,+1, 0,-1,-1, 0,+2, 0,-1,-1, 0,+1,+1,-1,-1,+2,-1,-1,+1, 0, 0, 0, 0, 0, 0}},   // s04 (key "05")
    {{+1,-1,-1,+1,+1, 0,-1,-1, 0,+1, 0,+1, 0,-2, 0,+2, 0,-2,+1,+1,-1,-1, 0,+2,-1,-1,+1, 0, 0, 0, 0, 0}},   // s05 (key "06")
    {{ 0, 0, 0, 0,+1, 0,-1, 0,-1, 0,+1,+1, 0,-1,-1,+1, 0, 0, 0,-1,+1,+1, 0,-1,-1, 0,+1, 0, 0, 0, 0, 0}},   // s06 (key "07")
    {{ 0, 0,+1, 0,-1, 0, 0, 0, 0,-1, 0,+1, 0, 0, 0, 0, 0, 0, 0, 0, 0,+1,-1,-1,+2,-1,-1,+1, 0, 0, 0, 0}},   // s07 (key "08")
    {{+1,-1, 0,+1,-2,+1, 0,-1,+1,+1,-1,-1,+1,+1, 0,-1,-1, 0,+1,+1, 0,-1,-1, 0,+1, 0, 0, 0, 0, 0, 0, 0}},   // s08 (key "09")
    {{ 0,+1, 0,-1,-1, 0,+1,+1, 0,-1,-1, 0,+2,-1,-1,+1,+1, 0,-1,-1,+1,+1,-2, 0,+1, 0, 0, 0, 0, 0, 0, 0}},   // s09 (key "10")
    {{ 0,+1, 0,-1,-1,+1,+1,-2,+1, 0,-1,+1, 0,+1,-1, 0,+1,-1, 0,-1, 0,+2,-1, 0, 0,-1,+1, 0, 0, 0, 0, 0}},   // s10 (key "11")
    {{ 0,+1, 0,-1, 0, 0,-1, 0,+2,-1,-1,+1,+1, 0,-1, 0,-1, 0,+1, 0, 0, 0, 0, 0,+1,-1,-1,+1, 0, 0, 0, 0}},   // s11 (key "12")
    {{+1,-1,-1,+2, 0,-1,-1, 0,+1,+1,-1, 0,+1,-2, 0,+1,+1,-1,-1,+1, 0, 0,+1, 0,-1,-1, 0,+1, 0, 0, 0, 0}},   // s12 (key "13")
    {{+1, 0,-2,+1,+1,-2,+1, 0,-1,+2, 0,-2, 0,+1,+1,-1, 0, 0,-1,+1,+1,-1,-1,+2, 0,-2, 0,+1, 0, 0, 0, 0}},   // s13 (key "14")
    {{ 0,+1,-1,-1,+1,+1, 0,-1, 0,-1, 0,+2, 0,-1, 0,-1, 0,+2,-1, 0,+1,-2, 0,+2,-1,-1,+1, 0, 0, 0, 0, 0}},   // s14 (key "15")
    {{ 0, 0,+1, 0,-2,+1,+1,-1, 0, 0,-1,+1, 0,-1,+1, 0, 0, 0, 0,+1, 0,-2,+1,+1,-1,-1, 0,+1, 0, 0, 0, 0}},   // s15 (key "16")
    {{+1, 0,-2, 0,+2,-1, 0,+1,-1, 0, 0,-1,+1,+1,-1, 0, 0,-1,+1,+1,-2,+1,+1,-2, 0,+1, 0, 0, 0, 0, 0, 0}},   // s16 (key "17")
    {{+1,-1, 0,+1,-2, 0,+2,-1, 0,+1,-1, 0, 0,-1, 0,+2, 0,-1, 0,-1,+1, 0,-1,+1, 0, 0, 0, 0, 0, 0, 0, 0}},   // s17 (key "18")
    {{+1, 0,-1, 0,-1, 0,+1,+1,-1, 0,+1,-2, 0,+2,-1,-1,+1, 0, 0,+1,-1,-1,+1, 0, 0, 0, 0, 0, 0, 0, 0, 0}},   // s18 (key "19")
    {{+1,-1,-1,+1, 0,+1, 0,-2, 0,+1, 0,+1,-1, 0,+1,-2, 0,+2,-1, 0,+1,-2, 0,+2,-1,-1,+1, 0, 0, 0, 0, 0}},   // s19 (key "20")
    {{ 0, 0,+1,-1,-1,+2,-1, 0,+1,-1, 0,-1, 0,+1,+1,-1, 0,+1,-2,+1,+1,-1, 0,-1, 0,+1, 0, 0, 0, 0, 0, 0}},   // s20 (key "21")
    {{+1,-1,-1,+2,-1, 0, 0,-1,+1, 0,+1, 0,-1,-1, 0,+2, 0,-2, 0,+2, 0,-1, 0,-1,+1, 0,-1,+1, 0, 0, 0, 0}},   // s21 (key "22")
    {{ 0, 0, 0, 0,+1, 0,-1, 0, 0, 0, 0, 0,-1,+1,+1,-2,+1, 0, 0,+1,-1,-1, 0,+2, 0,-2, 0,+1, 0, 0, 0, 0}},   // s22 (key "23")
    {{ 0, 0, 0, 0,+1,-1, 0, 0, 0,+1,-2,+1, 0,-1,+2, 0,-1,-1,+1,+1,-2,+1,+1,-1, 0,-1, 0,+1, 0, 0, 0, 0}},   // s23 (key "24")
    {{+1, 0,-2, 0,+1, 0, 0, 0,+1, 0,-2, 0,+2, 0,-1, 0, 0,-1,+1, 0, 0,+1,-2,+1,+1,-2, 0,+1, 0, 0, 0, 0}},   // s24 (key "25")
    {{+1,-1, 0,+1,-2, 0,+2,-1, 0, 0, 0,+1,-2, 0,+2,-1,-1,+1, 0,+1,-1, 0,+1,-1,-1, 0,+1, 0, 0, 0, 0, 0}},   // s25 (key "26")
    {{+1, 0,-2, 0,+1,+1,-1,-1,+1, 0,+1, 0,-2, 0,+2, 0,-1,-1,+1, 0,-1,+1,+1, 0,-2, 0,+1, 0, 0, 0, 0, 0}},   // s26 (key "27")
    {{+1, 0,-1,-1,+1,+1,-2,+1, 0, 0,+1,-2, 0,+1, 0,+1,-1,-1,+2, 0,-1, 0,-1,+1,+1,-2, 0,+1, 0, 0, 0, 0}},   // s27 (key "28")
    {{ 0,+1,-1,-1,+2, 0,-2, 0,+2, 0,-1, 0, 0, 0, 0,-1, 0,+2,-1,-1,+1, 0,+1,-1,-1,+1, 0, 0, 0, 0, 0, 0}},   // s28 (key "29")
    {{+1, 0,-2, 0,+2,-1,-1,+1, 0,+1, 0,-2, 0,+2, 0,-2,+1, 0, 0,+1,-2, 0,+1,+1, 0,-2, 0,+1, 0, 0, 0, 0}},   // s29 (key "30")
    {{ 0,+1,-1, 0,+1,-2, 0,+1,+1, 0,-2,+1,+1,-1, 0,-1, 0,+2, 0,-1,-1, 0,+2,-1,-1,+1, 0, 0, 0, 0, 0, 0}},   // s30 (key "31")
    {{ 0, 0,+1, 0,-2,+1,+1,-1,-1, 0,+1,+1, 0,-2, 0,+1, 0, 0, 0,+1, 0,-2,+1, 0, 0, 0,-1,+1, 0, 0, 0, 0}},   // s31 (key "32")
    {{+1, 0,-1, 0,-1,+1, 0,-1,+1, 0, 0,+1,-1, 0,+1,-1, 0, 0, 0,-1, 0,+1,+1, 0,-2, 0,+1, 0, 0, 0, 0, 0}},   // s32 (key "33")
    {{ 0, 0, 0, 0,+1, 0,-2, 0,+2,-1,-1,+2, 0,-1, 0, 0, 0,-1,+1,+1,-1, 0, 0, 0,-1, 0,+1, 0, 0, 0, 0, 0}},   // s33 (key "34")
    {{ 0, 0, 0,+1, 0,-1, 0, 0,-1, 0,+1, 0,+1, 0,-2,+1, 0, 0,+1,-1, 0,-1, 0,+1,+1,-1,-1,+1, 0, 0, 0, 0}},   // s34 (key "35")
    {{+1,-1, 0,+1,-1, 0, 0, 0,-1, 0,+2,-1, 0, 0, 0, 0,-1,+1, 0, 0, 0,+1,-1, 0, 0,-1,+1, 0, 0, 0, 0, 0}},   // s35 (key "36")
    {{+1,-1,-1,+1, 0,+1,-1,-1,+1,+1,-1,-1,+1, 0,+1, 0,-2,+1, 0, 0,+1,-1,-1, 0,+1, 0, 0, 0, 0, 0, 0, 0}},   // s36 (key "37")
    {{ 0,+1, 0,-2,+1,+1,-1, 0, 0, 0,-1, 0,+2,-1,-1,+1,+1, 0,-2, 0,+1,+1,-1,-1,+1, 0, 0, 0, 0, 0, 0, 0}},   // s37 (key "38")
    {{ 0,+1, 0,-1,-1,+1,+1,-2, 0,+2,-1,-1,+1,+1,-1, 0,+1,-1, 0,-1, 0,+2,-1,-1,+1, 0, 0, 0, 0, 0, 0, 0}},   // s38 (key "39")
    {{ 0,+1, 0,-1, 0, 0,-1, 0,+2,-1,-1,+1,+1,-1,-1,+1,+1, 0,-1, 0, 0, 0, 0, 0,-1, 0,+1, 0, 0, 0, 0, 0}},   // s39 (key "40")
    {{ 0,+1, 0,-1,-1,+1,+1,-2,+1,+1,-1, 0,-1, 0,+2,-1,-1,+1, 0,+1, 0,-2,+1, 0,-1,+1, 0, 0, 0, 0, 0, 0}},   // s40 (key "41")
    {{+1, 0,-1,-1,+1, 0,-1,+2, 0,-1,-1,+1,+1,-2, 0,+2,-1,-1,+2, 0,-2, 0,+2, 0,-1,-1, 0,+1, 0, 0, 0, 0}},   // s41 (key "42")
    {{+1, 0,-1, 0,-1, 0,+2, 0,-2, 0,+2,-1, 0,+1,-1, 0, 0, 0, 0,-1,+1, 0,-1,+1,+1,-1,-1,+1, 0, 0, 0, 0}},   // s42 (key "43")
    {{ 0,+1,-1,-1,+2, 0,-2, 0,+2,-1, 0, 0, 0, 0, 0,+1,-2,+1, 0,-1,+1, 0, 0, 0,+1,-1,-1,+1, 0, 0, 0, 0}},   // s43 (key "44")
    {{ 0,+1, 0,-1, 0, 0,-1,+1,+1,-2,+1, 0,-1,+2, 0,-2, 0,+1, 0, 0, 0, 0, 0,+1,-1,-1,+1, 0, 0, 0, 0, 0}},   // s44 (key "45")
    {{+1,-1,-1,+1,+1,-1,-1,+2, 0,-2, 0,+2,-1,-1,+1,+1, 0,-2,+1,+1,-2, 0,+2, 0,-2, 0,+1, 0, 0, 0, 0, 0}},   // s45 (key "46")
    {{ 0, 0,+1, 0,-2, 0,+1, 0,+1, 0,-2,+1, 0,-1,+2,-1, 0,+1,-1,-1,+1,+1,-2,+1,+1,-2, 0,+1, 0, 0, 0, 0}},   // s46 (key "47")
    {{+1, 0,-1,-1,+1, 0, 0,+1,-2,+1, 0,-1,+2, 0,-2,+1, 0,-1,+2, 0,-2,+1, 0, 0,+1,-2, 0,+1, 0, 0, 0, 0}},   // s47 (key "48")
    {{+1,-1,-1,+2,-1,-1,+2, 0,-2, 0,+2, 0,-1, 0, 0, 0,-1,+1,+1,-2,+1, 0,-1,+1,+1,-1,-1,+1, 0, 0, 0, 0}},   // s48 (key "49")
    {{ 0, 0,+1, 0,-2, 0,+1, 0,+1,-1, 0, 0, 0, 0, 0,+1,-1, 0, 0,-1,+1,+1,-1, 0,-1, 0,+1, 0, 0, 0, 0, 0}},   // s49 (key "50")
    {{+1,-1,-1,+1, 0, 0, 0,+1,-1, 0,+1,-1,-1, 0,+2, 0,-2,+1, 0, 0, 0,-1,+1, 0, 0, 0, 0, 0, 0, 0, 0, 0}},   // s50 (key "51")
    {{+1, 0,-1,-1,+1,+1,-1, 0, 0, 0, 0,-1,+1, 0,-1,+2,-1,-1,+2, 0,-1,-1,+1,+1,-1,-1, 0,+1, 0, 0, 0, 0}},   // s51 (key "52")
    {{ 0,+1, 0,-1,-1, 0,+2, 0,-2, 0,+1,+1,-1,-1,+1,+1, 0,-1, 0,-1,+1, 0,-1,+1,+1,-1,-1,+1, 0, 0, 0, 0}},   // s52 (key "53")
    {{+1, 0,-1,-1, 0,+1,+1, 0,-2,+1, 0, 0, 0, 0, 0, 0, 0,-1,+2,-1, 0, 0, 0, 0,-1,+1, 0, 0, 0, 0, 0, 0}},   // s53 (key "54")
    {{ 0, 0,+1,-1, 0,+1,-2, 0,+2,-1,-1,+2,-1, 0, 0, 0, 0,-1,+2, 0,-1, 0,-1,+1, 0,-1,+1, 0, 0, 0, 0, 0}},   // s54 (key "55")
    {{ 0, 0, 0, 0, 0, 0, 0,+1,-1,-1,+2, 0,-2, 0,+1, 0, 0,+1,-1, 0, 0,-1,+2,-1, 0, 0,-1,+1, 0, 0, 0, 0}},   // s55 (key "56")
    {{ 0, 0,+1, 0,-2,+1, 0, 0,+1,-1,-1,+1, 0, 0,+1,-2,+1,+1,-1,-1, 0,+1, 0,+1, 0,-2, 0,+1, 0, 0, 0, 0}},   // s56 (key "57")
    {{+1, 0,-1, 0,-1,+1, 0,-1,+2,-1,-1,+2,-1,-1,+2,-1, 0,+1,-1,-1, 0,+1, 0, 0,+1,-1,-1,+1, 0, 0, 0, 0}},   // s57 (key "58")
    {{+1, 0,-2, 0,+2, 0,-2, 0,+2, 0,-2, 0,+2, 0,-2, 0,+2, 0,-2, 0,+1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}},   // s58 (key "59")
    {{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}},   // s59 (key "00")
}};

// EU summer time: last Sunday of March 01:00 UTC to last Sunday of October.
long long lastSundayUtc(int y, unsigned m) {
    const long long d = civ::daysFromCivil(y, m, 31);
    const long long wd = civ::floorMod(d + 3, 7);   // 0 = Monday
    return (d - civ::floorMod(wd - 6, 7)) * 86400LL + 3600LL;
}
bool cestAt(long long ut) {
    const long long days = civ::floorDiv(ut, 86400);
    int y = 0; unsigned m = 0, dd = 0;
    civ::civilFromDays(days, y, m, dd);
    return ut >= lastSundayUtc(y, 3) && ut < lastSundayUtc(y, 10);
}

struct Scenario {
    const char* note = "";
    int rate = 12000;
    double offsetSamples = 0.0;
    double cn0 = std::numeric_limits<double>::quiet_NaN();
    double carrierHz = 0.0;
    bool invert = false;
    long long startUnix = 0;
    int minutes = 7;
    int outageFrom = -1, outageTo = -1;   // minutes with no carrier at all
    bool expectLock = true;
    unsigned seed = 1;
};

// The excursion starts 50.48 ms before the second it marks
// (AllouisDecoder.cpp, kSecondAfterStartSec).
constexpr double kLeadSec = 0.05048;

struct Sec {
    double t = 0.0;       // the UTC second, in stream time; its modulation starts kLeadSec before
    long long label = 0;
    int bit = 0;          // 0, 1, or 2 = no excursion (second 59)
    int sof = 0;
    bool off = false;
};

std::array<int, 60> timeCode(long long T, std::mt19937& rng) {
    std::array<int, 60> b{};
    std::uniform_int_distribution<int> coin(0, 1);
    const long long next = T + 60;
    const bool cest = cestAt(next);
    const long long local = next + (cest ? 7200 : 3600);
    const long long days = civ::floorDiv(local, 86400);
    const long long rem = civ::floorMod(local, 86400);
    int y = 0; unsigned mo = 0, d = 0;
    civ::civilFromDays(days, y, mo, d);
    const int hh = static_cast<int>(rem / 3600), mi = static_cast<int>(rem / 60 % 60);
    const int wd = static_cast<int>(civ::floorMod(days + 3, 7)) + 1;
    auto put = [&](int s, int v, std::initializer_list<int> w) {
        for (int x : w) b[static_cast<std::size_t>(s++)] = (v & x) ? 1 : 0;
    };
    auto bcd = [](int v) { return (v / 10) << 4 | (v % 10); };
    b[13] = coin(rng) && coin(rng);   // holidays, now and then
    b[17] = cest ? 1 : 0;
    b[18] = cest ? 0 : 1;
    b[20] = 1;
    put(21, bcd(mi), {1, 2, 4, 8, 16, 32, 64});
    put(29, bcd(hh), {1, 2, 4, 8, 16, 32});
    put(36, bcd(static_cast<int>(d)), {1, 2, 4, 8, 16, 32});
    put(42, wd, {1, 2, 4});
    put(45, bcd(static_cast<int>(mo)), {1, 2, 4, 8, 16});
    put(50, bcd(y % 100), {1, 2, 4, 8, 16, 32, 64, 128});
    auto par = [&](int a, int z) { int p = 0; for (int s = a; s < z; ++s) p ^= b[static_cast<std::size_t>(s)]; return p; };
    b[28] = par(21, 28);
    b[35] = par(29, 35);
    b[58] = par(36, 58);
    int ones = 0;
    for (int s = 21; s <= 58; ++s) ones += b[static_cast<std::size_t>(s)];
    b[3] = (ones >> 1) & 1; b[4] = (ones >> 2) & 1; b[5] = (ones >> 3) & 1; b[6] = (ones >> 4) & 1;
    return b;
}

std::vector<Sec> buildSeconds(const Scenario& sc) {
    std::mt19937 rng(sc.seed * 7919u);
    std::vector<Sec> out;
    const double off = sc.offsetSamples / sc.rate;
    for (int m = 0; m < sc.minutes; ++m) {
        const long long T = sc.startUnix + 60LL * m;
        const auto code = timeCode(T, rng);
        for (int s = 0; s < 60; ++s) {
            Sec x;
            x.t = 1.0 + off + static_cast<double>(T - sc.startUnix) + s;
            x.label = (T + s) * 1000LL;
            x.bit = s == 59 ? 2 : code[static_cast<std::size_t>(s)];
            x.sof = s;
            x.off = m >= sc.outageFrom && m < sc.outageTo;
            out.push_back(x);
        }
    }
    return out;
}

double excursion(double t) {
    if (t < 0.0 || t >= 0.1) return 0.0;
    const double u = t / 0.025;
    if (u < 1.0) return u;
    if (u < 3.0) return 2.0 - u;
    return u - 4.0;
}

// The phase of second `s` carrying `bit`, t seconds into it.
double phaseOf(int s, int bit, double t) {
    double ph = 0.0;
    if (bit != 2) ph += excursion(t) + (bit == 1 ? excursion(t - 0.1) : 0.0);
    if (t >= 0.2 && t < 1.0) {
        const double u = (t - 0.2) / 0.025;
        const int k = static_cast<int>(u);
        const auto& c = kCodes[static_cast<std::size_t>(s)];
        for (int i = 0; i < k; ++i) ph += c[static_cast<std::size_t>(i)];
        ph += c[static_cast<std::size_t>(k)] * (u - k);
    }
    return ph;
}

struct Result {
    bool ok = true;
    std::string why;
    int labels = 0, wrong = 0;
    double p99 = 0.0, bias = 0.0;
    std::vector<double> errMs;
    long long lastLabelSample = -1;
    ClockDecoderDiagnostics diag;
};

void fail(Result& r, const std::string& why) {
    if (r.ok) r.why = why;
    r.ok = false;
}

const Sec* nearest(const std::vector<Sec>& secs, double t) {
    auto it = std::lower_bound(secs.begin(), secs.end(), t, [](const Sec& s, double v) { return s.t < v; });
    const Sec* best = nullptr;
    if (it != secs.end()) best = &*it;
    if (it != secs.begin() && (!best || std::fabs((it - 1)->t - t) < std::fabs(best->t - t))) best = &*(it - 1);
    return best;
}

Result run(const Scenario& sc) {
    Result r;
    const auto secs = buildSeconds(sc);
    AllouisDecoder dec(sc.rate, 0.0);
    long long frameStart = 0;
    bool haveFrame = false;
    dec.onFrame = [&](const ClockFrameInfo& f) {
        frameStart = f.frameStartSample; haveFrame = true;
        if (std::getenv("ALS_STATS")) {
            const auto d = dec.diagnostics();
            std::printf("    frame at %.3f s: %s state=%d refusal=%d vote=%d q=%.2f snr=%.1f\n", f.frameStartSample / static_cast<double>(sc.rate),
                        f.minute < 0 ? "none" : civ::iso8601(civ::utcMsFromFields(f.year2, f.doy, f.hour, f.minute)).c_str(),
                        static_cast<int>(dec.state()), d.refusalReason, d.framesInWindow, d.voteQuality, d.pmSnrDb);
        }
    };
    dec.onSecond = [&](const ClockSecondInfo& i) {
        if (!i.edgeMeasured) return;
        const double t = i.edgeSampleExact / sc.rate;
        const Sec* s = nearest(secs, t);
        if (!s || std::fabs(s->t - t) > 0.1) return;
        r.errMs.push_back((t - s->t) * 1000.0);
    };
    dec.onTime = [&](const ClockTimeInfo& t) {
        if (!haveFrame) return;
        const long long base = civ::utcMsFromFields(t.year2, t.doy, t.hour, t.minute);
        const long long el = std::llround(static_cast<double>(t.lastEdgeSample - frameStart) / sc.rate);
        const long long got = base + el * 1000LL;
        const Sec* s = nearest(secs, static_cast<double>(t.lastEdgeSample) / sc.rate);
        ++r.labels;
        r.lastLabelSample = t.lastEdgeSample;
        if (!s || s->label != got) {
            ++r.wrong;
            char buf[160];
            std::snprintf(buf, sizeof buf, "labelled %s, truth %s", civ::iso8601(got).c_str(),
                          s ? civ::iso8601(s->label).c_str() : "?");
            fail(r, buf);
        }
    };

    std::mt19937 rng(sc.seed);
    const double sigma = std::isnan(sc.cn0) ? 0.0 : std::sqrt(sc.rate / std::pow(10.0, sc.cn0 / 10.0) / 2.0);
    std::normal_distribution<double> g(0.0, 1.0);
    const double endT = secs.back().t + 1.5;
    const long long total = static_cast<long long>(endT * sc.rate);
    std::vector<float> buf;
    std::size_t si = 0;
    const int block = sc.rate / 50;
    for (long long n0 = 0; n0 < total; n0 += block) {
        buf.clear();
        for (long long n = n0; n < n0 + block && n < total; ++n) {
            const double t = static_cast<double>(n) / sc.rate;
            while (si + 1 < secs.size() && secs[si + 1].t - kLeadSec <= t) ++si;
            double amp = 1.0, ph = 0.0;
            if (t >= secs[0].t - kLeadSec) {
                const Sec& s = secs[si];
                ph = phaseOf(s.sof, s.bit, t - (s.t - kLeadSec));
                if (s.off) amp = 0.0;
            }
            ph += 2.0 * kPi * sc.carrierHz * t + 0.4;
            double I = amp * std::cos(ph) + sigma * g(rng);
            double Q = amp * std::sin(ph) + sigma * g(rng);
            if (sc.invert) Q = -Q;
            buf.push_back(static_cast<float>(I));
            buf.push_back(static_cast<float>(Q));
        }
        dec.process(buf.data(), buf.size() / 2);
    }
    r.diag = dec.diagnostics();

    if (sc.expectLock && r.labels < 60) fail(r, "never locked (" + std::to_string(r.labels) + " labels)");
    if (!sc.expectLock && r.labels > 0) fail(r, "certified a time it should have refused");
    // After an outage, locked again before the end.
    if (sc.outageTo > 0 && r.lastLabelSample < static_cast<long long>((secs.back().t - 60.0) * sc.rate))
        fail(r, "did not lock again after the outage");
    if (sc.expectLock && r.errMs.size() > 40) {
        std::vector<double> a;
        double m = 0;
        for (std::size_t i = 20; i < r.errMs.size(); ++i) { a.push_back(std::fabs(r.errMs[i])); m += r.errMs[i]; }
        m /= static_cast<double>(a.size());
        std::sort(a.begin(), a.end());
        r.p99 = a[a.size() * 99 / 100];
        r.bias = m;
        const double cn0 = std::isnan(sc.cn0) ? 99.0 : sc.cn0;
        const double p99Lim = cn0 >= 60 ? 0.01 : cn0 >= 40 ? 0.2 : cn0 >= 35 ? 0.5 : 1.5;
        const double biasLim = cn0 >= 60 ? 0.002 : cn0 >= 40 ? 0.03 : cn0 >= 35 ? 0.08 : 0.3;
        if (r.p99 > p99Lim) fail(r, "edge p99 " + std::to_string(r.p99) + " ms (limit " + std::to_string(p99Lim) + ")");
        if (std::fabs(m) > biasLim) fail(r, "edges biased " + std::to_string(m * 1000.0) + " us");
    }
    return r;
}

// ---- the real recording ----------------------------------------------------

bool readWav(const std::string& path, int& rate, std::vector<float>& iq) {
    std::ifstream f(path, std::ios::binary);
    if (!f) return false;
    std::vector<char> d((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
    if (d.size() < 44 || std::memcmp(d.data(), "RIFF", 4) || std::memcmp(d.data() + 8, "WAVE", 4)) return false;
    std::size_t p = 12;
    int ch = 0, bits = 0;
    while (p + 8 <= d.size()) {
        std::uint32_t len = 0;
        std::memcpy(&len, d.data() + p + 4, 4);
        if (!std::memcmp(d.data() + p, "fmt ", 4)) {
            std::uint16_t c = 0, b = 0; std::uint32_t sr = 0;
            std::memcpy(&c, d.data() + p + 10, 2);
            std::memcpy(&sr, d.data() + p + 12, 4);
            std::memcpy(&b, d.data() + p + 22, 2);
            ch = c; rate = static_cast<int>(sr); bits = b;
        } else if (!std::memcmp(d.data() + p, "data", 4)) {
            if (ch != 2 || bits != 16) return false;
            const std::size_t n = std::min<std::size_t>(len, d.size() - p - 8) / 2;
            iq.resize(n);
            for (std::size_t i = 0; i < n; ++i) {
                std::int16_t v = 0;
                std::memcpy(&v, d.data() + p + 8 + 2 * i, 2);
                iq[i] = v / 32768.0f;
            }
            return true;
        }
        p += 8 + len + (len & 1);
    }
    return false;
}

// The recording's timeline: radiod's GPS capture time of every packet's first
// sample (tools/iqrecord.cpp). A sample's UTC is taken from the nearest stamped
// packet at or before it; the stamps jitter by microseconds, not samples.
struct Timeline {
    std::vector<long long> frame;
    std::vector<long long> ns;
    bool load(const std::string& path) {
        std::ifstream f(path);
        if (!f) return false;
        std::string line;
        while (std::getline(f, line)) {
            if (line.empty() || !std::isdigit(static_cast<unsigned char>(line[0]))) continue;
            std::stringstream ss(line);
            std::string a, b;
            std::getline(ss, a, ',');
            std::getline(ss, b, ',');
            const long long ts = std::stoll(b);
            if (ts == 0) continue;
            frame.push_back(std::stoll(a));
            ns.push_back(ts);
        }
        return !frame.empty();
    }
    double utcAt(double sample, int rate) const {
        auto it = std::upper_bound(frame.begin(), frame.end(), static_cast<long long>(std::floor(sample)));
        const std::size_t i = it == frame.begin() ? 0 : static_cast<std::size_t>(it - frame.begin() - 1);
        return static_cast<double>(ns[i]) * 1e-9 + (sample - static_cast<double>(frame[i])) / rate;
    }
};

bool live(const std::string& path) {
    int rate = 0;
    std::vector<float> iq;
    if (!readWav(path, rate, iq)) {
        std::printf("\nlive recording: %s not found or not 16-bit stereo; skipped\n", path.c_str());
        return true;
    }
    std::string csv = path;
    csv.replace(csv.size() - 4, 4, ".times.csv");
    Timeline tl;
    const bool haveTl = tl.load(csv);
    std::printf("\nlive recording: %s, %d Hz, %.0f s, %s\n", path.c_str(), rate, iq.size() / 2.0 / rate,
                haveTl ? "with capture stamps" : "NO capture stamps");
    AllouisDecoder dec(rate, 0.0);
    long long frameStart = 0;
    bool haveFrame = false;
    int frames = 0, decoded = 0, labels = 0, wrong = 0;
    std::vector<double> edgeMs;
    dec.onFrame = [&](const ClockFrameInfo& f) {
        frameStart = f.frameStartSample; haveFrame = true; ++frames;
        std::printf("  frame at %8.3f s: ", f.frameStartSample / static_cast<double>(rate));
        if (f.minute < 0) std::printf("no valid decode\n");
        else {
            ++decoded;
            std::printf("%s%s\n", civ::iso8601(civ::utcMsFromFields(f.year2, f.doy, f.hour, f.minute)).c_str(),
                        f.dst1 ? ", CEST" : ", CET");
        }
    };
    dec.onSecond = [&](const ClockSecondInfo& i) {
        if (!i.edgeMeasured || !haveTl) return;
        const double u = tl.utcAt(std::isfinite(i.edgeSampleExact) ? i.edgeSampleExact : i.edgeSample, rate);
        edgeMs.push_back((u - std::round(u)) * 1000.0);
    };
    dec.onTime = [&](const ClockTimeInfo& t) {
        if (!haveFrame) return;
        ++labels;
        if (!haveTl) return;
        const long long base = civ::utcMsFromFields(t.year2, t.doy, t.hour, t.minute);
        const long long el = std::llround(static_cast<double>(t.lastEdgeSample - frameStart) / rate);
        const long long got = base + el * 1000LL;
        const long long truth = std::llround(tl.utcAt(t.lastEdgeSample, rate)) * 1000LL;
        if (got != truth && wrong++ < 5)
            std::printf("    WRONG: labelled %s, stamps say %s\n", civ::iso8601(got).c_str(), civ::iso8601(truth).c_str());
    };
    const std::size_t block = static_cast<std::size_t>(rate / 50);
    for (std::size_t i = 0; i < iq.size() / 2; i += block)
        dec.process(iq.data() + 2 * i, std::min(block, iq.size() / 2 - i));
    const auto d = dec.diagnostics();

    bool ok = true;
    if (edgeMs.size() > 30) {
        std::vector<double> a(edgeMs.begin() + 20, edgeMs.end());
        double m = 0; for (double x : a) m += x; m /= a.size();
        double v = 0; for (double x : a) v += (x - m) * (x - m);
        double sd2 = 0; for (std::size_t i = 1; i < a.size(); ++i) sd2 += (a[i] - a[i - 1]) * (a[i] - a[i - 1]);
        std::printf("  edges against the capture stamps: %zu, mean %+.4f ms from the UTC second, "
                    "sd %.4f ms, second to second %.4f ms (path from Allouis 3.5 ms; the receiver chain is in this too)\n",
                    a.size(), m, std::sqrt(v / a.size()), std::sqrt(sd2 / (a.size() - 1) / 2.0));
    }
    std::printf("  end: carrier %.1f dB, timing SNR %.1f dB, %d frames (%d decoded), %d labelled seconds, %d wrong\n",
                d.toneSnrDb, d.pmSnrDb, frames, decoded, labels, wrong);
    if (decoded < 3) { std::printf("  FAIL: fewer than three minutes decoded\n"); ok = false; }
    if (labels < 60) { std::printf("  FAIL: never locked\n"); ok = false; }
    if (wrong > 0) { std::printf("  FAIL: %d seconds labelled wrong\n", wrong); ok = false; }
    std::printf("  %s\n", ok ? "PASS" : "FAIL");
    return ok;
}

} // namespace

int main(int argc, char** argv) {
    const double kClean = std::numeric_limits<double>::quiet_NaN();
    constexpr long long kDay = 1789984980;       // 2026-09-21T10:03Z, a Monday, CEST
    constexpr long long kSpring = 1774745820;    // 2026-03-29T00:57Z: CET -> CEST at 01:00Z

    std::vector<Scenario> scs;
    unsigned seed = 1;
    for (double off : {0.0, 0.37, 0.81})
        for (double cn0 : {kClean, 40.0, 30.0}) {
            Scenario s; s.note = "ALS162"; s.offsetSamples = off; s.cn0 = cn0; s.startUnix = kDay; s.seed = seed++;
            scs.push_back(s);
        }
    {
        Scenario s; s.note = "ALS162, 24 kHz"; s.rate = 24000; s.offsetSamples = 0.5; s.cn0 = 35; s.startUnix = kDay; s.seed = seed++;
        scs.push_back(s);
    }
    for (double hz : {1.3, -2.7}) {
        Scenario s; s.note = "carrier off DC"; s.carrierHz = hz; s.cn0 = 35; s.startUnix = kDay; s.seed = seed++;
        scs.push_back(s);
    }
    {
        Scenario s; s.note = "I/Q inverted"; s.invert = true; s.cn0 = 35; s.startUnix = kDay; s.seed = seed++;
        scs.push_back(s);
    }
    {
        // The lock limit: at 29 dB-Hz every seed tried locks and labels right;
        // at 28 most do not, the 100 ms excursions then losing more bits a
        // minute than erasure decoding can fill in.
        Scenario s; s.note = "weak"; s.cn0 = 29; s.startUnix = kDay; s.minutes = 9; s.seed = seed++;
        scs.push_back(s);
    }
    {
        Scenario s; s.note = "CET -> CEST"; s.cn0 = 35; s.startUnix = kSpring; s.minutes = 8; s.seed = seed++;
        scs.push_back(s);
    }
    {
        Scenario s; s.note = "outage, minutes 3-5"; s.cn0 = 35; s.startUnix = kDay; s.minutes = 10;
        s.outageFrom = 3; s.outageTo = 5; s.seed = seed++;
        scs.push_back(s);
    }

    bool all = true;
    std::printf("%-26s %5s %6s %6s %7s %8s %8s  %s\n", "scenario", "rate", "off", "C/N0", "labels", "p99 ms", "bias ms", "result");
    const char* only = std::getenv("ALS_ONLY");
    if (const char* extra = std::getenv("ALS_SEED")) for (Scenario& sc : scs) sc.seed += static_cast<unsigned>(std::atoi(extra)) * 1000u;
    for (const Scenario& sc : scs) {
        if (only && !std::strstr(sc.note, only)) continue;
        const Result r = run(sc);
        char cn[16];
        if (std::isnan(sc.cn0)) std::snprintf(cn, sizeof cn, "clean"); else std::snprintf(cn, sizeof cn, "%.0f", sc.cn0);
        std::printf("%-26s %5d %6.2f %6s %4d/%-2d %8.3f %+8.4f  %s%s\n", sc.note, sc.rate, sc.offsetSamples, cn,
                    r.labels, r.wrong, r.p99, r.bias, r.ok ? "ok" : "FAIL: ", r.ok ? "" : r.why.c_str());
        all = all && r.ok;
    }

    const std::string path = argc > 1 ? argv[1] : ALS162_TESTDATA;
    all = live(path) && all;
    std::printf("\n%s\n", all ? "ALL PASS" : "FAILURES");
    return all ? 0 : 1;
}
