// ubersdr-ntp-msftest: the MSF decoder against synthesised complex baseband,
// then against a real recording.
//
// Synthesised per NPL's "MSF 60 kHz time and date code" (2019): every second's
// carrier off at its start for 100 ms, bits A and B in the next two tenths,
// 500 ms off at second 00; the time code BCD most significant bit first in bit
// A, the identifier 01111110 in A 52-59, odd parity and summer time in B, UK
// clock time naming the next minute, and a leap second's minute running 61 s
// with the fields from 17A and 52B on moved one second later. Checked:
//
//   (a) every labelled second is the right UTC second -- including across the
//       GMT -> BST change and a leap second -- and the decoder locks where it
//       should;
//   (b) the edges, past the tracker's first twenty seconds, sit within
//       0.5 ms of truth wherever they fall on the sample grid, with the
//       unrounded edge unbiased to 50 us;
//   (c) WWVB, fed to it, certifies nothing.
//
// Then, if tools/testdata/msf_m9psy1_*.wav is present with its .times.csv (IQ
// from M9PSY-1, 125 km from Anthorn, with radiod's GPS capture time of every
// packet), decodes it and checks every labelled second against the UTC second
// those stamps put the edge in, and reports where the edges fall against UTC.
//
// Exit status 0 when every check passes.

#include "CivilTime.h"
#include "clock/MsfDecoder.h"

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

// UK summer time: last Sunday of March 01:00 UTC to last Sunday of October.
long long lastSundayUtc(int y, unsigned m) {
    const long long d = civ::daysFromCivil(y, m, 31);
    const long long wd = civ::floorMod(d + 3, 7);   // 0 = Monday
    return (d - civ::floorMod(wd - 6, 7)) * 86400LL + 3600LL;
}
bool bstAt(long long unix) {
    const long long days = civ::floorDiv(unix, 86400);
    int y = 0; unsigned m = 0, dd = 0;
    civ::civilFromDays(days, y, m, dd);
    return unix >= lastSundayUtc(y, 3) && unix < lastSundayUtc(y, 10);
}

struct Scenario {
    const char* note = "";
    int rate = 12000;
    double offsetSamples = 0.0;   // where second edges fall off the sample grid
    double cn0 = std::numeric_limits<double>::quiet_NaN();   // dB-Hz; NaN = clean
    double carrierHz = 0.0;
    double offLevel = 0.0;        // carrier amplitude while "off"
    bool invert = false;          // conjugate I/Q
    bool wwvb = false;            // send WWVB's AM code instead
    bool dropMarker = false;      // second 00 of minutes 3 and 6 sent as an ordinary second
    long long startUnix = 0;
    int minutes = 7;
    long long leapAt = 0;         // unix of the midnight a leap second precedes
    int dut1 = 3;                 // tenths
    bool expectLock = true;
    unsigned seed = 1;
};

struct Sec {
    double t = 0.0;               // stream seconds
    long long label = 0;          // UTC ms; -1 for 23:59:60
    double offMs[2] = {0, 0};     // carrier off from 0 to offMs[0]; and a second window
    double off2From = 0, off2To = 0;
};

// Bits A and B for the minute whose second 00 is unix T, `n` seconds long.
void timeCode(long long T, int n, int dut1, std::array<int, 62>& A, std::array<int, 62>& B) {
    A.fill(0); B.fill(0);
    const int o = n - 60;          // fields from 17A and 52B shift with the minute's length
    // The minute this code names starts at the next marker; unix time does not
    // count a leap second, so that is T + 60 however long this minute runs.
    const long long next = T + 60;
    const bool bst = bstAt(next);
    const long long local = next + (bst ? 3600 : 0);
    const long long days = civ::floorDiv(local, 86400);
    const long long rem = civ::floorMod(local, 86400);
    int y = 0; unsigned mo = 0, d = 0;
    civ::civilFromDays(days, y, mo, d);
    const int hh = static_cast<int>(rem / 3600), mi = static_cast<int>(rem / 60 % 60);
    const int wd = static_cast<int>(civ::floorMod(days + 4, 7));   // 0 = Sunday
    auto bcd = [](int v) { return (v / 10) << 4 | (v % 10); };
    auto put = [&](int p, int v, int nbits) {
        for (int i = nbits - 1; i >= 0; --i) A[static_cast<std::size_t>(p + o + (nbits - 1 - i))] = (v >> i) & 1;
    };
    put(17, bcd(y % 100), 8);
    put(25, bcd(static_cast<int>(mo)), 5);
    put(30, bcd(static_cast<int>(d)), 6);
    put(36, wd, 3);
    put(39, bcd(hh), 6);
    put(45, bcd(mi), 7);
    for (int j = 0; j < 8; ++j) A[static_cast<std::size_t>(52 + o + j)] = (j >= 1 && j <= 6) ? 1 : 0;
    auto oddPar = [&](int a, int z) { int c = 0; for (int p = a; p <= z; ++p) c += A[static_cast<std::size_t>(p + o)]; return (c & 1) ? 0 : 1; };
    B[static_cast<std::size_t>(54 + o)] = oddPar(17, 24);
    B[static_cast<std::size_t>(55 + o)] = oddPar(25, 35);
    B[static_cast<std::size_t>(56 + o)] = oddPar(36, 38);
    B[static_cast<std::size_t>(57 + o)] = oddPar(39, 51);
    B[static_cast<std::size_t>(58 + o)] = bst ? 1 : 0;
    // Summer time changing within the next 61 minutes.
    B[static_cast<std::size_t>(53 + o)] = bstAt(next + 3660) != bst ? 1 : 0;
    for (int k = 0; k < std::abs(dut1) && k < 8; ++k) B[static_cast<std::size_t>((dut1 > 0 ? 1 : 9) + k)] = 1;
}

// WWVB's AM code, roughly: 200/500/800 ms reduced, markers at 0,9,..,59.
void wwvbSecond(int s, std::mt19937& rng, double& offTo) {
    std::uniform_int_distribution<int> coin(0, 1);
    const bool marker = s == 0 || s % 10 == 9;
    offTo = marker ? 800 : coin(rng) ? 500 : 200;
}

std::vector<Sec> buildSeconds(const Scenario& sc) {
    std::vector<Sec> out;
    std::mt19937 rng(sc.seed * 7919u);
    const double off = sc.offsetSamples / sc.rate;
    double shift = 0.0;
    for (int m = 0; m < sc.minutes; ++m) {
        const long long T = sc.startUnix + 60LL * m;
        const bool leapMin = sc.leapAt > 0 && T + 60 == sc.leapAt;
        const int n = leapMin ? 61 : 60;
        std::array<int, 62> A{}, B{};
        timeCode(T, n, sc.dut1, A, B);
        for (int s = 0; s < n; ++s) {
            Sec x;
            x.t = 1.0 + off + static_cast<double>(T - sc.startUnix) + s + shift;
            x.label = s == 60 ? -1 : (T + s) * 1000LL;
            if (sc.wwvb) {
                double to = 0; wwvbSecond(s, rng, to);
                x.offMs[0] = to;
            } else if (s == 0 && !(sc.dropMarker && (m == 3 || m == 6))) {
                x.offMs[0] = 500;
            } else {
                // 0-100 off, then A, then B: contiguous off runs.
                const int a = A[static_cast<std::size_t>(s)], b = B[static_cast<std::size_t>(s)];
                x.offMs[0] = a ? (b ? 300 : 200) : 100;
                if (!a && b) { x.off2From = 200; x.off2To = 300; }
            }
            out.push_back(x);
        }
        if (leapMin) shift += 1.0;
    }
    return out;
}

// 1 inside [from, to] ms, 0 outside, with raised-cosine sides kRampMs wide
// centred on each end.
constexpr double kRampMs = 0.5;
double window(double ms, double from, double to) {
    if (!(to > from)) return 0.0;
    auto rise = [](double x) {   // 0 below -kRampMs/2, 1 above +kRampMs/2
        if (x <= -kRampMs / 2) return 0.0;
        if (x >= kRampMs / 2) return 1.0;
        return 0.5 - 0.5 * std::cos(kPi * (x + kRampMs / 2) / kRampMs);
    };
    return std::min(rise(ms - from), 1.0 - rise(ms - to));
}

struct Result {
    bool ok = true;
    std::string why;
    int labels = 0, wrong = 0;
    double p99 = 0.0;
    std::vector<double> errMs, exactErrMs;
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
    MsfDecoder dec(sc.rate, 0.0);

    long long frameStart = 0;
    bool haveFrame = false;
    dec.onFrame = [&](const ClockFrameInfo& f) {
        frameStart = f.frameStartSample; haveFrame = true;
        if (std::getenv("MSF_STATS")) {
            const auto d = dec.diagnostics();
            std::printf("    frame at %.3f s: %s from=%d state=%d refusal=%d vote=%d q=%.2f\n", f.frameStartSample / static_cast<double>(sc.rate),
                        f.minute < 0 ? "none" : civ::iso8601(civ::utcMsFromFields(f.year2, f.doy, f.hour, f.minute)).c_str(),
                        d.lastFrameFrom, static_cast<int>(dec.state()), d.refusalReason, d.framesInWindow, d.voteQuality);
        }
    };
    dec.onSecond = [&](const ClockSecondInfo& i) {
        if (!i.edgeMeasured) return;
        const double t = static_cast<double>(i.edgeSample) / sc.rate;
        const Sec* s = nearest(secs, t);
        if (!s || std::fabs(s->t - t) > 0.1) return;
        r.errMs.push_back((t - s->t) * 1000.0);
        if (std::isfinite(i.edgeSampleExact)) r.exactErrMs.push_back((i.edgeSampleExact / sc.rate - s->t) * 1000.0);
    };
    dec.onTime = [&](const ClockTimeInfo& t) {
        if (!haveFrame) return;
        const long long base = civ::utcMsFromFields(t.year2, t.doy, t.hour, t.minute);
        const long long el = std::llround(static_cast<double>(t.lastEdgeSample - frameStart) / sc.rate);
        const long long got = base + el * 1000LL;
        const Sec* s = nearest(secs, static_cast<double>(t.lastEdgeSample) / sc.rate);
        ++r.labels;
        if (!s || s->label != got) {
            ++r.wrong;
            char buf[160];
            std::snprintf(buf, sizeof buf, "labelled %s, truth %s", civ::iso8601(got).c_str(),
                          s ? (s->label < 0 ? "23:59:60" : civ::iso8601(s->label).c_str()) : "?");
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
            while (si + 1 < secs.size() && secs[si + 1].t <= t) ++si;
            double amp = 1.0;
            if (t >= secs[0].t) {
                const Sec& s = secs[si];
                const double ms = (t - s.t) * 1000.0;
                // Each keyed window with raised-cosine sides kRampMs wide,
                // centred on its nominal instants: band-limited, as a real
                // transmitter's is, so an edge between samples is still
                // recoverable, and its middle is exactly the nominal instant.
                double off = std::max(window(ms, 0.0, s.offMs[0]), window(ms, s.off2From, s.off2To));
                // The next second's fall starts ramping kRampMs/2 before it.
                if (si + 1 < secs.size()) off = std::max(off, window((t - secs[si + 1].t) * 1000.0, 0.0, secs[si + 1].offMs[0]));
                amp = 1.0 - (1.0 - sc.offLevel) * off;
            }
            const double ph = 2.0 * kPi * sc.carrierHz * t + 0.7;
            double I = amp * std::cos(ph) + sigma * g(rng);
            double Q = amp * std::sin(ph) + sigma * g(rng);
            if (sc.invert) Q = -Q;
            buf.push_back(static_cast<float>(I));
            buf.push_back(static_cast<float>(Q));
        }
        dec.process(buf.data(), buf.size() / 2);
    }
    r.diag = dec.diagnostics();
    if (std::getenv("MSF_STATS"))
        std::printf("    end: tone %d snr %.1f dB, segmented %d, contrast %.1f, anchored %d\n", r.diag.toneDetected,
                    r.diag.toneSnrDb, r.diag.phaseLocked, r.diag.pwmContrast, r.diag.anchored);

    if (sc.expectLock && r.labels < 60) fail(r, "never locked (" + std::to_string(r.labels) + " labels)");
    if (!sc.expectLock && r.labels > 0) fail(r, "certified a time it should have refused");
    // Past the tracker's first twenty edges: the 99th percentile and the mean,
    // against limits that scale with the noise. MSF on the air at M9PSY-1 is
    // near 69 dB-Hz; 40 is a weak signal and 30 a very weak one. (Dcf77test
    // holds DCF77's AM, a 15% cut, to a p99 of 2.5 ms at 40 dB-Hz.)
    if (sc.expectLock && r.errMs.size() > 40) {
        std::vector<double> a;
        double m = 0;
        for (std::size_t i = 20; i < r.exactErrMs.size(); ++i) { a.push_back(std::fabs(r.exactErrMs[i])); m += r.exactErrMs[i]; }
        m /= static_cast<double>(a.size());
        std::sort(a.begin(), a.end());
        const double p99 = a[a.size() * 99 / 100];
        const double cn0 = std::isnan(sc.cn0) ? 99.0 : sc.cn0;
        // p99: what the decoder achieves, with a margin -- regression guards.
        // Bias: what a run of this length can resolve. The tracked edge's
        // error is correlated over many seconds, so four minutes hold only a
        // few tens of independent samples of it: at 35 dB-Hz the mean is known
        // to about 50 us, at 30 dB-Hz to about 100.
        // 27 dB-Hz is where acquisition gives out: it locks and labels every
        // second right, but its edges are good to milliseconds, not tenths.
        // Limits set from eight seeds of each.
        const double p99Lim = cn0 >= 60 ? 0.05 : cn0 >= 40 ? 0.45 : cn0 >= 35 ? 1.5 : cn0 >= 30 ? 3.0 : 10.0;
        const double biasLim = cn0 >= 60 ? 0.005 : cn0 >= 40 ? 0.05 : cn0 >= 35 ? 0.15 : cn0 >= 30 ? 0.3 : 1.5;
        r.p99 = p99;
        if (std::getenv("MSF_STATS")) std::printf("    p99 %.3f ms, bias %+.4f ms\n", p99, m);
        if (p99 > p99Lim) fail(r, "edge p99 " + std::to_string(p99) + " ms (limit " + std::to_string(p99Lim) + ")");
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
    MsfDecoder dec(rate, 0.0);
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
            std::printf("%s  DUT1 %+.1f s%s\n", civ::iso8601(civ::utcMsFromFields(f.year2, f.doy, f.hour, f.minute)).c_str(),
                        f.dut1Tenths / 10.0, f.dst1 ? ", BST" : "");
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
        if (got != truth) {
            if (wrong++ < 5) std::printf("    WRONG: labelled %s, stamps say %s\n", civ::iso8601(got).c_str(), civ::iso8601(truth).c_str());
        }
    };
    const std::size_t block = static_cast<std::size_t>(rate / 50);
    for (std::size_t i = 0; i < iq.size() / 2; i += block)
        dec.process(iq.data() + 2 * i, std::min(block, iq.size() / 2 - i));
    const auto d = dec.diagnostics();

    bool ok = true;
    if (!edgeMs.empty()) {
        std::vector<double> a(edgeMs.begin() + std::min<std::size_t>(20, edgeMs.size() / 2), edgeMs.end());
        double m = 0; for (double x : a) m += x; m /= a.size();
        double v = 0; for (double x : a) v += (x - m) * (x - m);
        std::printf("  edges against the capture stamps: %zu, mean %+.3f ms after the UTC second, sd %.3f ms "
                    "(path from Anthorn 0.42 ms; the receiver chain and radiod's filter are in this too)\n",
                    a.size(), m, std::sqrt(v / a.size()));
    }
    std::printf("  end: carrier %.1f dB, contrast %.0f, %d frames (%d decoded), %d labelled seconds, %d wrong\n",
                d.toneSnrDb, d.pwmContrast, frames, decoded, labels, wrong);
    if (decoded < 3) { std::printf("  FAIL: fewer than three minutes decoded\n"); ok = false; }
    if (labels < 60) { std::printf("  FAIL: never locked\n"); ok = false; }
    if (wrong > 0) { std::printf("  FAIL: %d seconds labelled wrong\n", wrong); ok = false; }
    std::printf("  %s\n", ok ? "PASS" : "FAIL");
    return ok;
}

} // namespace

int main(int argc, char** argv) {
    const double kClean = std::numeric_limits<double>::quiet_NaN();
    // 2026-09-21T10:03Z, BST.
    constexpr long long kDay = 1789984980;
    // 2026-03-29T00:57Z: GMT -> BST at 01:00Z, three minutes in.
    constexpr long long kSpring = 1774745820;
    // 2016-12-31T23:54Z: the real leap second at 2017-01-01T00:00Z.
    constexpr long long kLeapStart = 1483228440;
    constexpr long long kLeapAt = 1483228800;

    std::vector<Scenario> scs;
    unsigned seed = 1;
    for (double off : {0.0, 0.37, 0.81})
        for (double cn0 : {kClean, 40.0, 30.0}) {
            Scenario s; s.note = "MSF"; s.offsetSamples = off; s.cn0 = cn0; s.startUnix = kDay; s.seed = seed++;
            scs.push_back(s);
        }
    {
        Scenario s; s.note = "MSF, 24 kHz"; s.rate = 24000; s.offsetSamples = 0.5; s.cn0 = 35; s.startUnix = kDay; s.seed = seed++;
        scs.push_back(s);
    }
    for (double hz : {1.1, -2.2}) {
        Scenario s; s.note = "carrier off DC"; s.carrierHz = hz; s.cn0 = 35; s.startUnix = kDay; s.seed = seed++;
        scs.push_back(s);
    }
    {
        Scenario s; s.note = "off is 10% not 0"; s.offLevel = 0.1; s.cn0 = 35; s.startUnix = kDay; s.seed = seed++;
        scs.push_back(s);
    }
    {
        Scenario s; s.note = "I/Q inverted"; s.invert = true; s.cn0 = 35; s.startUnix = kDay; s.seed = seed++;
        scs.push_back(s);
    }
    {
        Scenario s; s.note = "weak"; s.cn0 = 27; s.startUnix = kDay; s.minutes = 9; s.seed = seed++;
        scs.push_back(s);
    }
    {
        Scenario s; s.note = "minute marker missed"; s.dropMarker = true; s.cn0 = 35; s.startUnix = kDay; s.minutes = 9; s.seed = seed++;
        scs.push_back(s);
    }
    {
        Scenario s; s.note = "GMT -> BST"; s.cn0 = 35; s.startUnix = kSpring; s.minutes = 8; s.seed = seed++;
        scs.push_back(s);
    }
    {
        Scenario s; s.note = "leap second"; s.cn0 = 35; s.startUnix = kLeapStart; s.minutes = 10;
        s.leapAt = kLeapAt; s.seed = seed++;
        scs.push_back(s);
    }
    {
        Scenario s; s.note = "negative DUT1"; s.dut1 = -4; s.cn0 = 35; s.startUnix = kDay; s.seed = seed++;
        scs.push_back(s);
    }
    {
        Scenario s; s.note = "WWVB fed to it"; s.wwvb = true; s.offLevel = 0.14; s.cn0 = 40; s.startUnix = kDay;
        s.expectLock = false; s.seed = seed++;
        scs.push_back(s);
    }

    bool all = true;
    std::printf("%-26s %5s %6s %6s %7s %8s %8s  %s\n", "scenario", "rate", "off", "C/N0", "labels", "p99 ms", "bias ms", "result");
    const char* only = std::getenv("MSF_ONLY");
    if (const char* extra = std::getenv("MSF_SEED")) for (Scenario& sc : scs) sc.seed += static_cast<unsigned>(std::atoi(extra)) * 1000u;
    for (const Scenario& sc : scs) {
        if (only && !std::strstr(sc.note, only)) continue;
        const Result r = run(sc);
        double b = 0;
        std::size_t nb = 0;
        for (std::size_t i = std::min<std::size_t>(20, r.exactErrMs.size()); i < r.exactErrMs.size(); ++i) { b += r.exactErrMs[i]; ++nb; }
        if (nb) b /= nb;
        char cn[16];
        if (std::isnan(sc.cn0)) std::snprintf(cn, sizeof cn, "clean"); else std::snprintf(cn, sizeof cn, "%.0f", sc.cn0);
        std::printf("%-26s %5d %6.2f %6s %4d/%-2d %8.3f %+8.4f  %s%s\n", sc.note, sc.rate, sc.offsetSamples, cn,
                    r.labels, r.wrong, r.p99, b, r.ok ? "ok" : "FAIL: ", r.ok ? "" : r.why.c_str());
        all = all && r.ok;
    }

    const std::string path = argc > 1 ? argv[1] : MSF_TESTDATA;
    all = live(path) && all;
    std::printf("\n%s\n", all ? "ALL PASS" : "FAILURES");
    return all ? 0 : 1;
}
