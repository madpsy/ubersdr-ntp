// Offline accuracy test for the WWV/WWVH/WWVB time-code decoders.
//
// There is no receiver in a unit test, and a live one hides every decoder
// fault behind propagation, so this synthesises the broadcasts itself -- the
// same signal model as tools/wwvgen.py, per NIST SP 432 and SP 250-67 -- at
// 12 and 24 kHz, with the true second edges landing at chosen SUB-SAMPLE
// offsets into the stream (the WWVB 40-60% block phases included, which is
// where the old edge detector went blind), clean and with white noise. It also
// plays a leap-second minute: announced, announced but not inserted, and
// inserted without the warning bit.
//
// Each run is decoded the way Source consumes the decoders: a `time` event is
// composed against the last frame's second 0, and every later second edge
// extends it by whole seconds of samples until the lock drops. Every label
// that produces is compared against the truth for that edge. Checks:
//   (a) the decoder locks and labels the last minute of audio correctly, and
//       the 2024-12-31 -> 2025-01-01 runs certify a time after the rollover;
//   (b) edge error: WWVB within +/-2 ms of truth; WWV/WWVH within +/-3 ms of
//       that station's mean bias across all runs (the mean is reported -- it is
//       the chain-delay calibration, kNominalDelaySamples, not tested here);
//   (c) no wrong timestamp, ever. The only exception allowed is the leap second
//       itself when it was NOT announced: nothing can know that second is
//       23:59:60 until the one after it arrives.
//
// Exit status 0 when every check passes.

#include "CivilTime.h"
#include "clock/WwvDecoder.h"
#include "clock/WwvbDecoder.h"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <limits>
#include <map>
#include <mutex>
#include <random>
#include <string>
#include <thread>
#include <vector>

using namespace clockdec;
namespace civ = ubersdr_ntp;

namespace {

constexpr double kPi = 3.14159265358979323846;

enum class Kind { Wwv, Wwvh, Wwvb };

const char* kindName(Kind k) {
    switch (k) {
        case Kind::Wwv: return "WWV";
        case Kind::Wwvh: return "WWVH";
        default: return "WWVB";
    }
}

struct BitWeight { int second, weight; };
using FieldMap = std::vector<BitWeight>;

// NIST SP 432 (LSB-first) and SP 250-67 (MSB-first) field maps, as in the
// decoders and tools/wwvgen.py.
const FieldMap kWwvMin  = {{10, 1}, {11, 2}, {12, 4}, {13, 8}, {15, 10}, {16, 20}, {17, 40}};
const FieldMap kWwvHr   = {{20, 1}, {21, 2}, {22, 4}, {23, 8}, {25, 10}, {26, 20}};
const FieldMap kWwvDoy  = {{30, 1}, {31, 2}, {32, 4}, {33, 8}, {35, 10}, {36, 20},
                           {37, 40}, {38, 80}, {40, 100}, {41, 200}};
const FieldMap kWwvYr   = {{4, 1}, {5, 2}, {6, 4}, {7, 8}, {51, 10}, {52, 20}, {53, 40}, {54, 80}};
const FieldMap kWwvbMin = {{1, 40}, {2, 20}, {3, 10}, {5, 8}, {6, 4}, {7, 2}, {8, 1}};
const FieldMap kWwvbHr  = {{12, 20}, {13, 10}, {15, 8}, {16, 4}, {17, 2}, {18, 1}};
const FieldMap kWwvbDoy = {{22, 200}, {23, 100}, {25, 80}, {26, 40}, {27, 20},
                           {28, 10}, {30, 8}, {31, 4}, {32, 2}, {33, 1}};
const FieldMap kWwvbYr  = {{45, 80}, {46, 40}, {47, 20}, {48, 10}, {50, 8}, {51, 4}, {52, 2}, {53, 1}};

constexpr int kWwvLeapWarnSecond = 3;    // NIST SP 432
constexpr int kWwvbLeapWarnSecond = 56;  // NIST SP 250-67

void encode(int value, const FieldMap& map, std::vector<int>& sym) {
    std::vector<BitWeight> byWeight = map;
    std::sort(byWeight.begin(), byWeight.end(),
              [](const BitWeight& a, const BitWeight& b) { return a.weight > b.weight; });
    for (const BitWeight& bw : byWeight) {
        if (value >= bw.weight) { value -= bw.weight; sym[bw.second] = 1; }
    }
}

struct Scenario {
    Kind kind = Kind::Wwv;
    int rate = 12000;
    double offsetMs = 0.0;          // true second edges land this far into the stream
    double snrDb = std::numeric_limits<double>::quiet_NaN();   // NaN = clean
    unsigned seed = 1;
    long long startUnix = 0;        // POSIX time of the first whole minute
    int minutes = 10;
    long long rolloverUnix = 0;     // a certified time at/after this is required (0 = none)
    long long leapAfterUnix = 0;    // POSIX 00:00:00 following a leap second (0 = none)
    bool leapInsert = false;        // actually insert 23:59:60
    bool leapWarn = false;          // set the warning bit through the month
    // Unannounced leap only: the 23:59:60 edge itself may go out labelled as the
    // next minute's s0 -- nothing can tell it apart until the second after it.
    // Every OTHER edge must still be right.
    bool allowLeapEdgeWrong = false;
    // Station tag. `preset` goes in through presetStation before the first
    // sample (or pinStation when `pin`), as Source does for a restarted
    // decoder. `otherTickAmp` adds the other station's tick as well, 15 ms
    // later, the way both are heard on a shared frequency. `wantStation`
    // overrides the kind's own tag for the final check, and from `tagBySec`
    // on the decoder must hold that tag continuously (-1: final check only).
    ClockStation preset = ClockStation::Unknown;
    bool pin = false;
    double otherTickAmp = 0.0;
    const char* wantStation = nullptr;
    double tagBySec = -1.0;
    const char* note = "";
};

// One physical broadcast second: the POSIX time it names (-1 for 23:59:60)
// and its symbol (-1 = no pulse, 0/1 = bit, 2 = marker).
struct Second { long long posix; int sym; };

std::vector<Second> buildSeconds(const Scenario& sc) {
    std::vector<Second> secs;
    const bool b = sc.kind == Kind::Wwvb;
    for (int m = 0; m < sc.minutes; ++m) {
        const long long t = sc.startUnix + 60LL * m;
        const long long days = civ::floorDiv(t, 86400);
        const long long rem = t - days * 86400;
        int y = 0; unsigned mo = 0, d = 0;
        civ::civilFromDays(days, y, mo, d);
        const int doy = static_cast<int>(days - civ::daysFromCivil(y, 1, 1)) + 1;
        const int hour = static_cast<int>(rem / 3600);
        const int minute = static_cast<int>(rem / 60 % 60);

        std::vector<int> sym(60, 0);
        encode(minute, b ? kWwvbMin : kWwvMin, sym);
        encode(hour, b ? kWwvbHr : kWwvHr, sym);
        encode(doy, b ? kWwvbDoy : kWwvDoy, sym);
        encode(y % 100, b ? kWwvbYr : kWwvYr, sym);
        // The warning is up from early in the month until the leap second.
        const bool warn = sc.leapWarn && sc.leapAfterUnix > 0 && t < sc.leapAfterUnix &&
                          t >= sc.leapAfterUnix - 28LL * 86400;
        if (warn) sym[b ? kWwvbLeapWarnSecond : kWwvLeapWarnSecond] = 1;
        for (int s = 0; s < 60; ++s) {
            const bool marker = b ? (s == 0 || s % 10 == 9) : (s % 10 == 9);
            if (marker) sym[s] = 2;
        }
        if (!b) sym[0] = -1;   // WWV/WWVH minute hole

        for (int s = 0; s < 60; ++s) secs.push_back({t + s, sym[s]});
        // 23:59:60: a marker on WWVB (three in a row), a binary 0 on WWV/WWVH.
        if (sc.leapInsert && t + 60 == sc.leapAfterUnix) secs.push_back({-1, b ? 2 : 0});
    }
    return secs;
}

std::vector<float> render(const Scenario& sc, const std::vector<Second>& secs) {
    const bool b = sc.kind == Kind::Wwvb;
    const double tickHz = sc.kind == Kind::Wwvh ? 2200.0 : 2000.0;
    const double lowAmp = std::pow(10.0, -17.0 / 20.0);
    const double lowLen[3] = {0.200, 0.500, 0.800};
    const double pulseLen[3] = {0.170, 0.470, 0.770};

    double peak = b ? 1.0 : 2.3;
    double sigma = 0.0;
    if (!std::isnan(sc.snrDb)) {
        sigma = (1.0 / std::sqrt(2.0)) * std::pow(10.0, -sc.snrDb / 20.0);
        peak += 4.0 * sigma;
    }
    // wwvgen.py scales to int16 at 28000 peak; the decoders see int16 / 32768.
    const double scale = 28000.0 / peak / 32768.0;
    std::mt19937_64 rng(sc.seed);
    std::normal_distribution<double> gauss(0.0, 1.0);

    const std::size_t n = secs.size() * static_cast<std::size_t>(sc.rate);
    std::vector<float> out(n);
    const double off = sc.offsetMs / 1000.0;
    for (std::size_t i = 0; i < n; ++i) {
        const double t = static_cast<double>(i) / sc.rate - off;
        const long long p = static_cast<long long>(std::floor(t));
        const double tau = t - static_cast<double>(p);
        const int sym = (p >= 0 && p < static_cast<long long>(secs.size())) ? secs[p].sym : -1;
        double x;
        if (b) {
            const double amp = (sym >= 0 && tau < lowLen[sym]) ? lowAmp : 1.0;
            x = amp * std::sin(2.0 * kPi * 1000.0 * tau);
        } else {
            double env = 1.0;
            if (sym >= 0 && tau >= 0.030 && tau < 0.030 + pulseLen[sym])
                env += 0.5 * std::sin(2.0 * kPi * 100.0 * tau);
            x = env * std::sin(2.0 * kPi * 1000.0 * tau);
            if (tau < 0.005) x += 0.8 * std::sin(2.0 * kPi * tickHz * tau);
            const double otherHz = tickHz == 2000.0 ? 2200.0 : 2000.0;
            if (sc.otherTickAmp > 0.0 && tau >= 0.015 && tau < 0.020)
                x += sc.otherTickAmp * std::sin(2.0 * kPi * otherHz * (tau - 0.015));
        }
        if (sigma > 0.0) x += sigma * gauss(rng);
        out[i] = static_cast<float>(x * scale);
    }
    return out;
}

struct Result {
    bool pass = true;
    std::string why;
    double lockAtSec = -1.0;
    int timeGood = 0, timeBad = 0, labelGood = 0, labelBad = 0;
    int lastMinuteGood = 0;
    int wrongAtLeap = 0;               // wrong outputs for the 23:59:60 edge itself
    int wrongElsewhere = 0;            // wrong outputs for any other edge
    bool timeAfterRollover = false;
    std::vector<double> edgeErrMs;     // measured edges, after the first time event
    double unmeasuredMaxAbsMs = 0.0;   // edges flagged !edgeMeasured
    int unmeasured = 0;
    std::string firstWrong;
    const char* station = "?";
    double tickRatioDb = std::numeric_limits<double>::quiet_NaN();   // at the end
    double tagOffAtSec = -1.0;         // first second past tagBySec without the wanted tag
};

const char* stationName(ClockStation s) {
    return s == ClockStation::Wwv ? "WWV" : s == ClockStation::Wwvh ? "WWVH"
         : s == ClockStation::Wwvb ? "WWVB" : "unknown";
}

void fail(Result& r, const std::string& why) {
    r.pass = false;
    if (!r.why.empty()) r.why += "; ";
    r.why += why;
}

Result run(const Scenario& sc) {
    Result r;
    const std::vector<Second> secs = buildSeconds(sc);
    const std::vector<float> pcm = render(sc, secs);
    const double rate = sc.rate;
    const double offSamples = sc.offsetMs / 1000.0 * rate;
    const long long nSecs = static_cast<long long>(secs.size());

    auto physIndex = [&](std::int64_t edge) {
        return std::llround((static_cast<double>(edge) - offSamples) / rate);
    };
    auto truthMs = [&](long long p, long long& ms) {
        if (p < 0 || p >= nSecs || secs[static_cast<std::size_t>(p)].posix < 0) return false;
        ms = secs[static_cast<std::size_t>(p)].posix * 1000LL;
        return true;
    };

    // Source's composition state.
    bool haveFrame = false, haveAnchor = false, seenTime = false;
    std::int64_t frameStart = 0, anchorEdge = 0;
    long long anchorMs = 0;

    auto noteWrong = [&](const char* what, std::int64_t edge, long long decoded) {
        long long tm = 0;
        const long long p = physIndex(edge);
        const bool atLeap = p >= 0 && p < nSecs && secs[static_cast<std::size_t>(p)].posix < 0;
        (atLeap ? r.wrongAtLeap : r.wrongElsewhere) += 1;
        if (!r.firstWrong.empty()) return;
        const std::string truth = truthMs(p, tm) ? civ::iso8601(tm) : std::string("23:59:60");
        r.firstWrong = std::string(what) + " " + civ::iso8601(decoded) + " for true " + truth;
    };

    auto onFrame = [&](const ClockFrameInfo& f) {
        frameStart = f.frameStartSample;
        haveFrame = true;
    };
    auto onTime = [&](const ClockTimeInfo& t) {
        if (t.year2 < 0 || t.doy < 1 || t.hour < 0 || t.minute < 0 || !haveFrame) return;
        const long long base = civ::utcMsFromFields(t.year2, t.doy, t.hour, t.minute);
        const long long el = std::llround(static_cast<double>(t.lastEdgeSample - frameStart) / rate);
        const long long dec = base + el * 1000LL;
        long long tm = 0;
        if (truthMs(physIndex(t.lastEdgeSample), tm) && tm == dec) {
            ++r.timeGood;
            if (sc.rolloverUnix && tm >= sc.rolloverUnix * 1000LL) r.timeAfterRollover = true;
        } else {
            ++r.timeBad;
            noteWrong("time", t.lastEdgeSample, dec);
        }
        if (r.lockAtSec < 0) r.lockAtSec = static_cast<double>(t.lastEdgeSample) / rate;
        anchorEdge = t.lastEdgeSample;
        anchorMs = dec;
        haveAnchor = true;
        seenTime = true;
    };
    auto onSecond = [&](const ClockSecondInfo& i) {
        const long long p = physIndex(i.edgeSample);
        const double errMs =
            (static_cast<double>(i.edgeSample) - (static_cast<double>(p) * rate + offSamples)) /
            rate * 1000.0;
        if (seenTime) {
            if (i.edgeMeasured) {
                r.edgeErrMs.push_back(errMs);
            } else {
                ++r.unmeasured;
                r.unmeasuredMaxAbsMs = std::max(r.unmeasuredMaxAbsMs, std::fabs(errMs));
            }
        }
        if (!haveAnchor || i.edgeSample < anchorEdge) return;
        const double elapsed = static_cast<double>(i.edgeSample - anchorEdge) / rate;
        const long long whole = std::llround(elapsed);
        if (std::fabs(elapsed - static_cast<double>(whole)) > 0.25) return;
        const long long label = anchorMs + whole * 1000LL;
        long long tm = 0;
        if (truthMs(p, tm) && tm == label) {
            ++r.labelGood;
            if (p >= nSecs - 60) ++r.lastMinuteGood;
        } else {
            ++r.labelBad;
            noteWrong("label", i.edgeSample, label);
        }
    };
    auto onState = [&](ClockLockState s) {
        if (s != ClockLockState::Locked) haveAnchor = false;
    };

    constexpr std::size_t kChunk = 2400;   // 100-200 ms, like a stream
    if (sc.kind == Kind::Wwvb) {
        WwvbDecoder d(sc.rate);
        d.onSecond = onSecond; d.onFrame = onFrame; d.onTime = onTime; d.onStateChanged = onState;
        for (std::size_t i = 0; i < pcm.size(); i += kChunk)
            d.process(pcm.data() + i, std::min(kChunk, pcm.size() - i));
        r.station = "WWVB";
    }
    const char* want = sc.wantStation ? sc.wantStation
                     : sc.kind == Kind::Wwv ? "WWV" : sc.kind == Kind::Wwvh ? "WWVH" : "WWVB";
    if (sc.kind != Kind::Wwvb) {
        WwvDecoder d(sc.rate);
        d.onSecond = onSecond; d.onFrame = onFrame; d.onTime = onTime; d.onStateChanged = onState;
        if (sc.pin) d.pinStation(sc.preset);
        else d.presetStation(sc.preset);
        for (std::size_t i = 0; i < pcm.size(); i += kChunk) {
            d.process(pcm.data() + i, std::min(kChunk, pcm.size() - i));
            const double at = static_cast<double>(i) / rate;
            if (sc.tagBySec >= 0.0 && at >= sc.tagBySec && r.tagOffAtSec < 0.0 &&
                std::string(stationName(d.station())) != want)
                r.tagOffAtSec = at;
        }
        r.station = stationName(d.station());
        r.tickRatioDb = d.diagnostics().tickBandRatioDb;
    }

    // (a) + (c); (b) for WWVB here, WWV/WWVH once every run's mean is known.
    if (r.timeGood == 0) fail(r, "never locked");
    if (r.lastMinuteGood < 40) fail(r, "last minute not labelled (" + std::to_string(r.lastMinuteGood) + ")");
    if (sc.rolloverUnix && !r.timeAfterRollover) fail(r, "no certified time after the rollover");
    if (r.wrongElsewhere > 0 || (r.wrongAtLeap > 0 && !sc.allowLeapEdgeWrong))
        fail(r, "WRONG timestamps: " + std::to_string(r.timeBad) + " time, " +
                    std::to_string(r.labelBad) + " label -- first: " + r.firstWrong);
    if (std::string(r.station) != want) fail(r, std::string("station tagged ") + r.station);
    if (r.tagOffAtSec >= 0.0) {
        char buf[96];
        std::snprintf(buf, sizeof buf, "%s tag not held at %.0f s", want, r.tagOffAtSec);
        fail(r, buf);
    }
    if (sc.kind == Kind::Wwvb) {
        for (double e : r.edgeErrMs) {
            if (std::fabs(e) > 2.0) { fail(r, "WWVB edge error beyond 2 ms"); break; }
        }
    }
    return r;
}

struct Stats { double mean = 0.0, min = 0.0, max = 0.0; std::size_t n = 0; };

Stats stats(const std::vector<double>& v) {
    Stats s;
    if (v.empty()) return s;
    s.n = v.size();
    s.min = *std::min_element(v.begin(), v.end());
    s.max = *std::max_element(v.begin(), v.end());
    double sum = 0.0;
    for (double x : v) sum += x;
    s.mean = sum / static_cast<double>(v.size());
    return s;
}

} // namespace

int main() {
    const double kClean = std::numeric_limits<double>::quiet_NaN();
    // 2024-12-31T23:53Z: a leap year's day 366 rolling into 2025 while locked.
    constexpr long long kRollStart = 1735689180;
    constexpr long long kRollover = 1735689600;
    // 2016-12-31T23:52Z: the real 2016 leap second, 23:59:60 then 2017-01-01.
    constexpr long long kLeapStart = 1483228320;
    constexpr long long kLeapAfter = 1483228800;

    std::vector<Scenario> scs;
    unsigned seed = 1;
    for (Kind k : {Kind::Wwv, Kind::Wwvh, Kind::Wwvb}) {
        const bool b = k == Kind::Wwvb;
        // Offsets are fractions of the decimation block (WWV 5 ms, WWVB 10 ms),
        // off the sample grid; WWVB's include the old dead zone at 40-60%.
        const std::vector<double> offs = b
            ? std::vector<double>{0.0, 3.71, 4.23, 5.02, 5.57, 6.04, 9.13}
            : std::vector<double>{0.0, 1.29, 2.52, 3.87};
        for (int rate : {12000, 24000})
            for (double off : offs)
                for (double snr : {kClean, b ? 10.0 : 6.0}) {
                    Scenario sc;
                    sc.kind = k; sc.rate = rate; sc.offsetMs = off; sc.snrDb = snr;
                    sc.seed = seed++; sc.startUnix = kRollStart; sc.minutes = 10;
                    sc.rolloverUnix = kRollover; sc.note = "rollover";
                    scs.push_back(sc);
                }
    }
    for (Kind k : {Kind::Wwv, Kind::Wwvh, Kind::Wwvb}) {
        const bool b = k == Kind::Wwvb;
        struct Variant { bool insert, warn, allowLeapEdgeWrong; const char* note; };
        for (const Variant& v : {Variant{true, true, false, "leap announced"},
                                 Variant{false, true, false, "leap warned, none inserted"},
                                 Variant{true, false, true, "leap UNannounced"}})
            for (int rate : {12000, 24000})
                for (double snr : {kClean, b ? 10.0 : 6.0}) {
                    if (k == Kind::Wwvh && (rate != 12000 || !std::isnan(snr))) continue;
                    Scenario sc;
                    sc.kind = k; sc.rate = rate; sc.offsetMs = b ? 5.57 : 2.52; sc.snrDb = snr;
                    sc.seed = seed++; sc.startUnix = kLeapStart; sc.minutes = 12;
                    sc.leapAfterUnix = kLeapAfter; sc.leapInsert = v.insert;
                    sc.leapWarn = v.warn; sc.allowLeapEdgeWrong = v.allowLeapEdgeWrong;
                    sc.note = v.note;
                    scs.push_back(sc);
                }
    }

    // Station tag: carried into a restarted decoder, fixed by a WWV-only
    // carrier, and held with both stations heard. One rate and offset -- none
    // of this is about timing, and every other check still applies.
    auto tagScenario = [&](Kind k, const char* note) {
        Scenario sc;
        sc.kind = k; sc.rate = 12000; sc.offsetMs = 1.29; sc.seed = seed++;
        sc.startUnix = kRollStart; sc.minutes = 10; sc.rolloverUnix = kRollover; sc.note = note;
        return sc;
    };
    {
        Scenario sc = tagScenario(Kind::Wwv, "tag: preset WWV held from the first second");
        sc.preset = ClockStation::Wwv; sc.tagBySec = 0.0;
        scs.push_back(sc);
    }
    {
        Scenario sc = tagScenario(Kind::Wwvh, "tag: preset WWV on WWVH, corrected");
        sc.preset = ClockStation::Wwv; sc.tagBySec = 120.0;
        scs.push_back(sc);
    }
    {
        Scenario sc = tagScenario(Kind::Wwvh, "tag: pinned WWV is never judged");
        sc.preset = ClockStation::Wwv; sc.pin = true; sc.wantStation = "WWV"; sc.tagBySec = 0.0;
        scs.push_back(sc);
    }
    // WWVH's tick at 0.6 of WWV's reads about +1.4 dB here: between the
    // +0.8 dB that holds a tag and the +1.8 dB that adopts one. At 0.7 it reads
    // about +0.7 dB, below both.
    {
        Scenario sc = tagScenario(Kind::Wwv, "tag: both heard at +1.4 dB, WWV held");
        sc.otherTickAmp = 0.6; sc.preset = ClockStation::Wwv; sc.tagBySec = 0.0;
        scs.push_back(sc);
    }
    {
        Scenario sc = tagScenario(Kind::Wwv, "tag: both heard at +1.4 dB, none adopted");
        sc.otherTickAmp = 0.6; sc.wantStation = "unknown";
        scs.push_back(sc);
    }
    {
        Scenario sc = tagScenario(Kind::Wwv, "tag: both heard at +0.7 dB, WWV released");
        sc.otherTickAmp = 0.7; sc.preset = ClockStation::Wwv; sc.wantStation = "unknown";
        scs.push_back(sc);
    }

    std::vector<Result> results(scs.size());
    std::atomic<std::size_t> next{0};
    unsigned nThreads = std::max(1u, std::thread::hardware_concurrency());
    std::vector<std::thread> pool;
    for (unsigned t = 0; t < nThreads; ++t) {
        pool.emplace_back([&] {
            for (std::size_t i; (i = next.fetch_add(1)) < scs.size();) results[i] = run(scs[i]);
        });
    }
    for (std::thread& t : pool) t.join();

    // Station mean bias over every run (clean and noisy): the calibration the
    // +/-3 ms band is centred on.
    std::map<Kind, std::vector<double>> all;
    for (std::size_t i = 0; i < scs.size(); ++i)
        for (double e : results[i].edgeErrMs) all[scs[i].kind].push_back(e);
    std::map<Kind, Stats> stationStats;
    for (auto& kv : all) stationStats[kv.first] = stats(kv.second);

    for (std::size_t i = 0; i < scs.size(); ++i) {
        const Scenario& sc = scs[i];
        Result& r = results[i];
        if (sc.kind != Kind::Wwvb) {
            const double mu = stationStats[sc.kind].mean;
            for (double e : r.edgeErrMs) {
                if (std::fabs(e - mu) > 3.0) {
                    char buf[96];
                    std::snprintf(buf, sizeof buf, "edge %+.2f ms is beyond %s mean %+.2f +/-3 ms",
                                  e, kindName(sc.kind), mu);
                    fail(r, buf);
                    break;
                }
            }
        }
    }

    int failed = 0;
    std::printf("%-4s %-4s %-6s %-6s %-5s | %-6s %-9s %-10s | %-44s | %-14s | %s\n", "res", "stn",
                "rate", "offset", "snr", "lock", "time ok/X", "label ok/X",
                "measured edge error, ms (n mean min max)", "tag (tick dB)", "scenario");
    for (std::size_t i = 0; i < scs.size(); ++i) {
        const Scenario& sc = scs[i];
        const Result& r = results[i];
        const Stats s = stats(r.edgeErrMs);
        char snr[16];
        if (std::isnan(sc.snrDb)) std::snprintf(snr, sizeof snr, "clean");
        else std::snprintf(snr, sizeof snr, "%.0fdB", sc.snrDb);
        char tag[32];
        if (std::isnan(r.tickRatioDb)) std::snprintf(tag, sizeof tag, "%s", r.station);
        else std::snprintf(tag, sizeof tag, "%s %+.1f", r.station, r.tickRatioDb);
        std::printf("%-4s %-4s %-6d %5.2fms %-5s | %5.0fs %4d/%-4d %5d/%-4d | n=%-4zu %+6.2f %+6.2f %+6.2f "
                    "unmeas=%-3d | %-14s | %s%s%s\n",
                    r.pass ? "ok" : "FAIL", kindName(sc.kind), sc.rate, sc.offsetMs, snr,
                    r.lockAtSec, r.timeGood, r.timeBad, r.labelGood, r.labelBad, s.n, s.mean, s.min,
                    s.max, r.unmeasured, tag, sc.note, r.pass ? "" : " -- ", r.why.c_str());
        if (!r.pass) ++failed;
    }

    std::printf("\nstation edge bias over all runs (measured edges after lock):\n");
    for (auto& kv : stationStats) {
        const Stats& s = kv.second;
        std::printf("  %-4s n=%-6zu mean %+7.3f ms  min %+6.2f  max %+6.2f", kindName(kv.first), s.n,
                    s.mean, s.min, s.max);
        if (kv.first != Kind::Wwvb) {
            // edge = (window start + shift - kNominalDelaySamples) * decim at a
            // 200 Hz series: one series sample is 5 ms, so the value that would
            // put these synthetic edges on truth is 7 + mean / 5.
            std::printf("  -> synthetic-exact kNominalDelaySamples = %.3f (current 7)", 7.0 + s.mean / 5.0);
        }
        std::printf("\n");
    }
    std::printf("\n%zu scenarios, %d failed\n", scs.size(), failed);
    return failed == 0 ? 0 : 1;
}
