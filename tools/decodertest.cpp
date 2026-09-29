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
#include <cstdlib>
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
    // WWV/WWVH, which are IQ: where the carrier really is in the baseband and
    // where the decoder is told it is (the dial's offset), the receiver's
    // sample clock off by ppm, a second path (delay, amplitude, the two beating
    // at fadeHz), and NIST's 500/600 Hz tones at 50% outside the protected zone.
    double carrierHz = 0.0;
    double nominalHz = 0.0;
    double ppm = 0.0;
    double pathMs = 0.0, pathAmp = 0.0, fadeHz = 0.1;
    bool tones = false;
    // Edge error allowed, ms, for WWV/WWVH: every measured edge after lock
    // within this of the truth (after `expectMs`, the multipath's weighted mean).
    double tolMs = 0.1;
    double expectMs = 0.0;
    bool expectNoTiming = false;
    double meanTolMs = -1.0;       // >= 0: the run's mean edge error within this of expectMs   // no tick of the pinned station on air: nothing served
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

double trueRate(const Scenario& sc) { return sc.rate * (1.0 + sc.ppm * 1e-6); }

std::vector<float> render(const Scenario& sc, const std::vector<Second>& secs) {
    const bool b = sc.kind == Kind::Wwvb;
    const double lowAmp = std::pow(10.0, -17.0 / 20.0);
    const double lowLen[3] = {0.200, 0.500, 0.800};
    const double pulseLen[3] = {0.170, 0.470, 0.770};
    std::mt19937_64 rng(sc.seed);
    std::normal_distribution<double> gauss(0.0, 1.0);
    const double off = sc.offsetMs / 1000.0;

    if (b) {
        // WWVB is USB audio still: the carrier at 1000 Hz, PWM on its amplitude.
        double peak = 1.0, sigma = 0.0;
        if (!std::isnan(sc.snrDb)) {
            sigma = (1.0 / std::sqrt(2.0)) * std::pow(10.0, -sc.snrDb / 20.0);
            peak += 4.0 * sigma;
        }
        const double scale = 28000.0 / peak / 32768.0;
        const std::size_t n = secs.size() * static_cast<std::size_t>(sc.rate);
        std::vector<float> out(n);
        for (std::size_t i = 0; i < n; ++i) {
            const double t = static_cast<double>(i) / sc.rate - off;
            const long long p = static_cast<long long>(std::floor(t));
            const double tau = t - static_cast<double>(p);
            const int sym = (p >= 0 && p < static_cast<long long>(secs.size())) ? secs[p].sym : -1;
            const double amp = (sym >= 0 && tau < lowLen[sym]) ? lowAmp : 1.0;
            double x = amp * std::sin(2.0 * kPi * 1000.0 * tau);
            if (sigma > 0.0) x += sigma * gauss(rng);
            out[i] = static_cast<float>(x * scale);
        }
        return out;
    }

    // WWV/WWVH as the receiver's IQ: AM on a carrier at carrierHz. The
    // modulation, per NIST SP 432: the 100 Hz subcarrier at 50% from +30 ms for
    // the pulse's length (none in second 0); the tick, 5 ms of 1000 Hz (WWV) or
    // 1200 Hz (WWVH) at 80%, starting on the second, except in seconds 29 and
    // 59; in second 0 instead an 800 ms tone at the tick's frequency.
    const double tickHz = sc.kind == Kind::Wwvh ? 1200.0 : 1000.0;
    const double otherHz = tickHz == 1000.0 ? 1200.0 : 1000.0;
    // The tick's envelope, band-limited as radiod's filter and the
    // transmitter's audio chain leave it: its edges Gaussian with a 0.1 ms
    // sigma, nothing of note above 5 kHz. A rectangle sampled as it is would be
    // aliased, and its edge only findable to the nearest sample.
    auto tickEnv = [](double tau) {
        const double k = 1.0 / (std::sqrt(2.0) * 0.0001);
        if (tau < -0.001 || tau > 0.006) return 0.0;
        return 0.5 * (std::erf(tau * k) - std::erf((tau - 0.005) * k));
    };
    auto mod = [&](double t) {
        const long long p = static_cast<long long>(std::floor(t));
        const double tau = t - static_cast<double>(p);
        if (p < 0 || p >= static_cast<long long>(secs.size())) return 0.0;
        const int sym = secs[static_cast<std::size_t>(p)].sym;
        const int sof = static_cast<int>(((p % 60) + 60) % 60);
        double m = 0.0;
        if (sym >= 0 && tau >= 0.030 && tau < 0.030 + pulseLen[sym])
            m += 0.5 * std::sin(2.0 * kPi * 100.0 * tau);
        const bool minuteTone = sof == 0 && secs[static_cast<std::size_t>(p)].posix >= 0;
        if (minuteTone && tau < 0.8) m += 0.8 * std::sin(2.0 * kPi * tickHz * tau);
        else if (sof != 29 && sof != 59) m += 0.8 * tickEnv(tau) * std::sin(2.0 * kPi * tickHz * tau);
        // The next second's tick starts rising a few sigma before it.
        const long long nx = p + 1;
        const int nsof = static_cast<int>(nx % 60);
        if (tau > 0.999 && nx < static_cast<long long>(secs.size()) && nsof != 0 && nsof != 29 && nsof != 59)
            m += 0.8 * tickEnv(tau - 1.0) * std::sin(2.0 * kPi * tickHz * (tau - 1.0));
        if (sc.otherTickAmp > 0.0 && tau >= 0.015 && tau < 0.020)
            m += sc.otherTickAmp * std::sin(2.0 * kPi * otherHz * (tau - 0.015));
        if (sc.tones && tau >= 0.030 && tau < 0.990 && !minuteTone)
            m += 0.5 * std::sin(2.0 * kPi * ((p / 60) % 2 ? 600.0 : 500.0) * tau);
        return m;
    };

    double peak = 2.9 * (1.0 + sc.pathAmp), sigma = 0.0;
    if (!std::isnan(sc.snrDb)) {
        // snrDb is carrier over the noise in the whole 12 kHz (or 24 kHz): a
        // complex sigma per rail such that |noise|^2 averages 10^(-snr/10).
        sigma = std::sqrt(0.5 * std::pow(10.0, -sc.snrDb / 10.0));
        peak += 4.0 * sigma;
    }
    const double scale = 28000.0 / peak / 32768.0;
    const double rate = trueRate(sc);
    const std::size_t n = secs.size() * static_cast<std::size_t>(sc.rate);
    std::vector<float> out(2 * n);
    const double phi0 = 0.7 + 0.37 * sc.seed;
    for (std::size_t i = 0; i < n; ++i) {
        const double tt = static_cast<double>(i) / rate;
        const double t = tt - off;
        double re = (1.0 + mod(t)), im = 0.0;
        double ph = 2.0 * kPi * sc.carrierHz * tt + phi0;
        double zr = re * std::cos(ph), zi = re * std::sin(ph);
        if (sc.pathAmp > 0.0) {
            const double t2 = t - sc.pathMs / 1000.0;
            const double ph2 = ph + 2.0 * kPi * sc.fadeHz * tt + 1.9;
            const double r2 = sc.pathAmp * (1.0 + mod(t2));
            zr += r2 * std::cos(ph2); zi += r2 * std::sin(ph2);
        }
        (void)im;
        if (sigma > 0.0) { zr += sigma * gauss(rng); zi += sigma * gauss(rng); }
        out[2 * i] = static_cast<float>(zr * scale);
        out[2 * i + 1] = static_cast<float>(zi * scale);
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
    double servableAtSec = -1.0;       // first servable edge (WWV: the tick timer locked)
    double bcdMinusTickMs = std::numeric_limits<double>::quiet_NaN();
    double tickSnrDb = std::numeric_limits<double>::quiet_NaN();
    double carrierHz = std::numeric_limits<double>::quiet_NaN();
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
    const double rate = sc.rate;                 // the stream's nominal rate, as Source counts
    const double trate = sc.kind == Kind::Wwvb ? rate : trueRate(sc);   // its real one
    const double offSamples = sc.offsetMs / 1000.0 * trate;
    const long long nSecs = static_cast<long long>(secs.size());

    auto physIndex = [&](std::int64_t edge) {
        return std::llround((static_cast<double>(edge) - offSamples) / trate);
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
        const double at = std::isfinite(i.edgeSampleExact) ? i.edgeSampleExact
                                                           : static_cast<double>(i.edgeSample);
        const double errMs = (at - (static_cast<double>(p) * trate + offSamples)) / trate * 1000.0;
        if (i.edgeServable && r.servableAtSec < 0) r.servableAtSec = at / trate;
        if (seenTime) {
            if (i.edgeMeasured && i.edgeServable) {
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
        WwvDecoder d(sc.rate, sc.nominalHz);
        d.onSecond = onSecond; d.onFrame = onFrame; d.onTime = onTime; d.onStateChanged = onState;
        if (sc.pin) d.pinStation(sc.preset);
        else d.presetStation(sc.preset);
        const std::size_t frames = pcm.size() / 2;
        for (std::size_t i = 0; i < frames; i += kChunk) {
            d.process(pcm.data() + 2 * i, std::min(kChunk, frames - i));
            const double at = static_cast<double>(i) / rate;
            if (sc.tagBySec >= 0.0 && at >= sc.tagBySec && r.tagOffAtSec < 0.0 &&
                std::string(stationName(d.station())) != want)
                r.tagOffAtSec = at;
        }
        r.station = stationName(d.station());
        const ClockDecoderDiagnostics g = d.diagnostics();
        r.tickRatioDb = g.tickBandRatioDb;
        r.bcdMinusTickMs = g.bcdMinusTickMs;
        r.tickSnrDb = g.tickSnrDb;
        r.carrierHz = g.carrierOffsetHz;
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
    if (sc.kind != Kind::Wwvb) {
        for (double e : r.edgeErrMs) {
            if (std::fabs(e - sc.expectMs) > sc.tolMs) {
                char buf[96];
                std::snprintf(buf, sizeof buf, "tick edge %+.3f ms, beyond %+.3f +/-%.3f ms",
                              e, sc.expectMs, sc.tolMs);
                fail(r, buf);
                break;
            }
        }
        if (sc.meanTolMs >= 0.0 && !r.edgeErrMs.empty()) {
            double sum = 0.0;
            for (double e : r.edgeErrMs) sum += e;
            const double mean = sum / static_cast<double>(r.edgeErrMs.size());
            if (std::fabs(mean - sc.expectMs) > sc.meanTolMs) {
                char buf[96];
                std::snprintf(buf, sizeof buf, "mean edge %+.3f ms, beyond %+.3f +/-%.3f ms",
                              mean, sc.expectMs, sc.meanTolMs);
                fail(r, buf);
            }
        }
        if (sc.expectNoTiming) {
            if (r.servableAtSec >= 0.0) fail(r, "served a tick that is not the pinned station's");
        } else if (r.edgeErrMs.empty()) {
            fail(r, "no tick-timed edge");
        }
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

// A recording instead of a synthesis: tools/iqrecord's WAV (16-bit I/Q), fed
// to the decoder as Source feeds it, every second printed as CSV on stdout --
// the edge's sample index, whether it was measured and servable, the symbol --
// with the decoder's time events and a diagnostics line every ten seconds, for
// scripts to take on from there (the recording's capture stamps, the path).
int decodeWav(const char* path, double carrierOffsetHz) {
    std::FILE* f = std::fopen(path, "rb");
    if (!f) { std::fprintf(stderr, "cannot open %s\n", path); return 2; }
    unsigned char hdr[44];
    if (std::fread(hdr, 1, 44, f) != 44) { std::fclose(f); return 2; }
    auto u16 = [&](int o) { return static_cast<unsigned>(hdr[o] | hdr[o + 1] << 8); };
    auto u32 = [&](int o) { return static_cast<unsigned>(hdr[o] | hdr[o + 1] << 8 | hdr[o + 2] << 16 | hdr[o + 3] << 24); };
    const int channels = static_cast<int>(u16(22));
    const int rate = static_cast<int>(u32(24));
    if (channels != 2 || u16(34) != 16) { std::fprintf(stderr, "%s: want 16-bit stereo I/Q\n", path); return 2; }
    WwvDecoder d(rate, carrierOffsetHz);
    std::int64_t consumed = 0;
    d.onSecond = [&](const ClockSecondInfo& i) {
        double e = 0.0;
        for (float v : i.envelope) e += v;
        if (!i.envelope.empty()) e /= static_cast<double>(i.envelope.size());
        std::printf("sec,%lld,%.4f,%d,%d,%d,%d,%.3f,%.3f\n", static_cast<long long>(i.edgeSample), i.edgeSampleExact,
                    i.edgeMeasured ? 1 : 0, i.edgeServable ? 1 : 0, i.secondOfFrame, static_cast<int>(i.symbol),
                    i.confidence, e);
    };
    d.onFrame = [&](const ClockFrameInfo& fr) {
        std::printf("frame,%lld,%02d:%02d,doy %d,%.2f\n", static_cast<long long>(fr.frameStartSample), fr.hour,
                    fr.minute, fr.doy, fr.frameConfidence);
    };
    d.onTime = [&](const ClockTimeInfo& t) {
        std::printf("time,%lld,%.4f,20%02d doy %d %02d:%02d,sof %d,q %.2f,%s\n", static_cast<long long>(t.lastEdgeSample),
                    t.lastEdgeSampleExact, t.year2, t.doy, t.hour, t.minute, t.lastEdgeSecondOfFrame, t.quality,
                    stationName(t.station));
    };
    std::vector<std::int16_t> buf(2 * 2400);
    std::vector<float> iq(buf.size());
    double nextDiag = 10.0;
    for (;;) {
        const std::size_t got = std::fread(buf.data(), sizeof(std::int16_t), buf.size(), f);
        if (got < 2) break;
        for (std::size_t i = 0; i < got; ++i) iq[i] = buf[i] * (1.0f / 32768.0f);
        d.process(iq.data(), got / 2);
        consumed += static_cast<std::int64_t>(got / 2);
        if (static_cast<double>(consumed) / rate >= nextDiag) {
            nextDiag += 10.0;
            const ClockDecoderDiagnostics g = d.diagnostics();
            std::printf("diag,%.0f,state %d,station %s,fold %d,tick %d,tickSNR %.1f,bcd-tick %.2f,carrier %+.2f,"
                        "tag %.1f,frames %d,q %.2f\n",
                        static_cast<double>(consumed) / rate, static_cast<int>(d.state()), stationName(d.station()),
                        g.toneDetected ? 1 : 0, g.tickTiming ? 1 : 0, g.tickSnrDb, g.bcdMinusTickMs,
                        g.carrierOffsetHz, g.tickBandRatioDb, g.framesInWindow, g.voteQuality);
        }
    }
    std::fclose(f);
    std::printf("end,%lld,%d\n", static_cast<long long>(consumed), rate);
    return 0;
}

int main(int argc, char** argv) {
    if (argc > 1 && std::string(argv[1]) == "--wav") {
        if (argc < 3) { std::fprintf(stderr, "usage: %s --wav FILE [CARRIER_OFFSET_HZ]\n", argv[0]); return 2; }
        return decodeWav(argv[2], argc > 3 ? std::atof(argv[3]) : 0.0);
    }
    const char* only = argc > 1 ? argv[1] : nullptr;
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
                    sc.tolMs = std::isnan(snr) ? 0.01 : 0.4;
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
                    sc.tolMs = std::isnan(snr) ? 0.01 : 0.4;
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
        sc.tolMs = 0.02;
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
        sc.expectNoTiming = true;
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

    // The IQ front end: weak signals, a carrier off where the dial says, a
    // receiver clock off by tens of ppm, NIST's programme tones, and a second
    // path. snrDb here is carrier over the noise in the whole 12 kHz.
    auto iqScenario = [&](Kind k, double snr, const char* note) {
        Scenario sc;
        sc.kind = k; sc.rate = 12000; sc.offsetMs = 2.52; sc.snrDb = snr; sc.seed = seed++;
        sc.startUnix = kRollStart; sc.minutes = 10; sc.rolloverUnix = kRollover; sc.note = note;
        sc.tolMs = 0.4; sc.meanTolMs = 0.05;
        return sc;
    };
    for (Kind k : {Kind::Wwv, Kind::Wwvh}) {
        for (double snr : {0.0, -5.0, -10.0}) {
            Scenario sc = iqScenario(k, snr, "weak");
            sc.tolMs = snr <= -10.0 ? 1.5 : 0.8;
            sc.meanTolMs = snr <= -10.0 ? 0.15 : 0.05;
            scs.push_back(sc);
        }
    }
    {
        Scenario sc = iqScenario(Kind::Wwv, 3.0, "carrier +23.7 Hz, dial on it");
        sc.carrierHz = 23.7;
        scs.push_back(sc);
    }
    {
        Scenario sc = iqScenario(Kind::Wwv, 3.0, "dial 1 kHz below, carrier -8.1 Hz off that");
        sc.nominalHz = 1000.0; sc.carrierHz = 991.9;
        scs.push_back(sc);
    }
    {
        Scenario sc = iqScenario(Kind::Wwv, 3.0, "receiver clock +40 ppm");
        sc.ppm = 40.0;
        scs.push_back(sc);
    }
    for (double ppm : {-60.0, -20.0, 60.0}) for (Kind k : {Kind::Wwv, Kind::Wwvh}) {
        char* note = new char[40];
        std::snprintf(note, 40, "receiver clock %+.0f ppm", ppm);
        Scenario sc = iqScenario(k, 3.0, note);
        sc.ppm = ppm;
        // Past any real receiver's clock: the second or two before the slope
        // is fitted may be off by half a millisecond; the mean may not.
        if (std::fabs(ppm) >= 60.0) sc.tolMs = 0.6;
        scs.push_back(sc);
    }
    {
        Scenario sc = iqScenario(Kind::Wwv, 3.0, "500/600 Hz tones at 50%");
        sc.tones = true;
        scs.push_back(sc);
    }
    {
        // Two modes 1 ms apart, the second at 0.7, beating at 0.1 Hz. Coherent
        // AM weights each by its amplitude times the carrier they share: over
        // a fade cycle 1 and 0.49, so the tick is timed near 0.33 ms, the
        // modes' power-weighted mean -- but not every second: with the modes
        // in antiphase the carrier nearly cancels, the second path's weight
        // goes negative and the edge swings by up to 2 ms. That is what AM
        // demodulated on a carrier multipath has part-cancelled does; the
        // average over the fades is what is checked.
        Scenario sc = iqScenario(Kind::Wwv, 3.0, "two paths 1 ms apart, 0.7");
        sc.pathMs = 1.0; sc.pathAmp = 0.7; sc.fadeHz = 0.1;
        sc.expectMs = 0.329; sc.tolMs = 2.5; sc.meanTolMs = 0.3;
        scs.push_back(sc);
    }

    if (only) {
        std::vector<Scenario> keep;
        for (const Scenario& sc : scs) {
            char key[160];
            std::snprintf(key, sizeof key, "%s/%d/%.2f/%s/%s", kindName(sc.kind), sc.rate, sc.offsetMs,
                          std::isnan(sc.snrDb) ? "clean" : std::to_string(static_cast<int>(sc.snrDb)).c_str(), sc.note);
            if (std::string(key).find(only) != std::string::npos) keep.push_back(sc);
        }
        scs.swap(keep);
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


    int failed = 0;
    std::printf("%-4s %-4s %-6s %-6s %-5s | %-6s %-6s %-9s %-10s | %-44s | %-14s | %-22s | %s\n",
                "res", "stn", "rate", "offset", "snr", "tick", "lock", "time ok/X", "label ok/X",
                "served edge error, ms (n mean min max)", "tag (tick dB)",
                "tickSNR bcd-tick carrier", "scenario");
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
        char diag[48] = "";
        if (sc.kind != Kind::Wwvb)
            std::snprintf(diag, sizeof diag, "%4.1fdB %+6.2fms %+6.2fHz", r.tickSnrDb, r.bcdMinusTickMs, r.carrierHz);
        std::printf("%-4s %-4s %-6d %5.2fms %-5s | %5.0fs %5.0fs %4d/%-4d %5d/%-4d | n=%-4zu %+7.3f %+7.3f %+7.3f "
                    "unmeas=%-3d | %-14s | %-22s | %s%s%s\n",
                    r.pass ? "ok" : "FAIL", kindName(sc.kind), sc.rate, sc.offsetMs, snr, r.servableAtSec,
                    r.lockAtSec, r.timeGood, r.timeBad, r.labelGood, r.labelBad, s.n, s.mean, s.min,
                    s.max, r.unmeasured, tag, diag, sc.note, r.pass ? "" : " -- ", r.why.c_str());
        if (!r.pass) ++failed;
    }

    std::printf("\nstation edge bias over all runs (measured edges after lock):\n");
    for (auto& kv : stationStats) {
        const Stats& s = kv.second;
        std::printf("  %-4s n=%-6zu mean %+7.3f ms  min %+6.2f  max %+6.2f", kindName(kv.first), s.n,
                    s.mean, s.min, s.max);
        std::printf("\n");
    }
    // The BCD check's own delay: bcd - tick is (the smoothed shift less
    // kBcdEdgeDelaySamples) at a 200 Hz series, so the value that puts the
    // synthetic pulse on the tick is the current one plus the mean / 5 ms.
    std::vector<double> bcd;
    for (std::size_t i = 0; i < scs.size(); ++i)
        if (scs[i].kind != Kind::Wwvb && scs[i].pathAmp == 0.0 && std::isfinite(results[i].bcdMinusTickMs))
            bcd.push_back(results[i].bcdMinusTickMs);
    if (!bcd.empty()) {
        const Stats b = stats(bcd);
        std::printf("  BCD minus tick over %zu runs: mean %+.3f ms (min %+.3f max %+.3f) -> "
                    "synthetic-exact kBcdEdgeDelaySamples = current + %.3f\n",
                    b.n, b.mean, b.min, b.max, b.mean / 5.0);
    }
    std::printf("\n%zu scenarios, %d failed\n", scs.size(), failed);
    return failed == 0 ? 0 : 1;
}
