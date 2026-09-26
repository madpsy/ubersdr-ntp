// Offline test for the DCF77 decoder, AM and PM both.
//
// Synthesises DCF77 as an UberSDR "iq" session delivers it -- complex baseband,
// carrier at (or near) 0 Hz -- with the time code, the 15% carrier cut and the
// 512-chip phase modulation all per PTB, true second edges at sub-sample
// offsets, and white noise at a given carrier-to-noise density. Then decodes it
// the way Source consumes a decoder: a `time` event composed against the last
// frame's second 0, extended by whole seconds of samples.
//
// Checks, per scenario:
//   (a) it locks, and every label it produces is the true UTC of that edge --
//       across a CET->CEST change, and around an announced leap second, where
//       the 23:59:60 edge must not be labelled at all;
//   (b) edge error: every PM-timed edge past the tracker's first five within
//       0.25 ms of truth; AM-timed, with
//       no PM on air, 99% within 2.5 ms and a mean inside 0.3 ms once the
//       tracker has had twenty seconds; and whichever the scenario expects is
//       the one timing it at the end;
//   (c) AM and PM carrying DIFFERENT times never certify anything.
//
// Then, if tools/testdata/dcf77_live.wav is present (300 s of real DCF77 IQ
// from a KiwiSDR 35 km from Mainflingen), decodes that and reports what it can
// with the one truth it has -- the recording's README shows 09:20 UTC on Sunday
// 2026-05-24 -- plus whether consecutive minutes are one minute apart, whether
// AM and PM agreed, and how far AM's edges sit from PM's.
//
// Exit status 0 when every check passes.

#include "CivilTime.h"
#include "clock/Dcf77Decoder.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <fstream>
#include <limits>
#include <random>
#include <string>
#include <vector>

using namespace clockdec;
namespace civ = ubersdr_ntp;

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kChipSec = 120.0 / 77500.0;
constexpr double kDev = 15.6 * kPi / 180.0;

std::array<int, 512> chips() {
    std::array<int, 512> c{};
    unsigned l = 0;
    for (int i = 0; i < 512; ++i) {
        c[static_cast<std::size_t>(i)] = static_cast<int>(l & 1u);
        const unsigned chip = l & 1u;
        l >>= 1;
        if (chip ^ (l == 0 ? 1u : 0u)) l ^= 0x110u;
    }
    return c;
}

// EU summer time: last Sunday of March 01:00 UTC to last Sunday of October.
long long lastSundayUtc(int y, unsigned m) {
    const long long d = civ::daysFromCivil(y, m, 31);
    const long long wd = civ::floorMod(d + 3, 7);   // 0 = Monday
    return (d - civ::floorMod(wd - 6, 7)) * 86400LL + 3600LL;
}
bool cestAt(long long unix) {
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
    double carrierHz = 0.0;       // carrier's offset in the baseband
    double qrmHz = 0.0;           // a steady interfering tone, this far off DC...
    double qrmAmp = 0.0;          // ...at this amplitude (the carrier is 1)
    bool fade = false;            // the carrier gone for seconds 53-58 of minutes 3 and 6
    bool fillCut = false;         // second 42 of minutes 3 and 6 sent with no cut
    int fillMinute = -1;          // this minute (from 0) sent with no cut at...
    std::vector<int> fillSecs;    // ...these seconds
    bool am = true, pm = true;
    bool invert = false;          // conjugate I/Q, as a flipped spectrum would give
    bool conflict = false;        // AM sends a different minute from PM
    long long startUnix = 0;
    int minutes = 7;
    long long leapAt = 0;         // unix of the midnight a leap second precedes
    bool expectLock = true;
    bool expectPmTiming = true;
    double maxLabelGapSec = 0.0;  // once labelling, never longer than this without (0: unchecked)
    double minLabelGapSec = 0.0;  // once labelling, at least once this long without (0: unchecked)
    unsigned seed = 1;
};

struct Sec {
    double t = 0.0;               // stream seconds
    long long label = 0;          // UTC ms; -1 for 23:59:60
    bool faded = false;           // no carrier this second
    int amBit = 0;                // 0, 1, or 2 = no cut
    int pmBit = 0;
};

// The time code for the minute whose second 0 is `T` (unix, UTC).
std::array<int, 61> timeCode(long long T, long long leapAt, std::mt19937& rng) {
    std::array<int, 61> b{};
    std::uniform_int_distribution<int> coin(0, 1);
    for (int s = 1; s <= 14; ++s) b[static_cast<std::size_t>(s)] = coin(rng);   // weather
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
        for (int x : w) { b[static_cast<std::size_t>(s++)] = (v & x) ? 1 : 0; }
    };
    auto bcd = [](int v) { return (v / 10) << 4 | (v % 10); };
    b[17] = cest ? 1 : 0;
    b[18] = cest ? 0 : 1;
    b[19] = (leapAt > 0 && T >= leapAt - 3600 && T < leapAt) ? 1 : 0;
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
    return b;
}

std::vector<Sec> buildSeconds(const Scenario& sc) {
    std::mt19937 rng(sc.seed * 7919u);
    std::vector<Sec> out;
    const double off = sc.offsetSamples / sc.rate;
    double shift = 0.0;
    for (int m = 0; m < sc.minutes; ++m) {
        const long long T = sc.startUnix + 60LL * m;
        const bool leapMin = sc.leapAt > 0 && T + 60 == sc.leapAt;
        const auto code = timeCode(T, sc.leapAt, rng);
        const auto other = timeCode(T + 3600, sc.leapAt, rng);   // for the conflict case
        const int n = leapMin ? 61 : 60;
        for (int s = 0; s < n; ++s) {
            Sec x;
            x.t = 1.0 + off + static_cast<double>(T - sc.startUnix) + s + shift;
            x.label = s == 60 ? -1 : (T + s) * 1000LL;
            int bit = s < 59 ? code[static_cast<std::size_t>(s)] : 0;
            x.amBit = (s == 59 && !leapMin) || s == 60 ? 2 : bit;
            if (sc.conflict && s >= 21 && s <= 58) x.amBit = other[static_cast<std::size_t>(s)];
            x.pmBit = (s == 59 || s == 60) ? 0 : s <= 9 ? 1 : s <= 14 ? 0 : bit;
            x.faded = sc.fade && (m == 3 || m == 6) && s >= 53 && s <= 58;
            // A cut filled in -- what a burst of interference does to one
            // second's envelope -- which reads as a marker where none belongs.
            if (sc.fillCut && (m == 3 || m == 6) && s == 42) x.amBit = 2;
            if (m == sc.fillMinute && std::find(sc.fillSecs.begin(), sc.fillSecs.end(), s) != sc.fillSecs.end())
                x.amBit = 2;
            out.push_back(x);
        }
        if (leapMin) shift += 1.0;
    }
    return out;
}

struct Result {
    bool ok = true;
    std::string why;
    int labels = 0, wrong = 0;
    std::vector<double> pmErrMs, amErrMs;
    // PM edges again, at the decoder's own resolution rather than rounded to a
    // whole sample (ClockSecondInfo::edgeSampleExact).
    std::vector<double> pmExactErrMs;
    ClockDecoderDiagnostics diag;
    std::vector<std::uint8_t> from;
    // Measured seconds offered to serve time from (edgeServable), and any whose
    // flag disagreed with who was timing that second: only PM may serve.
    int servable = 0, servableMismatch = 0;
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
    const auto ch = chips();
    Dcf77Decoder dec(sc.rate, 0.0);

    long long frameStart = 0;
    bool haveFrame = false;
    long long lastLabelSample = -1;
    double worstGap = 0.0;
    dec.onFrame = [&](const ClockFrameInfo& f) {
        frameStart = f.frameStartSample;
        haveFrame = true;
        r.from.push_back(dec.diagnostics().lastFrameFrom);
        if (std::getenv("DCF77_STATS")) {
            const auto d = dec.diagnostics();
            std::printf("    frame from=%d state=%d pm=%d refusal=%d vote=%d q=%.2f\n", d.lastFrameFrom,
                        static_cast<int>(dec.state()), d.pmLocked, d.refusalReason, d.framesInWindow, d.voteQuality);
        }
    };
    dec.onSecond = [&](const ClockSecondInfo& i) {
        if (!i.edgeMeasured) return;
        if (i.edgeServable) ++r.servable;
        if (i.edgeServable != dec.diagnostics().timingFromPm) ++r.servableMismatch;
        const double t = static_cast<double>(i.edgeSample) / sc.rate;
        const Sec* s = nearest(secs, t);
        if (!s || std::fabs(s->t - t) > 0.1) return;
        const auto d = dec.diagnostics();
        (d.timingFromPm ? r.pmErrMs : r.amErrMs).push_back((t - s->t) * 1000.0);
        if (d.timingFromPm && std::isfinite(i.edgeSampleExact))
            r.pmExactErrMs.push_back((i.edgeSampleExact / sc.rate - s->t) * 1000.0);
    };
    dec.onTime = [&](const ClockTimeInfo& t) {
        if (!haveFrame) return;
        const long long base = civ::utcMsFromFields(t.year2, t.doy, t.hour, t.minute);
        const long long el = std::llround(static_cast<double>(t.lastEdgeSample - frameStart) / sc.rate);
        const long long got = base + el * 1000LL;
        const Sec* s = nearest(secs, static_cast<double>(t.lastEdgeSample) / sc.rate);
        if (lastLabelSample >= 0)
            worstGap = std::max(worstGap, static_cast<double>(t.lastEdgeSample - lastLabelSample) / sc.rate);
        lastLabelSample = t.lastEdgeSample;
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
            double amp = 1.0, ph = 0.0;
            if (t >= secs[0].t) {
                const Sec& s = secs[si];
                const double dt = t - s.t;
                if (sc.am && s.amBit != 2 && dt < (s.amBit ? 0.2 : 0.1)) amp = 0.15;
                if (s.faded) amp = 0.0;
                const double pt = dt - 0.2;
                if (sc.pm && pt >= 0.0 && pt < 512 * kChipSec) {
                    const int k = static_cast<int>(pt / kChipSec);
                    ph = (ch[static_cast<std::size_t>(k)] ^ s.pmBit) ? -kDev : kDev;
                }
            }
            ph += 2.0 * kPi * sc.carrierHz * t;
            double I = amp * std::cos(ph) + sigma * g(rng);
            double Q = amp * std::sin(ph) + sigma * g(rng);
            if (sc.qrmAmp > 0.0) {
                I += sc.qrmAmp * std::cos(2.0 * kPi * sc.qrmHz * t);
                Q += sc.qrmAmp * std::sin(2.0 * kPi * sc.qrmHz * t);
            }
            if (sc.invert) Q = -Q;
            buf.push_back(static_cast<float>(I));
            buf.push_back(static_cast<float>(Q));
        }
        dec.process(buf.data(), buf.size() / 2);
    }
    r.diag = dec.diagnostics();

    auto meanAbsMax = [](const std::vector<double>& v, double& mean, double& worst) {
        mean = 0.0; worst = 0.0;
        for (double x : v) { mean += x; worst = std::max(worst, std::fabs(x)); }
        if (!v.empty()) mean /= v.size();
    };
    double pm = 0, pmW = 0, am = 0, amW = 0;
    meanAbsMax(r.pmErrMs, pm, pmW);
    meanAbsMax(r.amErrMs, am, amW);
    if (sc.expectLock && r.labels < 60) fail(r, "never locked (" + std::to_string(r.labels) + " labels)");
    if (sc.maxLabelGapSec > 0.0 && worstGap > sc.maxLabelGapSec)
        fail(r, "went " + std::to_string(static_cast<int>(worstGap)) + " s without a label once locked");
    if (sc.minLabelGapSec > 0.0 && worstGap < sc.minLabelGapSec)
        fail(r, "never stopped labelling (worst gap " + std::to_string(static_cast<int>(worstGap)) + " s)");
    if (!sc.expectLock && r.labels > 0) fail(r, "certified a time it should have refused");
    if (sc.expectLock && sc.expectPmTiming && !r.diag.timingFromPm) fail(r, "not timed by PM at the end");
    if (sc.expectLock && !sc.expectPmTiming && r.diag.timingFromPm) fail(r, "timed by PM with no PM on air");
    // Only PM's edges may serve time: every measured second says so exactly
    // when PM timed it, and with no PM on air none does, locked or not.
    if (r.servableMismatch > 0)
        fail(r, std::to_string(r.servableMismatch) + " second(s) servable other than exactly when PM timed them");
    if (!sc.pm && r.servable > 0)
        fail(r, std::to_string(r.servable) + " AM-timed second(s) offered to serve time");
    if (sc.pm && sc.expectLock && sc.expectPmTiming && r.servable == 0) fail(r, "PM timing, yet nothing servable");
    // PM past its tracker's first five edges -- the first is one raw
    // measurement, and nothing downstream uses an edge that early (Source
    // needs a locked, anchored minute first).
    if (sc.expectPmTiming && r.pmErrMs.size() > 5) {
        double w = 0;
        for (std::size_t i = 5; i < r.pmErrMs.size(); ++i) w = std::max(w, std::fabs(r.pmErrMs[i]));
        if (w > 0.25) fail(r, "PM edge error " + std::to_string(w) + " ms");
    }
    // The unrounded edge (ClockSecondInfo::edgeSampleExact) must be unbiased:
    // its signed mean error within 20 us, wherever on the sample grid the true
    // edge falls. Rounding to a whole sample hides a bias on a grid whose edges
    // sit on samples, so the rounded edge cannot show it; this can. A half-
    // sample convention error in the correlation put it at +33 us at 12 kHz.
    if (sc.expectPmTiming && r.pmExactErrMs.size() > 5) {
        double m = 0, v = 0;
        const std::size_t n = r.pmExactErrMs.size() - 5;
        for (std::size_t i = 5; i < r.pmExactErrMs.size(); ++i) m += r.pmExactErrMs[i];
        m /= static_cast<double>(n);
        for (std::size_t i = 5; i < r.pmExactErrMs.size(); ++i) v += (r.pmExactErrMs[i] - m) * (r.pmExactErrMs[i] - m);
        if (std::getenv("DCF77_STATS"))
            std::printf("    unrounded PM edge: bias %+.4f ms, sd %.4f ms\n", m, std::sqrt(v / static_cast<double>(n)));
        if (std::fabs(m) > 0.020) fail(r, "unrounded PM edges biased " + std::to_string(m * 1000.0) + " us");
    }
    // AM timing is the fallback and is judged as one: past the tracker's first
    // twenty seconds, 99% of edges inside 2.5 ms and no bias worth the name.
    if (!sc.expectPmTiming && r.amErrMs.size() > 40) {
        std::vector<double> a(r.amErrMs.begin() + 20, r.amErrMs.end());
        double m = 0; for (double x : a) m += x; m /= a.size();
        for (double& x : a) x = std::fabs(x);
        std::sort(a.begin(), a.end());
        const double p99 = a[a.size() * 99 / 100];
        if (p99 > 2.5 || std::fabs(m) > 0.3)
            fail(r, "AM edges p99 " + std::to_string(p99) + " ms, mean " + std::to_string(m) + " ms");
    }
    return r;
}

// ---- the real recording ----------------------------------------------------

bool readWav(const char* path, int& rate, std::vector<float>& iq) {
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

bool live(const char* path) {
    int rate = 0;
    std::vector<float> iq;
    if (!readWav(path, rate, iq)) {
        std::printf("\nlive recording: %s not found or not 16-bit stereo; skipped\n", path);
        return true;
    }
    std::printf("\nlive recording: %s, %d Hz, %.0f s\n", path, rate, iq.size() / 2.0 / rate);
    Dcf77Decoder dec(rate, 0.0);
    struct F { long long start; int min, hour, doy, year2; std::uint8_t from; };
    std::vector<F> frames;
    std::vector<std::pair<double, double>> pmEdges;   // sample, fractional unknown -> rounded
    int labels = 0;
    long long firstLock = -1;
    dec.onFrame = [&](const ClockFrameInfo& f) {
        frames.push_back({f.frameStartSample, f.minute, f.hour, f.doy, f.year2, dec.diagnostics().lastFrameFrom});
    };
    dec.onSecond = [&](const ClockSecondInfo& i) {
        if (i.edgeMeasured && dec.diagnostics().timingFromPm)
            pmEdges.push_back({static_cast<double>(pmEdges.size()), static_cast<double>(i.edgeSample)});
    };
    dec.onTime = [&](const ClockTimeInfo& t) {
        if (firstLock < 0) firstLock = t.lastEdgeSample;
        ++labels;
    };
    const std::size_t block = static_cast<std::size_t>(rate / 50);
    for (std::size_t i = 0; i < iq.size() / 2; i += block)
        dec.process(iq.data() + 2 * i, std::min(block, iq.size() / 2 - i));
    const auto d = dec.diagnostics();

    static const char* kFrom[] = {"none", "AM only", "PM only", "AM+PM agree", "AM/PM DISAGREE"};
    bool ok = true;
    int agree = 0;
    for (std::size_t i = 0; i < frames.size(); ++i) {
        const F& f = frames[i];
        std::printf("  frame at %8.3f s: ", f.start / static_cast<double>(rate));
        if (f.min < 0) std::printf("no valid decode");
        else std::printf("%s", civ::iso8601(civ::utcMsFromFields(f.year2, f.doy, f.hour, f.min)).c_str());
        std::printf("  [%s]\n", kFrom[std::min<int>(f.from, 4)]);
        if (f.from == 3) ++agree;
        if (f.from == 4) ok = false;
        if (i > 0 && f.min >= 0 && frames[i - 1].min >= 0) {
            const long long a = civ::utcMsFromFields(frames[i - 1].year2, frames[i - 1].doy, frames[i - 1].hour, frames[i - 1].min);
            const long long b = civ::utcMsFromFields(f.year2, f.doy, f.hour, f.min);
            if (b - a != 60000) { std::printf("    NOT one minute after the previous\n"); ok = false; }
        }
    }
    // PM edge jitter: residual from a straight line through the PM-timed edges.
    double jitter = std::numeric_limits<double>::quiet_NaN();
    if (pmEdges.size() > 20) {
        // Seconds are numbered by rounding the spacing, so a coasted gap is fine.
        const double e0 = pmEdges.front().second;
        double sx = 0, sy = 0, sxx = 0, sxy = 0;
        const double n = static_cast<double>(pmEdges.size());
        std::vector<double> xs;
        for (auto& p : pmEdges) {
            const double x = std::round((p.second - e0) / rate);
            xs.push_back(x);
            sx += x; sy += p.second; sxx += x * x; sxy += x * p.second;
        }
        const double b = (n * sxy - sx * sy) / (n * sxx - sx * sx), a = (sy - b * sx) / n;
        double ss = 0;
        for (std::size_t i = 0; i < pmEdges.size(); ++i) {
            const double e = pmEdges[i].second - (a + b * xs[i]);
            ss += e * e;
        }
        jitter = std::sqrt(ss / n) * 1000.0 / rate;
        std::printf("  PM-timed edges: %zu, one second = %.3f samples, rms about the line %.3f ms "
                    "(the edges are whole samples: %.3f ms of that is rounding)\n",
                    pmEdges.size(), b, jitter, 1000.0 / rate / std::sqrt(12.0));
    }
    std::printf("  end state: %s, PM %s (SNR %.1f dB), timing from %s, AM edge - PM edge %+.3f ms, "
                "carrier %+.3f Hz, %d labelled seconds, first at %.1f s\n",
                d.pmLocked ? "PM locked" : "PM not locked", d.pmLocked ? "tracking" : "searching",
                d.pmSnrDb, d.timingFromPm ? "PM" : "AM", d.amMinusPmMs, d.carrierOffsetHz, labels,
                firstLock / static_cast<double>(rate));
    // The recording's own README shows it decoding "Sunday, 24/05/2026, 11:20
    // (CEST UTC+2), UTC 09:20": a truth for the date, the zone and the
    // conversion, independent of this decoder.
    bool sawTruth = false;
    for (const F& f : frames)
        if (f.min >= 0 && civ::iso8601(civ::utcMsFromFields(f.year2, f.doy, f.hour, f.min)) == "2026-05-24T09:20:00Z")
            sawTruth = true;
    if (!sawTruth) { std::printf("  FAIL: did not read 2026-05-24T09:20Z, which the recording's README shows\n"); ok = false; }
    if (!d.pmLocked || !d.timingFromPm) { std::printf("  FAIL: PM did not lock on the real signal\n"); ok = false; }
    if (agree < 2) { std::printf("  FAIL: fewer than two minutes where AM and PM agreed\n"); ok = false; }
    if (labels < 60) { std::printf("  FAIL: never locked\n"); ok = false; }
    if (std::isfinite(d.amMinusPmMs) && std::fabs(d.amMinusPmMs) > 2.0) {
        std::printf("  FAIL: AM and PM edges %+.2f ms apart\n", d.amMinusPmMs);
        ok = false;
    }
    std::printf("  %s\n", ok ? "PASS" : "FAIL");
    return ok;
}

} // namespace

int main(int argc, char** argv) {
    const double kClean = std::numeric_limits<double>::quiet_NaN();
    // 2026-09-21T10:03Z, an ordinary CEST morning.
    constexpr long long kDay = 1789984980;
    // 2026-03-29T00:57Z: CET -> CEST at 01:00Z, three minutes in.
    constexpr long long kSpring = 1774745820;
    // 2016-12-31T23:54Z: the real leap second at 2017-01-01T00:00Z, 00:59:60 CET.
    constexpr long long kLeapStart = 1483228440;
    constexpr long long kLeapAt = 1483228800;

    std::vector<Scenario> scs;
    unsigned seed = 1;
    for (double off : {0.0, 0.37, 0.81})
        for (double cn0 : {kClean, 40.0, 30.0}) {
            Scenario s; s.note = "AM+PM"; s.offsetSamples = off; s.cn0 = cn0; s.startUnix = kDay; s.seed = seed++;
            scs.push_back(s);
        }
    {
        Scenario s; s.note = "AM+PM, 24 kHz"; s.rate = 24000; s.offsetSamples = 0.5; s.cn0 = 35; s.startUnix = kDay; s.seed = seed++;
        scs.push_back(s);
    }
    // A receiver clock 18 and 34 ppm out, which is as far as a real one moves
    // an atomic carrier at 77.5 kHz.
    for (double hz : {1.4, -2.6}) {
        Scenario s; s.note = "carrier off DC"; s.carrierHz = hz; s.cn0 = 35; s.startUnix = kDay; s.seed = seed++;
        scs.push_back(s);
    }
    {
        // What a receiver with VLF interference showed: a line 10 Hz off,
        // stronger than DCF77, which must not be taken for the carrier.
        Scenario s; s.note = "QRM 10 Hz off, 2x carrier"; s.qrmHz = 10.2; s.qrmAmp = 2.0; s.cn0 = 35;
        s.startUnix = kDay; s.seed = seed++;
        scs.push_back(s);
    }
    {
        // What a real receiver showed: most of the IQ passband's power in a
        // signal a few kHz off the carrier, far above DCF77 and far above the
        // noise near it. The chip correlation integrates it away; a noise
        // estimate taken from y's power did not, and PM never tracked.
        Scenario s; s.note = "QRM +5.26 kHz, 10x carrier"; s.qrmHz = 5260.0; s.qrmAmp = 10.0; s.cn0 = 35;
        s.startUnix = kDay; s.seed = seed++;
        scs.push_back(s);
    }
    {
        // One second's cut filled in, in minutes 3 and 6, with no PM to frame
        // the minute. The marker is still at s59 both times, so the count of
        // seconds never slipped: the misread may cost the rest of its minute
        // (nothing certifies after a contradiction until s59), not the lock.
        Scenario s; s.note = "AM only, a cut filled"; s.pm = false; s.fillCut = true; s.cn0 = 40;
        s.startUnix = kDay; s.minutes = 9; s.expectPmTiming = false; s.maxLabelGapSec = 30.0; s.seed = seed++;
        scs.push_back(s);
    }
    {
        // The same misread once the lock is up (AM only locks by minute 7).
        // One confident misread is noise on a count s59 has confirmed: the
        // lock rides through it rather than losing the rest of the minute.
        Scenario s; s.note = "AM only, cut filled, locked"; s.pm = false; s.cn0 = 40;
        s.fillMinute = 9; s.fillSecs = {42};
        s.startUnix = kDay; s.minutes = 12; s.expectPmTiming = false; s.maxLabelGapSec = 2.0; s.seed = seed++;
        scs.push_back(s);
    }
    {
        // Two in one minute are a slipped count until s59 says otherwise:
        // nothing certifies from the second until the minute ends.
        Scenario s; s.note = "AM only, 2 cuts filled, locked"; s.pm = false; s.cn0 = 40;
        s.fillMinute = 9; s.fillSecs = {23, 42};
        s.startUnix = kDay; s.minutes = 12; s.expectPmTiming = false; s.minLabelGapSec = 15.0; s.seed = seed++;
        scs.push_back(s);
    }
    {
        Scenario s; s.note = "weak: PM holds, AM struggles"; s.cn0 = 24; s.startUnix = kDay; s.minutes = 9; s.seed = seed++;
        scs.push_back(s);
    }
    {
        // Seen live on a receiver 290 km out: the carrier gone for the last
        // five seconds of a minute, with nothing in the stream to say so.
        Scenario s; s.note = "5 s fades, minutes 3 and 6"; s.fade = true; s.cn0 = 35;
        s.startUnix = kDay; s.minutes = 9; s.seed = seed++;
        scs.push_back(s);
    }
    {
        Scenario s; s.note = "AM only (PM off air)"; s.pm = false; s.cn0 = 40; s.startUnix = kDay;
        s.expectPmTiming = false; s.seed = seed++;
        scs.push_back(s);
    }
    {
        Scenario s; s.note = "PM only (no carrier cut)"; s.am = false; s.cn0 = 35; s.startUnix = kDay; s.seed = seed++;
        scs.push_back(s);
    }
    {
        Scenario s; s.note = "I/Q inverted"; s.invert = true; s.cn0 = 35; s.startUnix = kDay; s.seed = seed++;
        scs.push_back(s);
    }
    {
        Scenario s; s.note = "CET -> CEST"; s.cn0 = 35; s.startUnix = kSpring; s.minutes = 8; s.seed = seed++;
        scs.push_back(s);
    }
    {
        Scenario s; s.note = "leap second announced"; s.cn0 = 35; s.startUnix = kLeapStart; s.minutes = 9;
        s.leapAt = kLeapAt; s.seed = seed++;
        scs.push_back(s);
    }
    {
        Scenario s; s.note = "AM and PM disagree"; s.conflict = true; s.cn0 = 40; s.startUnix = kDay;
        s.expectLock = false; s.seed = seed++;
        scs.push_back(s);
    }

    bool all = true;
    std::printf("%-30s %5s %6s %6s %7s  %-5s %7s %7s  %s\n", "scenario", "rate", "off", "C/N0",
                "labels", "time", "pm max", "am max", "result");
    const char* only = std::getenv("DCF77_ONLY");
    for (const Scenario& sc : scs) {
        if (only && !std::strstr(sc.note, only)) continue;
        const Result r = run(sc);
        auto worst = [](const std::vector<double>& v) {
            double w = 0; for (double x : v) w = std::max(w, std::fabs(x)); return w;
        };
        char cn[16];
        if (std::isnan(sc.cn0)) std::snprintf(cn, sizeof cn, "clean"); else std::snprintf(cn, sizeof cn, "%.0f", sc.cn0);
        std::printf("%-30s %5d %6.2f %6s %4d/%-2d  %-5s %7.3f %7.3f  %s%s\n", sc.note, sc.rate, sc.offsetSamples, cn,
                    r.labels, r.wrong, r.diag.timingFromPm ? "PM" : "AM", worst(r.pmErrMs), worst(r.amErrMs),
                    r.ok ? "ok" : "FAIL: ", r.ok ? "" : r.why.c_str());
        all = all && r.ok;
        if (std::getenv("DCF77_STATS")) {
            for (auto* v : {&r.amErrMs, &r.pmErrMs}) {
                if (v->empty()) continue;
                std::vector<double> a(*v); std::sort(a.begin(), a.end());
                double m = 0; for (double x : a) m += x; m /= a.size();
                std::printf("    %s n=%zu mean %+.3f p1 %+.3f p50 %+.3f p99 %+.3f first10:", v == &r.amErrMs ? "AM" : "PM",
                            a.size(), m, a[a.size() / 100], a[a.size() / 2], a[a.size() * 99 / 100]);
                for (std::size_t i = 0; i < std::min<std::size_t>(10, v->size()); ++i) std::printf(" %+.2f", (*v)[i]);
                std::printf("\n");
            }
        }
    }

    const char* path = argc > 1 ? argv[1] : DCF77_TESTDATA;
    all = live(path) && all;
    std::printf("\n%s\n", all ? "ALL PASS" : "FAILURES");
    return all ? 0 : 1;
}
