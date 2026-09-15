// Offline test for the daemon's clock.
//
// The served time is the daemon clock (a raw crystal, SampleClock.h) plus an
// offset the radio measures, moving at the crystal's rate error. Three things
// have to be right for that to hold, and each is checked here against a known
// truth rather than against the code's own idea of it:
//
//   1. OffsetEstimator tracks a drifting offset: the level to well under a
//      millisecond, the rate to a few ppm, through outliers; says honestly
//      when the rate is still unmeasured; recovers from a step in minutes;
//      and refuses a "rate" no crystal could have.
//   2. SampleClock fits cleanly on a clock that only counts, and demonstrably
//      does not on one being slewed the way a host's NTP client slews it --
//      the failure this clock exists to remove.
//   3. Selector compares sources measured at different instants at one
//      instant, averages their rates, and coasts along the rate when the
//      radio goes quiet instead of freezing the offset.
//
// Exit status 0 when every check passes.

#include "OffsetEstimator.h"
#include "SampleClock.h"
#include "Selector.h"

#include <algorithm>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <random>
#include <string>
#include <vector>

using namespace ubersdr_ntp;

namespace {

int g_ok = 0, g_failed = 0;

void check(const std::string& name, bool ok, const char* fmt = "", ...) {
    char detail[256] = {0};
    va_list ap;
    va_start(ap, fmt);
    std::vsnprintf(detail, sizeof detail, fmt, ap);
    va_end(ap);
    std::printf("  %-5s %s%s%s\n", ok ? "ok" : "FAIL", name.c_str(), detail[0] ? " -- " : "", detail);
    (ok ? g_ok : g_failed) += 1;
}

double medianOf(std::vector<double> v) {
    std::sort(v.begin(), v.end());
    const std::size_t m = v.size() / 2;
    return v.size() % 2 ? v[m] : 0.5 * (v[m - 1] + v[m]);
}

// Per-second offset samples: truth plus Gaussian edge scatter plus occasional
// gross outliers (an edge on a fade).
struct EdgeNoise {
    std::mt19937_64 rng;
    std::normal_distribution<double> gauss;
    std::uniform_real_distribution<double> uni{0.0, 1.0};
    EdgeNoise(unsigned seed, double sigma) : rng(seed), gauss(0.0, sigma) {}
    double operator()() {
        double e = gauss(rng);
        if (uni(rng) < 0.02) e += (uni(rng) < 0.5 ? -1.0 : 1.0) * 0.050;
        return e;
    }
};

void testDrift() {
    std::printf("\nOffsetEstimator: a crystal 40 ppm fast, 5 ms edge scatter, 2%% outliers at 50 ms\n");
    constexpr double kRate = -40e-6;   // a fast crystal: UTC minus it falls
    constexpr double t0 = 10000.0, o0 = 0.0123;
    auto truth = [&](double t) { return o0 + kRate * (t - t0); };
    EdgeNoise noise(7, 0.005);
    OffsetEstimator est;
    std::vector<std::pair<double, double>> all;

    for (int k = 0; k <= 1500; ++k) {
        const double t = t0 + k;
        const double y = truth(t) + noise();
        est.add(t, y);
        all.push_back({t, y});

        if (k == 60) {
            const OffsetEstimate& e = est.estimate();
            const double err = e.offsetAt(t) - truth(t);
            check("after 1 min the rate is reported unmeasured", !e.rateMeasured,
                  "measured=%d", e.rateMeasured);
            check("...and its doubt is inside the claim",
                  std::abs(err) <= e.rateTermSec + 4.0 * e.jitterSec / std::sqrt(e.samples),
                  "error %+.2f ms, rate term %.2f ms, jitter %.2f ms", err * 1e3, e.rateTermSec * 1e3,
                  e.jitterSec * 1e3);
        }
    }

    const double tEnd = t0 + 1500;
    const OffsetEstimate& e = est.estimate();
    const double err = e.offsetAt(tEnd) - truth(tEnd);
    const double ahead = e.offsetAt(tEnd + 120) - truth(tEnd + 120);

    // What the old estimator served: the median of the last two minutes, held.
    std::vector<double> lastTwo;
    for (const auto& p : all) if (p.first >= tEnd - 120) lastTwo.push_back(p.second);
    const double oldErr = medianOf(lastTwo) - truth(tEnd);

    check("rate measured after 25 min", e.rateMeasured);
    check("rate within 3 ppm", std::abs(e.rate - kRate) < 3e-6, "%+.2f ppm (true %+.1f, +/- %.2f)",
          e.rate * 1e6, kRate * 1e6, e.rateUncertainty * 1e6);
    check("level within 1 ms", std::abs(err) < 0.001, "%+.3f ms", err * 1e3);
    check("carried 2 min ahead, still within 1 ms", std::abs(ahead) < 0.001, "%+.3f ms", ahead * 1e3);
    check("jitter reports the scatter, not the outliers", e.jitterSec > 0.003 && e.jitterSec < 0.007,
          "%.2f ms", e.jitterSec * 1e3);
    std::printf("        (a plain 2-minute median, as before, is %+.2f ms out here)\n", oldErr * 1e3);
}

void testStep() {
    std::printf("\nOffsetEstimator: a 40 ms step after 15 min (a source relocked onto another edge)\n");
    constexpr double kRate = -13e-6;
    constexpr double t0 = 50000.0;
    auto truth = [&](double t) { return (t - t0 >= 900.0 ? 0.040 : 0.0) + kRate * (t - t0); };
    EdgeNoise noise(11, 0.004);
    OffsetEstimator est;
    for (int k = 0; k <= 2100; ++k) {
        const double t = t0 + k;
        est.add(t, truth(t) + noise());
        if (k == 1080) {
            const OffsetEstimate& e = est.estimate();
            const double err = e.offsetAt(t) - truth(t);
            check("3 min after the step the level has followed it", std::abs(err) < 0.002,
                  "%+.2f ms", err * 1e3);
            check("...because the history behind the step was dropped", e.levelShifts >= 1,
                  "level shifts %d", e.levelShifts);
        }
    }
    const OffsetEstimate& e = est.estimate();
    check("20 min after the step the rate is measured again, within 5 ppm",
          e.rateMeasured && std::abs(e.rate - kRate) < 5e-6, "measured=%d %+.2f ppm", e.rateMeasured,
          e.rate * 1e6);
}

void testWander() {
    std::printf("\nOffsetEstimator: path delay wandering +/-8 ms over minutes, on a -12 ppm crystal\n");
    constexpr double kRate = -12e-6;
    constexpr double t0 = 90000.0;
    constexpr double kPi = 3.14159265358979323846;
    // Measured offsets carry the wander; the crystal is only the line under it.
    auto wander = [&](double t) {
        return 0.005 * std::sin(2.0 * kPi * (t - t0) / 1200.0) +
               0.003 * std::sin(2.0 * kPi * (t - t0) / 420.0 + 1.3);
    };
    std::mt19937_64 rng(17);
    std::normal_distribution<double> gauss(0.0, 0.0006);
    OffsetEstimator est;
    double worstSigmas = 0.0, worstAt = 0.0, worstErr = 0.0, worstUnc = 0.0;
    int measuredChecks = 0;
    for (int k = 0; k <= 5400; ++k) {
        const double t = t0 + k;
        est.add(t, 0.02 + kRate * (t - t0) + wander(t) + gauss(rng));
        if (k % 60 != 0) continue;
        const OffsetEstimate& e = est.estimate();
        if (!e.rateMeasured) continue;
        ++measuredChecks;
        const double sigmas = std::abs(e.rate - kRate) / e.rateUncertainty;
        if (sigmas > worstSigmas) {
            worstSigmas = sigmas; worstAt = k; worstErr = e.rate - kRate; worstUnc = e.rateUncertainty;
        }
    }
    const OffsetEstimate& e = est.estimate();
    check("the rate is measured despite the wander", measuredChecks > 60, "%d of 90 checks", measuredChecks);
    check("the claimed rate uncertainty is honest (error within 3 of it, every minute)",
          worstSigmas <= 3.0, "worst %.1f at %.0f s: %+.2f ppm against +/- %.2f", worstSigmas, worstAt,
          worstErr * 1e6, worstUnc * 1e6);
    check("coasting on it would stay inside the claim (error within 15 ppm + its doubt)",
          std::abs(e.rate - kRate) <= 15e-6 + e.rateUncertainty, "%+.2f ppm +/- %.2f",
          (e.rate - kRate) * 1e6, e.rateUncertainty * 1e6);
    check("no false steps from wander", e.levelShifts == 0, "level shifts %d", e.levelShifts);
}

void testImplausibleRate() {
    std::printf("\nOffsetEstimator: a trend no crystal could have (2000 ppm)\n");
    OffsetEstimator est;
    EdgeNoise noise(13, 0.002);
    for (int k = 0; k <= 1200; ++k) est.add(1000.0 + k, 2000e-6 * k + noise());
    const OffsetEstimate& e = est.estimate();
    check("is not taken as a measured rate", !e.rateMeasured, "rate %+.0f ppm", e.rate * 1e6);
}

// A packet stream as SampleClock sees it: a receiver 20 ppm fast, 20 ms blocks,
// 50 ms of fixed path plus exponential queueing, stamped on some host clock.
struct FitResult { ClockFit fit; double edgeSpreadMs; };

FitResult runSampleClock(double (*clockAt)(double)) {
    constexpr int kRate = 12000, kBlock = 240;
    const double rateEff = kRate * (1.0 + 20e-6);
    std::mt19937_64 rng(3);
    std::exponential_distribution<double> queue(1.0 / 0.010);
    SampleClock sc(kRate, 4.0, 300.0);
    for (long long k = 1;; ++k) {
        const std::int64_t n = k * kBlock;
        const double captured = static_cast<double>(n) / rateEff;
        const double arrived = captured + 0.050 + queue(rng);
        if (arrived > 600.0) break;
        sc.observe(n, clockAt(arrived));
    }
    // Where the fit puts each second edge of the last minute, against where it
    // truly was on the same clock (captured plus the fixed path). The spread
    // of that error is what lands on the offset samples; its mean is a
    // constant the delay model owns.
    std::vector<double> err;
    for (int s = 540; s < 600; ++s) {
        const std::int64_t n = static_cast<std::int64_t>(std::llround(s * rateEff));
        double mapped = 0.0;
        if (!sc.hostTimeAt(n, mapped)) continue;
        err.push_back(mapped - clockAt(static_cast<double>(n) / rateEff + 0.050));
    }
    double mean = 0.0, ss = 0.0;
    for (double e : err) mean += e;
    mean /= static_cast<double>(err.size());
    for (double e : err) ss += (e - mean) * (e - mean);
    return {sc.fit(), std::sqrt(ss / static_cast<double>(err.size())) * 1e3};
}

// A crystal: counts, 13 ppm fast.
double crystalClock(double t) { return 1.7e9 + t * (1.0 + 13e-6); }

// The same crystal under an NTP client correcting an offset: slewed at +400
// ppm for a minute, then -400 for a minute, over and over -- within what was
// measured on the host where this was found (+177 to -500 ppm inside a minute).
double slewedClock(double t) {
    const double p = std::fmod(t, 120.0);
    const double tri = p < 60.0 ? p : 120.0 - p;
    return crystalClock(t) + 400e-6 * tri;
}

void testSampleClock() {
    std::printf("\nSampleClock: arrivals stamped on a counting crystal vs on a slewed host clock\n");
    const FitResult a = runSampleClock(crystalClock);
    const FitResult b = runSampleClock(slewedClock);
    std::printf("        crystal: residual %.3f ms, slope %s, receiver %+.1f ppm, edge spread %.3f ms\n",
                a.fit.residualRms * 1e3, a.fit.slopeHeld ? "REFUSED" : "accepted",
                (a.fit.secPerSample * 12000.0 - 1.0) * 1e6, a.edgeSpreadMs);
    std::printf("        slewed:  residual %.3f ms, slope %s, edge spread %.3f ms\n",
                b.fit.residualRms * 1e3, b.fit.slopeHeld ? "REFUSED" : "accepted", b.edgeSpreadMs);
    check("on a crystal the slope is accepted", a.fit.valid && !a.fit.slopeHeld);
    check("on a crystal the residual is under 1 ms", a.fit.residualRms < 0.001, "%.3f ms",
          a.fit.residualRms * 1e3);
    check("on a crystal edges map to within 0.5 ms", a.edgeSpreadMs < 0.5, "%.3f ms", a.edgeSpreadMs);
    check("a slewed clock is what broke it (ten times the residual)",
          b.fit.residualRms > 10.0 * a.fit.residualRms, "%.3f vs %.3f ms", b.fit.residualRms * 1e3,
          a.fit.residualRms * 1e3);
}

SourceSnapshot snapshotAt(const std::string& name, double atSec, double offsetSec, double rate,
                          double now) {
    SourceSnapshot s;
    s.name = name;
    s.enabled = true;
    s.clockState = "locked";
    s.station = "wwv";
    s.haveOffset = true;
    s.offsetSec = offsetSec;
    s.offsetAtSec = atSec;
    s.offsetRate = rate;
    s.offsetRateMeasured = true;
    s.offsetRateUncertainty = 1e-6;
    s.offsetAgeSec = now - atSec;
    s.offsetSamples = 100;
    s.dispersionSec = 0.020;
    s.weightDispersionSec = 0.003;
    s.jitterSec = 0.002;
    return s;
}

void testSelector() {
    std::printf("\nSelector: two sources measured 30 s apart, then 10 min with no radio\n");
    constexpr double kRate = -40e-6;
    auto truth = [&](double t) { return 0.25 + kRate * (t - 1000.0); };
    Selector sel(3600.0, 15.0, 1);
    constexpr double now = 5000.0;

    const std::vector<SourceSnapshot> snaps = {
        snapshotAt("a", now - 1.0, truth(now - 1.0), kRate, now),
        snapshotAt("b", now - 30.0, truth(now - 30.0), kRate, now),
    };
    const Combined c = sel.combine(snaps, now);
    const double err = c.utcAt(now) - (now + truth(now));
    check("combined and synchronised", c.valid && c.synchronised && c.used == 2, "used %d", c.used);
    check("both carried to one instant: served time within 0.05 ms", std::abs(err) < 5e-5,
          "%+.3f ms", err * 1e3);
    check("rate averaged", c.rateMeasured && std::abs(c.rate - kRate) < 1e-9, "%+.2f ppm", c.rate * 1e6);

    constexpr double later = now + 600.0;
    const Combined q = sel.combine({}, later);
    const double coastErr = q.utcAt(later) - (later + truth(later));
    check("coasting, still synchronised", q.valid && q.synchronised, "%s", q.note.c_str());
    check("coasting along the rate: within 0.05 ms after 10 min", std::abs(coastErr) < 5e-5,
          "%+.3f ms (a frozen offset would be %+.1f ms out)", coastErr * 1e3, -kRate * 600.0 * 1e3);
    check("coasting widens the dispersion", q.dispersionSec > c.dispersionSec, "%.2f -> %.2f ms",
          c.dispersionSec * 1e3, q.dispersionSec * 1e3);
    // One daemon second is 1 + rate seconds of UTC: a crystal 40 ppm fast has
    // counted a second before one has passed.
    const double at1 = q.utcAt(later + 1.0) - q.utcAt(later);
    {
        Selector split(3600.0, 15.0, 1);
        std::vector<SourceSnapshot> two = {
            snapshotAt("a", now - 1.0, truth(now - 1.0), -50e-6, now),
            snapshotAt("b", now - 1.0, truth(now - 1.0), -30e-6, now),
        };
        const Combined d = split.combine(two, now);
        check("sources whose rates disagree make the combined rate that uncertain",
              d.rateUncertainty >= 10e-6 - 1e-12, "rates -50/-30 ppm, combined %+.1f +/- %.1f ppm",
              d.rate * 1e6, d.rateUncertainty * 1e6);
    }
    check("the served clock ticks at UTC's rate, not the crystal's",
          std::abs(at1 - (1.0 + kRate)) < 1e-12, "%.9f s of UTC per daemon s (expected %.9f)", at1,
          1.0 + kRate);
}

} // namespace

int main() {
    std::printf("ubersdr-ntp clock test\n");
    testDrift();
    testStep();
    testWander();
    testImplausibleRate();
    testSampleClock();
    testSelector();
    std::printf("\n%d ok, %d failed\n", g_ok, g_failed);
    return g_failed ? 1 : 0;
}
