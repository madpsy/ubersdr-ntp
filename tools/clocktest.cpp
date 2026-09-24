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
//   4. TimeContinuity refuses a decoded time that jumps from the source's own
//      history -- the misread BCD bit that put a receiver four minutes, and
//      another four hours, out on 2026-09-17 -- takes a first time only once
//      it has agreed with itself, and replaces a history that was wrong once
//      the new time has held, so nothing it decides can stick for ever.
//
// Exit status 0 when every check passes.

#include "OffsetEstimator.h"
#include "SampleClock.h"
#include "Selector.h"
#include "TimeContinuity.h"

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

// A slow-polled peer on a drifting crystal: the 7 ms bug, and the fix.
//
// An upstream NTP peer polled every 64 s gets a 512-second level window
// (OffsetTuning::forPollInterval) and cannot fit its own rate for over half an
// hour. Until it does, the level is a median through samples carried forward at
// whatever rate it believes -- and believing ZERO on a crystal running 26 ppm
// means the median lands about half the window's drift behind the newest
// sample. That is ~6.7 ms, it looked exactly like the radio class reading
// early, and it is what setRatePrior exists to stop.
void testSlowPeerBorrowsTheRate() {
    std::printf("\nOffsetEstimator: a peer polled every 64 s, crystal 26 ppm, rate not yet fittable\n");
    constexpr double kRate = -26e-6;
    constexpr double kPoll = 64.0;
    constexpr double t0 = 50000.0, o0 = 0.0;
    auto truth = [&](double t) { return o0 + kRate * (t - t0); };

    const OffsetTuning t = OffsetTuning::forPollInterval(kPoll);
    OffsetEstimator without(t), with(t);
    // What the Selector would already know from the radio sources, which sample
    // every second and fit this within minutes.
    with.setRatePrior(kRate, 0.5e-6, true);

    EdgeNoise noise(11, 0.0005);
    double tEnd = t0;
    for (int k = 0; k <= 18; ++k) {           // 18 polls ~ 19 min, well short of a fit
        tEnd = t0 + k * kPoll;
        const double y = truth(tEnd) + noise();
        without.add(tEnd, y);
        with.add(tEnd, y);
    }

    const OffsetEstimate& a = without.estimate();
    const OffsetEstimate& b = with.estimate();
    const double errA = a.offsetAt(tEnd) - truth(tEnd);
    const double errB = b.offsetAt(tEnd) - truth(tEnd);

    check("neither has fitted its own rate yet", !a.rateMeasured && !b.rateMeasured,
          "without=%d with=%d", a.rateMeasured, b.rateMeasured);
    check("without a prior it assumes the crystal is perfect", !a.rateFromPrior && a.rate == 0.0,
          "rate %+.2f ppm", a.rate * 1e6);
    check("...and reads milliseconds late because of it", std::abs(errA) > 0.004,
          "%+.2f ms", errA * 1e3);
    check("with the system's rate it is marked borrowed", b.rateFromPrior && b.rate == kRate,
          "fromPrior=%d rate %+.2f ppm", b.rateFromPrior, b.rate * 1e6);
    check("...and the level is right to under a millisecond", std::abs(errB) < 0.001,
          "%+.3f ms", errB * 1e3);
    check("...and it no longer claims 50 ppm of doubt", b.rateUncertainty < 1e-6,
          "%.2f ppm", b.rateUncertainty * 1e6);
    check("a borrowed rate is still not a measured one, so the combine ignores it",
          !b.rateMeasured, "measured=%d", b.rateMeasured);
    std::printf("        (assuming zero rate here costs %+.2f ms; borrowing it costs %+.3f ms)\n",
                errA * 1e3, errB * 1e3);
}

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
    s.kind = SourceKind::Radio;
    s.primaryClass = true;
    s.active = true;
    // `ready` is the field the Selector actually reads -- a source says in one
    // bool whether it has anything worth considering, so that the Selector does
    // not have to know what a decoder lock or a reach register is. clockState
    // is what the status report renders; setting only that would build a
    // snapshot that looks locked and is not a candidate.
    s.ready = true;
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

// An upstream NTP peer's snapshot, for the two-class tests below.
SourceSnapshot peerAt(const std::string& name, double atSec, double offsetSec, double now,
                      int stratum = 2) {
    SourceSnapshot s;
    s.name = name;
    s.enabled = true;
    s.active = true;
    s.ready = true;
    s.kind = SourceKind::Ntp;
    s.primaryClass = false;          // overridden by the caller when ntp is primary
    s.clockState = "locked";
    s.haveOffset = true;
    s.offsetSec = offsetSec;
    s.offsetAtSec = atSec;
    s.offsetRate = 0.0;
    s.offsetRateMeasured = true;
    s.offsetRateUncertainty = 1e-6;
    s.offsetAgeSec = now - atSec;
    s.offsetSamples = 40;
    s.dispersionSec = 0.010;
    s.weightDispersionSec = 0.002;
    s.jitterSec = 0.001;
    s.ntp.stratum = stratum;
    s.ntp.refid = "198.51.100.7";
    s.ntp.address = "198.51.100.7:123";
    s.ntp.addressRefid = 0xC6336407u;   // 198.51.100.7
    s.ntp.rootDelaySec = 0.012;
    s.ntp.rootDispersionSec = 0.004;
    s.ntp.delaySec = 0.008;
    s.ntp.reach = 0377;
    return s;
}

// Runs the selector forward over `seconds`, one call a second, and returns the
// last result. combine() is driven by the `now` it is handed, so a test can
// cover an hour in a millisecond -- and the hold-downs and the five-minute
// residual average are all measured in that time rather than in calls.
Combined run(Selector& sel, std::vector<SourceSnapshot>& snaps, double& now, double seconds,
             void (*each)(std::vector<SourceSnapshot>&, double) = nullptr) {
    Combined c;
    for (double t = 0.0; t < seconds; t += 1.0) {
        now += 1.0;
        if (each) each(snaps, now);
        c = sel.combine(snaps, now);
    }
    return c;
}

bool used(const Combined& c, const std::string& name) {
    return std::find(c.usedNames.begin(), c.usedNames.end(), name) != c.usedNames.end();
}

std::string reasonFor(const Combined& c, const std::string& name) {
    const auto it = c.notUsedReasons.find(name);
    return it == c.notUsedReasons.end() ? std::string() : it->second;
}

// --- both classes at once -------------------------------------------------
void testAlwaysMode() {
    std::printf("\nTwo classes, secondary \"always\": both contribute to one answer\n");

    ClockConfig k;
    k.primary = SourceKind::Radio;
    k.secondary = SecondaryMode::Always;
    Selector sel(3600.0, 15.0, 1, k);

    double now = 1000.0;
    std::vector<SourceSnapshot> snaps = {
        snapshotAt("wwv10", now, 0.100, 0.0, now),
        snapshotAt("wwv15", now, 0.102, 0.0, now),
        peerAt("pool-a", now, 0.101, now),
        peerAt("pool-b", now, 0.099, now),
    };
    const Combined c = sel.combine(snaps, now);

    check("all four are candidates", c.candidates == 4, "%d", c.candidates);
    check("all four are used", c.used == 4, "%d", c.used);
    check("a radio source and a peer are both in the answer",
          used(c, "wwv10") && used(c, "pool-a"));
    check("serving from both classes", c.serving == ServingClass::Both,
          "%s", servingClassName(c.serving));
    // A radio reference is in the selected set, so this is a stratum-1 radio
    // clock -- which is what NTP would say of any server with a refclock among
    // its selected peers.
    check("stratum stays 1 while a radio source is in the answer", c.stratum == 1,
          "%d", c.stratum);
    check("...so the refid is the station, not an address",
          !c.refidIsAddress && c.refid == "WWV", "%s", c.refid.c_str());
    check("...and the root delay is zero: no NTP path above a radio clock",
          c.rootDelaySec == 0.0, "%.1f ms", c.rootDelaySec * 1000.0);
    check("the combined offset sits among the sources",
          std::abs(c.offsetSec - 0.1005) < 0.002, "%+.2f ms", c.offsetSec * 1000.0);
}

void testStratumFromUpstream() {
    std::printf("\nServing from upstreams alone: the stratum has to say so\n");

    ClockConfig k;
    k.secondary = SecondaryMode::Always;
    Selector sel(3600.0, 15.0, 1, k);

    double now = 1000.0;
    std::vector<SourceSnapshot> snaps = {
        peerAt("pool-a", now, 0.100, now, 2),
        peerAt("pool-b", now, 0.101, now, 3),
    };
    const Combined c = sel.combine(snaps, now);

    check("both peers are used", c.used == 2, "%d", c.used);
    // One more than the LOWEST stratum among the survivors. Claiming stratum 1
    // here would tell every client this was a radio clock.
    check("stratum is one below the best upstream", c.stratum == 3, "%d", c.stratum);
    check("the refid becomes the server's address", c.refidIsAddress,
          "%s", c.refid.c_str());
    check("...carried as the four bytes the packet needs",
          c.refidAddress == 0xC6336407u, "%08x", c.refidAddress);
    // There IS an NTP path above us now, and its length is not zero.
    check("the root delay is the path back to the primary reference",
          c.rootDelaySec > 0.015 && c.rootDelaySec < 0.025,
          "%.1f ms", c.rootDelaySec * 1000.0);
    check("serving from the secondary class", c.serving == ServingClass::Secondary,
          "%s", servingClassName(c.serving));
}

// --- standby, and the failover ---------------------------------------------
void testStandbyAndFailover() {
    std::printf("\nSecondary \"standby\": measured continuously, held out until needed\n");

    ClockConfig k;
    k.primary = SourceKind::Radio;
    k.secondary = SecondaryMode::Standby;
    k.failoverAfterSec = 60.0;
    k.failbackAfterSec = 300.0;
    Selector sel(3600.0, 15.0, 1, k);

    double now = 1000.0;
    std::vector<SourceSnapshot> snaps = {
        snapshotAt("wwv10", now, 0.100, 0.0, now),
        peerAt("pool-a", now, 0.104, now),
    };
    // Keep both measurements fresh as the clock advances, so nothing drops out
    // for staleness while the hold-downs are being tested.
    auto refresh = [](std::vector<SourceSnapshot>& v, double t) {
        for (SourceSnapshot& s : v) {
            if (!s.ready) continue;
            s.offsetAtSec = t;
            s.offsetAgeSec = 0.0;
        }
    };

    Combined c = run(sel, snaps, now, 130.0, refresh);
    check("the radio serves", used(c, "wwv10") && c.used == 1, "%d used", c.used);
    check("the peer is held out", !used(c, "pool-a"));
    // Held out is not broken, and the page has to say which of the two it is.
    check("...and said to be standing by, not failing",
          reasonFor(c, "pool-a").find("standing by") != std::string::npos,
          "%s", reasonFor(c, "pool-a").c_str());
    check("serving from the primary", c.serving == ServingClass::Primary,
          "%s", servingClassName(c.serving));

    // The class delta: the whole reason standby is worth having over cold.
    check("the two classes are compared anyway", c.classDelta.valid);
    check("...and the difference is the planted 4 ms",
          std::abs(c.classDelta.averagedSec - (-0.004)) < 0.0005,
          "%+.2f ms", c.classDelta.averagedSec * 1000.0);
    check("...measured over the time both have been up",
          c.classDelta.settledForSec > 100.0, "%.0f s", c.classDelta.settledForSec);

    // --- the radio goes away ------------------------------------------------
    snaps[0].ready = false;
    snaps[0].haveOffset = false;
    snaps[0].clockState = "unlocked";
    snaps[0].notReadyReason = "no tick: nothing at 1000 Hz to time a second from";

    c = run(sel, snaps, now, 30.0, refresh);
    check("30 s in, it coasts rather than switching", c.serving == ServingClass::Coasting,
          "%s", servingClassName(c.serving));
    check("...and says the swap is coming, with how long is left",
          c.failoverInSec > 0.0 && c.failoverInSec < 35.0, "%.0f s", c.failoverInSec);

    c = run(sel, snaps, now, 45.0, refresh);
    check("past the hold-down it fails over to the peer",
          c.serving == ServingClass::Secondary && used(c, "pool-a"),
          "%s", servingClassName(c.serving));
    check("...and the stratum follows the source it is now serving from",
          c.stratum == 3, "%d", c.stratum);
    check("...and it is still synchronised throughout", c.synchronised);

    // --- the radio comes back, and must not be believed at once -------------
    snaps[0] = snapshotAt("wwv10", now, 0.100, 0.0, now);
    c = run(sel, snaps, now, 120.0, refresh);
    check("a returning radio source does not take back immediately",
          c.serving == ServingClass::Secondary, "%s", servingClassName(c.serving));
    check("...and says how long it must stay healthy first",
          c.failbackInSec > 0.0, "%.0f s", c.failbackInSec);
    check("...and the radio is named as recovering, not as broken",
          reasonFor(c, "wwv10").find("taking back") != std::string::npos,
          "%s", reasonFor(c, "wwv10").c_str());

    c = run(sel, snaps, now, 200.0, refresh);
    check("after the failback interval the radio takes back",
          c.serving == ServingClass::Primary && used(c, "wwv10"),
          "%s", servingClassName(c.serving));
    check("...and the stratum returns to 1", c.stratum == 1, "%d", c.stratum);
}

// --- flapping must not become oscillation ----------------------------------
//
// The failure this guards against is specific and it is not hypothetical: a
// decoder coming out of a fade does not come back cleanly, it locks and loses
// the lock repeatedly for several minutes. If the served time followed that,
// every cycle would be a step of the whole radio-to-network difference --
// several milliseconds, sometimes tens -- landing on clients as a sawtooth,
// which is worse for them than either class alone would have been.
//
// Two flap rates, because they fail differently. A flap shorter than the
// failover hold-down must not switch at all; a flap longer than it must switch
// ONCE and then stay put, because the failback interval is longer than the
// flap's healthy half. And through both, the clock must stay synchronised --
// stability that was bought by refusing to serve would be no bargain.
void testNoOscillation() {
    std::printf("\nA flapping primary must not make the served time oscillate\n");

    ClockConfig k;
    k.primary = SourceKind::Radio;
    k.secondary = SecondaryMode::Standby;
    k.failoverAfterSec = 60.0;
    k.failbackAfterSec = 300.0;

    // Counts class switches and watches for any loss of synchronisation over a
    // run in which the radio is up for `upSec` and down for `downSec`, round
    // and round.
    struct Result {
        int switches = 0;
        int unsyncSeconds = 0;
        double worstStep = 0.0;   // largest jump in the served offset between seconds
        ServingClass ended = ServingClass::None;
    };

    auto flap = [&](double upSec, double downSec, double totalSec) {
        Selector sel(3600.0, 15.0, 1, k);
        double now = 1000.0;
        // The two classes 6 ms apart, which is about what a real radio delay
        // model leaves: every switch costs a step of that size, so a count of
        // switches is a count of steps a client would see.
        std::vector<SourceSnapshot> snaps = {
            snapshotAt("wwv10", now, 0.106, 0.0, now),
            peerAt("pool-a", now, 0.100, now),
        };

        Result r;
        ServingClass last = ServingClass::None;
        double lastOffset = 0.0;
        bool haveLast = false;
        bool radioUp = true;
        double phaseLeft = upSec;

        for (double t = 0.0; t < totalSec; t += 1.0) {
            now += 1.0;
            phaseLeft -= 1.0;
            if (phaseLeft <= 0.0) {
                radioUp = !radioUp;
                phaseLeft = radioUp ? upSec : downSec;
                if (radioUp) {
                    // It comes back from nothing, as a decoder does.
                    snaps[0] = snapshotAt("wwv10", now, 0.106, 0.0, now);
                } else {
                    snaps[0].ready = false;
                    snaps[0].haveOffset = false;
                    snaps[0].clockState = "unlocked";
                    snaps[0].notReadyReason = "no tick";
                }
            }
            for (SourceSnapshot& s : snaps) {
                if (!s.ready) continue;
                s.offsetAtSec = now;
                s.offsetAgeSec = 0.0;
            }

            const Combined c = sel.combine(snaps, now);
            // Coasting is not a class change: it is the same answer carried
            // forward, and a client sees no step from it.
            const ServingClass serving =
                (c.serving == ServingClass::Coasting || c.serving == ServingClass::None)
                    ? last : c.serving;
            if (last != ServingClass::None && serving != last) ++r.switches;
            if (serving != ServingClass::None) last = serving;
            if (!c.synchronised) ++r.unsyncSeconds;
            if (haveLast) r.worstStep = std::max(r.worstStep, std::abs(c.offsetSec - lastOffset));
            lastOffset = c.offsetSec;
            haveLast = true;
        }
        r.ended = last;
        return r;
    };

    // A fast flap: 30 s down is shorter than the 60 s hold-down, so the
    // failover never fires and the gaps are covered by coasting on the crystal.
    {
        const Result r = flap(30.0, 30.0, 1800.0);
        check("a 30 s flap under the 60 s hold-down never switches class at all",
              r.switches == 0, "%d switches in 30 min", r.switches);
        check("...and the clock stays synchronised throughout", r.unsyncSeconds == 0,
              "%d s unsynchronised", r.unsyncSeconds);
        check("...so a client sees no step from it", r.worstStep < 0.001,
              "worst step %.3f ms", r.worstStep * 1000.0);
    }

    // A slow flap: 200 s down clears the hold-down, so it fails over once --
    // and then the 200 s healthy half never reaches the 300 s failback
    // interval, so it stays put instead of switching back and forth every
    // cycle. This is the whole reason the two hold-downs are different numbers.
    {
        const Result r = flap(200.0, 200.0, 3600.0);
        check("a 200 s flap over the hold-down switches once, not once a cycle",
              r.switches == 1, "%d switches in an hour of flapping (9 cycles)", r.switches);
        check("...and settles on the secondary", r.ended == ServingClass::Secondary,
              "%s", servingClassName(r.ended));
        check("...and the clock stays synchronised throughout", r.unsyncSeconds == 0,
              "%d s unsynchronised", r.unsyncSeconds);
        check("...with one step, of about the difference between the classes",
              r.worstStep > 0.001 && r.worstStep < 0.020,
              "worst step %.1f ms", r.worstStep * 1000.0);
    }

    // And once the primary is genuinely well again, it does take back --
    // hysteresis that never releases is not hysteresis, it is a one-way door.
    {
        Selector sel(3600.0, 15.0, 1, k);
        double now = 1000.0;
        std::vector<SourceSnapshot> snaps = {
            snapshotAt("wwv10", now, 0.106, 0.0, now),
            peerAt("pool-a", now, 0.100, now),
        };
        auto refresh = [](std::vector<SourceSnapshot>& v, double t) {
            for (SourceSnapshot& s : v) {
                if (!s.ready) continue;
                s.offsetAtSec = t;
                s.offsetAgeSec = 0.0;
            }
        };
        run(sel, snaps, now, 60.0, refresh);
        snaps[0].ready = false;
        snaps[0].haveOffset = false;
        Combined c = run(sel, snaps, now, 90.0, refresh);
        check("it failed over to start with", c.serving == ServingClass::Secondary,
              "%s", servingClassName(c.serving));

        snaps[0] = snapshotAt("wwv10", now, 0.106, 0.0, now);
        c = run(sel, snaps, now, 400.0, refresh);
        check("a primary healthy for longer than the failback interval does take back",
              c.serving == ServingClass::Primary, "%s", servingClassName(c.serving));
    }
}

// --- the refusal test must not cross the classes ---------------------------
void testRefusalIsWithinAClass() {
    std::printf("\nA source is judged by its own class and never by the other\n");

    ClockConfig k;
    k.secondary = SecondaryMode::Always;
    Selector sel(3600.0, 15.0, 1, k);

    double now = 1000.0;
    // Three radio sources that agree with each other, and three peers that
    // agree with each other, with the two classes 100 ms apart. That is far
    // past the +/-30 ms refusal limit -- and it is not a fault: it is what a
    // badly modelled radio path looks like, and the figure the class delta
    // exists to report.
    std::vector<SourceSnapshot> snaps = {
        snapshotAt("wwv10", now, 0.200, 0.0, now),
        snapshotAt("wwv15", now, 0.201, 0.0, now),
        snapshotAt("wwv20", now, 0.199, 0.0, now),
        peerAt("pool-a", now, 0.100, now),
        peerAt("pool-b", now, 0.101, now),
        peerAt("pool-c", now, 0.099, now),
    };
    auto refresh = [](std::vector<SourceSnapshot>& v, double t) {
        for (SourceSnapshot& s : v) { s.offsetAtSec = t; s.offsetAgeSec = 0.0; }
    };

    Combined c = run(sel, snaps, now, 400.0, refresh);

    bool anyRefused = false;
    for (const SourceResidual& r : c.residuals) if (r.refused) anyRefused = true;
    check("100 ms between the classes refuses nobody", !anyRefused);
    check("every source is still a candidate", c.candidates == 6, "%d", c.candidates);
    check("...and the gap is reported instead",
          c.classDelta.valid && std::abs(c.classDelta.averagedSec - 0.100) < 0.002,
          "%+.0f ms", c.classDelta.averagedSec * 1000.0);
    // Each source's residual is against its OWN class, so they are all small
    // despite the classes being 100 ms apart.
    double worst = 0.0;
    for (const SourceResidual& r : c.residuals) worst = std::max(worst, std::abs(r.averagedSec));
    check("...and each residual is against its own class, so all are small",
          worst < 0.005, "worst %+.1f ms", worst * 1000.0);

    // Now move ONE radio source away from its own class. That is a misdecode,
    // there is no arrangement of receivers that explains it, and it must be
    // refused -- which is the behaviour the cross-class exemption must not
    // have broken.
    Selector sel2(3600.0, 15.0, 1, k);
    double now2 = 1000.0;
    std::vector<SourceSnapshot> snaps2 = {
        snapshotAt("wwv10", now2, 0.200, 0.0, now2),
        snapshotAt("wwv15", now2, 0.201, 0.0, now2),
        snapshotAt("wwv20", now2, 0.300, 0.0, now2),   // 100 ms from its peers
        peerAt("pool-a", now2, 0.100, now2),
        peerAt("pool-b", now2, 0.101, now2),
        peerAt("pool-c", now2, 0.099, now2),
    };
    Combined c2 = run(sel2, snaps2, now2, 400.0, refresh);

    bool oddRefused = false;
    for (const SourceResidual& r : c2.residuals) if (r.name == "wwv20") oddRefused = r.refused;
    check("a radio source 100 ms from its OWN class is still refused", oddRefused);
    check("...and only that one", c2.candidates == 5, "%d", c2.candidates);
    check("...with a reason naming the class it disagreed with",
          reasonFor(c2, "wwv20").find("radio") != std::string::npos,
          "%s", reasonFor(c2, "wwv20").c_str());
}

// --- cold standby -----------------------------------------------------------
void testColdActivation() {
    std::printf("\nSecondary \"cold\": nothing is connected until it is needed\n");

    ClockConfig k;
    k.primary = SourceKind::Radio;
    k.secondary = SecondaryMode::Cold;
    k.failoverAfterSec = 60.0;
    k.failbackAfterSec = 300.0;
    Selector sel(3600.0, 15.0, 1, k);

    check("before the first combine the standby is already down",
          !sel.activation().secondaryActive);

    double now = 1000.0;
    // The cold peer is present but has nothing: that is what a disconnected
    // source looks like, and the Selector must not need it to be ready in
    // order to decide to bring it up.
    SourceSnapshot cold = peerAt("pool-a", now, 0.0, now);
    cold.ready = false;
    cold.haveOffset = false;
    cold.active = false;
    cold.notReadyReason = "held in cold standby";

    std::vector<SourceSnapshot> snaps = { snapshotAt("wwv10", now, 0.100, 0.0, now), cold };
    auto refresh = [](std::vector<SourceSnapshot>& v, double t) {
        for (SourceSnapshot& s : v) {
            if (!s.ready) continue;
            s.offsetAtSec = t;
            s.offsetAgeSec = 0.0;
        }
    };

    run(sel, snaps, now, 60.0, refresh);
    check("while the radio is healthy it stays down", !sel.activation().secondaryActive);
    check("...and says why", sel.activation().reason.find("healthy") != std::string::npos,
          "%s", sel.activation().reason.c_str());

    // The radio drops. Bringing a cold source up takes minutes, so it must
    // start AT ONCE rather than after the failover hold-down -- otherwise the
    // hold-down is spent doing nothing.
    snaps[0].ready = false;
    snaps[0].haveOffset = false;
    run(sel, snaps, now, 2.0, refresh);
    check("the moment the radio drops, the standby is brought up",
          sel.activation().secondaryActive);
    check("...before the failover hold-down has expired, because acquiring takes time",
          sel.activation().reason.find("warming up") != std::string::npos,
          "%s", sel.activation().reason.c_str());

    // It acquires, and then serves.
    snaps[1] = peerAt("pool-a", now, 0.104, now);
    Combined c = run(sel, snaps, now, 90.0, refresh);
    check("once it has acquired and the hold-down passes, it serves",
          c.serving == ServingClass::Secondary, "%s", servingClassName(c.serving));

    // The radio returns. The standby has to stay up through the whole failback
    // interval, or a receiver that re-locks and loses it would tear the
    // standby down between attempts.
    snaps[0] = snapshotAt("wwv10", now, 0.100, 0.0, now);
    run(sel, snaps, now, 120.0, refresh);
    check("a returning radio does not stand the standby down at once",
          sel.activation().secondaryActive);
    run(sel, snaps, now, 250.0, refresh);
    check("...but it is stood down once the radio has proven itself",
          !sel.activation().secondaryActive);
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

// A receiver decoding once a minute. Truth is a crystal 12 ppm off UTC; each
// reading is truth plus a couple of milliseconds of edge scatter, and a misread
// adds whatever the bad bit was worth. Every admitted reading refreshes the
// history the way the source's filter does.
struct ContinuityRig {
    TimeContinuity tc;
    double t = 1000.0;               // daemon clock
    double base = -1.07;             // UTC minus daemon at t = 0
    double rate = -12e-6;
    std::mt19937_64 rng{7};
    std::normal_distribution<double> scatter{0.0, 0.002};

    double truth(double at) const { return base + rate * at; }
    TimeContinuity::Verdict reading(double misreadSec = 0.0, bool leapWindow = false) {
        const double implied = truth(t) + scatter(rng) + misreadSec;
        const TimeContinuity::Verdict v = tc.judge(implied, t, leapWindow);
        if (v.admitted()) tc.noteFiltered(truth(t), rate, t);
        return v;
    }
    void minute() { t += 60.0; }
};

const char* outcomeName(TimeContinuity::Outcome o) {
    using O = TimeContinuity::Outcome;
    switch (o) {
        case O::Accepted: return "accepted";
        case O::FirstConfirmed: return "first confirmed";
        case O::Confirming: return "confirming";
        case O::Refused: return "refused";
        case O::Adopted: return "adopted";
        case O::LeapSecond: return "leap second";
    }
    return "?";
}

void testTieBreak() {
    std::printf("\nSelector: two receivers that do not overlap at all\n");
    constexpr double now = 5000.0;
    auto at = [&](const std::string& n, double off, double wd) {
        SourceSnapshot s = snapshotAt(n, now - 1.0, off, 0.0, now);
        s.offsetRateMeasured = false;
        s.offsetRateUncertainty = 0.0;
        s.weightDispersionSec = wd;
        return s;
    };
    // The 2026-09-17 case: one right, one four hours out, and the wrong one
    // sorting first. With nothing served yet, the better-measured is kept,
    // whichever order they come in.
    for (int order = 0; order < 2; ++order) {
        Selector sel(3600.0, 15.0, 1);
        std::vector<SourceSnapshot> v = {at("wrong", -14400.0, 0.006), at("right", 0.25, 0.002)};
        if (order) std::swap(v[0], v[1]);
        const Combined c = sel.combine(v, now);
        check(order ? "...and in the other order" : "no history: the better-measured one is kept, not the lower",
              c.used == 1 && c.usedNames[0] == "right", "used %s",
              c.usedNames.empty() ? "none" : c.usedNames[0].c_str());
    }
    // Serving from one, a second appears that does not overlap it and is
    // better measured: the served time does not jump to it.
    for (int order = 0; order < 2; ++order) {
        Selector sel(3600.0, 15.0, 1);
        sel.combine({at("serving", 0.25, 0.004)}, now - 10.0);
        std::vector<SourceSnapshot> v = {at("serving", 0.25, 0.004), at("newcomer", -240.0, 0.001)};
        if (order) std::swap(v[0], v[1]);
        const Combined c = sel.combine(v, now);
        check(order ? "...and in the other order" : "with a served time, the continuous one is kept, however well the other measures",
              c.used == 1 && c.usedNames[0] == "serving" && std::abs(c.offsetSec - 0.25) < 1e-6,
              "used %s, offset %+.3f s", c.usedNames.empty() ? "none" : c.usedNames[0].c_str(),
              c.offsetSec);
        const auto why = c.notUsedReasons.find("newcomer");
        check("...and the other is told why", why != c.notUsedReasons.end() &&
              why->second.find("continuous with the served time") != std::string::npos,
              "%s", why == c.notUsedReasons.end() ? "(no reason)" : why->second.c_str());
    }
}

void testTimeContinuity() {
    std::printf("\ncontinuity of decoded times\n");
    using O = TimeContinuity::Outcome;

    {
        ContinuityRig r;
        const auto a = r.reading(); r.minute();
        const auto b = r.reading(); r.minute();
        const auto c = r.reading();
        check("the first time is not used on one reading", a.outcome == O::Confirming, "%s", outcomeName(a.outcome));
        check("...nor on two a minute apart", b.outcome == O::Confirming, "%s", outcomeName(b.outcome));
        check("...but is on three over two minutes", c.outcome == O::FirstConfirmed, "%s", outcomeName(c.outcome));
    }
    {
        ContinuityRig r;
        const auto a = r.reading(-240.0); r.minute();
        const auto b = r.reading(); r.minute();
        const auto c = r.reading(); r.minute();
        const bool noHistoryYet = !r.tc.haveHistory();
        const auto d = r.reading(); r.minute();
        const auto e = r.reading(-240.0);
        check("a misread first time is not taken", a.outcome == O::Confirming && noHistoryYet &&
              b.outcome == O::Confirming && c.outcome == O::Confirming);
        check("...the correct one after it is, once it has agreed three times", d.outcome == O::FirstConfirmed,
              "%s", outcomeName(d.outcome));
        check("...and the misread is then refused against it", e.outcome == O::Refused, "%+.1f s", e.jumpSec);
    }

    auto locked = [] {
        ContinuityRig r;
        for (int i = 0; i < 10; ++i) { r.reading(); r.minute(); }
        return r;
    };
    {
        // 2026-09-17 07:00: -240 s on K3FEF.
        ContinuityRig r = locked();
        const auto bad = r.reading(-240.0); r.minute();
        const auto good = r.reading();
        check("a four-minute misread is refused", bad.outcome == O::Refused, "%+.3f s", bad.jumpSec);
        check("...by what it jumped, to the second", std::abs(bad.jumpSec + 240.0) < 0.01);
        check("...and the next good decode is taken", good.outcome == O::Accepted, "%s", outcomeName(good.outcome));
    }
    {
        // 2026-09-17 18:00: -14 400 s on K3GMQ, for a few minutes.
        ContinuityRig r = locked();
        bool allRefused = true;
        for (int i = 0; i < 3; ++i) { allRefused &= r.reading(-14400.0).outcome == O::Refused; r.minute(); }
        const auto good = r.reading();
        check("a four-hour misread held for three minutes is refused throughout", allRefused);
        check("...and the time carries on where it was", good.outcome == O::Accepted, "%+.3f s", good.jumpSec);
    }
    {
        ContinuityRig r = locked();
        const auto v = r.reading(-1.0);
        check("a one-second misread is refused", v.outcome == O::Refused, "%+.3f s", v.jumpSec);
        ContinuityRig q = locked();
        const auto w = q.reading(0.3);
        check("300 ms is not a jump (the consensus's job, not this)", w.outcome == O::Accepted);
    }
    {
        // A misread that comes and goes never adds up to an adoption.
        ContinuityRig r = locked();
        bool adopted = false;
        for (int i = 0; i < 40; ++i) {
            const auto v = r.reading(i % 3 == 2 ? 0.0 : -240.0);
            adopted |= v.outcome == O::Adopted;
            r.minute();
        }
        check("an intermittent misread, 40 min of it, is never taken", !adopted);
    }
    {
        // The history was the wrong one: a step that stays (the machine was
        // suspended with the daemon clock stopped, say) is taken after ten
        // minutes, not before, and not never.
        ContinuityRig r = locked();
        r.base += 37.0;
        double adoptedAfter = -1.0;
        const double start = r.t;
        for (int i = 0; i < 20 && adoptedAfter < 0.0; ++i) {
            if (r.reading().outcome == O::Adopted) adoptedAfter = r.t - start;
            r.minute();
        }
        check("a real step that stays is taken after 10 min", adoptedAfter >= 600.0 && adoptedAfter <= 660.0,
              "after %.0f s", adoptedAfter);
        const auto next = r.reading();
        check("...and is the history from then on", next.outcome == O::Accepted, "%+.3f s", next.jumpSec);
    }
    {
        // Back after a day with no decodes: the crystal has moved a second,
        // along the rate the history knows.
        ContinuityRig r = locked();
        r.t += 86400.0;
        const auto v = r.reading();
        check("a day away, the crystal's drift is not a jump", v.outcome == O::Accepted, "%+.3f s", v.jumpSec);
    }
    {
        ContinuityRig r = locked();
        r.base -= 1.0;
        const auto v = r.reading(0.0, true);
        check("an announced leap second is taken at once", v.outcome == O::LeapSecond, "%s", outcomeName(v.outcome));
        ContinuityRig q = locked();
        q.base -= 1.0;
        const auto w = q.reading(0.0, false);
        check("...and the same second anywhere else is not", w.outcome == O::Refused, "%s", outcomeName(w.outcome));
    }
}

// CaptureClock maps samples through the packet nearest them, exactly: a
// packet's stamp is the capture time of its first sample, so no fit and no
// jitter -- and a step in the stamps (radiod re-anchoring after it lost
// samples) applies from that packet on, without disturbing the samples before.
void testCaptureClock() {
    std::printf("\ncapture clock\n");
    const int rate = 12000;
    const int frame = 240;   // 20 ms
    CaptureClock c(rate);
    double t = 0.0;
    check("no marks: no time", !c.hostTimeAt(0, t));

    const double t0 = 1000.0;
    for (int i = 0; i < 100; ++i) c.observe(static_cast<std::int64_t>(i) * frame, t0 + i * 0.020);
    bool ok = c.hostTimeAt(0, t);
    check("first sample of the first packet", ok && std::fabs(t - t0) < 1e-9, "%.9f", t - t0);
    ok = c.hostTimeAt(50 * frame + 120, t);
    check("mid-packet: its packet's capture plus the offset at the stream rate",
          ok && std::fabs(t - (t0 + 1.0 + 0.010)) < 1e-9, "%.9f", t - (t0 + 1.010));

    // radiod lost 4.045 ms of samples at the USB and re-anchored: from packet
    // 100 on, every sample was captured that much later than the count says.
    const double lost = 0.004045;
    for (int i = 100; i < 200; ++i) c.observe(static_cast<std::int64_t>(i) * frame, t0 + i * 0.020 + lost);
    ok = c.hostTimeAt(150 * frame, t);
    check("after a re-anchor: the step applies at once",
          ok && std::fabs(t - (t0 + 3.0 + lost)) < 1e-9, "%.6f ms", (t - (t0 + 3.0)) * 1e3);
    ok = c.hostTimeAt(60 * frame, t);
    check("before it: untouched", ok && std::fabs(t - (t0 + 1.2)) < 1e-9, "%.6f ms", (t - (t0 + 1.2)) * 1e3);

    // Packets with no capture time leave a gap; a sample in it extrapolates
    // from the nearest mark.
    CaptureClock g(rate);
    g.observe(0, t0);
    g.observe(10 * frame, t0 + 0.200);
    ok = g.hostTimeAt(3 * frame, t);
    check("in a short gap: from the nearer mark", ok && std::fabs(t - (t0 + 0.060)) < 1e-9);
    check("far from every mark: no time",
          !g.hostTimeAt(10 * frame + static_cast<std::int64_t>(3.0 * rate), t));

    // A sample index that goes backwards is a new stream.
    g.observe(0, t0 + 50.0);
    ok = g.hostTimeAt(0, t);
    check("a new stream replaces the old marks", ok && std::fabs(t - (t0 + 50.0)) < 1e-9 && g.marks() == 1);
}

// HostSlewGuard: capture times are trusted while the host clock runs at its
// usual rate against the daemon clock, and not while it is being slewed, nor
// for kExposureSec after, while the slew still sits in radiod's anchor window.
void testHostSlewGuard() {
    std::printf("\nhost slew guard\n");
    HostSlewGuard g;
    // The crystal runs -12 ppm against a disciplined host clock: that is the
    // usual rate, not a slew.
    const double crystal = -12e-6;
    double dmr = 0.0, now = 0.0;
    std::string why;
    auto run = [&](double seconds, double hostRate) {
        for (double s = 0; s < seconds; s += 1.0) {
            now += 1.0;
            dmr += crystal - hostRate;
            g.sample(now, dmr);
        }
    };
    check("nothing measured yet: not steady", !g.steady(now, &why), "%s", why.c_str());
    run(30.0, 0.0);
    check("a crystal's rate, from the start: steady", g.steady(now, &why), "%s", why.c_str());
    run(300.0, 0.0);
    check("five minutes steady", g.steady(now) && std::fabs(g.deviationPpm()) < 1.0, "%.2f ppm",
          g.deviationPpm());

    run(60.0, 500e-6);   // the host's NTP client slews at 500 ppm for a minute
    check("while slewed: not steady", !g.steady(now, &why), "%s", why.c_str());
    run(10.0, 0.0);
    check("just after: still not trusted (radiod's window still holds it)", !g.steady(now, &why), "%s",
          why.c_str());
    run(30.0, 0.0);
    check("half a minute after: steady again -- the slew did not become the baseline",
          g.steady(now, &why), "%s (%.2f ppm)", why.c_str(), g.deviationPpm());

    run(60.0, 5e-6);     // an ordinary disciplined correction
    check("a 5 ppm correction is not a slew worth refusing", g.steady(now, &why), "%s", why.c_str());
}

int main() {
    std::printf("ubersdr-ntp clock test\n");
    testDrift();
    testSlowPeerBorrowsTheRate();
    testStep();
    testWander();
    testImplausibleRate();
    testSampleClock();
    testCaptureClock();
    testHostSlewGuard();
    testSelector();
    testAlwaysMode();
    testStratumFromUpstream();
    testStandbyAndFailover();
    testNoOscillation();
    testRefusalIsWithinAClass();
    testColdActivation();
    testTieBreak();
    testTimeContinuity();
    std::printf("\n%d ok, %d failed\n", g_ok, g_failed);
    return g_failed ? 1 : 0;
}
