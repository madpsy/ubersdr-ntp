#pragma once

// A source's offset from the daemon clock, and how fast that offset moves.
//
// Every measured second edge gives one sample: UTC minus the daemon clock at
// that edge, with the delay model applied. The daemon clock is a free-running
// crystal (SampleClock.h), so the true offset is not a constant but a line: it
// moves at the crystal's rate error against UTC -- tens of ppm, and steady. A
// median over a window, which is what this used to be, reports the middle of
// that line: a minute stale for a two-minute window, which at 50 ppm is 3 ms of
// bias on every answer. So the level and the rate are estimated apart:
//
//   RATE from a long window (half an hour, and not before ten minutes of it),
//   by least squares with gross outliers trimmed. It is a property of the
//   crystal and changes slowly. What limits it is not the second edges'
//   scatter but the path delay's wander -- milliseconds over minutes, which
//   over a short span passes for rate -- so its uncertainty is judged from
//   one-minute means, which carry that wander, not from the per-second
//   scatter, and as if the whole window might be a single excursion of it.
//
//   LEVEL from a short window (two minutes): each sample carried forward to the
//   newest along that rate, then the median, with the MAD as the jitter. The
//   median, not the mean, because one edge landing on a fade is tens of
//   milliseconds out; the short window, because a genuine step -- a source that
//   relocked onto a different edge -- should work its way out in two minutes,
//   not twenty.
//
// A step also bends the long fit, so the fit is checked for one: when the short
// window sits off the line by more than its noise allows, everything older than
// the short window is dropped and the rate is fitted again from what remains.
//
// Until there is enough history to measure the rate, none is assumed, and the
// doubt that leaves is reported rather than hidden: the level carries up to
// kUnmeasuredRatePpm times the distance it was carried forward.
//
// HOLDING THROUGH A JITTER SPIKE
//
// The median survives any disturbance that covers under half the level window,
// and nothing that covers more. A radio path can be disturbed for longer than
// a minute -- skywave on a long LF path at night scatters DCF77's PM edges by
// hundreds of microseconds, still inside the PM tracker's one-chip tolerance,
// so the decoder neither complains nor loses lock -- and then the level follows
// the scatter, and with one radio source in use the served time follows the
// level. Seen live on DCF77 at 1061 km, 04:40 UTC: jitter 14 us -> 312 us for
// about a minute, level 140 us down and then 130 us past where it started.
// Every sanity check further on (level shift at 10 ms, continuity at 0.5 s,
// the Selector's cross-check with no second receiver) is blind to that.
//
// So a source can ask (OffsetTuning::holdJitterSpikes) for the level to be
// held when its jitter jumps well past ITS OWN usual jitter: the last level
// published while the jitter was calm, carried along the rate, until the
// jitter has been calm again for a minute; then the gap between the held and
// the live level is slewed out rather than stepped. The measurement is not
// thrown away -- the live level and jitter are still reported, and the jitter
// still feeds the dispersion, because a spike is a real doubt whichever of the
// two levels is right. It is bounded: a hold that outlasts kMaxHoldSec gives
// way to the live level, because a disturbance that long is a change, and a
// daemon running for months must not be stuck on a level that has gone. Until
// the source has ten minutes of its own jitter history there is no baseline to
// judge a spike against, and nothing is held.

#include <algorithm>
#include <cstddef>
#include <deque>

namespace ubersdr_ntp {

// The windows the two estimates above are taken over.
//
// They were constants, and for the radio sources they still are: the defaults
// here are those values unchanged. They had to become a parameter when a source
// arrived that does not produce a sample a second. An upstream NTP peer polled
// every 64 seconds puts TWO samples in a 120-second level window, and the level
// needs five to be valid, so the estimator would have reported no offset for
// that peer for ever -- not badly, but never.
//
// Everything here is therefore expressed against how often samples arrive, and
// forPollInterval() derives a set from the poll interval. A window is a span of
// time because the things it is fighting -- the path delay's wander, a step, a
// crystal's drift -- happen in time and not in samples; a minimum count is a
// count because a median of three is not a median. Both have to hold.
struct OffsetTuning {
    double levelWindowSec = 120.0;
    int minLevelSamples = 5;

    double rateWindowSec = 1800.0;
    double minRateSpanSec = 600.0;
    int minRateSamples = 120;

    // Block length for judging the rate's uncertainty (see fitLine).
    double blockSec = 60.0;
    int minBlockSamples = 5;
    int minBlocks = 5;

    // Hold the level through a spike in the jitter (see above). Radio sources
    // only: a peer polled every 64 s has too few samples for a jitter history
    // worth judging against, and its own clock filter already does this job.
    bool holdJitterSpikes = false;

    // A set suited to a source sampled once every `pollSec` seconds.
    //
    // The counts are what a robust statistic needs and no more -- a median and
    // a MAD over four samples, a line through twenty -- because a peer polled
    // every 64 s gathers samples slowly and a count meant for a source
    // producing one a second would put the first usable rate hours away. The
    // spans are the radio figures stretched to hold those counts, never
    // shortened: the wander they are there to average over does not get faster
    // because the polling got slower.
    static OffsetTuning forPollInterval(double pollSec) {
        if (!(pollSec > 0.0)) return OffsetTuning{};
        OffsetTuning t;
        t.minLevelSamples = 4;
        t.levelWindowSec = std::max(120.0, 8.0 * pollSec);
        t.minRateSamples = 20;
        t.minRateSpanSec = std::max(600.0, 30.0 * pollSec);
        t.rateWindowSec = std::max(1800.0, 120.0 * pollSec);
        t.minBlockSamples = 3;
        t.blockSec = std::max(60.0, 5.0 * pollSec);
        t.minBlocks = 5;
        return t;
    }
};

struct OffsetEstimate {
    bool valid = false;          // enough samples in the level window to be a filtered value
    int samples = 0;             // samples in the level window
    double atSec = 0.0;          // daemon time offsetSec refers to: the newest sample
    double offsetSec = 0.0;      // UTC minus the daemon clock at atSec
    double jitterSec = 0.0;      // spread of the level window about the rate line (1.4826 MAD)

    bool rateMeasured = false;   // fitted from THIS source's own samples
    bool rateFromPrior = false;  // not fitted here; borrowed from the system (setRatePrior)
    double rate = 0.0;           // d(offset)/d(daemon time); 1e-6 is 1 ppm
    double rateUncertainty = 0.0;
    double rateSpanSec = 0.0;    // how much time the rate was fitted over
    int rateSamples = 0;         // ...and how many samples survived trimming
    double rateTermSec = 0.0;    // how far the rate's uncertainty could move offsetSec
    int levelShifts = 0;         // times history was dropped for a step

    // Holding through a jitter spike (OffsetTuning::holdJitterSpikes). While
    // `held`, offsetSec is the last calm level carried along the rate and
    // liveOffsetSec is what the window says; otherwise they differ only by a
    // gap still being slewed out after a hold (rejoinGapSec).
    double liveOffsetSec = 0.0;
    bool held = false;
    double heldForSec = 0.0;
    double rejoinGapSec = 0.0;       // offsetSec - liveOffsetSec, shrinking
    double jitterBaselineSec = 0.0;  // the source's usual jitter; 0 until known
    double spikeThresholdSec = 0.0;  // jitter above this starts a hold; 0 until known
    int holds = 0;                   // holds started
    int holdsTimedOut = 0;           // ...that gave way to the live level at kMaxHoldSec

    double offsetAt(double daemonSec) const { return offsetSec + rate * (daemonSec - atSec); }
};

class OffsetEstimator {
public:
    OffsetEstimator() = default;
    explicit OffsetEstimator(const OffsetTuning& t) : m_t(t) {}

    void add(double atSec, double offsetSec);
    void clear();
    bool empty() const { return m_samples.empty(); }
    double newestAt() const { return m_newestAt; }

    // THE CRYSTAL'S DRIFT IS ONE NUMBER, NOT ONE PER SOURCE
    //
    // Every source here measures the same physical thing: UTC against this
    // daemon's clock. The Selector's combine says so in as many words, and
    // averages the sources' rates because of it. But a source that cannot fit
    // the rate from its own samples used to assume ZERO, and that is not a
    // neutral assumption -- it is a claim that a crystal known to be running
    // 26 ppm fast is perfect.
    //
    // It cost about 7 ms. A peer polled every 64 s gets a 512-second level
    // window (forPollInterval above), the daemon clock moves 13 ms across that
    // window at 26 ppm, and a median through samples carried forward at a rate
    // of zero lands half the ramp behind the newest one. The radio sources,
    // sampling every second, fit their own rate in minutes and never showed it.
    // The peer needed two hours to fit one, and read 7 ms late until it did --
    // which then looked exactly like the radio being 7 ms early.
    //
    // So: a source that has not fitted a rate borrows the one the system has
    // already determined. `rateMeasured` stays false, which keeps it out of the
    // Selector's rate average, so nothing here is circular -- it consumes that
    // average, it does not feed it.
    void setRatePrior(double rateSec, double uncertaintySec, bool known) {
        m_priorRate = rateSec;
        m_priorUncertainty = uncertaintySec;
        m_priorKnown = known;
    }

    // Recomputed only when a sample has arrived since the last call.
    const OffsetEstimate& estimate();

private:
    struct Sample { double at; double offset; };
    void recompute();
    void holdThroughSpikes();

    OffsetTuning m_t;

    std::deque<Sample> m_samples;
    double m_newestAt = 0.0;
    bool m_dirty = true;
    int m_levelShifts = 0;
    OffsetEstimate m_est;

    // Spike hold; see holdThroughSpikes.
    struct JitterRecord { double at; double jitter; };
    std::deque<JitterRecord> m_jitterHistory;
    double m_nextJitterRecordAt = 0.0;
    bool m_haveCalm = false;
    double m_calmAt = 0.0, m_calmOffset = 0.0;   // last level published while calm
    bool m_holding = false;
    double m_holdStart = 0.0;
    double m_calmSince = -1.0;                   // while holding: calm again since
    bool m_rearmOnCalm = false;                  // after a timeout: no hold until calm
    double m_gap = 0.0, m_gapSlew = 0.0;         // being slewed out after a hold
    double m_lastHoldAt = 0.0;
    int m_holds = 0, m_holdsTimedOut = 0;

    // The system's rate, handed in from outside; see setRatePrior.
    double m_priorRate = 0.0;
    double m_priorUncertainty = 0.0;
    bool m_priorKnown = false;
};

} // namespace ubersdr_ntp
