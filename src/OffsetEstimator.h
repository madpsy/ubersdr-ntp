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

#include <cstddef>
#include <deque>

namespace ubersdr_ntp {

struct OffsetEstimate {
    bool valid = false;          // enough samples in the level window to be a filtered value
    int samples = 0;             // samples in the level window
    double atSec = 0.0;          // daemon time offsetSec refers to: the newest sample
    double offsetSec = 0.0;      // UTC minus the daemon clock at atSec
    double jitterSec = 0.0;      // spread of the level window about the rate line (1.4826 MAD)

    bool rateMeasured = false;   // false: rate is taken as zero and rateUncertainty is the bound
    double rate = 0.0;           // d(offset)/d(daemon time); 1e-6 is 1 ppm
    double rateUncertainty = 0.0;
    double rateSpanSec = 0.0;    // how much time the rate was fitted over
    int rateSamples = 0;         // ...and how many samples survived trimming
    double rateTermSec = 0.0;    // how far the rate's uncertainty could move offsetSec
    int levelShifts = 0;         // times history was dropped for a step

    double offsetAt(double daemonSec) const { return offsetSec + rate * (daemonSec - atSec); }
};

class OffsetEstimator {
public:
    void add(double atSec, double offsetSec);
    void clear();
    bool empty() const { return m_samples.empty(); }
    double newestAt() const { return m_newestAt; }

    // Recomputed only when a sample has arrived since the last call.
    const OffsetEstimate& estimate();

private:
    struct Sample { double at; double offset; };
    void recompute();

    std::deque<Sample> m_samples;
    double m_newestAt = 0.0;
    bool m_dirty = true;
    int m_levelShifts = 0;
    OffsetEstimate m_est;
};

} // namespace ubersdr_ntp
