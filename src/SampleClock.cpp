#include "SampleClock.h"

#include <cmath>
#include <ctime>

namespace ubersdr_ntp {

double realtimeNow() {
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    return static_cast<double>(ts.tv_sec) + static_cast<double>(ts.tv_nsec) * 1e-9;
}

double monotonicNow() {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return static_cast<double>(ts.tv_sec) + static_cast<double>(ts.tv_nsec) * 1e-9;
}

double realtimeMinusMonotonic() {
    const double m0 = monotonicNow();
    const double r = realtimeNow();
    const double m1 = monotonicNow();
    return r - 0.5 * (m0 + m1);
}

SampleClock::SampleClock(int sampleRate, double bucketSec, double windowSec)
    : m_rate(sampleRate > 0 ? sampleRate : 12000),
      m_bucketSec(bucketSec),
      m_windowSec(windowSec) {}

void SampleClock::setSampleRate(int rate) {
    std::lock_guard<std::mutex> lk(m_mu);
    if (rate <= 0 || rate == m_rate) return;
    // A rate change means every residual already collected was computed against
    // the wrong divisor. Nothing about the old envelope survives it.
    m_rate = rate;
    m_envelope.clear();
    m_haveBucket = false;
    m_fit = ClockFit{};
}

void SampleClock::reset() {
    std::lock_guard<std::mutex> lk(m_mu);
    m_envelope.clear();
    m_haveBucket = false;
    m_fit = ClockFit{};
    m_lastExcess = 0.0;
}

void SampleClock::observe(std::int64_t endSample, double hostSec) {
    std::lock_guard<std::mutex> lk(m_mu);

    const double residual = hostSec - static_cast<double>(endSample) / m_rate;
    const Point p{endSample, hostSec, residual};

    // How far above the fitted line this arrival landed. Reported, not used:
    // it is the live jitter figure for the status block, and a path whose
    // excess delay is climbing is the first sign a source is about to become
    // useless long before it loses lock.
    if (m_fit.valid) {
        const double predicted = m_fit.anchorSec + m_fit.secPerSample * static_cast<double>(endSample);
        m_lastExcess = hostSec - predicted;
    }

    if (!m_haveBucket) {
        m_haveBucket = true;
        m_bucketBest = p;
        m_bucketStartSec = hostSec;
        return;
    }

    // Within the bucket, keep only the fastest packet.
    if (residual < m_bucketBest.residual) m_bucketBest = p;

    if (hostSec - m_bucketStartSec < m_bucketSec) return;

    m_envelope.push_back(m_bucketBest);
    m_haveBucket = false;

    // Age the window off by time rather than by count, so a source that has
    // been dropping packets does not end up with a window covering an hour.
    while (m_envelope.size() > 2 &&
           m_envelope.back().hostSec - m_envelope.front().hostSec > m_windowSec) {
        m_envelope.pop_front();
    }

    refit();
}

void SampleClock::refit() {
    const std::size_t n = m_envelope.size();
    if (n < 2) {
        m_fit = ClockFit{};
        return;
    }

    // Ordinary least squares of hostSec against sample index. The x values run
    // to 10^9 within an hour at 12 kHz, so both axes are centred on the window
    // mean before the sums are taken — regressing raw sample indices loses the
    // millisecond in the double long before the slope is any good.
    double sx = 0.0, sy = 0.0;
    for (const Point& p : m_envelope) {
        sx += static_cast<double>(p.sample);
        sy += p.hostSec;
    }
    const double mx = sx / static_cast<double>(n);
    const double my = sy / static_cast<double>(n);

    double sxx = 0.0, sxy = 0.0;
    for (const Point& p : m_envelope) {
        const double dx = static_cast<double>(p.sample) - mx;
        sxx += dx * dx;
        sxy += dx * (p.hostSec - my);
    }

    ClockFit f;
    if (sxx <= 0.0) {
        // Every point at one sample index: no slope is determinable, so hold
        // the nominal rate and let the intercept do the work.
        f.secPerSample = 1.0 / m_rate;
    } else {
        f.secPerSample = sxy / sxx;
    }

    // A slope implying the receiver's sample clock is more than 1000 ppm off
    // ours is not a rate error, it is a fit through too short a window or a
    // stream with a discontinuity in it. Fall back to nominal rather than
    // extrapolate a wild slope across a minute of samples.
    const double nominal = 1.0 / m_rate;
    if (!(f.secPerSample > nominal * 0.999 && f.secPerSample < nominal * 1.001)) {
        f.secPerSample = nominal;
    }

    f.anchorSec = my - f.secPerSample * mx;

    double ss = 0.0;
    for (const Point& p : m_envelope) {
        const double r = p.hostSec - (f.anchorSec + f.secPerSample * static_cast<double>(p.sample));
        ss += r * r;
    }
    f.residualRms = std::sqrt(ss / static_cast<double>(n));
    f.spanSec = m_envelope.back().hostSec - m_envelope.front().hostSec;
    f.points = static_cast<int>(n);
    f.valid = true;

    m_fit = f;
}

ClockFit SampleClock::fit() const {
    std::lock_guard<std::mutex> lk(m_mu);
    return m_fit;
}

bool SampleClock::hostTimeAt(std::int64_t sample, double& hostSec) const {
    std::lock_guard<std::mutex> lk(m_mu);
    if (!m_fit.valid) return false;
    hostSec = m_fit.anchorSec + m_fit.secPerSample * static_cast<double>(sample);
    return true;
}

double SampleClock::lastExcessDelay() const {
    std::lock_guard<std::mutex> lk(m_mu);
    return m_lastExcess;
}

} // namespace ubersdr_ntp
