#include "SampleClock.h"

#include <algorithm>
#include <cmath>
#include <ctime>

namespace ubersdr_ntp {

namespace {

// The most a receiver's sample clock can credibly differ from ours. A crystal
// is tens of ppm at worst and radiod resamples to the requested rate, so this
// is already generous; past it the slope is describing a damaged stream, not a
// rate. See the use site for why the bound is not looser.
constexpr double kMaxPlausiblePpm = 200.0;

} // namespace

namespace {

double readClock(clockid_t id) {
    struct timespec ts;
    clock_gettime(id, &ts);
    return static_cast<double>(ts.tv_sec) + static_cast<double>(ts.tv_nsec) * 1e-9;
}

// Where the daemon clock's zero is, fixed at the first read: the host clock at
// that moment, so the two start out together and every offset stays a small
// number while the host is roughly right. Nothing depends on the choice beyond
// that, and nothing ever moves it.
double daemonBase() {
    static const double base = [] {
        const double r0 = readClock(CLOCK_MONOTONIC_RAW);
        const double rt = readClock(CLOCK_REALTIME);
        const double r1 = readClock(CLOCK_MONOTONIC_RAW);
        return rt - 0.5 * (r0 + r1);
    }();
    return base;
}

} // namespace

double daemonNow() { return readClock(CLOCK_MONOTONIC_RAW) + daemonBase(); }

double realtimeNow() { return readClock(CLOCK_REALTIME); }

double monotonicNow() { return readClock(CLOCK_MONOTONIC); }

double daemonMinusRealtime() {
    const double d0 = daemonNow();
    const double r = realtimeNow();
    const double d1 = daemonNow();
    return 0.5 * (d0 + d1) - r;
}

double daemonClockResolution() {
    struct timespec res{};
    if (clock_getres(CLOCK_MONOTONIC_RAW, &res) != 0) return 1e-9;
    return static_cast<double>(res.tv_sec) + static_cast<double>(res.tv_nsec) * 1e-9;
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

    // A slope implying the receiver's sample clock is far off ours is not a
    // rate error, it is a fit through too short a window or a stream with a
    // discontinuity in it. Fall back to nominal rather than extrapolate a wild
    // slope across a minute of samples.
    //
    // The bound is 200 ppm because that is already generous for the thing being
    // measured: a receiver's sample clock is a crystal, tens of ppm at worst,
    // and radiod resamples to the rate that was asked for. Anything beyond that
    // is the stream misbehaving, and the old 1000 ppm bound was loose enough to
    // pass a fit that put 12 ms of bias on every edge at the window's edge --
    // wide enough to matter, narrow enough to look like a plausible source.
    const double nominal = 1.0 / m_rate;
    const double fitted = f.secPerSample;
    if (!(fitted > nominal * (1.0 - kMaxPlausiblePpm * 1e-6) &&
          fitted < nominal * (1.0 + kMaxPlausiblePpm * 1e-6))) {
        f.secPerSample = nominal;
        f.slopeHeld = true;
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

    // The distance the slope is actually worked over: from the window's centre,
    // which is where a regression is most certain, out to the newest sample,
    // which is where every timestamp that matters is taken.
    const double dWork = std::abs(static_cast<double>(m_envelope.back().sample) - mx);
    if (n > 2 && sxx > 0.0) {
        // Textbook standard error of an OLS slope: residual variance over the
        // spread in x. It falls as the window lengthens (both through n and,
        // much faster, through sxx), which is why a fit that has just been
        // rebuilt is correctly reported as the weak evidence it is.
        const double slopeStdErr = std::sqrt((ss / static_cast<double>(n - 2)) / sxx);
        f.slopeUncertaintySec = slopeStdErr * dWork;
    } else {
        // Two points determine a line exactly and say nothing about how well.
        // The span is the only thing left to go on.
        f.slopeUncertaintySec = f.residualRms;
    }
    if (f.slopeHeld) {
        f.slopeUncertaintySec = std::max(f.slopeUncertaintySec,
                                         std::abs(fitted - nominal) * dWork);
    }
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
