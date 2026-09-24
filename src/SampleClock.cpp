#include "SampleClock.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <ctime>
#include <iterator>
#include <vector>

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

bool SampleClock::hostTimeAt(double sample, double& hostSec) const {
    std::lock_guard<std::mutex> lk(m_mu);
    if (!m_fit.valid) return false;
    hostSec = m_fit.anchorSec + m_fit.secPerSample * sample;
    return true;
}

double SampleClock::lastExcessDelay() const {
    std::lock_guard<std::mutex> lk(m_mu);
    return m_lastExcess;
}

// ---------------------------------------------------------------------------
// CaptureClock

CaptureClock::CaptureClock(int sampleRate) : m_rate(sampleRate) {}

void CaptureClock::setSampleRate(int rate) {
    std::lock_guard<std::mutex> lk(m_mu);
    if (rate != m_rate) m_marks.clear();
    m_rate = rate;
}

void CaptureClock::reset() {
    std::lock_guard<std::mutex> lk(m_mu);
    m_marks.clear();
}

void CaptureClock::observe(std::int64_t firstSample, double captureSec) {
    std::lock_guard<std::mutex> lk(m_mu);
    if (!m_marks.empty() && firstSample <= m_marks.back().sample) m_marks.clear();
    m_marks.push_back({firstSample, captureSec});
    if (m_rate <= 0) return;
    const std::int64_t keep = static_cast<std::int64_t>(kHistorySec * m_rate);
    while (m_marks.size() > 1 && firstSample - m_marks.front().sample > keep) m_marks.pop_front();
}

bool CaptureClock::hostTimeAt(double sample, double& hostSec) const {
    std::lock_guard<std::mutex> lk(m_mu);
    if (m_marks.empty() || m_rate <= 0) return false;

    // The marks are in sample order: the nearest is at the lower bound or just
    // before it.
    auto it = std::lower_bound(m_marks.begin(), m_marks.end(), sample,
                               [](const Mark& m, double s) { return static_cast<double>(m.sample) < s; });
    const Mark* best = nullptr;
    if (it != m_marks.end()) best = &*it;
    if (it != m_marks.begin()) {
        const Mark& prev = *std::prev(it);
        if (!best || sample - static_cast<double>(prev.sample) <= static_cast<double>(best->sample) - sample)
            best = &prev;
    }
    const double dist = (sample - static_cast<double>(best->sample)) / m_rate;
    if (std::fabs(dist) > kMaxExtrapolateSec) return false;
    // At the stream's nominal rate: radiod's clock is the GPSDO, and over the
    // fraction of a second to the nearest mark the daemon crystal's tens of ppm
    // are nanoseconds.
    hostSec = best->sec + dist;
    return true;
}

std::size_t CaptureClock::marks() const {
    std::lock_guard<std::mutex> lk(m_mu);
    return m_marks.size();
}

double CaptureClock::spanSec() const {
    std::lock_guard<std::mutex> lk(m_mu);
    if (m_marks.size() < 2 || m_rate <= 0) return 0.0;
    return static_cast<double>(m_marks.back().sample - m_marks.front().sample) / m_rate;
}

// ---------------------------------------------------------------------------
// HostSlewGuard

void HostSlewGuard::reset() {
    m_hist.clear();
    m_deviationPpm = std::numeric_limits<double>::quiet_NaN();
    m_calmSince = -1.0;
}

void HostSlewGuard::sample(double daemonSec, double dmr) {
    if (!m_hist.empty() && daemonSec - m_hist.back().daemonSec < 1.0) return;
    m_hist.push_back({daemonSec, dmr});
    while (m_hist.size() > 2 && daemonSec - m_hist.front().daemonSec > kHistorySec) m_hist.pop_front();

    // d(daemon - realtime)/dt is the crystal's rate less the host clock's. Its
    // rate over the last kShortSec, from the oldest reading inside that span.
    auto slopeBetween = [](const Reading& a, const Reading& b) {
        return (b.dmr - a.dmr) / (b.daemonSec - a.daemonSec);
    };
    const Reading& now = m_hist.back();
    const Reading* from = nullptr;
    for (const Reading& r : m_hist) {
        if (daemonSec - r.daemonSec <= kShortSec) { from = &r; break; }
    }
    if (!from || now.daemonSec - from->daemonSec < 0.5 * kShortSec) {
        m_deviationPpm = std::numeric_limits<double>::quiet_NaN();
        m_calmSince = -1.0;
        return;
    }
    const double shortRate = slopeBetween(*from, now);

    // The usual rate: the median of consecutive kShortSec slopes across the
    // history, so a spell of slewing inside it does not become the baseline.
    std::vector<double> slopes;
    for (std::size_t i = 0; i < m_hist.size();) {
        std::size_t j = i + 1;
        while (j < m_hist.size() && m_hist[j].daemonSec - m_hist[i].daemonSec < kShortSec) ++j;
        if (j >= m_hist.size()) break;
        slopes.push_back(slopeBetween(m_hist[i], m_hist[j]));
        i = j;
    }
    double deviation;
    if (slopes.size() >= 5) {
        std::nth_element(slopes.begin(), slopes.begin() + slopes.size() / 2, slopes.end());
        deviation = std::fabs(shortRate - slopes[slopes.size() / 2]) * 1e6;
    } else {
        // Too little history for a baseline: allow a crystal's worth of rate.
        deviation = std::max(0.0, std::fabs(shortRate) * 1e6 - kCrystalAllowancePpm);
    }
    m_deviationPpm = deviation;
    if (deviation > kMaxDeviationPpm) m_calmSince = -1.0;
    else if (m_calmSince < 0.0) m_calmSince = daemonSec;
}

bool HostSlewGuard::steady(double daemonSec, std::string* why) const {
    char buf[160];
    if (std::isnan(m_deviationPpm)) {
        if (why) *why = "measuring the host clock's rate";
        return false;
    }
    if (m_calmSince < 0.0) {
        if (why) {
            std::snprintf(buf, sizeof buf, "host clock being slewed (%.0f ppm off its usual rate)",
                          m_deviationPpm);
            *why = buf;
        }
        return false;
    }
    if (daemonSec - m_calmSince < kExposureSec) {
        if (why) *why = "host clock only just steady";
        return false;
    }
    return true;
}

double HostSlewGuard::deviationPpm() const { return m_deviationPpm; }

} // namespace ubersdr_ntp
