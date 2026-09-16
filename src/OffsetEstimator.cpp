#include "OffsetEstimator.h"

#include <algorithm>
#include <cmath>
#include <vector>

namespace ubersdr_ntp {

namespace {

// The windows and minimum counts now live in OffsetTuning (OffsetEstimator.h),
// because a source polled once a minute needs different ones from a source
// producing a sample a second. The DEFAULTS there are the figures that were
// here, and the reasoning behind the two that are not obvious is:
//
//   minRateSpanSec  the least history a rate may be fitted from. The
//                   per-second scatter is not what limits it -- that is under a
//                   millisecond once the sample clock is sound -- but the path
//                   delay's wander, milliseconds over a few minutes, which over
//                   a short span is indistinguishable from rate. Measured live,
//                   fits over two to ten minutes of one receiver read anything
//                   from +23 to -21 ppm against a crystal public NTP put at -12.
//
//   blockSec        how long a block is when judging that rate's uncertainty;
//                   see fitLine, which explains what the blocks are for.

// What a crystal may be assumed to be out by before its rate has been measured.
// Generous: the one this was written on measured 13 ppm against public NTP, and
// a PC's oscillator is typically within tens.
constexpr double kUnmeasuredRatePpm = 50.0;

// A measured rate must be better known than the assumption it replaces, or the
// assumption is kept: half the bound, so a noisy early slope does not pull the
// level further than zero would have.
constexpr double kMaxMeasuredUncertaintyPpm = kUnmeasuredRatePpm / 2.0;

// Beyond this the "rate" is a step the fit has bent itself around, or a
// machine that was suspended -- not a crystal.
constexpr double kMaxRatePpm = 500.0;

// The smallest departure of the short window from the long line that counts as
// a step, whatever the noise says. Path wander alone moves the short window
// several milliseconds off a half-hour line, and dropping history for that
// would keep the rate from ever being measured; a step smaller than this is
// absorbed by the level window's own median within two minutes anyway, and
// shows in the rate's block uncertainty while it passes through.
constexpr double kStepFloorSec = 0.010;

constexpr double kMadToSigma = 1.4826;

double median(std::vector<double> v) {
    const std::size_t mid = v.size() / 2;
    std::nth_element(v.begin(), v.begin() + static_cast<std::ptrdiff_t>(mid), v.end());
    if (v.size() % 2) return v[mid];
    const double hi = v[mid];
    const double lo = *std::max_element(v.begin(), v.begin() + static_cast<std::ptrdiff_t>(mid));
    return 0.5 * (lo + hi);
}

struct Point { double x; double y; };

struct LineFit {
    bool ok = false;
    double slope = 0.0, intercept = 0.0;
    double slopeSe = 0.0;
    double sigma = 0.0;       // residual scatter of the kept points
    double span = 0.0;
    double xmin = 0.0;
    int kept = 0;
};

// Least squares of y on x, twice: once through everything, then again through
// what lies within five sigma (from the MAD) of the first line, so a handful of
// edges on a fade cannot tilt it.
//
// The slope's uncertainty is the larger of the textbook standard error and what
// the wander could have done to it. The textbook figure treats every per-second
// residual as independent, and they are not: the path delay wanders by
// milliseconds over minutes, the fit bends to follow it, and a thousand
// correlated residuals then vouch for a slope far more strongly than they can.
// Live, it claimed 21 ppm +/- 0.5 for a crystal public NTP measured at 12.
//
// One-minute block means average away the per-second scatter and keep the
// wander. But the wander's own timescale is minutes to tens of minutes, so the
// blocks are correlated too, and a standard error over them still claims too
// much (tested: six times too little against wander periods of 7 and 20
// minutes). So the window is taken as possibly one excursion of the wander --
// one degree of freedom -- and the uncertainty is what a slope fitted through a
// single such excursion could be off by: sqrt(12) times the blocks' scatter
// about the line, over the span. Conservative, which is the right direction
// for a figure a coasting server grows its dispersion by.
LineFit fitLine(const std::vector<Point>& pts, const OffsetTuning& t) {
    LineFit f;
    if (static_cast<int>(pts.size()) < t.minRateSamples) return f;

    auto ols = [](const std::vector<Point>& p, const std::vector<bool>& keep, LineFit& out) {
        double n = 0.0, mx = 0.0, my = 0.0, xmin = 1e300, xmax = -1e300;
        for (std::size_t i = 0; i < p.size(); ++i) {
            if (!keep[i]) continue;
            n += 1.0; mx += p[i].x; my += p[i].y;
            xmin = std::min(xmin, p[i].x); xmax = std::max(xmax, p[i].x);
        }
        if (n < 3.0) return false;
        mx /= n; my /= n;
        double sxx = 0.0, sxy = 0.0;
        for (std::size_t i = 0; i < p.size(); ++i) {
            if (!keep[i]) continue;
            const double dx = p[i].x - mx;
            sxx += dx * dx;
            sxy += dx * (p[i].y - my);
        }
        if (!(sxx > 0.0)) return false;
        out.slope = sxy / sxx;
        out.intercept = my - out.slope * mx;
        double ss = 0.0;
        for (std::size_t i = 0; i < p.size(); ++i) {
            if (!keep[i]) continue;
            const double r = p[i].y - (out.intercept + out.slope * p[i].x);
            ss += r * r;
        }
        out.kept = static_cast<int>(n);
        out.span = xmax - xmin;
        out.xmin = xmin;
        out.sigma = std::sqrt(ss / (n - 2.0));
        out.slopeSe = std::sqrt((ss / (n - 2.0)) / sxx);
        return true;
    };

    std::vector<bool> keep(pts.size(), true);
    LineFit first;
    if (!ols(pts, keep, first)) return f;

    std::vector<double> res(pts.size());
    for (std::size_t i = 0; i < pts.size(); ++i)
        res[i] = pts[i].y - (first.intercept + first.slope * pts[i].x);
    const double medRes = median(res);
    std::vector<double> dev(res.size());
    for (std::size_t i = 0; i < res.size(); ++i) dev[i] = std::abs(res[i] - medRes);
    const double sigma = std::max(kMadToSigma * median(dev), 1e-4);
    for (std::size_t i = 0; i < pts.size(); ++i) keep[i] = std::abs(res[i] - medRes) <= 5.0 * sigma;

    if (!ols(pts, keep, f)) return LineFit{};

    const int nBlocks = static_cast<int>(f.span / t.blockSec) + 1;
    std::vector<double> bx(static_cast<std::size_t>(nBlocks), 0.0), br(bx.size(), 0.0);
    std::vector<int> bn(bx.size(), 0);
    for (std::size_t i = 0; i < pts.size(); ++i) {
        if (!keep[i]) continue;
        const auto b = static_cast<std::size_t>(
            std::min(nBlocks - 1, static_cast<int>((pts[i].x - f.xmin) / t.blockSec)));
        bx[b] += pts[i].x;
        br[b] += pts[i].y - (f.intercept + f.slope * pts[i].x);
        ++bn[b];
    }
    std::vector<Point> blocks;
    for (std::size_t b = 0; b < bx.size(); ++b) {
        if (bn[b] >= t.minBlockSamples) blocks.push_back({bx[b] / bn[b], br[b] / bn[b]});
    }
    if (static_cast<int>(blocks.size()) >= t.minBlocks && f.span > 0.0) {
        double ss = 0.0;
        for (const Point& p : blocks) ss += p.y * p.y;
        const double blockScatter = std::sqrt(ss / static_cast<double>(blocks.size() - 2));
        f.slopeSe = std::max(f.slopeSe, std::sqrt(12.0) * blockScatter / f.span);
    }

    f.ok = f.kept >= t.minRateSamples && f.span >= t.minRateSpanSec &&
           static_cast<int>(blocks.size()) >= t.minBlocks;
    return f;
}

} // namespace

void OffsetEstimator::add(double atSec, double offsetSec) {
    m_samples.push_back({atSec, offsetSec});
    m_newestAt = m_samples.size() == 1 ? atSec : std::max(m_newestAt, atSec);
    while (!m_samples.empty() && m_samples.front().at < m_newestAt - m_t.rateWindowSec) {
        m_samples.pop_front();
    }
    m_dirty = true;
}

void OffsetEstimator::clear() {
    m_samples.clear();
    m_newestAt = 0.0;
    m_dirty = true;
}

const OffsetEstimate& OffsetEstimator::estimate() {
    if (m_dirty) recompute();
    m_dirty = false;
    return m_est;
}

void OffsetEstimator::recompute() {
    m_est = OffsetEstimate{};
    if (m_samples.empty()) {
        m_est.levelShifts = m_levelShifts;
        return;
    }
    const double ref = m_newestAt;

    // At most twice: a step found in the first fit drops the history behind
    // it, and the second fit is of what is left, which holds no older samples
    // for a step to be found in.
    LineFit fit;
    for (int attempt = 0; attempt < 2; ++attempt) {
        std::vector<Point> pts;
        pts.reserve(m_samples.size());
        bool older = false;
        for (const Sample& s : m_samples) {
            pts.push_back({s.at - ref, s.offset});
            if (s.at < ref - m_t.levelWindowSec) older = true;
        }
        fit = fitLine(pts, m_t);
        if (!fit.ok || !older) break;

        const bool plausible = std::abs(fit.slope) <= kMaxRatePpm * 1e-6;
        std::vector<double> shortRes;
        for (const Point& p : pts) {
            if (p.x >= -m_t.levelWindowSec) shortRes.push_back(p.y - (fit.intercept + fit.slope * p.x));
        }
        const double limit = std::max(kStepFloorSec,
                                      4.0 * fit.sigma / std::sqrt(static_cast<double>(shortRes.size())));
        if (plausible && std::abs(median(shortRes)) <= limit) break;

        while (!m_samples.empty() && m_samples.front().at < ref - m_t.levelWindowSec) m_samples.pop_front();
        // Samples arrive in time order, so the front is the oldest; this is
        // belt and braces for an edge the fit placed a little out of order.
        m_samples.erase(std::remove_if(m_samples.begin(), m_samples.end(),
                                       [&](const Sample& s) { return s.at < ref - m_t.levelWindowSec; }),
                        m_samples.end());
        ++m_levelShifts;
        fit = LineFit{};
    }

    const bool measured = fit.ok && std::abs(fit.slope) <= kMaxRatePpm * 1e-6 &&
                          fit.slopeSe <= kMaxMeasuredUncertaintyPpm * 1e-6;
    m_est.rateMeasured = measured;
    m_est.rate = measured ? fit.slope : 0.0;
    m_est.rateUncertainty = measured ? fit.slopeSe : kUnmeasuredRatePpm * 1e-6;
    m_est.rateSpanSec = fit.ok ? fit.span : 0.0;
    m_est.rateSamples = fit.ok ? fit.kept : 0;
    m_est.levelShifts = m_levelShifts;

    std::vector<double> level;
    double carried = 0.0;
    for (const Sample& s : m_samples) {
        if (s.at < ref - m_t.levelWindowSec) continue;
        level.push_back(s.offset + m_est.rate * (ref - s.at));
        carried += ref - s.at;
    }
    const double med = median(level);
    std::vector<double> dev(level.size());
    for (std::size_t i = 0; i < level.size(); ++i) dev[i] = std::abs(level[i] - med);

    m_est.atSec = ref;
    m_est.offsetSec = med;
    m_est.jitterSec = kMadToSigma * median(dev);
    m_est.samples = static_cast<int>(level.size());
    // The median sits where the typical sample does, so the rate's doubt acts
    // over the mean distance a sample was carried.
    m_est.rateTermSec = m_est.rateUncertainty * (carried / static_cast<double>(level.size()));
    m_est.valid = m_est.samples >= m_t.minLevelSamples;
}

} // namespace ubersdr_ntp
