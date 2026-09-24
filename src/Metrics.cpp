#include "Metrics.h"

#include "Selector.h"
#include "SourceSnapshot.h"

#include <algorithm>
#include <cmath>

namespace ubersdr_ntp {

void MetricTier::add(double unix, double value) {
    const double start = std::floor(unix / m_width) * m_width;
    if (!m_haveCurrent || start > m_current.start) {
        if (m_haveCurrent) {
            m_done.push_back(m_current);
            while (m_done.size() > m_capacity) m_done.pop_front();
        }
        m_current = MetricBucket{start, value, value, value, 1};
        m_haveCurrent = true;
        return;
    }
    // Same bucket -- or an earlier one, which only a clock stepping backwards
    // produces. Folded into the current bucket rather than reopening a closed
    // one: a minute that averaged a little across a step is a smaller lie than
    // a history whose buckets are out of order.
    m_current.sum += value;
    m_current.min = std::min(m_current.min, value);
    m_current.max = std::max(m_current.max, value);
    ++m_current.count;
}

std::vector<MetricBucket> MetricTier::points(double now) const {
    // The window is the tier's capacity back from the bucket `now` falls in,
    // so a series that stopped reporting ages out of the chart rather than
    // showing its last hour as if it were this one.
    const double oldest = std::floor(now / m_width) * m_width - m_width * m_capacity;
    std::vector<MetricBucket> out;
    out.reserve(m_done.size() + 1);
    for (const MetricBucket& b : m_done) {
        if (b.start > oldest) out.push_back(b);
    }
    if (m_haveCurrent && m_current.start > oldest) out.push_back(m_current);
    return out;
}

void MetricHistory::add(const MetricSeriesInfo& info, double unix, double value) {
    if (!std::isfinite(value) || !std::isfinite(unix)) return;
    std::lock_guard<std::mutex> lk(m_mu);
    auto it = m_index.find(info.id);
    if (it == m_index.end()) {
        if (m_series.size() >= kMaxSeries) return;
        it = m_index.emplace(info.id, m_series.size()).first;
        m_series.push_back(Entry{info});
    }
    Entry& e = m_series[it->second];
    e.hour.add(unix, value);
    e.day.add(unix, value);
}

std::vector<MetricHistory::Series> MetricHistory::snapshot(Range r, double now) const {
    std::lock_guard<std::mutex> lk(m_mu);
    std::vector<Series> out;
    out.reserve(m_series.size());
    for (const Entry& e : m_series) {
        out.push_back(Series{e.info, (r == Range::Hour ? e.hour : e.day).points(now)});
    }
    return out;
}

std::size_t MetricHistory::seriesCount() const {
    std::lock_guard<std::mutex> lk(m_mu);
    return m_series.size();
}

void StateTimeline::set(double unix, const std::string& state) {
    if (!m_changes.empty() && m_changes.back().state == state) return;
    // A clock stepped backwards would put this change before the last; it is
    // stamped at the last instead, so the spans stay in order.
    const double at = m_changes.empty() ? unix : std::max(unix, m_changes.back().start);
    m_changes.push_back(Span{at, state});
    // Drop what no window reaches: a change is still needed while the one
    // after it is inside the kept period, because it says what state held
    // at the start of that period.
    while (m_changes.size() > 1 && m_changes[1].start < at - kKeepSec) m_changes.pop_front();
    while (m_changes.size() > kMaxChanges) m_changes.pop_front();
}

std::vector<StateTimeline::Span> StateTimeline::since(double from) const {
    std::vector<Span> out;
    for (std::size_t i = 0; i < m_changes.size(); ++i) {
        const bool nextInside = i + 1 < m_changes.size() && m_changes[i + 1].start > from;
        if (m_changes[i].start >= from || nextInside || i + 1 == m_changes.size()) {
            out.push_back(m_changes[i]);
        }
    }
    return out;
}

void MetricHistory::setServing(double unix, const std::string& state) {
    if (!std::isfinite(unix)) return;
    std::lock_guard<std::mutex> lk(m_mu);
    m_serving.set(unix, state);
}

std::vector<StateTimeline::Span> MetricHistory::serving(Range r, double now) const {
    // The same window the buckets cover, measured from the start of the
    // oldest bucket, so the bar lines up with the charts beneath it.
    const double width = r == Range::Hour ? kHourWidthSec : kDayWidthSec;
    const double from = std::floor(now / width) * width + width - windowSec(r);
    std::lock_guard<std::mutex> lk(m_mu);
    return m_serving.since(from);
}

namespace {

double median(std::vector<double> v) {
    std::sort(v.begin(), v.end());
    const std::size_t m = v.size() / 2;
    return v.size() % 2 ? v[m] : (v[m - 1] + v[m]) / 2.0;
}

} // namespace

void sampleMetrics(MetricHistory& h, const std::vector<SourceSnapshot>& sources,
                   const Combined& c, double unix) {
    // The served figures, exactly as the page's headline shows them.
    if (c.synchronised) {
        h.add({"served.offset_ms", "Clock offset", "ms", "served", ""}, unix,
              c.hostOffsetSec * 1000.0);
        h.add({"served.dispersion_ms", "Root dispersion", "ms", "served", ""}, unix,
              c.dispersionSec * 1000.0);
    }

    // Each class's median offset over its ready sources: the figure on either
    // side of the primary-and-secondary card.
    std::vector<double> primary, secondary;
    for (const SourceSnapshot& s : sources) {
        if (!s.ready || !s.haveOffset) continue;
        (s.primaryClass ? primary : secondary).push_back(s.hostOffsetSec * 1000.0);
    }
    if (!primary.empty()) {
        h.add({"class.primary_ms", "Primary median offset", "ms", "class", ""}, unix,
              median(primary));
    }
    if (!secondary.empty()) {
        h.add({"class.secondary_ms", "Secondary median offset", "ms", "class", ""}, unix,
              median(secondary));
    }
    // The instant difference, not the five-minute average the card shows:
    // averaging an average into minute buckets would smear the history twice.
    if (c.classDelta.valid) {
        h.add({"class.delta_ms", "Primary − secondary", "ms", "class", ""}, unix,
              c.classDelta.instantSec * 1000.0);
    }

    std::map<std::string, const SourceResidual*> residuals;
    for (const SourceResidual& r : c.residuals) residuals[r.name] = &r;

    for (const SourceSnapshot& s : sources) {
        const bool radio = s.kind == SourceKind::Radio;
        const std::string group = radio ? "radio" : "ntp";
        const std::string base = "source." + s.name + ".";
        if (s.ready && s.haveOffset) {
            h.add({base + "offset_ms", s.name + " offset", "ms", group, s.name}, unix,
                  s.hostOffsetSec * 1000.0);
            // How tightly its own measurements agree (OffsetEstimator: 1.4826
            // MAD about the rate line). In microseconds, because the spread is
            // wide: a capture-timed DCF77 edge scatters by about one, an
            // internet NTP server by hundreds.
            h.add({base + "jitter_us", s.name + " jitter", "µs", group, s.name}, unix,
                  s.jitterSec * 1e6);
        }
        // Against the median of its class, which is the agreement the
        // consensus judges it on. Only with someone to disagree with.
        if (auto it = residuals.find(s.name); it != residuals.end() && it->second->peers >= 1) {
            h.add({base + "residual_ms", s.name + " residual", "ms", group, s.name}, unix,
                  it->second->instantSec * 1000.0);
        }
        if (radio) {
            if (s.link == LinkState::Streaming && s.toneDetected) {
                h.add({base + "snr_db", s.name + " tick SNR", "dB", group, s.name}, unix,
                      s.toneSnrDb);
            }
        } else if (s.ready && s.ntp.delaySec > 0.0) {
            h.add({base + "delay_ms", s.name + " round trip", "ms", group, s.name}, unix,
                  s.ntp.delaySec * 1000.0);
        }
    }
}

} // namespace ubersdr_ntp
