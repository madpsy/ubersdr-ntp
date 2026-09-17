#pragma once

// A short memory for the figures on the status page.
//
// The page shows each figure as it is now. What it cannot show is whether now
// is normal: an offset of -1.6 ms means one thing if it has sat there all
// afternoon and another if it was +4 ms twenty minutes ago, and the
// difference between the classes is only interesting as a trend. So a handful
// of series are kept at two resolutions -- one-minute averages over the last
// hour, thirty-minute averages over the last day -- and served to the page to
// draw under the figures they belong to.
//
// Bounded by construction: a series is a fixed number of buckets per tier,
// and the series themselves are the served figures plus a few per configured
// source, whose number does not grow at runtime. MetricHistory refuses new
// series past kMaxSeries anyway, so nothing a source reports can make it grow.
//
// In memory only. The log file is the durable record, and a day of history is
// what the page needs to say whether the present is ordinary.

#include <cstdint>
#include <deque>
#include <map>
#include <mutex>
#include <string>
#include <vector>

namespace ubersdr_ntp {

struct MetricBucket {
    double start = 0.0;   // UTC, seconds, aligned to the tier's width
    double sum = 0.0;
    double min = 0.0;
    double max = 0.0;
    std::uint32_t count = 0;

    double mean() const { return count ? sum / count : 0.0; }
};

// One resolution of one series: completed buckets plus the one filling now.
class MetricTier {
public:
    MetricTier(double widthSec, std::size_t capacity) : m_width(widthSec), m_capacity(capacity) {}

    void add(double unix, double value);

    // Every bucket that starts within the last `capacity` widths of `now`,
    // oldest first, the one still filling last.
    std::vector<MetricBucket> points(double now) const;

    double width() const { return m_width; }
    std::size_t capacity() const { return m_capacity; }

private:
    double m_width;
    std::size_t m_capacity;
    std::deque<MetricBucket> m_done;
    MetricBucket m_current;
    bool m_haveCurrent = false;
};

struct MetricSeriesInfo {
    std::string id;       // stable key, e.g. "served.offset_ms" or "source.r1.snr_db"
    std::string label;    // for people
    std::string unit;     // "ms", "dB"
    std::string group;    // served, class, radio, ntp
    std::string source;   // the source it belongs to, empty for the others
};

// Which of a small set of states something was in, over time: kept as the
// instants it changed, not as samples, so a day of a state that never changed
// is one entry. Bounded both ways -- changes older than the longest window are
// dropped, and past kMaxChanges the oldest go regardless, so a state flapping
// four times a second cannot grow it.
class StateTimeline {
public:
    static constexpr std::size_t kMaxChanges = 2000;
    static constexpr double kKeepSec = 25.0 * 3600.0;

    // Records `state` as current from `unix` on, if it differs from the last.
    void set(double unix, const std::string& state);

    struct Span {
        double start;
        std::string state;
    };
    // The changes since `from`, preceded by the one in effect at `from` (whose
    // start is then before it), oldest first.
    std::vector<Span> since(double from) const;

private:
    std::deque<Span> m_changes;
};

class MetricHistory {
public:
    static constexpr std::size_t kMaxSeries = 256;

    enum class Range { Hour, Day };
    static constexpr double kHourWidthSec = 60.0;
    static constexpr std::size_t kHourBuckets = 60;
    static constexpr double kDayWidthSec = 1800.0;
    static constexpr std::size_t kDayBuckets = 48;

    // One sample. The series is created on first use with `info`; afterwards
    // only the id is looked at. Non-finite values are dropped: a gap is the
    // honest record of a figure that did not exist.
    void add(const MetricSeriesInfo& info, double unix, double value);

    struct Series {
        MetricSeriesInfo info;
        std::vector<MetricBucket> points;
    };
    // Every series, in the order they were first added, which is config
    // order for per-source series and so gives the page a stable colour for
    // each source.
    std::vector<Series> snapshot(Range r, double now) const;

    std::size_t seriesCount() const;

    // Which class served the time, as a timeline, for the page's changeover
    // bar. Called every pass; only a change is stored.
    void setServing(double unix, const std::string& state);
    std::vector<StateTimeline::Span> serving(Range r, double now) const;
    static double windowSec(Range r) {
        return r == Range::Hour ? kHourWidthSec * kHourBuckets : kDayWidthSec * kDayBuckets;
    }

private:
    struct Entry {
        MetricSeriesInfo info;
        MetricTier hour{kHourWidthSec, kHourBuckets};
        MetricTier day{kDayWidthSec, kDayBuckets};
    };
    mutable std::mutex m_mu;
    std::vector<Entry> m_series;
    std::map<std::string, std::size_t> m_index;
    StateTimeline m_serving;
};

struct Combined;
struct SourceSnapshot;

// Takes one sample of everything worth a history from one pass of the main
// loop: the served offset and dispersion, each class's median offset and the
// difference between them, and per source its offset, its residual against
// its own class, and one figure of its own kind -- a receiver's tick SNR, an
// upstream's round trip. A figure that does not exist this pass (no lock, not
// synchronised) is not sampled, and shows as a gap.
void sampleMetrics(MetricHistory& h, const std::vector<SourceSnapshot>& sources,
                   const Combined& c, double unix);

} // namespace ubersdr_ntp
