#pragma once

// Turning several receivers into one time.
//
// The reason for having more than one source is that HF propagation makes any
// single one unreliable: 10 MHz from Fort Collins is excellent in the
// afternoon and gone at 3am, 5 MHz is the other way round, and a receiver at
// the other end of the country hears a different subset again. Redundancy here
// is not about hardware failure, it is about the band.
//
// That makes the combining problem the same one NTP itself solves, so it is
// solved the same way:
//
//   1. CANDIDATES. A source counts only if it is locked, its measurements are
//      fresh, and it has enough of them to have been filtered.
//
//   2. INTERSECTION (Marzullo, as refined by Mills). Each candidate asserts an
//      interval [offset - dist, offset + dist] that it believes contains the
//      true offset. Find the largest set of intervals with a point in common.
//      A source that is confidently wrong — and a time-code decoder CAN be
//      confidently wrong, because a deep fade biases every frame in the voter's
//      window the same way — asserts an interval that overlaps nobody, and is
//      discarded rather than averaged in. This is the step that makes two
//      sources worth more than twice one.
//
//   3. COMBINE. Weight the survivors by 1/dist^2 and by the configured weight.
//      A source claiming 8 ms then counts for sixteen times one claiming 32 ms,
//      which is the right ratio when the claim is honest and the reason the
//      dispersion each source reports has to be honest.
//
// With one source there is nothing to intersect and step 2 does nothing; the
// answer is that source's offset and its dispersion, which is the correct and
// slightly humbling result.

#include "Source.h"

#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace ubersdr_ntp {

struct Combined {
    bool valid = false;         // an offset is available at all
    bool synchronised = false;  // ...and it is fresh enough to serve as stratum 1
    double offsetSec = 0.0;     // add to this host's clock to get UTC
    double dispersionSec = 0.0; // how far out that could be
    double ageSec = 0.0;        // since the last contributing measurement
    int used = 0;               // sources that survived the intersection
    int candidates = 0;         // sources that were eligible to be considered
    std::string refid = "WWV";  // NTP reference identifier of the dominant station
    bool leapPending = false;
    std::vector<std::string> usedNames;
    std::vector<std::string> rejectedNames;
    std::string note;           // why it is not synchronised, when it is not
};

class Selector {
public:
    Selector(double coastSeconds, double coastDriftPpm, int minSources);

    // Recomputes from the current snapshots. Called on a timer and by the NTP
    // server; cheap enough to call per request but not called per request, so
    // a burst of clients cannot turn into a burst of work.
    Combined combine(const std::vector<SourceSnapshot>& snaps, double nowRealtime);

    // The last result, for the NTP server to answer from without recomputing.
    Combined current() const;

private:
    mutable std::mutex m_mu;
    Combined m_last;

    // The coasting state: the last good offset and when it was good.
    bool m_haveLast = false;
    double m_lastGoodOffset = 0.0;
    double m_lastGoodDispersion = 0.0;
    double m_lastGoodAt = 0.0;

    double m_coastSeconds;
    double m_coastDriftPpm;
    int m_minSources;
};

} // namespace ubersdr_ntp
