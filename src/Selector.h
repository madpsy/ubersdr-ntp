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
//   3. COMBINE. Weight the survivors by 1/uncertainty^2 and by the configured
//      weight. A source claiming 8 ms then counts for sixteen times one claiming
//      32 ms, which is the right ratio when the claim is honest and the reason
//      the dispersion each source reports has to be honest.
//
//      The uncertainty used here is the source's OWN -- its jitter and how well
//      its sample clock is known -- and not the interval it asserts in step 2.
//      The interval includes the delay model's uncertainty, which is the same
//      for every source because they all run the same model: weighting by a
//      number they share compresses the ratio between a good source and a bad
//      one until it stops meaning anything. Step 2 wants the honest interval,
//      step 3 wants the difference between them, and they are not the same
//      number.
//
// With one source there is nothing to intersect and step 2 does nothing; the
// answer is that source's offset and its dispersion, which is the correct and
// slightly humbling result.
//
// TWO CLASSES
//
// The sources are not all radio any more. Some are upstream NTP servers, and
// one class is the primary while the other is the secondary; see ClockConfig
// for what the three secondary modes mean. Three things change here, and only
// three.
//
//   WHO IS ELIGIBLE. In `always` mode, everybody, and steps 1-3 run over both
//   classes at once exactly as written. In `standby` and `cold` the secondary's
//   candidates are held out of the intersection while the primary can serve,
//   and swapped in when it cannot -- after a hold-down, and back again after a
//   longer one, because a decoder coming out of a fade re-locks and loses it
//   repeatedly for several minutes and every swap is a step in the served time.
//
//   WHO MAY CONVICT WHOM. Step 2's refusal test rests on an argument that only
//   holds inside the radio class: every receiver is hearing ONE transmitter and
//   ONE second edge, so anything past 30 ms is a misdecode rather than a delay
//   model. A radio source and an NTP server have no such relationship -- they
//   are two independent measurements of UTC, and a disagreement between them is
//   the radio delay model's error, which is a real quantity with a real value
//   that may well exceed 30 ms. So residuals are computed WITHIN a class, and a
//   source is never refused for disagreeing with the other class.
//
//   WHAT STRATUM COMES OUT. A radio source is stratum 0 -- a reference, not a
//   server -- so serving from one is stratum 1. An upstream at stratum 2 makes
//   us stratum 3. The rule is NTP's own: one more than the lowest stratum among
//   the survivors, which means a mixed set is still stratum 1, because a radio
//   reference really is in it.
//
// THE DIFFERENCE BETWEEN THE CLASSES IS WORTH MORE THAN THE FAILOVER
//
// When both classes are measured at once -- `always`, or `standby`, which is
// why standby exists -- the difference between their consensuses is the first
// absolute reference this daemon has ever had. Every radio source shares the
// chain-delay constant, the codec delay and the decoder's edge bias, and those
// terms cancel exactly in any comparison between receivers: no number of
// receivers can measure them. An NTP server does not share them. So
// `classDelta` below is a direct measurement of the sum of the constants the
// delay model cannot see, which the README has always had to state as an
// unvalidated estimate of about 3 ms. It is reported and never applied: a
// correction that made the radio agree with the network by construction would
// turn the one independent check in the system into a tautology.

#include "SourceSnapshot.h"

#include <cstdint>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace ubersdr_ntp {

// What one source's corrected offset says against the others'.
//
// Every source is listening to the same transmitter, so once each one's delay
// model has done its job they must all report the same offset -- not nearly,
// exactly, because the event they are timing is one event. Whatever is left
// over is model error by construction, and it is the only part of the delay
// budget that can be observed at all: the terms every source shares (the
// receiver chain, the codec, the decoder's bias) cancel in the comparison and
// no amount of agreement between receivers can measure them. What survives is
// the part that differs -- the network path and the propagation path -- which
// is also the part that goes wrong.
struct SourceResidual {
    std::string name;
    double instantSec = 0.0;  // against the median of the OTHERS, right now
    double averagedSec = 0.0; // ...smoothed, because one edge proves nothing
    bool haveAverage = false;
    int peers = 0;            // how many others IN ITS OWN CLASS it was compared against
    bool refused = false;     // ...and it disagreed with them past explaining
    double settledForSec = 0.0; // how long it has been measured at all
    SourceKind kind = SourceKind::Radio;
};

// Where the served time is coming from at this instant.
enum class ServingClass {
    None,        // nothing usable and nothing to coast from
    Primary,
    Secondary,
    Both,        // `always` mode with candidates in each
    Coasting,    // the crystal, on the last good offset
};
const char* servingClassName(ServingClass c);

// What the two classes say about each other.
//
// Only meaningful while both are being measured at once, which is `always` and
// `standby` but not `cold`. See the header: this is the one measurement in the
// system that can see the constants every radio source shares, because the
// class it is measured against does not share them.
struct ClassDelta {
    bool valid = false;
    double instantSec = 0.0;    // primary consensus minus secondary consensus, now
    double averagedSec = 0.0;   // ...smoothed over the same tau as a source residual
    bool haveAverage = false;
    double settledForSec = 0.0;
    int primarySources = 0;     // how many went into each side
    int secondarySources = 0;
};

struct Combined {
    bool valid = false;         // an offset is available at all
    bool synchronised = false;  // ...and it is fresh enough to serve as stratum 1
    // The served time is the daemon clock (SampleClock.h) plus this offset, as
    // of atSec, moving at `rate`: use utcAt(), which is what every served
    // timestamp is formed from.
    double offsetSec = 0.0;
    double atSec = 0.0;
    double rate = 0.0;              // d(offsetSec)/d(daemon time); 1e-6 is 1 ppm
    double rateUncertainty = 0.0;
    bool rateMeasured = false;
    // Served time minus THIS HOST's clock, at atSec: the correction the host
    // would need. For display and the API's offset_ms; never used to serve.
    double hostOffsetSec = 0.0;
    double dispersionSec = 0.0; // how far out that could be
    double ageSec = 0.0;        // since the last contributing measurement
    int used = 0;               // sources that survived the intersection
    int candidates = 0;         // sources that were eligible to be considered

    // NTP's own stratum rule: one more than the lowest among the survivors,
    // where a radio source counts as 0 because a reference is not a server. So
    // radio-only is 1, an upstream at stratum 2 makes this 3, and a mixed set
    // is 1 because a radio reference really is in it.
    int stratum = 1;
    // The NTP path above this server, which is zero while any radio source is
    // in use -- there is no path above a radio clock -- and the selected
    // upstream's root delay plus the round trip to it when there is not.
    double rootDelaySec = 0.0;

    // The reference identifier, as text for people. When the served time comes
    // from an upstream rather than the radio it is that server's address, which
    // is what RFC 5905 requires above stratum 1, and refidAddress carries the
    // four bytes the packet needs.
    std::string refid = "WWV";
    bool refidIsAddress = false;
    std::uint32_t refidAddress = 0;

    // Which class is serving, how the failover stands, and what the two classes
    // say about each other.
    ServingClass serving = ServingClass::None;
    ClassDelta classDelta;
    int primaryCandidates = 0;
    int secondaryCandidates = 0;
    // Seconds until the pending swap, or 0 when none is pending. One of the two
    // at most: the primary is either failing or recovering, never both.
    double failoverInSec = 0.0;
    double failbackInSec = 0.0;
    // Why the serving class is what it is, in words for the status page.
    std::string servingNote;

    bool leapPending = false;
    std::vector<std::string> usedNames;
    std::vector<std::string> rejectedNames;
    // Why each source is not in use, by name, in words meant for the status
    // page. A source in use has no entry.
    std::map<std::string, std::string> notUsedReasons;
    // Sources refused for long enough that they should start over from
    // nothing. The Selector decides; the caller owns the sources and acts.
    std::vector<std::string> reacquireNames;
    std::string note;           // why it is not synchronised, when it is not
    std::vector<SourceResidual> residuals;

    // UTC at a daemon-clock instant.
    double utcAt(double daemonSec) const { return daemonSec + offsetSec + rate * (daemonSec - atSec); }
};

class Selector {
public:
    // The clock configuration is taken by value and kept: the primary class and
    // the secondary's mode do not change at runtime, and passing them per call
    // would let two callers disagree about which class was which.
    Selector(double coastSeconds, double coastDriftPpm, int minSources,
             ClockConfig clock = ClockConfig{});

    // Which sources the caller should be holding connections for, given where
    // the failover currently stands. Empty in every mode but `cold`, where it
    // is the whole of how a cold standby is brought up: the Selector decides
    // that the primary has gone, the caller owns the sources and acts.
    //
    // Returned as a decision per class rather than per source, because
    // activation is a property of the class -- a cold secondary comes up
    // entirely or not at all.
    struct Activation {
        bool secondaryActive = false;
        std::string reason;
    };
    Activation activation() const;

    // Recomputes from the current snapshots. Called on a timer and by the NTP
    // server; cheap enough to call per request but not called per request, so
    // a burst of clients cannot turn into a burst of work.
    // `now` is the daemon clock (daemonNow()).
    Combined combine(const std::vector<SourceSnapshot>& snaps, double now);

    // The last result, for the NTP server to answer from without recomputing.
    Combined current() const;

private:
    mutable std::mutex m_mu;
    Combined m_last;

    // Per-source agreement with the consensus, smoothed over TIME rather than
    // over calls: combine() runs four times a second, so a per-call smoothing
    // factor describes a window that changes whenever that rate does, and a
    // figure meant to average out a fade would instead average out five
    // seconds of it.
    struct ResidualState {
        double averagedSec = 0.0;
        double lastAtSec = 0.0;
        double firstAtSec = 0.0;
        // Whether the refusal has been logged. combine() runs four times a
        // second, and a refusal logged on every call buried everything else;
        // it is said once when it starts and once when it ends, and the status
        // block carries it in between.
        bool refusalLogged = false;
        // Whether the average holds any reading taken against enough peers
        // to mean something. Until then averagedSec and firstAtSec are unset.
        bool seeded = false;
    };

    // Self-recovery, kept apart from ResidualState because that is discarded
    // when a source starts over and this must survive it: the backoff is
    // about the RECEIVER, not about one lock.
    struct ReacquireState {
        // When the current refusal began, 0 if none. Cleared only when the
        // source is judged and accepted: time it spends unjudgeable (too few
        // peers) or out of the candidates is not acceptance and does not reset it.
        double refusedSinceSec = 0.0;
        int count = 0;                  // consecutive re-acquisitions without being accepted
    };
    std::map<std::string, ReacquireState> m_reacquire;
    std::map<std::string, ResidualState> m_residualAvg;

    // The coasting state: the last good offset and when it was good.
    bool m_haveLast = false;
    double m_lastGoodOffset = 0.0;     // at m_lastGoodAt
    double m_lastGoodRate = 0.0;       // ...and the rate it was moving at, to coast along
    double m_lastGoodRateUncertainty = 0.0;
    bool m_lastGoodRateMeasured = false;
    double m_lastGoodDispersion = 0.0;
    double m_lastGoodAt = 0.0;
    // When the newest measurement behind that offset was taken. Not the same
    // as m_lastGoodAt, which is refreshed every combine while synchronised:
    // coasting from it would report an age up to a candidate's maximum age too
    // young, and the reference timestamp with it.
    double m_lastGoodMeasuredAt = 0.0;

    // --- the failover state machine ---------------------------------------
    //
    // Two timers and a latch. The latch is which class is serving; the timers
    // are how long the condition for changing it has held, because both
    // directions need hysteresis and they need different amounts of it. See
    // ClockConfig for why failing over is cheap and failing back is not.
    bool m_onSecondary = false;
    double m_primaryLostSince = 0.0;     // 0: the primary has candidates now
    double m_primaryHealthySince = 0.0;  // 0: it does not
    // Whether the secondary is being asked to hold connections. Only `cold`
    // ever sets this false; read by activation() from another thread.
    bool m_secondaryActive = true;
    std::string m_activationReason;

    // The smoothed difference between the two classes' consensuses. Kept here
    // rather than recomputed because it is an average over five minutes and
    // combine() runs four times a second.
    struct ClassDeltaState {
        bool seeded = false;
        double averagedSec = 0.0;
        double lastAtSec = 0.0;
        double firstAtSec = 0.0;
    };
    ClassDeltaState m_classDelta;

    double m_coastSeconds;
    double m_coastDriftPpm;
    int m_minSources;
    ClockConfig m_clock;
};

} // namespace ubersdr_ntp
