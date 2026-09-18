#pragma once

// UTC does not jump: the continuity check on a radio source's decoded times.
//
// A time code is a few dozen bits, and a bit can be misread. A fade that biases
// the weight-4 bit of the minutes, or of the hours, produces a frame that is
// self-consistent in every way the decoder can check, votes clean, and says it
// is four minutes, or four hours, from the truth. Both have happened live
// (2026-09-17: -240 s on one receiver in the morning, -14 400 s on the other in
// the evening), the voter certified both, and with two receivers configured the
// consensus could not tell which was wrong -- so the served time followed. No
// agreement among frames can catch it, and the decoder's +-24 h plausibility
// bound against the host clock is far too wide to (and the host clock is set
// from this daemon anyway).
//
// What does catch it is that UTC is continuous and the daemon clock is the raw
// oscillator: their difference moves at a crystal's rate and nothing else. So
// a source's own filtered offset, carried forward along its own rate, predicts
// where its next decode must land to a fraction of a second, and a decode that
// lands whole seconds away has been misread, whatever its vote quality.
//
// No other source is consulted. This is a source checking itself, which is what
// keeps a receiver from being overruled by a network it may be there to
// distrust, and what still works with a single receiver.
//
// The history itself could be wrong, and that must not stick for ever either:
//
//   - The FIRST time has no history. It is taken only once it has agreed with
//     itself kFirstReadings times over at least kFirstSpanSec, so one misread
//     at the moment of locking cannot become the history everything after it
//     is judged against.
//   - A history that is wrong anyway -- a first time that slipped through, the
//     machine suspended while the daemon clock stood still -- is replaced by a
//     new time once that time has held in every reading, with none agreeing
//     with the old history in between, for kAdoptSpanSec. A misread comes and
//     goes; a real step stays.
//
// A leap second is the one genuine jump in UTC: exactly one second, in the
// first hours of a month, after a warning. It is let straight through there.
//
// Pure logic, no locking: the owner serialises calls.

namespace ubersdr_ntp {

class TimeContinuity {
public:
    // How far a decode may land from the history before it is a misread rather
    // than a measurement. Everything real that moves a receiver's offset is
    // milliseconds: propagation, the delay model, an edge tracker on the wrong
    // part of the pulse (about 100 ms, and the consensus's job). Half a second
    // is past all of them and short of the smallest thing a misread time code
    // can do, which is a whole second.
    static constexpr double kJumpLimitSec = 0.5;
    // ...widened by what the daemon clock could have drifted since the history
    // was last refreshed: a generous crystal, well past the ~12 ppm measured
    // here, so a source back after a day away is still judged fairly.
    static constexpr double kDriftPpm = 50.0;
    // Two readings of one new time agree when they are this close.
    static constexpr double kAgreeSec = 0.25;
    // The first time: this many readings in a row, over at least this long.
    // Two minutes less a few seconds, since readings come a minute apart and
    // are dated by wherever the edges fall.
    static constexpr int kFirstReadings = 3;
    static constexpr double kFirstSpanSec = 115.0;
    // A time that contradicts the history: held, in every reading, for this
    // long. Longer than any misread seen live (a few minutes at most), short
    // enough that a wrong history corrects itself unattended.
    static constexpr int kAdoptReadings = 6;
    static constexpr double kAdoptSpanSec = 600.0;

    enum class Outcome {
        Accepted,        // agrees with the history
        FirstConfirmed,  // the first time, now confirmed: the history starts here
        Confirming,      // the first time, not yet confirmed
        Refused,         // a jump from the history
        Adopted,         // a jump that held long enough to replace the history
        LeapSecond,      // the announced one-second step
    };

    struct Verdict {
        Outcome outcome = Outcome::Accepted;
        double jumpSec = 0.0;    // against the history (0 with none)
        int readings = 0;        // of the pending time, this one included
        double spanSec = 0.0;    // ...from its first reading to this one
        // Whether the offsets filtered from the old history must be dropped:
        // they describe a time this source no longer believes.
        bool discardFiltered() const {
            return outcome == Outcome::Adopted || outcome == Outcome::LeapSecond;
        }
        bool admitted() const {
            return outcome != Outcome::Confirming && outcome != Outcome::Refused;
        }
    };

    // One decoded time, as the offset it implies (UTC minus the daemon clock,
    // delay model applied) at daemon instant `atSec`. `leapWindow`: this is the
    // start of a month and a leap warning was believed within the last day.
    Verdict judge(double impliedOffsetSec, double atSec, bool leapWindow);

    // The filtered offset the source now holds, which is the history from here
    // on. Called whenever the filter produces one.
    void noteFiltered(double offsetSec, double rate, double atSec);

    bool haveHistory() const { return m_have; }

private:
    void notePending(double offsetSec, double atSec);
    void startHistory(double offsetSec, double atSec);

    bool m_have = false;
    double m_offset = 0.0;
    double m_rate = 0.0;
    double m_at = 0.0;

    bool m_pending = false;
    double m_pendingOffset = 0.0;
    double m_pendingFirstAt = 0.0;
    double m_pendingLastAt = 0.0;
    int m_pendingReadings = 0;
};

} // namespace ubersdr_ntp
