#include "TimeContinuity.h"

#include <cmath>

namespace ubersdr_ntp {

void TimeContinuity::notePending(double offsetSec, double atSec) {
    const bool agrees =
        m_pending &&
        std::abs(offsetSec - m_pendingOffset) <=
            kAgreeSec + std::abs(atSec - m_pendingLastAt) * kDriftPpm * 1e-6;
    if (!agrees) {
        m_pending = true;
        m_pendingFirstAt = atSec;
        m_pendingReadings = 0;
    }
    ++m_pendingReadings;
    m_pendingOffset = offsetSec;
    m_pendingLastAt = atSec;
}

void TimeContinuity::startHistory(double offsetSec, double atSec) {
    // The rate is kept across a replacement: it is the crystal's, and the
    // crystal did not change when the history did. Zero on the first.
    m_have = true;
    m_offset = offsetSec;
    m_at = atSec;
    m_pending = false;
    m_pendingReadings = 0;
}

TimeContinuity::Verdict TimeContinuity::judge(double impliedOffsetSec, double atSec,
                                              bool leapWindow) {
    Verdict v;

    if (!m_have) {
        notePending(impliedOffsetSec, atSec);
        v.readings = m_pendingReadings;
        v.spanSec = m_pendingLastAt - m_pendingFirstAt;
        if (v.readings >= kFirstReadings && v.spanSec >= kFirstSpanSec) {
            startHistory(impliedOffsetSec, atSec);
            v.outcome = Outcome::FirstConfirmed;
        } else {
            v.outcome = Outcome::Confirming;
        }
        return v;
    }

    const double since = atSec - m_at;
    const double predicted = m_offset + m_rate * since;
    const double limit = kJumpLimitSec + std::abs(since) * kDriftPpm * 1e-6;
    v.jumpSec = impliedOffsetSec - predicted;

    if (std::abs(v.jumpSec) <= limit) {
        // Agreeing with the history ends any run of disagreement: the new time
        // has to hold in EVERY reading to replace it.
        m_pending = false;
        m_pendingReadings = 0;
        v.outcome = Outcome::Accepted;
        return v;
    }

    if (leapWindow && std::abs(std::abs(v.jumpSec) - 1.0) <= kAgreeSec) {
        startHistory(impliedOffsetSec, atSec);
        v.outcome = Outcome::LeapSecond;
        return v;
    }

    notePending(impliedOffsetSec, atSec);
    v.readings = m_pendingReadings;
    v.spanSec = m_pendingLastAt - m_pendingFirstAt;
    if (v.readings >= kAdoptReadings && v.spanSec >= kAdoptSpanSec) {
        startHistory(impliedOffsetSec, atSec);
        v.outcome = Outcome::Adopted;
    } else {
        v.outcome = Outcome::Refused;
    }
    return v;
}

void TimeContinuity::noteFiltered(double offsetSec, double rate, double atSec) {
    m_have = true;
    m_offset = offsetSec;
    m_rate = rate;
    m_at = atSec;
}

} // namespace ubersdr_ntp
