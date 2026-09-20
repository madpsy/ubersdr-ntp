#pragma once

// What the Selector, the status report and main() see of a source, whichever
// kind it is.
//
// There are two implementations — Source, one UberSDR receiver decoding a
// time-signal broadcast, and NtpPeer, one upstream NTP server — and almost
// nothing in common between how they work. What they do have in common is the
// only thing anything above them needs: each one measures UTC minus the DAEMON
// clock (SampleClock.h), reports how well it knows that, and says why not when
// it does not know it at all.
//
// Measuring against the daemon clock rather than the host clock is what makes
// them combinable without a conversion. Both take their timestamps on the same
// free-running oscillator, so their offsets are two estimates of one quantity
// and the difference between them is real disagreement rather than two reads of
// a clock something else is steering in between.
//
// ACTIVATION
//
// A source can be held idle. That is what `cold` secondary mode is (see
// ClockConfig): nothing is connected until the primary class fails, because a
// long-lived UberSDR session occupies a listener slot on a receiver that may
// not be ours, and polling a public NTP server that is not being used is asking
// somebody else's machine to do work for nothing. setActive(false) is therefore
// a real disconnection, not a flag on the output — and setActive(true) has to
// acquire from nothing, which is why the mode costs minutes on failover and why
// `standby` exists.

#include "SourceSnapshot.h"

#include <string>
#include <vector>

namespace ubersdr_ntp {

class TimeSource {
public:
    virtual ~TimeSource() = default;

    // Start and stop the source's own threads. Bracketing the object's life;
    // not the same as activation, which happens while it is running.
    virtual void start() = 0;
    virtual void stop() = 0;

    virtual const std::string& name() const = 0;
    virtual SourceKind kind() const = 0;

    // Everything it knows about itself, copied out under its own lock.
    virtual SourceSnapshot snapshot() const = 0;

    // Drop whatever it has and acquire from nothing, at the Selector's word
    // that it has disagreed with its peers for too long. Safe from any thread;
    // acted on within about a second.
    virtual void requestReacquire(const std::string& why) = 0;

    // Connect (or keep connected) / disconnect and stay disconnected. `why` is
    // shown in the status report so a source sitting idle says which decision
    // put it there rather than looking broken. Idempotent: calling it with the
    // state the source is already in does nothing, which matters because the
    // Selector calls it on every pass.
    virtual void setActive(bool on, const std::string& why) = 0;
    virtual bool active() const = 0;

    // The daemon clock's drift against UTC, as the combination currently knows
    // it, handed down every pass. One crystal, one rate: a source that cannot
    // fit it from its own samples uses this instead of assuming zero. See
    // OffsetEstimator::setRatePrior for why assuming zero was expensive.
    //
    // Sources that fit their own rate ignore it, and a source using it is kept
    // out of the average it came from, so this only ever flows downwards.
    virtual void setSystemRate(double /*rateSec*/, double /*uncertaintySec*/, bool /*known*/) {}

    // The decoded times refused since the last call, as how far each was from
    // what the source's own history says the time is, in seconds -- and then
    // forgotten, so the caller sees each one once. For the history chart: a
    // refusal that happens once a month is a curiosity, one that happens
    // every night is a receiver worth looking at. Only a radio source has any.
    virtual std::vector<double> takeRejectedJumps() { return {}; }
};

} // namespace ubersdr_ntp
