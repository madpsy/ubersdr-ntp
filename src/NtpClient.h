#pragma once

// One upstream NTP server, polled as a client.
//
// The mirror of NtpServer.cpp: that one answers mode-3 requests, this one sends
// them. It exists because the radio sources and the network fail to different
// things — the band dies every night and takes every receiver hearing the same
// transmitter with it, the network dies for its own unrelated reasons — and a
// clock meant to run unattended for months wants a reference that survives each.
//
// WHAT IT MEASURES, AND WHY IT IS THE SAME QUANTITY A RECEIVER MEASURES
//
// The four NTPv4 timestamps give the offset and the round trip:
//
//     offset = ((t2 - t1) + (t3 - t4)) / 2
//     delay  =  (t4 - t1) - (t3 - t2)
//
// with t1 and t4 taken HERE and t2 and t3 reported by the server. Taking t1 and
// t4 on the daemon clock (SampleClock.h) rather than the host clock makes that
// offset "UTC minus the daemon clock" — bit for bit the same quantity a WWV
// decoder produces, measured against the same free-running oscillator. That is
// what lets one Selector intersect and average the two kinds without a
// conversion step in between, and it is the reason this is a peer of the radio
// sources rather than a special case bolted onto the side of them.
//
// The offset is exact only if the path is symmetric. It is not, quite, and the
// asymmetry is invisible from here — the same structural blindness the radio
// delay model has, for the same reason: a constant one-way bias is
// indistinguishable from a clock that is wrong. What CAN be done is to prefer
// the least-delayed sample, because delay above the minimum is queueing, and
// queueing is what asymmetry is made of. Hence the clock filter below.
//
// THE CLOCK FILTER
//
// Eight samples in a shift register, and the smallest round trip among them is
// the reference. This is NTP's own filter and it is the same argument
// SampleClock.h makes at length for the audio path: transport delay is bounded
// below and unbounded above, so the minimum is the estimator that matters and
// the mean is a statistic of the load.
//
// The minimum is used as a GATE rather than as the output. Handing on
// whichever sample currently has the lowest delay sounds right and starves the
// estimator: a peer whose first poll happens to be its fastest hands on nothing
// for the next seven, because the winner has not changed and the same packet
// must not be counted eight times — eight minutes of silence at a 64-second
// poll, against an estimator that needs four samples before it reports
// anything. So every sample is handed on whose excess delay above the best in
// the register is small enough that the bias it could carry does not matter,
// and the rest are dropped as queueing spikes. See filterAdd.
//
// A spike is not a fault in the path, though, and a peer that only drops them
// can starve: the best round trip stays in the register for eight polls, so one
// unusually fast reply makes every ordinary one look queued for the next eight
// minutes, and an anycast server like time.cloudflare.com, whose round trip
// moves from reply to reply, will do that routinely. So the gate widens with
// the spread of the register's clean half, a run of spikes is taken as the path
// having changed rather than as more spikes, and a dropped or lost reply is
// followed a couple of seconds later by another try rather than a whole poll.
//
// NAMES
//
// A server given by name is looked up again when the answer's DNS TTL runs
// out, capped at an hour. An anycast or pool name is several machines and the
// set moves; the TTL is how long the name's owner says the answer holds, and
// keeping an address past it is how a client ends up polling a server that has
// been taken out of rotation. The address in use is kept if the new answer
// still includes it -- a pool's answers come back in a different order every
// time, and hopping between members on every lookup would reset the filter for
// nothing. A lookup that fails keeps the address that works. An address
// literal has no TTL and is never looked up.
//
// SPOOFING
//
// The transmit timestamp this sends is sixty-four random bits, not the clock,
// and a reply is refused unless its originate field returns them verbatim. An
// off-path attacker forging a reply has to guess that field; a real clock is
// guessable to within the poll interval, and random bits are not. chrony does
// the same. It costs nothing, because t1 is recorded locally and the originate
// field was never read for anything else.
//
// THREADS
//
// One per peer, which sends, waits for the reply with a timeout, and sleeps
// until the next poll. Everything the outside world reads goes through m_mu.

#include "Config.h"
#include "OffsetEstimator.h"
#include "TimeSource.h"

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace ubersdr_ntp {

// The addresses this host answers on, gathered once at startup.
//
// For loop detection: a server whose reference identifier is one of our own
// addresses is synchronised to THIS daemon, and taking time from it would close
// a loop — we would be disciplining ourselves through a machine that is
// disciplining itself through us, and the pair would wander off together with
// nothing left anywhere in the circle that had heard a radio. It is a real
// configuration mistake and a cheap one to make: point a host's chrony at this
// daemon, then list that host here as a fallback.
std::vector<std::uint32_t> localIpv4Addresses();

class NtpPeer final : public TimeSource {
public:
    NtpPeer(NtpSourceConfig cfg, std::vector<std::uint32_t> localAddrs);
    ~NtpPeer() override;

    NtpPeer(const NtpPeer&) = delete;
    NtpPeer& operator=(const NtpPeer&) = delete;

    void start() override;
    void stop() override;

    const std::string& name() const override { return m_cfg.name; }
    SourceKind kind() const override { return SourceKind::Ntp; }
    SourceSnapshot snapshot() const override;

    // Throw away the filter and the offset history and resolve the name again,
    // at the Selector's word that this peer has disagreed with its own class
    // for too long. A peer cannot "re-acquire" the way a decoder can — there is
    // no lock to take again — but a name that resolves to several addresses can
    // land on a different one, and a server that has gone wrong is often one
    // address out of a pool that has.
    void requestReacquire(const std::string& why) override;

    void setActive(bool on, const std::string& why) override;
    void setSystemRate(double rateSec, double uncertaintySec, bool known) override;
    bool active() const override { return m_active.load(); }

private:
    void run();

    // One send-and-wait. Returns false if nothing usable came back; `err` then
    // says why, in words meant for the status page.
    bool pollOnce(std::string& err);

    bool ensureSocket(std::string& err);   // connect if not connected
    // Resolve the name and connect(2) the UDP socket to one of its addresses,
    // keeping the current one if the answer still includes it, and set when to
    // look again from the answer's TTL. On failure an open socket is kept.
    bool connectPeer(std::string& err);
    void closeSocket();
    void dropTiming(const char* why);      // caller must NOT hold m_mu

    // One accepted measurement, into the filter. Caller holds m_mu.
    struct Sample {
        double offsetSec = 0.0;
        double delaySec = 0.0;
        double dispersionSec = 0.0;  // what it was worth when it was taken
        double atSec = 0.0;          // daemon clock at t4
        std::uint64_t seq = 0;
        bool valid = false;
    };
    void filterAdd(const Sample& s);       // caller holds m_mu
    void recompute();                      // caller holds m_mu

    NtpSourceConfig m_cfg;
    std::vector<std::uint32_t> m_localAddrs;

    std::thread m_thread;
    std::atomic<bool> m_running{false};
    std::atomic<bool> m_active{true};
    std::atomic<bool> m_reacquire{false};
    std::mutex m_wake;
    std::condition_variable m_wakeCv;

    int m_fd = -1;                  // polling thread only
    std::string m_resolvedText;     // ...and what it resolved to, for the log
    double m_resolvedAt = 0.0;      // when the socket was connected
    double m_resolveDueAt = 0.0;    // monotonic time the name is looked up again
    bool m_literal = false;         // the server is an address, not a name
    bool m_resolveFailing = false;  // the last lookup failed (log once a run)
    int m_consecutiveTimeouts = 0;

    // Quick retries: after a lost reply or a spike the next poll comes a
    // couple of seconds later instead of a whole interval, a few times in a
    // row at most. m_retryable is set by pollOnce/filterAdd for the poll just
    // made.
    bool m_retryable = false;
    int m_quickRetries = 0;

    // How long the burst still has to run. Counted down rather than timed, so a
    // burst interrupted by an unreachable server resumes rather than expires.
    int m_burstLeft = 0;
    // What the poll interval has been stretched to by a kiss-o'-death RATE, or
    // 0 for the configured one. A server that says it is being asked too often
    // is not negotiating.
    double m_backoffPollSec = 0.0;
    // Consecutive accepted replies since the last refusal, for easing that
    // backoff back down. A peer stuck at a useless poll interval because of one
    // kiss on its opening burst is a fallback that is not one.
    int m_goodRun = 0;

    mutable std::mutex m_mu;
    SourceSnapshot m_snap;
    std::string m_activeReason;
    std::string m_reacquireWhy;

    static constexpr int kFilterSize = 8;
    Sample m_filter[kFilterSize];
    std::uint64_t m_seq = 0;
    std::uint64_t m_handedOn = 0;   // seq of the newest sample given to the estimator
    int m_spikeRun = 0;             // consecutive samples dropped as spikes

    OffsetEstimator m_offsets;
};

} // namespace ubersdr_ntp
