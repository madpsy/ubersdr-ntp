#pragma once

// An NTPv4 server (RFC 5905) answering from the radio clock.
//
// Server mode only: it answers mode-3 client requests and nothing else. In
// particular it never answers mode 6 or mode 7 — the control and private
// modes — because those are the ones behind every ntpd reflection-amplification
// advisory of the last decade, and this program has no use for either.
//
// STRATUM AND HONESTY
//
// A radio clock is stratum 1 by definition: its reference is not another NTP
// server. That does not make it accurate, and the fields that say how accurate
// are the ones that matter here. Root dispersion carries the whole error budget
// from the Selector, propagation model included.
//
// Stratum 1 is no longer unconditional, because the time can now come from an
// upstream NTP server instead — as a fallback, or alongside the radio; see
// ClockConfig. When it does, this server is one stratum below that upstream and
// says so, and the root delay stops being zero (there is no NTP path above a
// radio clock; there is one above a pool server) and becomes the length of the
// path back to the primary reference. Claiming stratum 1 off a pool server
// would be a lie of the kind everything else here takes trouble to avoid, and a
// client told this was a radio clock would weigh it accordingly.
//
// Precision is the clock's reading resolution, as RFC 5905 defines it, and
// nothing more: a client adds it to root dispersion, so carrying the error
// budget in both would count it twice.
//
// TIMESTAMPS
//
// Every timestamp is this host's clock plus the combined offset, so a client
// synchronising against this server ends up on UTC rather than on our error.
// Receive timestamps come from the kernel via SO_TIMESTAMPNS where it is
// available: the difference between when the packet arrived and when this
// process got round to looking at it is real, and on a loaded machine it is
// larger than the jitter of the radio path.

#include "RateLimiter.h"
#include "Selector.h"

#include <atomic>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace ubersdr_ntp {

struct NtpStats {
    std::uint64_t requests = 0;
    std::uint64_t answered = 0;
    std::uint64_t ignored = 0;       // wrong mode, wrong version, too short
    std::uint64_t rateLimited = 0;    // over the limit and not answered with the time
    std::uint64_t kodSent = 0;        // ...of which sent a RATE kiss-o'-death
    std::uint64_t unsynchronised = 0; // answered with LI=3 / stratum 0
    std::uint64_t sendErrors = 0;
};

class NtpServer {
public:
    NtpServer(NtpConfig cfg, Selector& selector);
    ~NtpServer();

    // Binds every configured address. Returns false with `err` set if none
    // could be bound — a time server that is not listening is a silent
    // failure, so this is fatal rather than a warning.
    bool start(std::string& err);
    void stop();

    NtpStats stats() const;

private:
    void serve(int fd, const std::string& label);

    NtpConfig m_cfg;
    Selector& m_selector;

    // Paired: the label is only for the log, but indexing the configured list
    // by fd position names the wrong address as soon as one of them failed to
    // bind -- which is exactly when a clear log matters.
    struct Listener { int fd; std::string label; };
    std::vector<Listener> m_fds;
    std::vector<std::thread> m_threads;
    std::atomic<bool> m_running{false};

    mutable std::mutex m_mu;
    NtpStats m_stats;

    RateLimiter m_limiter;
};

} // namespace ubersdr_ntp
