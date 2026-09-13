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
// are the ones that matter here. Root delay is genuinely zero — there is no NTP
// path above us — and root dispersion carries the whole error budget from the
// Selector, propagation model included. Precision is the clock's reading
// resolution, as RFC 5905 defines it, and nothing more: a client adds it to
// root dispersion, so carrying the error budget in both would count it twice.
//
// TIMESTAMPS
//
// Every timestamp is this host's clock plus the combined offset, so a client
// synchronising against this server ends up on UTC rather than on our error.
// Receive timestamps come from the kernel via SO_TIMESTAMPNS where it is
// available: the difference between when the packet arrived and when this
// process got round to looking at it is real, and on a loaded machine it is
// larger than the jitter of the radio path.

#include "Selector.h"

#include <atomic>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

namespace ubersdr_ntp {

struct NtpStats {
    std::uint64_t requests = 0;
    std::uint64_t answered = 0;
    std::uint64_t ignored = 0;       // wrong mode, wrong version, too short
    std::uint64_t rateLimited = 0;
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
    bool rateLimitAllows(const std::string& key, double now);

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

    // Per-client token buckets. Keyed by address text rather than by a packed
    // sockaddr so one map covers v4 and v6, and swept rather than allowed to
    // grow, because the key space is the internet.
    struct Bucket { double tokens; double at; };
    std::unordered_map<std::string, Bucket> m_buckets;
    double m_lastSweep = 0.0;
    // A hard ceiling on m_buckets, for a spoofed-source flood the periodic
    // sweep cannot keep up with. ~100 bytes an entry, so ~10 MB at the cap;
    // see rateLimitAllows for what happens beyond it.
    static constexpr std::size_t kMaxBuckets = 100000;
};

} // namespace ubersdr_ntp
