// Per-client response limiting for the NTP service: see RateLimitConfig.
//
// A token bucket per client, where a client is its address cut to the
// configured prefix. Separate from NtpServer so the decisions can be tested
// without a socket.
#pragma once

#include "Config.h"
#include "IpPrefix.h"

#include <cstdint>
#include <mutex>
#include <random>
#include <string>
#include <unordered_map>
#include <vector>

namespace ubersdr_ntp {

enum class RateVerdict {
    Answer,  // within the limit, exempt, or not limited at all
    Leak,    // over the limit, answered anyway (rate_limit.leak)
    Kod,     // over the limit: send a RATE kiss-o'-death
    Drop,    // over the limit: send nothing
};

class RateLimiter {
public:
    explicit RateLimiter(RateLimitConfig cfg);

    // `now` in seconds on any monotonic clock.
    RateVerdict check(const struct sockaddr_storage& from, double now);

    // Clients currently tracked, for the tests.
    std::size_t tracked() const;

    // A hard ceiling on the buckets, for a spoofed-source flood the periodic
    // sweep cannot keep up with. ~100 bytes an entry, so ~10 MB at the cap;
    // see check() for what happens beyond it.
    static constexpr std::size_t kMaxBuckets = 100000;

private:
    struct Bucket {
        double tokens;
        double at;
        double kodAt;  // when this client was last sent a kiss-o'-death
    };

    RateLimitConfig m_cfg;
    std::vector<IpPrefix> m_exempt;
    mutable std::mutex m_mu;
    // Keyed by family and the masked address bytes, so one map covers v4 and
    // v6, and swept rather than allowed to grow, because the key space is the
    // internet.
    std::unordered_map<std::string, Bucket> m_buckets;
    double m_lastSweep = 0.0;
    std::minstd_rand m_rng{std::random_device{}()};
};

} // namespace ubersdr_ntp
