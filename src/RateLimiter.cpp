#include "RateLimiter.h"

#include <algorithm>
#include <limits>

namespace ubersdr_ntp {

RateLimiter::RateLimiter(RateLimitConfig cfg) : m_cfg(std::move(cfg)) {
    // Config::validate has already refused anything that does not parse.
    for (const std::string& e : m_cfg.exempt) {
        IpPrefix p;
        if (IpPrefix::parse(e, p)) m_exempt.push_back(p);
    }
}

std::size_t RateLimiter::tracked() const {
    std::lock_guard<std::mutex> lk(m_mu);
    return m_buckets.size();
}

RateVerdict RateLimiter::check(const struct sockaddr_storage& from, double now) {
    if (m_cfg.intervalSec <= 0.0) return RateVerdict::Answer;

    const IpAddr addr = ipAddrOf(from);
    if (addr.family == 0) return RateVerdict::Answer;
    for (const IpPrefix& p : m_exempt) {
        if (p.contains(addr)) return RateVerdict::Answer;
    }
    const IpAddr client = masked(addr, addr.family == AF_INET ? m_cfg.ipv4Prefix : m_cfg.ipv6Prefix);
    std::string key(17, '\0');
    key[0] = static_cast<char>(client.family == AF_INET ? 4 : 6);
    std::copy(client.b.begin(), client.b.end(), key.begin() + 1);

    std::lock_guard<std::mutex> lk(m_mu);

    const double burst = static_cast<double>(m_cfg.burst);
    // How long an idle bucket takes to fill again: past this, forgetting it
    // loses nothing.
    const double full = burst * m_cfg.intervalSec;

    // Sweep occasionally so the map cannot grow without bound.
    if (now - m_lastSweep > 60.0) {
        const double idle = std::max(60.0, full);
        for (auto it = m_buckets.begin(); it != m_buckets.end();) {
            if (now - it->second.at > idle) it = m_buckets.erase(it);
            else ++it;
        }
        m_lastSweep = now;
    }

    auto it = m_buckets.find(key);
    if (it == m_buckets.end()) {
        // The minute sweep bounds the map for honest traffic, not for a flood
        // of spoofed source addresses, which can add a fresh key every packet.
        // So there is a hard cap: when it is reached, sweep now (anything idle
        // long enough to be full again), and if the map is still full, answer
        // this client WITHOUT tracking it. At most one such sweep a second: under
        // a sustained flood every packet is a new key, and an O(n) walk of the
        // map per packet, under the lock every listener shares, would be a
        // denial of service by itself.
        //
        // Fail-open rather than fail-closed: refusing untracked keys would let
        // the same flood lock out every legitimate client that had not been
        // seen before it began, which turns a memory bound into a denial of
        // service. The cost of fail-open is that a flood of >kMaxBuckets
        // distinct clients is not rate-limited -- but each of those gets one
        // reply the size of its request, gain 1, which is exactly what it
        // would get if it were within its limit anyway.
        if (m_buckets.size() >= kMaxBuckets) {
            if (now - m_lastSweep < 1.0) return RateVerdict::Answer;
            for (auto sit = m_buckets.begin(); sit != m_buckets.end();) {
                if (now - sit->second.at > full) sit = m_buckets.erase(sit);
                else ++sit;
            }
            m_lastSweep = now;
            if (m_buckets.size() >= kMaxBuckets) return RateVerdict::Answer;
        }
        m_buckets.emplace(key, Bucket{burst - 1.0, now, -std::numeric_limits<double>::infinity()});
        return RateVerdict::Answer;
    }

    Bucket& b = it->second;
    b.tokens = std::min(burst, b.tokens + (now - b.at) / m_cfg.intervalSec);
    b.at = now;
    if (b.tokens >= 1.0) {
        b.tokens -= 1.0;
        return RateVerdict::Answer;
    }

    if (m_cfg.leak > 0.0 && std::uniform_real_distribution<double>(0.0, 1.0)(m_rng) < m_cfg.leak) {
        return RateVerdict::Leak;
    }
    // The kiss-o'-death is limited too, to one per client per interval (and
    // never more than one a second), so a client that ignores it is not
    // answered at its own rate with them.
    if (m_cfg.kod && now - b.kodAt >= std::max(1.0, m_cfg.intervalSec)) {
        b.kodAt = now;
        return RateVerdict::Kod;
    }
    return RateVerdict::Drop;
}

} // namespace ubersdr_ntp
