#include "ClientTally.h"

#include "IpPrefix.h"

#include <algorithm>
#include <cmath>

namespace ubersdr_ntp {

namespace {
// Minute k's bucket. Floored: a window reaching back before minute 0 -- in the
// first hour of a monotonic clock that starts at boot -- has negative minutes,
// and C++'s % keeps their sign.
std::size_t slot(long long k, int n) { return static_cast<std::size_t>(((k % n) + n) % n); }
} // namespace

// Moves an entry's buckets on to `minute`, clearing the ones it has skipped:
// bucket k % 60 holds minute k, so a bucket not written since is an hour old.
void ClientTally::advance(Entry& e, long long minute) {
    if (minute <= e.lastMinute) return;
    const long long gap = std::min<long long>(minute - e.lastMinute, kMinutes);
    for (long long k = 1; k <= gap; ++k) {
        const std::size_t i = slot(e.lastMinute + k, kMinutes);
        e.requests[i] = 0;
        e.limited[i] = 0;
    }
    e.lastMinute = minute;
}

// The past hour as seen at `minute`: the buckets from minute - 59 on, which an
// entry last moved before `minute` holds only up to its lastMinute.
void ClientTally::sum(const Entry& e, long long minute, std::uint64_t& requests, std::uint64_t& limited) {
    requests = limited = 0;
    for (long long k = std::max(minute - kMinutes + 1, e.lastMinute - kMinutes + 1);
         k <= std::min(minute, e.lastMinute); ++k) {
        const std::size_t i = slot(k, kMinutes);
        requests += e.requests[i];
        limited += e.limited[i];
    }
}

void ClientTally::record(const struct sockaddr_storage& from, double now, bool limited) {
    const IpAddr a = ipAddrOf(from);
    if (a.family == 0) return;
    const std::string key = publicLabel(a);
    const long long minute = static_cast<long long>(std::floor(now / 60.0));

    std::lock_guard<std::mutex> lk(m_mu);
    auto it = m_clients.find(key);
    if (it == m_clients.end()) {
        if (m_clients.size() >= kMaxClients) {
            if (now - m_lastSweep < 1.0) return;
            m_lastSweep = now;
            for (auto sit = m_clients.begin(); sit != m_clients.end();) {
                std::uint64_t r = 0, l = 0;
                sum(sit->second, minute, r, l);
                if (r <= 1) sit = m_clients.erase(sit);
                else ++sit;
            }
            if (m_clients.size() >= kMaxClients) return;
        }
        Entry e;
        e.lastMinute = minute;
        it = m_clients.emplace(key, e).first;
    }
    Entry& e = it->second;
    advance(e, minute);
    const std::size_t i = slot(minute, kMinutes);
    if (e.requests[i] < UINT32_MAX) ++e.requests[i];
    if (limited && e.limited[i] < UINT32_MAX) ++e.limited[i];
}

std::vector<TopClient> ClientTally::top(std::size_t n, double now) const {
    const long long minute = static_cast<long long>(std::floor(now / 60.0));
    std::vector<TopClient> all;
    {
        std::lock_guard<std::mutex> lk(m_mu);
        all.reserve(m_clients.size());
        for (const auto& [key, e] : m_clients) {
            TopClient t;
            t.client = key;
            sum(e, minute, t.requests, t.limited);
            if (t.requests > 0) all.push_back(std::move(t));
        }
    }
    const std::size_t k = std::min(n, all.size());
    std::partial_sort(all.begin(), all.begin() + static_cast<std::ptrdiff_t>(k), all.end(),
                      [](const TopClient& x, const TopClient& y) {
                          return x.requests != y.requests ? x.requests > y.requests : x.client < y.client;
                      });
    all.resize(k);
    return all;
}

} // namespace ubersdr_ntp
