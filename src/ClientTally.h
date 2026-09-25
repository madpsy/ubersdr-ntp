// Who is asking the NTP service the time, over the past hour: requests per
// client, and how many of them were over the rate limit, for the status page's
// top-clients list.
//
// A client is its publicLabel() (IpPrefix.h): a LAN address whole, a public one
// cut to its /24 or /64. The cut is made as the request is counted, so a public
// address is never held here in full.
#pragma once

#include <array>
#include <cstdint>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include <sys/socket.h>

namespace ubersdr_ntp {

struct TopClient {
    std::string client;
    std::uint64_t requests = 0;  // in the past hour
    std::uint64_t limited = 0;   // ...of which over the rate limit
};

class ClientTally {
public:
    // `now` in seconds on any monotonic clock.
    void record(const struct sockaddr_storage& from, double now, bool limited);

    // The `n` busiest clients over the past hour, busiest first.
    std::vector<TopClient> top(std::size_t n, double now) const;

    // A spoofed-source flood adds a client a packet. Past this many, clients
    // quiet for the hour or seen only once are dropped (at most one sweep a
    // second), and if that frees nothing a new client is not counted. ~500
    // bytes an entry, so ~2 MB at the cap.
    static constexpr std::size_t kMaxClients = 4096;

private:
    static constexpr int kMinutes = 60;
    struct Entry {
        std::array<std::uint32_t, kMinutes> requests{};
        std::array<std::uint32_t, kMinutes> limited{};
        long long lastMinute = 0;  // the minute the buckets were last moved to
    };
    static void advance(Entry& e, long long minute);
    static void sum(const Entry& e, long long minute, std::uint64_t& requests, std::uint64_t& limited);

    mutable std::mutex m_mu;
    std::unordered_map<std::string, Entry> m_clients;
    double m_lastSweep = -1e18;
};

} // namespace ubersdr_ntp
