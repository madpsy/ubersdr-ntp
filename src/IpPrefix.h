// An IPv4 or IPv6 address prefix: "10.0.0.0/8", "2001:db8::/32", or a bare
// address, which is the whole address. For ntp.rate_limit.exempt, and for
// grouping a client's address into the prefix that counts as one client.
#pragma once

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>

#include <array>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <string>

namespace ubersdr_ntp {

// An address as 16 bytes and a family, IPv4 in the first four. An IPv4-mapped
// IPv6 address (::ffff:a.b.c.d) is taken as the IPv4 address it carries, so a
// client counts as the same client on either socket.
struct IpAddr {
    int family = 0;  // AF_INET or AF_INET6; 0 for anything else
    std::array<std::uint8_t, 16> b{};
    int bits() const { return family == AF_INET ? 32 : 128; }
};

inline IpAddr ipAddrOf(const struct sockaddr_storage& sa) {
    IpAddr a;
    if (sa.ss_family == AF_INET) {
        const auto* s = reinterpret_cast<const struct sockaddr_in*>(&sa);
        a.family = AF_INET;
        std::memcpy(a.b.data(), &s->sin_addr, 4);
    } else if (sa.ss_family == AF_INET6) {
        const auto* s = reinterpret_cast<const struct sockaddr_in6*>(&sa);
        if (IN6_IS_ADDR_V4MAPPED(&s->sin6_addr)) {
            a.family = AF_INET;
            std::memcpy(a.b.data(), reinterpret_cast<const std::uint8_t*>(&s->sin6_addr) + 12, 4);
        } else {
            a.family = AF_INET6;
            std::memcpy(a.b.data(), &s->sin6_addr, 16);
        }
    }
    return a;
}

// The address with everything past `len` bits cleared.
inline IpAddr masked(IpAddr a, int len) {
    for (int i = 0; i < 16; ++i) {
        const int keep = len - 8 * i;
        if (keep >= 8) continue;
        a.b[static_cast<std::size_t>(i)] &= keep <= 0 ? 0 : static_cast<std::uint8_t>(0xff << (8 - keep));
    }
    return a;
}

struct IpPrefix {
    IpAddr net;
    int len = 0;

    bool contains(const IpAddr& a) const {
        return a.family == net.family && masked(a, len).b == net.b;
    }

    // False, with nothing changed, for anything that is not an address or an
    // address/length with the length in range for its family.
    static bool parse(const std::string& text, IpPrefix& out) {
        const std::size_t slash = text.find('/');
        const std::string host = text.substr(0, slash);
        IpPrefix p;
        if (::inet_pton(AF_INET, host.c_str(), p.net.b.data()) == 1) {
            p.net.family = AF_INET;
        } else if (::inet_pton(AF_INET6, host.c_str(), p.net.b.data()) == 1) {
            p.net.family = AF_INET6;
        } else {
            return false;
        }
        p.len = p.net.bits();
        if (slash != std::string::npos) {
            const std::string n = text.substr(slash + 1);
            if (n.empty() || n.size() > 3 || n.find_first_not_of("0123456789") != std::string::npos) return false;
            p.len = std::atoi(n.c_str());
            if (p.len > p.net.bits()) return false;
        }
        p.net = masked(p.net, p.len);
        out = p;
        return true;
    }
};

} // namespace ubersdr_ntp
