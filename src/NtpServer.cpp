#include "NtpServer.h"

#include "CivilTime.h"
#include "Log.h"
#include "SampleClock.h"

#include <algorithm>
#include <arpa/inet.h>
#include <cerrno>
#include <cmath>
#include <cstring>
#include <netinet/in.h>
#include <poll.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

namespace ubersdr_ntp {

namespace {

constexpr const char* kTag = "ntp";

// Seconds between 1900-01-01 (the NTP epoch) and 1970-01-01 (the Unix epoch).
constexpr std::uint64_t kNtpUnixDelta = 2208988800ULL;

constexpr int kPacketSize = 48;     // an NTPv4 packet with no extension fields

// An NTP 64-bit timestamp: seconds since 1900 in the high word, binary
// fractions of a second in the low.
//
// Era handling is the truncation to 32 bits, and it is correct rather than
// merely conventional: a client and server that disagree about the era
// disagree by 136 years, which no sane client will accept, and both sides
// wrapping the same way is what makes 2036 a non-event for a server that only
// ever reflects the client's own timestamp back.
struct NtpTime {
    std::uint32_t sec;
    std::uint32_t frac;
};

NtpTime toNtpTime(double unixSec) {
    const double ntp = unixSec + static_cast<double>(kNtpUnixDelta);
    const double whole = std::floor(ntp);
    NtpTime t;
    t.sec = static_cast<std::uint32_t>(static_cast<std::uint64_t>(whole) & 0xFFFFFFFFULL);
    double frac = ntp - whole;
    if (frac < 0.0) frac = 0.0;
    if (frac >= 1.0) frac = 0.9999999999;
    t.frac = static_cast<std::uint32_t>(frac * 4294967296.0);
    return t;
}

void put32(std::uint8_t* p, std::uint32_t v) {
    p[0] = static_cast<std::uint8_t>(v >> 24);
    p[1] = static_cast<std::uint8_t>(v >> 16);
    p[2] = static_cast<std::uint8_t>(v >> 8);
    p[3] = static_cast<std::uint8_t>(v);
}

// A 32-bit short-format value: 16 bits of seconds, 16 of fraction. Used for
// root delay and root dispersion.
std::uint32_t toShortFormat(double seconds) {
    if (seconds < 0.0) seconds = 0.0;
    if (seconds > 65535.0) seconds = 65535.0;
    return static_cast<std::uint32_t>(seconds * 65536.0);
}

// The precision field: the base-2 logarithm of the claimed resolution, as a
// signed 8-bit value.
//
// Deliberately derived from the dispersion rather than from clock_getres. The
// host clock resolves nanoseconds and this server's time is good to some tens
// of milliseconds; reporting -29 because the syscall is precise would be a lie
// that some clients weight their selection on.
std::int8_t precisionFor(double dispersionSec) {
    if (!(dispersionSec > 0.0)) return -20;
    int p = static_cast<int>(std::floor(std::log2(dispersionSec)));
    if (p < -20) p = -20;
    if (p > 0) p = 0;
    return static_cast<std::int8_t>(p);
}

std::string addressText(const struct sockaddr_storage& sa) {
    char host[INET6_ADDRSTRLEN] = {0};
    if (sa.ss_family == AF_INET) {
        const auto* s = reinterpret_cast<const struct sockaddr_in*>(&sa);
        inet_ntop(AF_INET, &s->sin_addr, host, sizeof host);
    } else if (sa.ss_family == AF_INET6) {
        const auto* s = reinterpret_cast<const struct sockaddr_in6*>(&sa);
        inet_ntop(AF_INET6, &s->sin6_addr, host, sizeof host);
    } else {
        return "?";
    }
    return host;
}

} // namespace

NtpServer::NtpServer(NtpConfig cfg, Selector& selector)
    : m_cfg(std::move(cfg)), m_selector(selector) {}

NtpServer::~NtpServer() { stop(); }

bool NtpServer::start(std::string& err) {
    int bound = 0;
    std::string lastErr;

    for (const std::string& addr : m_cfg.listen) {
        const bool v6 = addr.find(':') != std::string::npos;
        const int family = v6 ? AF_INET6 : AF_INET;

        const int fd = ::socket(family, SOCK_DGRAM, 0);
        if (fd < 0) { lastErr = std::string("socket: ") + std::strerror(errno); continue; }

        int one = 1;
        ::setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof one);
        if (v6) {
            // v6-only, so binding both "0.0.0.0" and "::" does not collide on
            // a system where the v6 wildcard would otherwise cover v4 too.
            ::setsockopt(fd, IPPROTO_IPV6, IPV6_V6ONLY, &one, sizeof one);
        }
        // Kernel receive timestamps. The gap between a packet arriving and this
        // process reading it is real delay that the client would otherwise see
        // as our asymmetry; asking the kernel removes it. Best-effort: a
        // platform without it falls back to reading the clock on wake-up.
#ifdef SO_TIMESTAMPNS
        ::setsockopt(fd, SOL_SOCKET, SO_TIMESTAMPNS, &one, sizeof one);
#endif

        struct sockaddr_storage sa{};
        socklen_t salen = 0;
        if (v6) {
            auto* s = reinterpret_cast<struct sockaddr_in6*>(&sa);
            s->sin6_family = AF_INET6;
            s->sin6_port = htons(static_cast<std::uint16_t>(m_cfg.port));
            if (inet_pton(AF_INET6, addr.c_str(), &s->sin6_addr) != 1) {
                lastErr = "not a valid IPv6 address: " + addr;
                ::close(fd);
                continue;
            }
            salen = sizeof(struct sockaddr_in6);
        } else {
            auto* s = reinterpret_cast<struct sockaddr_in*>(&sa);
            s->sin_family = AF_INET;
            s->sin_port = htons(static_cast<std::uint16_t>(m_cfg.port));
            if (inet_pton(AF_INET, addr.c_str(), &s->sin_addr) != 1) {
                lastErr = "not a valid IPv4 address: " + addr;
                ::close(fd);
                continue;
            }
            salen = sizeof(struct sockaddr_in);
        }

        if (::bind(fd, reinterpret_cast<struct sockaddr*>(&sa), salen) != 0) {
            lastErr = "bind " + addr + ":" + std::to_string(m_cfg.port) + ": " + std::strerror(errno);
            if (errno == EACCES && m_cfg.port < 1024) {
                lastErr += " — port " + std::to_string(m_cfg.port) +
                           " is privileged; run as root, grant CAP_NET_BIND_SERVICE "
                           "(setcap 'cap_net_bind_service=+ep' on the binary, or "
                           "AmbientCapabilities= in the unit), or set ntp.port to "
                           "something above 1023";
            }
            ::close(fd);
            continue;
        }

        m_fds.push_back(Listener{fd, addr});
        ++bound;
        LOG_INFO(kTag, "listening on %s:%d", addr.c_str(), m_cfg.port);
    }

    if (bound == 0) {
        err = lastErr.empty() ? "no listen addresses configured" : lastErr;
        return false;
    }

    m_running.store(true);
    for (const Listener& l : m_fds) {
        const int fd = l.fd;
        const std::string label = l.label;
        m_threads.emplace_back([this, fd, label] { serve(fd, label); });
    }
    return true;
}

void NtpServer::stop() {
    if (!m_running.exchange(false)) return;
    // Shutdown, then close: a thread blocked in recvmsg on a socket that is
    // merely closed can sit there indefinitely, and on some kernels the fd
    // number is reused underneath it.
    for (const Listener& l : m_fds) ::shutdown(l.fd, SHUT_RDWR);
    for (std::thread& t : m_threads) if (t.joinable()) t.join();
    for (const Listener& l : m_fds) ::close(l.fd);
    m_fds.clear();
    m_threads.clear();
}

bool NtpServer::rateLimitAllows(const std::string& key, double now) {
    if (m_cfg.rateLimitPerClient <= 0.0) return true;

    std::lock_guard<std::mutex> lk(m_mu);

    // Sweep occasionally so the map cannot grow without bound. Anything that
    // has not been seen for a minute has a full bucket anyway, so dropping it
    // loses nothing.
    if (now - m_lastSweep > 60.0) {
        for (auto it = m_buckets.begin(); it != m_buckets.end();) {
            if (now - it->second.at > 60.0) it = m_buckets.erase(it);
            else ++it;
        }
        m_lastSweep = now;
    }

    const double burst = std::max(1.0, m_cfg.rateLimitPerClient);
    auto it = m_buckets.find(key);
    if (it == m_buckets.end()) {
        m_buckets.emplace(key, Bucket{burst - 1.0, now});
        return true;
    }
    Bucket& b = it->second;
    b.tokens = std::min(burst, b.tokens + (now - b.at) * m_cfg.rateLimitPerClient);
    b.at = now;
    if (b.tokens < 1.0) return false;
    b.tokens -= 1.0;
    return true;
}

void NtpServer::serve(int fd, const std::string& label) {
    std::uint8_t buf[1024];

    while (m_running.load()) {
        // Poll before recvmsg so stop() is noticed promptly even on a platform
        // where shutdown() does not wake a blocked receive.
        struct pollfd pfd{fd, POLLIN, 0};
        const int pr = ::poll(&pfd, 1, 500);
        if (pr <= 0) {
            if (pr < 0 && errno != EINTR) {
                LOG_WARN(kTag, "%s: poll: %s", label.c_str(), std::strerror(errno));
            }
            continue;
        }

        struct sockaddr_storage from{};
        struct iovec iov{buf, sizeof buf};
        // Big enough for the SCM_TIMESTAMPNS control message with room to spare.
        alignas(struct cmsghdr) char control[256];
        struct msghdr mh{};
        mh.msg_name = &from;
        mh.msg_namelen = sizeof from;
        mh.msg_iov = &iov;
        mh.msg_iovlen = 1;
        mh.msg_control = control;
        mh.msg_controllen = sizeof control;

        const ssize_t n = ::recvmsg(fd, &mh, 0);
        if (n < 0) {
            if (errno == EINTR || errno == EAGAIN) continue;
            if (m_running.load()) LOG_WARN(kTag, "%s: recvmsg: %s", label.c_str(), std::strerror(errno));
            continue;
        }

        // The moment the packet actually arrived, from the kernel if it said.
        double recvRealtime = realtimeNow();
#ifdef SO_TIMESTAMPNS
        for (struct cmsghdr* cm = CMSG_FIRSTHDR(&mh); cm; cm = CMSG_NXTHDR(&mh, cm)) {
            if (cm->cmsg_level == SOL_SOCKET && cm->cmsg_type == SO_TIMESTAMPNS) {
                struct timespec ts;
                std::memcpy(&ts, CMSG_DATA(cm), sizeof ts);
                recvRealtime = static_cast<double>(ts.tv_sec) + static_cast<double>(ts.tv_nsec) * 1e-9;
                break;
            }
        }
#endif

        {
            std::lock_guard<std::mutex> lk(m_mu);
            m_stats.requests++;
        }

        if (n < kPacketSize) {
            std::lock_guard<std::mutex> lk(m_mu);
            m_stats.ignored++;
            continue;
        }

        const std::uint8_t li_vn_mode = buf[0];
        const int version = (li_vn_mode >> 3) & 0x07;
        const int mode = li_vn_mode & 0x07;

        // Mode 3 is a client asking the time. Mode 1 (symmetric active) is a
        // peer offering to exchange it, which is answered because it costs
        // nothing and some clients use it. Everything else — and in particular
        // modes 6 and 7, the control and private modes that every ntpd
        // amplification advisory is about — is dropped without a reply.
        if (mode != 3 && mode != 1) {
            std::lock_guard<std::mutex> lk(m_mu);
            m_stats.ignored++;
            continue;
        }
        if (version < 1 || version > 4) {
            std::lock_guard<std::mutex> lk(m_mu);
            m_stats.ignored++;
            continue;
        }

        const std::string peer = addressText(from);
        if (!rateLimitAllows(peer, monotonicNow())) {
            std::lock_guard<std::mutex> lk(m_mu);
            m_stats.rateLimited++;
            continue;
        }

        const Combined c = m_selector.current();

        // Not `!c.valid`: a source that locked hours ago and has since gone
        // leaves a combined result that is still VALID (there is a last known
        // offset) but no longer SYNCHRONISED (it is past the coast limit). The
        // earlier test let that case through, so a server configured not to
        // answer while unsynchronised answered anyway with stratum 16 — which
        // is the correct content, but not what was asked for.
        if (!c.synchronised && !m_cfg.answerWhenUnsynchronised) {
            std::lock_guard<std::mutex> lk(m_mu);
            m_stats.ignored++;
            continue;
        }

        // --- build the reply ------------------------------------------------
        std::uint8_t out[kPacketSize];
        std::memset(out, 0, sizeof out);

        int leap = 0;
        int stratum = 1;
        if (!c.synchronised) {
            leap = 3;       // unsynchronised
            stratum = 16;   // and therefore unusable, which is the honest answer
        } else if (c.leapPending && m_cfg.honourLeapWarning) {
            // WWV asserts its warning bit for the whole month leading up to a
            // leap second; NTP's leap indicator means "in the last minute of
            // today". Narrowing it to the last day of the month is what keeps
            // the two from meaning different things. The broadcast carries no
            // sign, and every leap second since 1972 has been an insertion, so
            // an insertion is what is announced.
            const double utc = recvRealtime + c.offsetSec;
            if (isLastDayOfMonth(static_cast<long long>(utc * 1000.0))) leap = 1;
        }

        // Reply version matches the request: a v3 client handed a v4 reply is
        // within its rights to discard it.
        out[0] = static_cast<std::uint8_t>((leap << 6) | (version << 3) | 4);  // mode 4, server
        out[1] = static_cast<std::uint8_t>(stratum);
        // Poll interval: echo what the client asked for, clamped to the range
        // RFC 5905 defines. A server has no opinion about how often to be asked.
        {
            std::int8_t poll = static_cast<std::int8_t>(buf[2]);
            if (poll < 4) poll = 4;
            if (poll > 17) poll = 17;
            out[2] = static_cast<std::uint8_t>(poll);
        }
        out[3] = static_cast<std::uint8_t>(precisionFor(c.dispersionSec));

        // Root delay is zero and means it: there is no NTP path above this
        // server. The entire error budget is in root dispersion.
        put32(out + 4, 0);
        put32(out + 8, toShortFormat(c.dispersionSec));

        // Reference identifier: four ASCII characters naming the radio source,
        // as RFC 5905 specifies for stratum 1.
        std::string refid = c.refid.empty() ? "WWV" : c.refid;
        refid.resize(4, '\0');
        std::memcpy(out + 12, refid.data(), 4);

        // Reference timestamp: when this server's clock was last set from the
        // radio, which is the age of the newest contributing measurement.
        const double corrected = recvRealtime + c.offsetSec;
        const NtpTime ref = toNtpTime(corrected - c.ageSec);
        put32(out + 16, ref.sec);
        put32(out + 20, ref.frac);

        // Originate: the client's transmit timestamp, echoed back verbatim.
        // Verbatim matters — it is how the client matches the reply to its
        // request, and reformatting it through a double would corrupt the low
        // bits some clients use as a nonce.
        std::memcpy(out + 24, buf + 40, 8);

        // Receive and transmit, both on the corrected clock.
        const NtpTime rec = toNtpTime(corrected);
        put32(out + 32, rec.sec);
        put32(out + 36, rec.frac);

        const NtpTime xmt = toNtpTime(realtimeNow() + c.offsetSec);
        put32(out + 40, xmt.sec);
        put32(out + 44, xmt.frac);

        const ssize_t sent = ::sendto(fd, out, sizeof out, 0,
                                      reinterpret_cast<struct sockaddr*>(&from), mh.msg_namelen);
        std::lock_guard<std::mutex> lk(m_mu);
        if (sent < 0) {
            m_stats.sendErrors++;
            // Not logged per packet: an unreachable client is the client's
            // problem and a noisy one would fill the log with it.
            if (m_stats.sendErrors % 1000 == 1) {
                LOG_WARN(kTag, "sendto %s: %s", peer.c_str(), std::strerror(errno));
            }
        } else {
            m_stats.answered++;
            if (!c.synchronised) m_stats.unsynchronised++;
        }
    }
}

NtpStats NtpServer::stats() const {
    std::lock_guard<std::mutex> lk(m_mu);
    return m_stats;
}

} // namespace ubersdr_ntp
