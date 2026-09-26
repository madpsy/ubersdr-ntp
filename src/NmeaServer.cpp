#include "NmeaServer.h"

#include "Log.h"
#include "SampleClock.h"

#include <algorithm>
#include <arpa/inet.h>
#include <cerrno>
#include <cmath>
#include <cstring>
#include <ctime>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <poll.h>
#include <pthread.h>
#include <sched.h>
#include <sys/prctl.h>
#include <sys/socket.h>
#include <unistd.h>

namespace ubersdr_ntp {

namespace {

constexpr const char* kTag = "nmea";

// A client's send buffer, kept small: a second's sentences are under 200
// bytes, so this is a minute or so of them, and a client that has not read
// that much is not reading. The kernel doubles it and applies its own floor.
constexpr int kSendBuffer = 8192;

// How long before the second the thread stops waiting on the sockets and
// spins: a wake-up from sleep comes hundreds of microseconds late on an idle
// machine whose CPU has dropped into a deep power state, and the spin takes
// that out. As Pps.cpp, and for the same reason.
constexpr double kSpinSec = 0.001;

std::string peerText(const struct sockaddr_storage& sa) {
    char host[INET6_ADDRSTRLEN] = {0};
    if (sa.ss_family == AF_INET) {
        const auto* s = reinterpret_cast<const struct sockaddr_in*>(&sa);
        inet_ntop(AF_INET, &s->sin_addr, host, sizeof host);
        return std::string(host) + ":" + std::to_string(ntohs(s->sin_port));
    }
    if (sa.ss_family == AF_INET6) {
        const auto* s = reinterpret_cast<const struct sockaddr_in6*>(&sa);
        inet_ntop(AF_INET6, &s->sin6_addr, host, sizeof host);
        return "[" + std::string(host) + "]:" + std::to_string(ntohs(s->sin6_port));
    }
    return "?";
}

} // namespace

NmeaServer::NmeaServer(NmeaTcpConfig cfg, ClockFn clock, PositionFn position, int realtimePriority)
    : m_cfg(std::move(cfg)), m_clock(std::move(clock)), m_position(std::move(position)),
      m_rtPriority(realtimePriority) {
    m_stats.enabled = m_cfg.enabled;
    m_stats.port = m_cfg.port;
    m_stats.sentences = m_cfg.sentences;
    m_stats.maxClients = m_cfg.maxClients;
}

NmeaServer::~NmeaServer() { stop(); }

bool NmeaServer::start(std::string& err) {
    if (!m_cfg.enabled || m_running.load()) return true;
    std::string lastErr;
    int port = m_cfg.port;
    std::vector<std::string> bound;

    for (const std::string& addr : m_cfg.listen) {
        const bool v6 = addr.find(':') != std::string::npos;
        const int fd = ::socket(v6 ? AF_INET6 : AF_INET, SOCK_STREAM | SOCK_NONBLOCK | SOCK_CLOEXEC, 0);
        if (fd < 0) { lastErr = std::string("socket: ") + std::strerror(errno); continue; }
        int one = 1;
        ::setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof one);
        // v6-only, so "0.0.0.0" and "::" can both be bound, as for NTP.
        if (v6) ::setsockopt(fd, IPPROTO_IPV6, IPV6_V6ONLY, &one, sizeof one);

        struct sockaddr_storage sa{};
        socklen_t salen = 0;
        bool parsed = false;
        if (v6) {
            auto* s = reinterpret_cast<struct sockaddr_in6*>(&sa);
            s->sin6_family = AF_INET6;
            s->sin6_port = htons(static_cast<std::uint16_t>(port));
            parsed = inet_pton(AF_INET6, addr.c_str(), &s->sin6_addr) == 1;
            salen = sizeof(struct sockaddr_in6);
        } else {
            auto* s = reinterpret_cast<struct sockaddr_in*>(&sa);
            s->sin_family = AF_INET;
            s->sin_port = htons(static_cast<std::uint16_t>(port));
            parsed = inet_pton(AF_INET, addr.c_str(), &s->sin_addr) == 1;
            salen = sizeof(struct sockaddr_in);
        }
        if (!parsed) {
            lastErr = "nmea_tcp.listen: not a valid address: " + addr;
            ::close(fd);
            continue;
        }
        if (::bind(fd, reinterpret_cast<struct sockaddr*>(&sa), salen) != 0 || ::listen(fd, 8) != 0) {
            lastErr = "NMEA bind " + addr + ":" + std::to_string(port) + ": " + std::strerror(errno);
            ::close(fd);
            continue;
        }
        // Port 0 is the kernel's choice (the tests); the other addresses then
        // take the same one.
        if (port == 0) {
            struct sockaddr_storage got{};
            socklen_t len = sizeof got;
            if (::getsockname(fd, reinterpret_cast<struct sockaddr*>(&got), &len) == 0) {
                port = got.ss_family == AF_INET6
                           ? ntohs(reinterpret_cast<struct sockaddr_in6*>(&got)->sin6_port)
                           : ntohs(reinterpret_cast<struct sockaddr_in*>(&got)->sin_port);
            }
        }
        m_listenFds.push_back(fd);
        bound.push_back(addr);
        LOG_INFO(kTag, "NMEA 0183 over TCP on %s:%d", addr.c_str(), port);
    }

    if (m_listenFds.empty()) {
        err = lastErr.empty() ? "nmea_tcp.listen is empty" : lastErr;
        std::lock_guard<std::mutex> lk(m_mu);
        m_stats.state = "error";
        m_stats.detail = err;
        return false;
    }
    m_boundPort = port;
    {
        std::lock_guard<std::mutex> lk(m_mu);
        m_stats.port = port;
        m_stats.listen = bound;
        m_stats.state = "waiting";
        m_stats.detail = "starting";
    }
    m_running.store(true);
    m_thread = std::thread([this] { run(); });
    return true;
}

void NmeaServer::stop() {
    if (!m_running.exchange(false)) return;
    if (m_thread.joinable()) m_thread.join();
    for (const Client& c : m_clients) ::close(c.fd);
    m_clients.clear();
    for (int fd : m_listenFds) ::close(fd);
    m_listenFds.clear();
}

NmeaTcpStats NmeaServer::stats() const {
    std::lock_guard<std::mutex> lk(m_mu);
    NmeaTcpStats out = m_stats;
    const double now = monotonicNow();
    for (NmeaClientInfo& c : out.clients) c.connectedSec = now - c.connectedSec;
    return out;
}

void NmeaServer::acceptFrom(int listenFd) {
    for (;;) {
        struct sockaddr_storage from{};
        socklen_t len = sizeof from;
        const int fd = ::accept4(listenFd, reinterpret_cast<struct sockaddr*>(&from), &len,
                                 SOCK_NONBLOCK | SOCK_CLOEXEC);
        if (fd < 0) {
            if (errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR)
                LOG_DEBUG(kTag, "accept: %s", std::strerror(errno));
            return;
        }
        const std::string who = peerText(from);
        if (static_cast<int>(m_clients.size()) >= m_cfg.maxClients) {
            ::close(fd);
            std::lock_guard<std::mutex> lk(m_mu);
            ++m_stats.refused;
            if (m_stats.refused % 100 == 1)
                LOG_WARN(kTag, "refused %s: already %d clients (nmea_tcp.max_clients)", who.c_str(), m_cfg.maxClients);
            continue;
        }
        int one = 1;
        ::setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof one);
        const int sndbuf = kSendBuffer;
        ::setsockopt(fd, SOL_SOCKET, SO_SNDBUF, &sndbuf, sizeof sndbuf);
        // Ahead of bulk traffic in the host's queues, as the NTP replies.
        const int prio = 6;
        ::setsockopt(fd, SOL_SOCKET, SO_PRIORITY, &prio, sizeof prio);
        const double since = monotonicNow();
        m_clients.push_back(Client{fd, who, since});
        LOG_DEBUG(kTag, "client %s connected", who.c_str());
        std::lock_guard<std::mutex> lk(m_mu);
        ++m_stats.connections;
        m_stats.clients.push_back(NmeaClientInfo{who, since});
    }
}

void NmeaServer::closeClient(std::size_t i) {
    LOG_DEBUG(kTag, "client %s gone", m_clients[i].address.c_str());
    ::close(m_clients[i].fd);
    m_clients.erase(m_clients.begin() + static_cast<std::ptrdiff_t>(i));
    std::lock_guard<std::mutex> lk(m_mu);
    m_stats.clients.erase(m_stats.clients.begin() + static_cast<std::ptrdiff_t>(i));
}

void NmeaServer::run() {
    if (m_rtPriority > 0) {
        struct sched_param sp{};
        sp.sched_priority = m_rtPriority;
        const int rc = ::pthread_setschedparam(::pthread_self(), SCHED_FIFO, &sp);
        if (rc != 0) {
            LOG_WARN(kTag, "cannot send at real-time priority %d (%s): the sentences go out at normal "
                     "priority, later and less evenly under load", m_rtPriority, std::strerror(rc));
        }
    }
    // The wake-up is the timestamp, so no slack in it: a normal thread's
    // default of 50 us would be the spin's to make up every second.
    ::prctl(PR_SET_TIMERSLACK, 1UL, 0UL, 0UL, 0UL);

    double lastSlot = -1e9;   // daemon time of the last second sent
    PpsLabeler labels;
    const bool wantRmc = std::find(m_cfg.sentences.begin(), m_cfg.sentences.end(), "rmc") != m_cfg.sentences.end();

    while (m_running.load()) {
        Combined c = m_clock();
        const double now = daemonNow();

        // The next second: of the served clock when there is one, of this
        // host's clock otherwise (then only for an RMC saying V). As Pps.cpp.
        long long second = 0;
        double at = 0.0;
        if (c.valid) {
            second = static_cast<long long>(std::floor(c.utcAt(std::max(now, lastSlot + 0.5)))) + 1;
            at = daemonAtUtc(c, static_cast<double>(second));
        } else {
            const double host = now - daemonMinusRealtime();
            at = now + (std::floor(host) + 1.0 - host);
        }

        // Serve the sockets until then. The clock is read again each time
        // round, so an estimate that moves while waiting moves the instant too.
        for (;;) {
            if (!m_running.load()) return;
            if (c.valid) {
                const Combined c2 = m_clock();
                if (c2.valid) { c = c2; at = daemonAtUtc(c, static_cast<double>(second)); }
            }
            const double left = std::min(at - kSpinSec - daemonNow(), 0.2);   // prompt to stop()
            if (left <= 0.0) break;

            std::vector<struct pollfd> pfds;
            for (int fd : m_listenFds) pfds.push_back({fd, POLLIN, 0});
            for (const Client& cl : m_clients) pfds.push_back({cl.fd, POLLIN, 0});
            struct timespec ts{};
            ts.tv_sec = static_cast<time_t>(left);
            ts.tv_nsec = static_cast<long>((left - static_cast<double>(ts.tv_sec)) * 1e9);
            const int pr = ::ppoll(pfds.data(), pfds.size(), &ts, nullptr);
            if (pr <= 0) continue;

            const std::size_t nl = m_listenFds.size();
            // Clients first, from the back, so closing one does not move the
            // ones still to be looked at; then the listeners, which append.
            for (std::size_t k = pfds.size(); k-- > nl;) {
                if (!pfds[k].revents) continue;
                const std::size_t i = k - nl;
                char buf[1024];
                bool gone = (pfds[k].revents & (POLLERR | POLLHUP | POLLNVAL)) != 0;
                while (!gone) {
                    const ssize_t n = ::recv(m_clients[i].fd, buf, sizeof buf, 0);
                    if (n > 0) continue;   // whatever it says is ignored
                    if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) break;
                    if (n < 0 && errno == EINTR) continue;
                    gone = true;
                }
                if (gone) closeClient(i);
            }
            for (std::size_t k = 0; k < nl; ++k)
                if (pfds[k].revents & POLLIN) acceptFrom(m_listenFds[k]);
        }

        const bool sync = c.valid && c.synchronised;
        std::optional<PpsLabel> label;
        if (c.valid) {
            label = labels.next(second, c.leapPending, std::fabs(at - lastSlot - 1.0) < 0.1);
        } else {
            labels.reset();
        }
        lastSlot = at;

        PpsPosition pos;
        std::string from;
        if (wantRmc) pos = m_position(from);
        std::string out;
        for (const std::string& n : m_cfg.sentences) {
            if (n == "rmc") out += nmeaRmc(label, sync, pos);
            else if (n == "zda" && sync && label) out += nmeaZda(*label);
        }

        // Built first, then the spin: the lookups above are not the client's delay.
        while (m_running.load() && daemonNow() < at) {}
        if (!m_running.load()) return;

        double late = -1.0;
        std::uint64_t delivered = 0, dropped = 0;
        if (!out.empty()) {
            const std::uint64_t lines = static_cast<std::uint64_t>(std::count(out.begin(), out.end(), '$'));
            for (std::size_t i = m_clients.size(); i-- > 0;) {
                if (late < 0.0) late = daemonNow() - at;
                const ssize_t n = ::send(m_clients[i].fd, out.data(), out.size(), MSG_NOSIGNAL | MSG_DONTWAIT);
                if (n == static_cast<ssize_t>(out.size())) {
                    delivered += lines;
                    continue;
                }
                // A partial send would leave half a sentence for the next
                // second's to follow: better closed than garbled.
                if (n >= 0 || errno == EAGAIN || errno == EWOULDBLOCK) {
                    LOG_INFO(kTag, "dropped %s: not reading", m_clients[i].address.c_str());
                    ++dropped;
                }
                closeClient(i);
            }
        }

        std::lock_guard<std::mutex> lk(m_mu);
        m_stats.sent += delivered;
        m_stats.dropped += dropped;
        m_stats.positionFrom = pos.valid ? from : std::string();
        if (sync) {
            m_stats.state = "serving";
            m_stats.detail.clear();
        } else {
            m_stats.state = "waiting";
            m_stats.detail = c.note.empty() ? "the daemon is not synchronised" : "not synchronised: " + c.note;
        }
        if (late >= 0.0) {
            m_late[m_lateNext] = late * 1e6;
            m_lateNext = (m_lateNext + 1) % m_late.size();
            m_lateN = std::min(m_lateN + 1, m_late.size());
            double sum = 0.0, mx = 0.0;
            for (std::size_t i = 0; i < m_lateN; ++i) { sum += m_late[i]; mx = std::max(mx, m_late[i]); }
            m_stats.lastLateUs = late * 1e6;
            m_stats.meanLateUs = sum / static_cast<double>(m_lateN);
            m_stats.maxLateUs = mx;
        }
    }
}

} // namespace ubersdr_ntp
