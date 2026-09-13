#include "HttpApi.h"

#include "CivilTime.h"
#include "Log.h"
#include "SampleClock.h"

#include "../third_party/json.hpp"

#include <algorithm>
#include <arpa/inet.h>
#include <cerrno>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <poll.h>
#include <sstream>
#include <thread>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>

namespace ubersdr_ntp {

namespace {

constexpr const char* kTag = "http";

// A connection cap, because this accepts from the network and every connection
// costs a thread. Generous for a status service; small enough that a stuck
// client cannot exhaust the process.
constexpr int kMaxConnections = 64;

// Of those, how many may be SSE streams. A stream holds its slot for as long as
// the client cares to listen, so without a separate, smaller cap 64 open pages
// (or one script opening 64 streams) would leave nothing for /api/health --
// which is the one request a monitoring system makes, and it would read the
// 503 as the daemon being down. Requests other than streams are short, so the
// remaining slots are effectively always available to them.
constexpr int kMaxStreams = 16;

// Send and receive timeout on every accepted socket. The sockets are blocking,
// and a client that stops reading would otherwise pin its thread in send()
// forever: a stuck stream never notices, and stop() is left waiting on it.
constexpr int kSocketTimeoutSec = 5;

// How often the event stream sends the full status document alongside the
// one-second ticks. The ticks carry everything the summary needs; this is for
// the per-source detail, which does not change at 1 Hz and is twenty times the
// size.
constexpr double kFullStatusEverySec = 5.0;

// Read the whole request head, or give up. A status service has no use for a
// client that takes longer than this to send one line, and a socket held open
// sending nothing is the cheapest denial there is.
constexpr int kRequestTimeoutMs = 5000;

#include "IndexHtml.inc"

// Replace, not the strict default: strings in these documents include bytes
// that came off the network unvalidated (a source's link detail is
// IXWebSocket's raw HTTP status reason), nlohmann throws type_error.316 on
// invalid UTF-8, and an exception out of a detached connection thread is
// std::terminate. U+FFFD in one field is the right outcome.
std::string dumpJson(const nlohmann::json& j, bool pretty) {
    return j.dump(pretty ? 2 : -1, ' ', false, nlohmann::json::error_handler_t::replace);
}

bool writeAll(int fd, const char* data, std::size_t len) {
    std::size_t sent = 0;
    while (sent < len) {
        const ssize_t n = ::send(fd, data + sent, len - sent, MSG_NOSIGNAL);
        if (n > 0) { sent += static_cast<std::size_t>(n); continue; }
        if (n < 0 && (errno == EINTR)) continue;
        // EAGAIN on these blocking sockets means SO_SNDTIMEO expired: the
        // client has not taken a byte in kSocketTimeoutSec. Give up on it
        // rather than waiting again, or a reader that trickles one byte every
        // few seconds keeps a thread indefinitely.
        return false;
    }
    return true;
}

bool writeAll(int fd, const std::string& s) { return writeAll(fd, s.data(), s.size()); }

std::string httpResponse(int code, const char* description, const char* contentType,
                         const std::string& body, bool headOnly,
                         const char* extraHeader = nullptr) {
    std::ostringstream o;
    o << "HTTP/1.1 " << code << ' ' << description << "\r\n"
      << "Content-Type: " << contentType << "\r\n"
      << "Content-Length: " << body.size() << "\r\n"
      // Nothing here is ever worth caching: every response is a measurement.
      << "Cache-Control: no-store\r\n"
      // A browser-based client on another origin — a dashboard, a page somebody
      // wrote — should be able to read this. It is public, read-only measurement
      // data with no credentials attached, so a permissive policy leaks nothing.
      << "Access-Control-Allow-Origin: *\r\n"
      << "Connection: close\r\n";
    if (extraHeader) o << extraHeader << "\r\n";
    o << "\r\n";
    if (!headOnly) o << body;
    return o.str();
}

void splitUri(const std::string& uri, std::string& path, std::string& query) {
    const std::size_t q = uri.find('?');
    if (q == std::string::npos) { path = uri; query.clear(); return; }
    path = uri.substr(0, q);
    query = uri.substr(q + 1);
}

std::string queryValue(const std::string& query, const std::string& key) {
    std::size_t pos = 0;
    while (pos < query.size()) {
        std::size_t amp = query.find('&', pos);
        if (amp == std::string::npos) amp = query.size();
        const std::string pair = query.substr(pos, amp - pos);
        const std::size_t eq = pair.find('=');
        if (eq != std::string::npos && pair.compare(0, eq, key) == 0) return pair.substr(eq + 1);
        pos = amp + 1;
    }
    return {};
}

bool truthy(const std::string& v) { return !v.empty() && v != "0" && v != "false" && v != "no"; }

// The compact per-second payload: the corrected time, what is being served,
// and one line per source. Everything a live display or the summary half of the
// page needs, at a size that is reasonable to send every second.
std::string tickJson(const Combined& c, const StatusInput& in, double nowRealtime) {
    nlohmann::json j;
    const double utc = nowRealtime + c.offsetSec;
    const long long utcMs = static_cast<long long>(std::llround(utc * 1000.0));

    j["unix"] = utc;
    j["unix_ms"] = utcMs;
    j["utc"] = iso8601(utcMs);
    j["synchronised"] = c.synchronised;
    j["stratum"] = c.synchronised ? 1 : 16;
    j["refid"] = c.refid;
    j["offset_ms"] = c.offsetSec * 1000.0;
    j["dispersion_ms"] = c.dispersionSec * 1000.0;
    j["reference_age_seconds"] = c.ageSec;
    j["sources_used"] = c.used;
    j["sources_candidate"] = c.candidates;
    j["leap_pending"] = c.leapPending;
    if (!c.note.empty()) j["note"] = c.note;
    j["used_names"] = c.usedNames;
    j["uptime_seconds"] = in.uptimeSec;
    j["version"] = in.version;

    nlohmann::json n;
    n["port"] = in.ntpPort;
    n["requests"] = in.ntp.requests;
    n["answered"] = in.ntp.answered;
    n["ignored"] = in.ntp.ignored;
    n["rate_limited"] = in.ntp.rateLimited;
    j["ntp"] = std::move(n);

    nlohmann::json arr = nlohmann::json::array();
    int locked = 0;
    for (const SourceSnapshot& s : in.sources) {
        if (s.clockState == "locked") ++locked;
        nlohmann::json o;
        o["name"] = s.name;
        o["link"] = linkStateName(s.link);
        o["state"] = s.clockState;
        o["station"] = s.station;
        o["tone_snr_db"] = s.toneSnrDb;
        o["tone_detected"] = s.toneDetected;
        // NaN has no JSON spelling; null means there is no tick to compare.
        o["tick_band_ratio_db"] = std::isfinite(s.tickBandRatioDb)
                                      ? nlohmann::json(s.tickBandRatioDb) : nlohmann::json(nullptr);
        o["phase_locked"] = s.phaseLocked;
        o["anchored"] = s.anchored;
        o["frames_in_window"] = s.framesInWindow;
        o["window_size"] = s.windowSize;
        o["refusal"] = s.refusal;
        o["last_quality"] = s.lastQuality;
        o["have_offset"] = s.haveOffset;
        o["offset_ms"] = s.offsetSec * 1000.0;
        o["dispersion_ms"] = s.dispersionSec * 1000.0;
        o["samples"] = s.offsetSamples;
        o["in_use"] = std::find(c.usedNames.begin(), c.usedNames.end(), s.name) != c.usedNames.end();
        arr.push_back(std::move(o));
    }
    j["sources_locked"] = locked;
    j["sources_total"] = static_cast<int>(in.sources.size());
    j["sources"] = std::move(arr);

    return dumpJson(j, false);
}

} // namespace

HttpApi::HttpApi(HttpConfig cfg, Selector& selector, StatusProvider provider)
    : m_cfg(std::move(cfg)), m_selector(selector), m_provider(std::move(provider)) {}

HttpApi::~HttpApi() { stop(); }

bool HttpApi::start(std::string& err) {
    if (!m_cfg.enabled) {
        LOG_INFO(kTag, "disabled in configuration");
        return true;
    }

    const bool v6 = m_cfg.listen.find(':') != std::string::npos;
    m_listenFd = ::socket(v6 ? AF_INET6 : AF_INET, SOCK_STREAM, 0);
    if (m_listenFd < 0) { err = std::string("socket: ") + std::strerror(errno); return false; }

    int one = 1;
    ::setsockopt(m_listenFd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof one);

    struct sockaddr_storage sa{};
    socklen_t salen = 0;
    if (v6) {
        ::setsockopt(m_listenFd, IPPROTO_IPV6, IPV6_V6ONLY, &one, sizeof one);
        auto* s = reinterpret_cast<struct sockaddr_in6*>(&sa);
        s->sin6_family = AF_INET6;
        s->sin6_port = htons(static_cast<std::uint16_t>(m_cfg.port));
        if (inet_pton(AF_INET6, m_cfg.listen.c_str(), &s->sin6_addr) != 1) {
            err = "http.listen is not a valid IPv6 address: " + m_cfg.listen;
            ::close(m_listenFd); m_listenFd = -1; return false;
        }
        salen = sizeof(struct sockaddr_in6);
    } else {
        auto* s = reinterpret_cast<struct sockaddr_in*>(&sa);
        s->sin_family = AF_INET;
        s->sin_port = htons(static_cast<std::uint16_t>(m_cfg.port));
        if (inet_pton(AF_INET, m_cfg.listen.c_str(), &s->sin_addr) != 1) {
            err = "http.listen is not a valid IPv4 address: " + m_cfg.listen;
            ::close(m_listenFd); m_listenFd = -1; return false;
        }
        salen = sizeof(struct sockaddr_in);
    }

    if (::bind(m_listenFd, reinterpret_cast<struct sockaddr*>(&sa), salen) != 0) {
        err = "http bind " + m_cfg.listen + ":" + std::to_string(m_cfg.port) + ": " +
              std::strerror(errno);
        ::close(m_listenFd); m_listenFd = -1; return false;
    }
    if (::listen(m_listenFd, 16) != 0) {
        err = std::string("http listen: ") + std::strerror(errno);
        ::close(m_listenFd); m_listenFd = -1; return false;
    }

    m_running.store(true);
    m_acceptor = std::thread([this] { accept(); });
    LOG_INFO(kTag, "listening on http://%s:%d/ (read-only)", m_cfg.listen.c_str(), m_cfg.port);
    return true;
}

void HttpApi::stop() {
    if (!m_running.exchange(false)) return;
    // Shutdown, join, THEN close. Closing first would pull the fd out from
    // under an acceptor still in poll/accept on it, and the number could be
    // reused by another socket before the acceptor notices m_running.
    if (m_listenFd >= 0) ::shutdown(m_listenFd, SHUT_RDWR);
    if (m_acceptor.joinable()) m_acceptor.join();
    if (m_listenFd >= 0) {
        ::close(m_listenFd);
        m_listenFd = -1;
    }

    // Break every live connection out of whatever it is blocked in. An SSE
    // stream sits in poll with a timeout of up to a second; shutting its socket
    // down makes that return immediately, and the next write fails.
    {
        std::lock_guard<std::mutex> lk(m_fdMu);
        for (int fd : m_liveFds) ::shutdown(fd, SHUT_RDWR);
    }

    // Then wait for them, because they hold references to the Selector and the
    // status provider, both of which are about to be destroyed. The deadline is
    // a backstop against a thread wedged in a kernel call. Returning past it
    // would let the caller destroy what that thread is still using -- a
    // use-after-free at exit, which can corrupt the log's last lines or hang
    // in a destructor -- so the process ends here instead, without running
    // destructors. Non-zero, because a shutdown that had to abandon a thread
    // is worth systemd recording as a failure.
    const double deadline = monotonicNow() + 5.0;
    while (m_connections.load() > 0 && monotonicNow() < deadline) {
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    if (m_connections.load() > 0) {
        LOG_ERROR(kTag, "%d connection(s) did not close within 5s; exiting without cleanup",
                  m_connections.load());
        std::_Exit(EXIT_FAILURE);
    }
}

void HttpApi::accept() {
    while (m_running.load()) {
        struct pollfd pfd{m_listenFd, POLLIN, 0};
        const int pr = ::poll(&pfd, 1, 500);
        if (pr <= 0) continue;

        struct sockaddr_storage from{};
        socklen_t fromLen = sizeof from;
        const int fd = ::accept(m_listenFd, reinterpret_cast<struct sockaddr*>(&from), &fromLen);
        if (fd < 0) {
            if (errno == EINTR || errno == EAGAIN) continue;
            if (m_running.load()) LOG_DEBUG(kTag, "accept: %s", std::strerror(errno));
            continue;
        }

        // Before anything is written to it, including the 503 below: that
        // write runs on the acceptor thread, which must not be the one a
        // non-reading client wedges.
        const struct timeval tv{kSocketTimeoutSec, 0};
        ::setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof tv);
        ::setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof tv);

        if (m_connections.load() >= kMaxConnections) {
            const std::string body = "too many connections\n";
            writeAll(fd, httpResponse(503, "Service Unavailable", "text/plain; charset=utf-8",
                                      body, false));
            ::close(fd);
            continue;
        }

        m_connections.fetch_add(1);
        {
            std::lock_guard<std::mutex> lk(m_fdMu);
            m_liveFds.insert(fd);
        }
        std::thread([this, fd] {
            // Nothing may escape this thread: it is detached, so an exception
            // is std::terminate and takes the NTP server down with a status
            // page. Whatever went wrong, the connection is closed and counted
            // out below as for any other request.
            try {
                serveConnection(fd);
            } catch (const std::exception& e) {
                LOG_WARN(kTag, "connection aborted: %s", e.what());
            } catch (...) {
                LOG_WARN(kTag, "connection aborted by an unknown exception");
            }
            {
                std::lock_guard<std::mutex> lk(m_fdMu);
                m_liveFds.erase(fd);
            }
            ::close(fd);
            m_connections.fetch_sub(1);
        }).detach();
    }
}

void HttpApi::serveConnection(int fd) {
    // /api/time's receive timestamp: when the request's first bytes arrived,
    // set at the first successful recv below. Not when this thread started --
    // that is when the TCP connection was accepted, and a client may take a
    // while to send its request after connecting, which would read as delay.
    double arrived = 0.0;

    int one = 1;
    ::setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof one);

    // Read until the end of the request head. Only the request line matters
    // here, but the headers have to be consumed or a client that pipelined
    // them sees them reflected as a body.
    std::string head;
    const double deadline = monotonicNow() + kRequestTimeoutMs / 1000.0;
    char buf[4096];
    while (head.find("\r\n\r\n") == std::string::npos && head.find("\n\n") == std::string::npos) {
        const int remaining = static_cast<int>((deadline - monotonicNow()) * 1000.0);
        if (remaining <= 0) return;
        struct pollfd pfd{fd, POLLIN, 0};
        if (::poll(&pfd, 1, remaining) <= 0) return;
        const ssize_t n = ::recv(fd, buf, sizeof buf, 0);
        if (n <= 0) return;
        if (arrived == 0.0) arrived = realtimeNow();
        head.append(buf, static_cast<std::size_t>(n));
        if (head.size() > 16384) return;   // no legitimate GET is this large
    }

    const std::size_t eol = head.find_first_of("\r\n");
    if (eol == std::string::npos) return;
    std::istringstream rl(head.substr(0, eol));
    std::string method, uri, version;
    rl >> method >> uri >> version;
    if (method.empty() || uri.empty()) return;

    // Read-only, enforced rather than merely intended. GET and HEAD are the
    // whole surface; anything else is refused with the header that says so,
    // which is what a client deserves instead of a 404.
    const bool headOnly = (method == "HEAD");
    if (method != "GET" && !headOnly) {
        writeAll(fd, httpResponse(405, "Method Not Allowed", "text/plain; charset=utf-8",
                                  "this service is read-only; only GET and HEAD are accepted\n",
                                  false, "Allow: GET, HEAD"));
        return;
    }

    std::string path, query;
    splitUri(uri, path, query);

    if (path == "/" || path == "/index.html") {
        writeAll(fd, httpResponse(200, "OK", "text/html; charset=utf-8",
                                  kIndexHtml, headOnly));
        return;
    }

    if (path == "/api/events") {
        if (headOnly) {
            writeAll(fd, httpResponse(200, "OK", "text/event-stream", "", true));
            return;
        }
        serveEvents(fd);
        return;
    }

    if (path == "/api/time") {
        double clientSec = 0.0;
        // `t` in Unix seconds, or `t_ms` in milliseconds — whichever the client
        // finds easier. Only echoed back as the originate timestamp, so a
        // nonsensical value costs nothing.
        const std::string t = queryValue(query, "t");
        const std::string tms = queryValue(query, "t_ms");
        if (!t.empty()) clientSec = std::strtod(t.c_str(), nullptr);
        else if (!tms.empty()) clientSec = std::strtod(tms.c_str(), nullptr) / 1000.0;

        // Compact by default here: a program is usually reading it.
        const std::string body = renderTimeJson(m_selector.current(), arrived, clientSec,
                                                truthy(queryValue(query, "pretty")));
        writeAll(fd, httpResponse(200, "OK", "application/json; charset=utf-8",
                                  body + "\n", headOnly));
        return;
    }

    if (path == "/api/status" || path == "/api/sources") {
        const StatusInput in = m_provider();
        // Pretty by default here: this document is read by people in a terminal
        // at least as often as by programs.
        const std::string p = queryValue(query, "pretty");
        const bool pretty = p.empty() ? true : truthy(p);
        std::string body;
        if (path == "/api/sources") {
            // The same document with the sources array lifted out, rather than
            // a second serialiser that could disagree with the first.
            try {
                auto j = nlohmann::json::parse(renderStatusJson(in, false));
                body = dumpJson(j["sources"], pretty);
            } catch (const std::exception&) {
                body = renderStatusJson(in, pretty);
            }
        } else {
            body = renderStatusJson(in, pretty);
        }
        writeAll(fd, httpResponse(200, "OK", "application/json; charset=utf-8",
                                  body + "\n", headOnly));
        return;
    }

    if (path == "/api/health") {
        // Deliberately tiny and deliberately honest: 200 when a time is being
        // served, 503 when it is not, so a health check need not parse
        // anything. A daemon that is running but unsynchronised is not healthy
        // for the purpose anyone checks.
        const Combined c = m_selector.current();
        std::ostringstream b;
        b << "{\"ok\":" << (c.synchronised ? "true" : "false")
          << ",\"synchronised\":" << (c.synchronised ? "true" : "false")
          << ",\"stratum\":" << (c.synchronised ? 1 : 16)
          << ",\"dispersion_ms\":" << (c.dispersionSec * 1000.0) << "}\n";
        writeAll(fd, httpResponse(c.synchronised ? 200 : 503,
                                  c.synchronised ? "OK" : "Service Unavailable",
                                  "application/json; charset=utf-8", b.str(), headOnly));
        return;
    }

    writeAll(fd, httpResponse(404, "Not Found", "text/plain; charset=utf-8",
                              "not found\n\navailable:\n"
                              "  /              status page\n"
                              "  /api/events    one event per corrected second (SSE)\n"
                              "  /api/time      the time, for clients that do not speak NTP\n"
                              "  /api/status    everything this daemon knows\n"
                              "  /api/sources   just the per-source array\n"
                              "  /api/health    200 when synchronised, 503 when not\n",
                              headOnly));
}

void HttpApi::serveEvents(int fd) {
    // Counted in and out by a guard, so an exception on the way out of here
    // (caught in the connection thread) cannot leak a stream slot for good.
    struct StreamSlot {
        std::atomic<int>& n;
        ~StreamSlot() { n.fetch_sub(1); }
    };
    // Claimed first and checked after, so two clients racing for the last slot
    // cannot both get it.
    const int streams = m_streams.fetch_add(1) + 1;
    StreamSlot slot{m_streams};
    if (streams > kMaxStreams) {
        writeAll(fd, httpResponse(503, "Service Unavailable", "text/plain; charset=utf-8",
                                  "too many event streams\n", false, "Retry-After: 10"));
        return;
    }

    std::ostringstream hdr;
    hdr << "HTTP/1.1 200 OK\r\n"
        << "Content-Type: text/event-stream; charset=utf-8\r\n"
        << "Cache-Control: no-store\r\n"
        << "Access-Control-Allow-Origin: *\r\n"
        // Without this an nginx or Apache in front buffers the stream and the
        // ticks arrive in clumps, which is precisely the property a one-second
        // tick is for.
        << "X-Accel-Buffering: no\r\n"
        << "Connection: close\r\n\r\n"
        // How long a browser waits before reconnecting a dropped stream. Two
        // seconds, because losing two ticks is a visible stall on a clock.
        << "retry: 2000\n\n";

    if (!writeAll(fd, hdr.str())) return;

    // Both documents immediately, so a client that has just connected draws
    // everything at once. Sending only the status and letting the first tick
    // wait for the next second boundary left the page's clock and summary
    // blank for up to a second while its detail panels were already filled in,
    // which reads as a half-broken page rather than as a page that is about to
    // finish loading.
    {
        const StatusInput in = m_provider();
        const std::string status = "event: status\ndata: " + renderStatusJson(in, false) + "\n\n";
        if (!writeAll(fd, status)) return;
        const std::string tick = "event: tick\ndata: " +
                                 tickJson(m_selector.current(), in, realtimeNow()) + "\n\n";
        if (!writeAll(fd, tick)) return;
    }

    double lastFull = monotonicNow();

    while (m_running.load()) {
        // Sleep to the next CORRECTED second boundary: the instant the
        // broadcast's own second rolls over, not this host's. A display driven
        // from this stream therefore ticks with WWV, and it keeps doing so
        // across a change in the offset — the boundary moves with the
        // correction rather than drifting away from it.
        const Combined c = m_selector.current();
        const double now = realtimeNow();
        const double corrected = now + c.offsetSec;
        double wait = std::ceil(corrected) - corrected;
        // Exactly on the boundary, go to the next one rather than firing twice.
        if (wait < 0.002) wait += 1.0;

        struct pollfd pfd{fd, POLLIN, 0};
        // Poll rather than sleep, so a client that closes the connection is
        // noticed within the second rather than on the next failed write.
        const int pr = ::poll(&pfd, 1, static_cast<int>(wait * 1000.0));
        if (pr > 0) {
            char discard[512];
            const ssize_t n = ::recv(fd, discard, sizeof discard, 0);
            if (n <= 0) break;   // the client has gone
            // Anything a client sends on an SSE stream is ignored: this is a
            // read-only service and the stream is one-directional. Back to
            // waiting for the boundary rather than falling through, which
            // would send an extra tick for every packet the client sent.
            continue;
        }

        if (!m_running.load()) break;

        const Combined c2 = m_selector.current();
        const StatusInput in = m_provider();
        const std::string tick = "event: tick\ndata: " +
                                 tickJson(c2, in, realtimeNow()) + "\n\n";
        if (!writeAll(fd, tick)) break;

        // The per-source detail does not change at 1 Hz and is twenty times the
        // size of a tick, so it goes out every few seconds instead.
        const double t = monotonicNow();
        if (t - lastFull >= kFullStatusEverySec) {
            lastFull = t;
            const std::string full = "event: status\ndata: " + renderStatusJson(in, false) + "\n\n";
            if (!writeAll(fd, full)) break;
        }
    }
}

} // namespace ubersdr_ntp
