#include "HttpApi.h"

#include "CivilTime.h"
#include "Log.h"
#include "SampleClock.h"

#include "../third_party/json.hpp"

#include <algorithm>
#include <arpa/inet.h>
#include <cctype>
#include <cerrno>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <poll.h>
#include <set>
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

// Splits a comma-separated query value. Empty items are dropped.
std::set<std::string> csvSet(const std::string& v) {
    std::set<std::string> out;
    std::size_t pos = 0;
    while (pos <= v.size()) {
        std::size_t comma = v.find(',', pos);
        if (comma == std::string::npos) comma = v.size();
        if (comma > pos) out.insert(v.substr(pos, comma - pos));
        pos = comma + 1;
    }
    return out;
}

// Enough of percent-decoding for a source name or a type list: %XX and '+'.
std::string urlDecode(const std::string& v) {
    std::string out;
    for (std::size_t i = 0; i < v.size(); ++i) {
        if (v[i] == '+') { out += ' '; continue; }
        if (v[i] == '%' && i + 2 < v.size() && std::isxdigit(static_cast<unsigned char>(v[i + 1])) &&
            std::isxdigit(static_cast<unsigned char>(v[i + 2]))) {
            out += static_cast<char>(std::strtol(v.substr(i + 1, 2).c_str(), nullptr, 16));
            i += 2;
            continue;
        }
        out += v[i];
    }
    return out;
}

// GET /api/eventlog. Every event held, newest first, with the type catalogue
// alongside so a client can label and filter without knowing the daemon's
// vocabulary in advance. The filters are all optional and combine as AND:
//
//   type=a,b        only these types
//   category=a,b    only these categories (daemon, clock, class, source, radio, ntp)
//   source=name     only events about this source
//   kind=radio|ntp  only events about this kind of source or class
//   severity=s      this severity or worse (info, notice, warning, error)
//   since_id=n      only events newer than n
//   limit=n         at most n of them
std::string eventLogJson(const EventLog* log, const std::string& query, bool pretty) {
    nlohmann::json j;
    j["capacity"] = EventLog::kCapacity;
    j["latest_id"] = log ? log->latestId() : 0;

    nlohmann::json types = nlohmann::json::array();
    for (const EventTypeInfo& t : eventTypes()) {
        types.push_back({{"type", t.name},
                         {"label", t.label},
                         {"category", t.category},
                         {"severity", eventSeverityName(t.severity)},
                         {"description", t.description}});
    }
    j["types"] = std::move(types);
    j["severities"] = {"info", "notice", "warning", "error"};

    const std::set<std::string> wantTypes = csvSet(urlDecode(queryValue(query, "type")));
    const std::set<std::string> wantCats = csvSet(urlDecode(queryValue(query, "category")));
    const std::string wantSource = urlDecode(queryValue(query, "source"));
    const std::string wantKind = urlDecode(queryValue(query, "kind"));
    const std::string sev = queryValue(query, "severity");
    int minSev = 0;
    for (int i = 0; i <= 3; ++i) {
        if (sev == eventSeverityName(static_cast<EventSeverity>(i))) minSev = i;
    }
    const std::string sinceText = queryValue(query, "since_id");
    const std::uint64_t since = sinceText.empty() ? 0 : std::strtoull(sinceText.c_str(), nullptr, 10);
    const std::string limitText = queryValue(query, "limit");
    const long limit = limitText.empty() ? -1 : std::strtol(limitText.c_str(), nullptr, 10);

    nlohmann::json events = nlohmann::json::array();
    if (log) {
        for (const Event& e : log->all()) {
            if (limit >= 0 && static_cast<long>(events.size()) >= limit) break;
            const EventTypeInfo& info = eventTypeInfo(e.type);
            if (e.id <= since) continue;
            if (!wantTypes.empty() && !wantTypes.count(info.name)) continue;
            if (!wantCats.empty() && !wantCats.count(info.category)) continue;
            if (!wantSource.empty() && e.source != wantSource) continue;
            if (!wantKind.empty() && (!e.haveKind || wantKind != sourceKindName(e.kind))) continue;
            if (static_cast<int>(info.severity) < minSev) continue;

            events.push_back(eventJson(e));
        }
    }
    j["events"] = std::move(events);
    return dumpJson(j, pretty);
}

// GET /api/metrics. One range at a time -- `range=hour` (the default: 60
// one-minute buckets) or `range=day` (48 half-hour ones) -- and optionally only
// some groups, `group=served,class,radio,ntp`. Each point is
// [bucket start (UTC s), mean, min, max, samples], oldest first; the last one
// is still filling. Values to a microsecond of a millisecond is more precision
// than any of them has, and rounding keeps a day of every source small.
std::string metricsJson(const MetricHistory* h, const std::string& query, bool pretty,
                        double now) {
    const bool day = queryValue(query, "range") == "day";
    const std::set<std::string> groups = csvSet(urlDecode(queryValue(query, "group")));
    const auto r3 = [](double v) { return std::round(v * 1000.0) / 1000.0; };

    nlohmann::json j;
    j["range"] = day ? "day" : "hour";
    j["bucket_seconds"] = day ? MetricHistory::kDayWidthSec : MetricHistory::kHourWidthSec;
    j["buckets"] = day ? MetricHistory::kDayBuckets : MetricHistory::kHourBuckets;
    j["now"] = now;
    nlohmann::json series = nlohmann::json::array();
    if (h) {
        for (const auto& s : h->snapshot(day ? MetricHistory::Range::Day
                                             : MetricHistory::Range::Hour, now)) {
            if (!groups.empty() && !groups.count(s.info.group)) continue;
            nlohmann::json pts = nlohmann::json::array();
            for (const MetricBucket& b : s.points) {
                pts.push_back({b.start, r3(b.mean()), r3(b.min), r3(b.max), b.count});
            }
            series.push_back({{"id", s.info.id},
                              {"label", s.info.label},
                              {"unit", s.info.unit},
                              {"group", s.info.group},
                              {"source", s.info.source.empty() ? nlohmann::json(nullptr)
                                                               : nlohmann::json(s.info.source)},
                              {"points", std::move(pts)}});
        }
    }
    j["series"] = std::move(series);

    // Which class served, as spans: [start, state], oldest first, each
    // lasting until the next and the last until `now`. The first may start
    // before the window, and says what held at its beginning.
    nlohmann::json spans = nlohmann::json::array();
    if (h) {
        for (const auto& sp : h->serving(day ? MetricHistory::Range::Day
                                             : MetricHistory::Range::Hour, now)) {
            spans.push_back({sp.start, sp.state});
        }
    }
    j["serving"] = std::move(spans);
    return dumpJson(j, pretty);
}

// The compact per-second payload: the corrected time, what is being served,
// and one line per source. Everything a live display or the summary half of the
// page needs, at a size that is reasonable to send every second.
std::string tickJson(const Combined& c, const StatusInput& in, double nowDaemon) {
    nlohmann::json j;
    const double utc = c.utcAt(nowDaemon);
    const long long utcMs = static_cast<long long>(std::llround(utc * 1000.0));

    j["unix"] = utc;
    j["unix_ms"] = utcMs;
    j["utc"] = iso8601(utcMs);
    j["synchronised"] = c.synchronised;
    j["stratum"] = c.synchronised ? c.stratum : 16;
    j["refid"] = c.refid;
    j["root_delay_ms"] = c.rootDelaySec * 1000.0;
    // The correction this HOST would need, which is what a person reading
    // "offset" wants to know.
    j["offset_ms"] = (utc - (nowDaemon - daemonMinusRealtime())) * 1000.0;
    // The served clock against the daemon's own, and how fast that moves: what
    // a client timing repeated requests needs to tell a change in the server's
    // estimate from the rate it is expected to move at (see the status page).
    j["clock_offset_ms"] = (utc - nowDaemon) * 1000.0;
    j["clock_rate_ppm"] = c.rate * 1e6;
    j["dispersion_ms"] = c.dispersionSec * 1000.0;
    j["reference_age_seconds"] = c.ageSec;
    j["sources_used"] = c.used;
    j["sources_candidate"] = c.candidates;
    j["leap_pending"] = c.leapPending;
    if (!c.note.empty()) j["note"] = c.note;
    j["used_names"] = c.usedNames;
    j["uptime_seconds"] = in.uptimeSec;
    j["version"] = in.version;
    j["events_latest_id"] = in.eventsLatestId;

    // The two classes, every second, because the page's headline figures now
    // include which class is serving and how far apart the two are -- and a
    // figure that only refreshes with the full status block would lag a
    // failover by several seconds on the one screen someone is watching it on.
    {
        nlohmann::json k;
        k["primary"] = sourceKindName(in.primaryKind);
        k["secondary"] = sourceKindName(in.secondaryKind);
        k["secondary_mode"] = secondaryModeName(in.secondaryMode);
        k["secondary_active"] = in.secondaryActive;
        k["serving"] = servingClassName(c.serving);
        k["serving_note"] = c.servingNote;
        k["primary_candidates"] = c.primaryCandidates;
        k["secondary_candidates"] = c.secondaryCandidates;
        k["failover_in_seconds"] = c.failoverInSec > 0.0 ? nlohmann::json(c.failoverInSec)
                                                         : nlohmann::json(nullptr);
        k["failback_in_seconds"] = c.failbackInSec > 0.0 ? nlohmann::json(c.failbackInSec)
                                                         : nlohmann::json(nullptr);
        nlohmann::json cd;
        cd["valid"] = c.classDelta.valid;
        if (c.classDelta.valid) {
            cd["delta_ms"] = c.classDelta.averagedSec * 1000.0;
            cd["instant_ms"] = c.classDelta.instantSec * 1000.0;
            cd["settled_for_seconds"] = c.classDelta.settledForSec;
            cd["primary_sources"] = c.classDelta.primarySources;
            cd["secondary_sources"] = c.classDelta.secondarySources;
        }
        k["class_delta"] = std::move(cd);
        j["clock"] = std::move(k);
    }

    // Per-source agreement, which the page draws as a column: each source
    // against the others in its own class.
    {
        nlohmann::json ag = nlohmann::json::array();
        for (const SourceResidual& r : c.residuals) {
            ag.push_back({{"name", r.name},
                          {"kind", sourceKindName(r.kind)},
                          {"residual_ms", r.averagedSec * 1000.0},
                          {"peers", r.peers},
                          {"refused", r.refused}});
        }
        j["agreement"] = std::move(ag);
    }

    nlohmann::json n;
    n["port"] = in.ntpPort;
    n["requests"] = in.ntp.requests;
    n["answered"] = in.ntp.answered;
    n["ignored"] = in.ntp.ignored;
    n["rate_limited"] = in.ntp.rateLimited;
    j["ntp"] = std::move(n);

    nlohmann::json arr = nlohmann::json::array();
    int ready = 0;
    for (const SourceSnapshot& s : in.sources) {
        if (s.ready) ++ready;
        nlohmann::json o;
        o["name"] = s.name;
        o["kind"] = sourceKindName(s.kind);
        o["primary_class"] = s.primaryClass;
        o["active"] = s.active;
        o["ready"] = s.ready;
        o["not_ready_reason"] = s.notReadyReason.empty() ? nlohmann::json(nullptr)
                                                         : nlohmann::json(s.notReadyReason);
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
        // DCF77: whose second edges the offset is taken from, "pm" or "am".
        if (s.station == "dcf77") o["timing"] = s.timingFromPm ? "pm" : "am";
        o["last_quality"] = s.lastQuality;
        o["have_offset"] = s.haveOffset;
        o["offset_ms"] = s.hostOffsetSec * 1000.0;
        o["dispersion_ms"] = s.dispersionSec * 1000.0;
        o["jitter_ms"] = s.jitterSec * 1000.0;
        o["weight_dispersion_ms"] = s.weightDispersionSec * 1000.0;
        o["samples"] = s.offsetSamples;
        o["in_use"] = std::find(c.usedNames.begin(), c.usedNames.end(), s.name) != c.usedNames.end();
        const auto why = c.notUsedReasons.find(s.name);
        o["not_used_reason"] = why != c.notUsedReasons.end()
                                   ? nlohmann::json(why->second) : nlohmann::json(nullptr);
        if (s.kind == SourceKind::Ntp) {
            nlohmann::json p;
            p["server"] = s.ntp.server;
            p["address"] = s.ntp.address;
            p["stratum"] = s.ntp.stratum;
            p["refid"] = s.ntp.refid;
            p["reach"] = s.ntp.reach;
            {
                char b[8];
                std::snprintf(b, sizeof b, "%03o", s.ntp.reach);
                p["reach_octal"] = b;
            }
            p["poll_seconds"] = s.ntp.pollSec;
            p["delay_ms"] = s.ntp.delaySec * 1000.0;
            p["root_distance_ms"] = s.ntp.rootDistanceSec * 1000.0;
            p["stopped"] = s.ntp.stopped;
            p["kiss_code"] = s.ntp.kissCode;
            o["ntp"] = std::move(p);
        }
        arr.push_back(std::move(o));
    }
    j["sources_ready"] = ready;
    // Kept under its old name too: an external client written against the
    // previous shape should not break over a word.
    j["sources_locked"] = ready;
    j["sources_total"] = static_cast<int>(in.sources.size());
    j["sources"] = std::move(arr);

    return dumpJson(j, false);
}

} // namespace

HttpApi::HttpApi(HttpConfig cfg, Selector& selector, StatusProvider provider,
                 const EventLog* events, const MetricHistory* metrics)
    : m_cfg(std::move(cfg)),
      m_selector(selector),
      m_provider(std::move(provider)),
      m_events(events),
      m_metrics(metrics) {}

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
        // "::" is the one address anyone writes meaning "everything", so let
        // it also accept IPv4 rather than making them choose a family. Any
        // other v6 address is a specific interface and stays v6-only.
        const int v6only = (m_cfg.listen == "::") ? 0 : 1;
        ::setsockopt(m_listenFd, IPPROTO_IPV6, IPV6_V6ONLY, &v6only, sizeof v6only);
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
    // /api/time's receive timestamp, on the daemon clock: when the request's
    // first bytes arrived, set at the first successful recv below. Not when this thread started --
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
        if (arrived == 0.0) arrived = daemonNow();
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

    if (path == "/api/eventlog") {
        const std::string p = queryValue(query, "pretty");
        const bool pretty = p.empty() ? true : truthy(p);
        writeAll(fd, httpResponse(200, "OK", "application/json; charset=utf-8",
                                  eventLogJson(m_events, query, pretty) + "\n", headOnly));
        return;
    }

    if (path == "/api/metrics") {
        // Compact by default: this one is read by the page, and a day of every
        // source pretty-printed is mostly whitespace.
        const Combined c = m_selector.current();
        const double d = daemonNow();
        const double now = c.valid ? c.utcAt(d) : d - daemonMinusRealtime();
        writeAll(fd, httpResponse(200, "OK", "application/json; charset=utf-8",
                                  metricsJson(m_metrics, query,
                                              truthy(queryValue(query, "pretty")), now) + "\n",
                                  headOnly));
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
                              "  /api/eventlog  the last 100 events worth knowing about\n"
                              "  /api/metrics   recent history: ?range=hour or ?range=day\n"
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
                                 tickJson(m_selector.current(), in, daemonNow()) + "\n\n";
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
        const double corrected = c.utcAt(daemonNow());
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
                                 tickJson(c2, in, daemonNow()) + "\n\n";
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
