#include "Source.h"

#include "CivilTime.h"
#include "Log.h"
#include "Version.h"
#include "../third_party/json.hpp"
#include "../third_party/pcm_v4.hpp"

#include <ixwebsocket/IXNetSystem.h>
#include <ixwebsocket/IXWebSocket.h>

#include <curl/curl.h>
#include <opus/opus.h>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <random>
#include <unistd.h>
#include <functional>
#include <memory>
#include <sstream>

using nlohmann::json;

namespace ubersdr_ntp {

// The lossless decoder lives behind a pimpl so pcm_v4.hpp — 1000 lines of
// codec that nothing else needs — stays out of Source.h and out of every
// translation unit that merely wants a SourceSnapshot.
class PcmV4Reader {
public:
    ubersdr::PCMv4StreamDecoder dec;
};

const char* linkStateName(LinkState s) {
    switch (s) {
        case LinkState::Idle:       return "idle";
        case LinkState::Connecting: return "connecting";
        case LinkState::Streaming:  return "streaming";
        case LinkState::Backoff:    return "backoff";
        case LinkState::Stopped:    return "stopped";
    }
    return "?";
}

namespace {

// How long the offset window reaches back. Two minutes is long enough to
// average a couple of hundred second-edge measurements and short enough that a
// genuine step — a source that resynchronised onto a different edge — works its
// way out rather than being averaged with the old value for ever.
constexpr double kOffsetWindowSec = 120.0;

// Offsets older than this stop counting as current. A source that locked and
// then faded should stop contributing long before its last reading is useless,
// and the selector needs a definite answer rather than a decaying one.
constexpr double kOffsetStaleSec = 180.0;

// An anchor is extended forwards one second at a time. Past this, the extension
// has run further than a voted timestamp can vouch for and the next `time`
// event should have arrived — WWV sends one a minute.
constexpr double kAnchorMaxAgeSec = 300.0;

// Opus's constant contribution to the delay. The encoder's own lookahead is
// 6.5 ms at any rate; the measured end-to-end shift of the decoder's second
// edge through an encode/decode round trip at 12 kHz is 5-10 ms, the balance
// being reconstruction. 8 ms is the middle of what was measured, and it is a
// bias rather than noise: across clean and 3 dB-SNR signals the shift did not
// move.
constexpr double kOpusDelaySec = 0.008;

// Where the WWV/WWVH decoder puts a second edge relative to the true edge in
// its input: 13.6 ms EARLY, the matched filter's chain delay being taken as 7
// series samples where it is 4.27. Measured, not estimated: tools/decodertest
// generates WWV and WWVH and reports this mean over every edge it checks
// (-13.645 / -13.637 ms, spread about ±1 ms). The WWVB decoder's edges are
// exact to 0.02 ms, so it has no such term. Kept as a correction here rather
// than fixed in the decoder, whose tracker is built around the upstream value.
constexpr double kWwvDecoderEdgeBiasSec = -0.013645;

// The delay from RF reaching the SDR to the audio leaving UberSDR's WebSocket:
// radiod's demodulator and filters, its block framing, the server's handling.
// It is a property of the software, the same on every instance, so it is one
// constant rather than something each operator has to calibrate.
//
// Measured live, 2026-09-13, against a north-eastern US receiver hearing WWV on
// 10 and 15 MHz for 14 minutes of lock, from a host disciplined by ntpd to about
// 1.4 ms: with this term equal to the decoder bias above, the served offset
// averaged +0.1 ms and stayed within ±2.6 ms. One receiver and one session, so
// good to perhaps ±2.5 ms; worth refining against more receivers.
constexpr double kUberSdrChainDelaySec = 0.0136;

// Floor on how well the delay model can be trusted, whatever it computed. The
// receiver's own buffering between radiod and the WebSocket is inside this and
// nothing here can see it.
constexpr double kDelayUncertaintyFloorSec = 0.015;
// And a proportional part, because a long path is a worse-known path: more
// hops, more spread between them, more of the virtual height assumption.
constexpr double kDelayUncertaintyFraction = 0.25;

std::string makeUuidV4() {
    // The server requires a canonical lowercase v4 UUID and binds it to this
    // host's IP, so it identifies the session rather than securing anything —
    // but it is also what a receiver operator sees in their session list, so a
    // fresh one per process avoids colliding with another copy of this daemon.
    static std::random_device rd;
    static std::mt19937_64 gen(rd() ^ (static_cast<std::uint64_t>(::getpid()) << 32));
    std::uniform_int_distribution<std::uint64_t> d;
    std::uint64_t a = d(gen), b = d(gen);

    a = (a & 0xFFFFFFFFFFFF0FFFULL) | 0x0000000000004000ULL;   // version 4
    b = (b & 0x3FFFFFFFFFFFFFFFULL) | 0x8000000000000000ULL;   // variant 1

    char buf[40];
    std::snprintf(buf, sizeof buf, "%08x-%04x-%04x-%04x-%012llx",
                  static_cast<unsigned>(a >> 32),
                  static_cast<unsigned>((a >> 16) & 0xFFFF),
                  static_cast<unsigned>(a & 0xFFFF),
                  static_cast<unsigned>(b >> 48),
                  static_cast<unsigned long long>(b & 0xFFFFFFFFFFFFULL));
    return buf;
}

struct CurlBuf { std::string data; };

std::size_t curlWrite(char* ptr, std::size_t size, std::size_t nmemb, void* userdata) {
    auto* b = static_cast<CurlBuf*>(userdata);
    b->data.append(ptr, size * nmemb);
    return size * nmemb;
}

// One HTTP request, returning the status code, the body, and the network round
// trip. The RTT is not incidental: half of it is the best available estimate of
// the one-way network delay from the receiver to here, which is a term in the
// timing budget.
//
// The round trip is the TCP HANDSHAKE, and which of curl's timings that is
// matters more than it looks. CURLINFO_APPCONNECT_TIME is the obvious choice
// and it is wrong: it covers DNS, the TCP handshake AND the TLS handshake, so
// it is two to three round trips, and halving it overestimates the one-way
// delay by the same factor. Measured against this receiver, TCP was 107 ms and
// TLS-complete 221 ms; taking half of the latter put 110 ms into the budget
// where 54 ms belonged, and the served time was 56 ms wrong as a direct result.
//
// CONNECT_TIME minus NAMELOOKUP_TIME is the SYN/SYN-ACK exchange and nothing
// else, which is one round trip by definition.
long httpRequest(const std::string& url, const std::string* postBody,
                 const std::string& userAgent, bool verifyTls,
                 std::string& body, double& rttSec, std::string& err) {
    CURL* curl = curl_easy_init();
    if (!curl) { err = "curl_easy_init failed"; return -1; }

    CurlBuf buf;
    struct curl_slist* headers = nullptr;
    headers = curl_slist_append(headers, "Content-Type: application/json");
    const std::string ua = "User-Agent: " + userAgent;
    headers = curl_slist_append(headers, ua.c_str());

    curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
    if (postBody) curl_easy_setopt(curl, CURLOPT_POSTFIELDS, postBody->c_str());
    curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, curlWrite);
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, &buf);
    curl_easy_setopt(curl, CURLOPT_TIMEOUT, 15L);
    curl_easy_setopt(curl, CURLOPT_FOLLOWLOCATION, 1L);
    if (!verifyTls) {
        curl_easy_setopt(curl, CURLOPT_SSL_VERIFYPEER, 0L);
        curl_easy_setopt(curl, CURLOPT_SSL_VERIFYHOST, 0L);
    }

    const double t0 = monotonicNow();
    const CURLcode res = curl_easy_perform(curl);
    const double wall = monotonicNow() - t0;
    rttSec = 0.0;

    long code = -1;
    if (res == CURLE_OK) {
        curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &code);
        double connectTime = 0.0, lookupTime = 0.0;
        if (curl_easy_getinfo(curl, CURLINFO_CONNECT_TIME, &connectTime) == CURLE_OK &&
            curl_easy_getinfo(curl, CURLINFO_NAMELOOKUP_TIME, &lookupTime) == CURLE_OK &&
            connectTime > lookupTime) {
            rttSec = connectTime - lookupTime;
        } else {
            // A reused connection reports no handshake at all. The whole
            // request is then an upper bound, and an upper bound on a delay
            // that is about to be halved is better than zero — which would
            // silently drop the largest correctable term in the budget.
            rttSec = wall;
        }
    } else {
        err = curl_easy_strerror(res);
        rttSec = wall;
    }

    body = std::move(buf.data);
    curl_slist_free_all(headers);
    curl_easy_cleanup(curl);
    return code;
}

std::string httpToWs(const std::string& url) {
    if (url.rfind("https://", 0) == 0) return "wss://" + url.substr(8);
    if (url.rfind("http://", 0) == 0)  return "ws://" + url.substr(7);
    return url;
}

} // namespace

// ---------------------------------------------------------------------------

Source::Source(SourceConfig cfg)
    : m_cfg(std::move(cfg)),
      m_sessionId(makeUuidV4()),
      m_clock(12000) {
    m_snap.name = m_cfg.name;
    m_snap.url = m_cfg.url;
    m_snap.enabled = m_cfg.enabled;
    m_snap.carrierHz = m_cfg.carrierHz;
    m_snap.dialHz = m_cfg.dialHz;
    m_snap.format = m_cfg.format;
    m_snap.weight = m_cfg.weight;
    m_snap.station = m_cfg.dialHz < kWwvbCeilingHz ? "wwvb" : "unknown";
    m_linkSince = monotonicNow();
}

Source::~Source() {
    stop();
    if (m_opus) { opus_decoder_destroy(m_opus); m_opus = nullptr; }
}

void Source::start() {
    if (!m_cfg.enabled) {
        std::lock_guard<std::mutex> lk(m_mu);
        m_snap.link = LinkState::Stopped;
        m_snap.linkDetail = "disabled in configuration";
        return;
    }
    if (m_running.exchange(true)) return;
    m_thread = std::thread([this] { supervise(); });
}

void Source::stop() {
    if (!m_running.exchange(false)) return;
    // The wake mutex is taken, empty, before notifying. Without it the flag can
    // flip and the notification fire between the supervisor testing its
    // predicate and blocking, and a lost wakeup there costs a whole backoff
    // interval — up to 37 s — on SIGTERM.
    { std::lock_guard<std::mutex> lk(m_wake); }
    m_wakeCv.notify_all();

    // The socket is NOT stopped here. ix::WebSocket::stop() joins the socket's
    // thread unguarded and the supervisor stops the same socket on its way out,
    // so doing it from both threads is a double join. Nor does it need to be:
    // the supervisor never waits on the socket, only on m_wakeCv, which the
    // notify above ends at once — it then stops the socket itself, as promptly
    // as this thread could have.
    if (m_thread.joinable()) m_thread.join();
    {
        std::lock_guard<std::mutex> lk(m_wsMu);
        m_ws.reset();
    }
    std::lock_guard<std::mutex> lk(m_mu);
    m_snap.link = LinkState::Stopped;
}

// ---------------------------------------------------------------------------
// Lifecycle

void Source::supervise() {
    const char* tag = m_cfg.name.c_str();
    LOG_INFO(tag, "starting: %s dial %.6f MHz (carrier %.6f MHz) format %s",
             m_cfg.url.c_str(), m_cfg.dialHz / 1e6, m_cfg.carrierHz / 1e6,
             formatName(m_cfg.format));

    // At boot for the receiver's name and coordinates, and again after each
    // successful connection for as long as the coordinates are still unknown —
    // see the stream loop. Once only was not enough: a receiver unreachable at
    // the moment this daemon started left the propagation term at zero for the
    // life of the process, which is 9-15 ms of bias on a transatlantic path.
    fetchDescription();

    // Reconnection policy: forever, with exponential backoff capped at 30 s,
    // plus jitter.
    //
    // FOREVER, because no failure here is worth giving up on. A receiver that
    // is full will have room later, a band that is dead will open, a tunnel
    // that is down will come back -- and a time server that stopped trying at
    // 3am and was still stopped at noon is worse than useless, because it looks
    // like it is working.
    //
    // CAPPED AT 30 s, because /connection is rate limited at ten requests a
    // minute per IP and a daemon that hammers it turns a transient refusal into
    // a persistent one. Thirty seconds is also about the longest gap that does
    // not look broken on the status page.
    //
    // JITTERED, because every source starts at the same instant and fails at
    // the same instant -- one receiver going down puts all of its sources into
    // the same backoff, and without jitter they then retry in lockstep for
    // ever, arriving as exactly the burst the rate limit exists to stop.
    //
    // A FRESH SESSION ID each attempt, in sessionHandshake: the server binds a
    // UUID to an IP and remembers that it kicked one, so carrying a rejected id
    // into the next attempt inherits the rejection.
    constexpr double kBackoffMin = 2.0;
    constexpr double kBackoffMax = 30.0;
    constexpr double kJitterFraction = 0.25;
    // Backoff resets once a connection has PROVED itself, not merely opened. A
    // receiver that accepts the WebSocket and then drops it a second later --
    // which is what hitting a session limit looks like from here -- would
    // otherwise reset the backoff on every attempt and be retried every two
    // seconds indefinitely.
    constexpr double kProvenAfterSec = 30.0;

    double backoff = kBackoffMin;
    // Consecutive failures, for throttling the log. A source unreachable for
    // six hours should not have written seven hundred identical warnings.
    int consecutiveFailures = 0;
    double lastFailureLogAt = 0.0;

    // Per source rather than shared, so two sources do not draw the same
    // jitter from a common generator and stay in lockstep regardless.
    std::mt19937 jitterGen(static_cast<std::uint32_t>(
        std::hash<std::string>{}(m_cfg.name)) ^ static_cast<std::uint32_t>(::getpid()));

    auto backoffDelay = [&]() {
        std::uniform_real_distribution<double> d(1.0 - kJitterFraction, 1.0 + kJitterFraction);
        return backoff * d(jitterGen);
    };

    // The first three failures are always logged -- that is the interesting
    // part, and what someone starting the daemon is watching for. After that,
    // one line every five minutes carries the count instead.
    auto shouldLogFailure = [&]() {
        if (consecutiveFailures <= 3) return true;
        const double now = monotonicNow();
        if (now - lastFailureLogAt >= 300.0) { lastFailureLogAt = now; return true; }
        return false;
    };

    while (m_running.load()) {
        {
            std::lock_guard<std::mutex> lk(m_mu);
            m_snap.link = LinkState::Connecting;
            m_snap.linkDetail.clear();
            m_snap.connectAttempts++;
            m_linkSince = monotonicNow();
        }

        std::string err;
        if (!sessionHandshake(err)) {
            ++consecutiveFailures;
            const double delay = backoffDelay();
            if (shouldLogFailure()) {
                if (consecutiveFailures <= 3) {
                    LOG_WARN(tag, "session handshake failed: %s (retry in %.0fs)",
                             err.c_str(), delay);
                } else {
                    LOG_WARN(tag, "still failing after %d attempts: %s (retrying every ~%.0fs)",
                             consecutiveFailures, err.c_str(), backoff);
                }
            }
            {
                std::lock_guard<std::mutex> lk(m_mu);
                m_snap.link = LinkState::Backoff;
                m_snap.linkDetail = err;
                m_linkSince = monotonicNow();
            }
            std::unique_lock<std::mutex> lk(m_wake);
            m_wakeCv.wait_for(lk, std::chrono::duration<double>(delay),
                              [this] { return !m_running.load(); });
            backoff = std::min(backoff * 2.0, kBackoffMax);
            continue;
        }

        // A fresh socket per attempt. Reusing one that has already failed its
        // handshake leaves IXWebSocket's internal state describing the old
        // attempt, and the URL has to change anyway when the session id does.
        auto ws = std::make_shared<ix::WebSocket>();
        {
            std::lock_guard<std::mutex> lk(m_wsMu);
            m_ws = ws;
        }
        ws->setUrl(buildWsUrl());
        ws->setExtraHeaders({{"User-Agent", kUserAgent}});
        ws->disableAutomaticReconnection();   // reconnection is this loop's job,
                                                // because it must re-POST /connection first
        ws->setHandshakeTimeout(20);
        // WebSocket-level pings, separate from the JSON keepalive below. These
        // keep NAT and proxy state alive; the JSON one is what the UberSDR
        // session timer watches.
        ws->setPingInterval(20);

        if (!m_cfg.verifyTls) {
            ix::SocketTLSOptions tls;
            tls.caFile = "NONE";
            tls.disable_hostname_validation = true;
            ws->setTLSOptions(tls);
        }

        ws->setOnMessageCallback([this](const ix::WebSocketMessagePtr& msg) {
            switch (msg->type) {
                case ix::WebSocketMessageType::Open:
                    onOpen();
                    break;
                case ix::WebSocketMessageType::Close:
                    onClose(msg->closeInfo.reason.empty() ? "closed" : msg->closeInfo.reason);
                    break;
                case ix::WebSocketMessageType::Error:
                    onClose("error: " + msg->errorInfo.reason);
                    break;
                case ix::WebSocketMessageType::Message:
                    if (msg->binary) onBinary(msg->str);
                    else onText(msg->str);
                    break;
                default:
                    break;
            }
        });

        m_socketOpen.store(false);
        ws->start();

        // Keepalive and liveness, until the socket goes away or we are stopped.
        //
        // The liveness check matters as much as the keepalive: a WebSocket over
        // a path that has silently gone away stays "open" until the OS gives up
        // on the TCP connection, which can be minutes. Audio arrives 50 times a
        // second, so ten seconds of silence is already unambiguous.
        double lastPing = monotonicNow();
        double lastRttProbe = monotonicNow();
        // A thread-local copy: m_linkSince is written under m_mu by the
        // WebSocket thread, and reading it here without the lock is a race on a
        // double for no benefit -- this loop already knows when it started.
        const double connectStarted = monotonicNow();
        bool sawOpen = false;
        bool proven = false;
        while (m_running.load()) {
            {
                std::unique_lock<std::mutex> lk(m_wake);
                m_wakeCv.wait_for(lk, std::chrono::seconds(1),
                                  [this] { return !m_running.load(); });
            }
            if (!m_running.load()) break;

            if (m_socketOpen.load()) {
                // Once per connection, after it has opened: the receiver has
                // just answered HTTP, so this is the best moment to retry a
                // description that failed. After the open rather than between
                // /connection and the socket, so a slow description cannot
                // delay the WebSocket past the session the handshake reserved.
                bool locationKnown;
                {
                    std::lock_guard<std::mutex> lk(m_mu);
                    locationKnown = m_snap.receiverLocation.valid;
                }
                if (!sawOpen && !locationKnown) fetchDescription();

                sawOpen = true;
                const double now = monotonicNow();
                // Proved itself: it has been delivering for long enough that
                // this was a real connection rather than an accept followed by
                // a refusal.
                if (!proven && now - connectStarted >= kProvenAfterSec) {
                    proven = true;
                    backoff = kBackoffMin;
                    consecutiveFailures = 0;
                }
                if (now - lastPing >= 30.0) {
                    ws->send("{\"type\":\"ping\"}");
                    lastPing = now;
                }

                // Re-probe the path. A session can run for hours and the
                // one-way delay measured at connect time is not a constant —
                // it moves with the route and with congestion, and it is the
                // largest correctable term in the budget. /health is the
                // cheapest thing the server serves.
                if (now - lastRttProbe >= 300.0) {
                    lastRttProbe = now;
                    probeRtt();
                }

                double silentFor;
                {
                    std::lock_guard<std::mutex> lk(m_mu);
                    silentFor = m_lastAudioAt > 0.0 ? now - m_lastAudioAt : now - m_linkSince;
                }
                if (silentFor > 10.0) {
                    LOG_WARN(tag, "no audio for %.0fs — reconnecting", silentFor);
                    break;
                }
            } else if (sawOpen) {
                break;   // it opened and has now closed
            } else if (monotonicNow() - connectStarted > 30.0) {
                LOG_WARN(tag, "connection did not open within 30s — retrying");
                break;
            }
        }

        ws->stop();
        {
            std::lock_guard<std::mutex> lk(m_wsMu);
            m_ws.reset();
        }
        ws.reset();
        m_socketOpen.store(false);
        resetStream("disconnected");

        if (!m_running.load()) break;

        {
            std::lock_guard<std::mutex> lk(m_mu);
            m_snap.link = LinkState::Backoff;
            m_linkSince = monotonicNow();
        }
        // A disconnection after a proven connection starts the backoff over:
        // it was working, so the next attempt deserves to be prompt. One that
        // never settled keeps climbing.
        if (!proven) ++consecutiveFailures;
        const double delay = backoffDelay();
        LOG_INFO(tag, "reconnecting in %.0fs%s", delay,
                 proven ? "" : " (this connection never settled)");
        {
            std::unique_lock<std::mutex> lk(m_wake);
            m_wakeCv.wait_for(lk, std::chrono::duration<double>(delay),
                              [this] { return !m_running.load(); });
        }
        backoff = std::min(backoff * 2.0, kBackoffMax);
    }

    {
        std::lock_guard<std::mutex> lk(m_mu);
        m_snap.link = LinkState::Stopped;
    }
    LOG_INFO(tag, "stopped");
}

bool Source::sessionHandshake(std::string& err) {
    // A fresh session id per connection attempt. The server binds a UUID to an
    // IP and remembers that it was kicked; carrying a rejected one into the
    // next attempt inherits the rejection.
    m_sessionId = makeUuidV4();

    json body;
    body["user_session_id"] = m_sessionId;
    if (!m_cfg.password.empty()) body["password"] = m_cfg.password;
    // Non-strict: a --password given on the command line is not validated as
    // UTF-8 the way one from the JSON config is, and the strict dump() throws
    // on invalid bytes — uncaught, on this thread, which terminates the daemon.
    // Replacing them sends a password the server will refuse, and says so.
    const std::string payload = body.dump(-1, ' ', false, json::error_handler_t::replace);

    std::string resp, curlErr;
    double rtt = 0.0;
    const long code = httpRequest(m_cfg.url + "/connection", &payload, kUserAgent,
                                  m_cfg.verifyTls, resp, rtt, curlErr);
    if (code < 0) { err = "POST /connection: " + curlErr; return false; }

    {
        std::lock_guard<std::mutex> lk(m_mu);
        recordRtt(rtt);
        updateDelayModel();
    }

    if (code != 200) {
        // The body carries the reason — "receiver full", "requires a password"
        // — and a log saying only "HTTP 403" sends someone reading packet
        // captures for something the server already explained.
        std::string reason;
        try {
            const json j = json::parse(resp);
            if (j.contains("reason")) reason = j["reason"].get<std::string>();
        } catch (const std::exception&) { /* not JSON; fall through */ }
        err = "POST /connection returned " + std::to_string(code);
        if (!reason.empty()) err += ": " + reason;
        return false;
    }

    try {
        const json j = json::parse(resp);
        if (j.contains("allowed") && !j["allowed"].get<bool>()) {
            err = "receiver refused the connection";
            if (j.contains("reason")) err += ": " + j["reason"].get<std::string>();
            return false;
        }
    } catch (const std::exception& e) {
        err = std::string("POST /connection returned unparseable JSON: ") + e.what();
        return false;
    }

    return true;
}

bool Source::fetchDescription() {
    // Informational, and the source of the receiver's coordinates — which is
    // what makes the propagation delay computable rather than guessed. A
    // failure here is not fatal: the delay model falls back to whatever was
    // configured, and the supervisor tries again on the next connection.
    std::string resp, err;
    double rtt = 0.0;
    const long code = httpRequest(m_cfg.url + "/api/description", nullptr, kUserAgent,
                                  m_cfg.verifyTls, resp, rtt, err);
    if (code != 200) {
        LOG_DEBUG(m_cfg.name.c_str(), "/api/description unavailable (%ld %s)", code, err.c_str());
        return false;
    }

    try {
        const json j = json::parse(resp);
        std::lock_guard<std::mutex> lk(m_mu);
        const bool hadLocation = m_snap.receiverLocation.valid;
        m_snap.receiverName.clear();   // rebuilt below; a retry must not append twice
        recordRtt(rtt);
        if (j.contains("receiver") && j["receiver"].is_object()) {
            const json& r = j["receiver"];
            if (r.contains("name") && r["name"].is_string())
                m_snap.receiverName = r["name"].get<std::string>();
            if (r.contains("callsign") && r["callsign"].is_string() && !r["callsign"].get<std::string>().empty()) {
                if (!m_snap.receiverName.empty()) m_snap.receiverName += " ";
                m_snap.receiverName += "(" + r["callsign"].get<std::string>() + ")";
            }
            if (r.contains("gps") && r["gps"].is_object()) {
                const json& g = r["gps"];
                if (g.contains("lat") && g.contains("lon") &&
                    g["lat"].is_number() && g["lon"].is_number()) {
                    const double lat = g["lat"].get<double>();
                    const double lon = g["lon"].get<double>();
                    // 0,0 is the Atlantic, and it is also what an unconfigured
                    // receiver reports. Treating it as a location would put
                    // every such source on a path it is not on.
                    if (std::abs(lat) > 0.01 || std::abs(lon) > 0.01) {
                        m_snap.receiverLocation = GeoPoint{lat, lon, true};
                    }
                }
            }
        }
        updateDelayModel();
        // Said at INFO the first time and when coordinates newly appear; a
        // receiver that publishes none is retried on every reconnection, and
        // repeating "no coordinates" each time would bury the log.
        const bool worthSaying = !m_haveDescription || (!hadLocation && m_snap.receiverLocation.valid);
        m_haveDescription = true;
        if (worthSaying) {
            LOG_INFO(m_cfg.name.c_str(), "receiver: %s%s, http rtt %.0f ms",
                     m_snap.receiverName.empty() ? "(unnamed)" : m_snap.receiverName.c_str(),
                     m_snap.receiverLocation.valid ? "" : ", no coordinates published",
                     m_snap.httpRttMs);
            if (m_snap.receiverLocation.valid) {
                LOG_INFO(m_cfg.name.c_str(), "path: %s", m_snap.pathDescription.c_str());
            }
        }
        return true;
    } catch (const std::exception& e) {
        LOG_DEBUG(m_cfg.name.c_str(), "/api/description unparseable: %s", e.what());
        return false;
    }
}

void Source::probeRtt() {
    std::string resp, err;
    double rtt = 0.0;
    const long code = httpRequest(m_cfg.url + "/health", nullptr, kUserAgent,
                                  m_cfg.verifyTls, resp, rtt, err);
    if (code < 0) {
        LOG_DEBUG(m_cfg.name.c_str(), "rtt probe failed: %s", err.c_str());
        return;
    }
    std::lock_guard<std::mutex> lk(m_mu);
    const double before = m_snap.httpRttMs;
    recordRtt(rtt);
    updateDelayModel();
    if (std::abs(m_snap.httpRttMs - before) > 5.0) {
        LOG_INFO(m_cfg.name.c_str(), "network round trip now %.0f ms (was %.0f); "
                 "one-way delay term %.1f ms",
                 m_snap.httpRttMs, before, m_snap.networkSec * 1000.0);
    }
}

std::string Source::buildWsUrl() const {
    std::ostringstream u;
    u << httpToWs(m_cfg.url) << "/ws"
      << "?frequency=" << m_cfg.dialHz
      << "&mode=usb"
      // The passband has to reach 2.2 kHz or the WWV/WWVH second tick — which
      // is recovered entirely from its 2000/2200 Hz audio image — is filtered
      // away, and the decoder sits in `acquiring` for ever with nothing to say
      // why. 0-3000 is what the frontend's own clock panel asks for.
      << "&bandwidthLow=0&bandwidthHigh=3000"
      << "&format=" << (m_cfg.format == AudioFormat::Opus ? "opus" : "pcm-zstd")
      // Version 4 for both: it is the only version whose Opus frames carry a
      // self-describing header, and the lossless path at version 4 is the
      // predictive codec rather than anything zstd. The query parameter is
      // still spelt "pcm-zstd" for compatibility with older servers.
      << "&version=4"
      << "&user_session_id=" << m_sessionId;
    if (!m_cfg.password.empty()) {
        // Passwords here are the receiver's bypass password, not a secret of
        // ours, and the server takes it in the query string. Percent-encode it
        // so one containing & or = does not silently truncate the URL.
        char* esc = curl_easy_escape(nullptr, m_cfg.password.c_str(),
                                     static_cast<int>(m_cfg.password.size()));
        if (esc) { u << "&password=" << esc; curl_free(esc); }
    }
    return u.str();
}

// ---------------------------------------------------------------------------
// WebSocket events

void Source::onOpen() {
    m_socketOpen.store(true);
    {
        std::lock_guard<std::mutex> lk(m_mu);
        m_snap.link = LinkState::Streaming;
        m_snap.linkDetail.clear();
        m_linkSince = monotonicNow();
        m_lastAudioAt = 0.0;
    }
    LOG_INFO(m_cfg.name.c_str(), "connected");
    std::shared_ptr<ix::WebSocket> ws;
    {
        std::lock_guard<std::mutex> lk(m_wsMu);
        ws = m_ws;
    }
    if (ws) ws->send("{\"type\":\"get_status\"}");
}

void Source::onClose(const std::string& reason) {
    const bool wasOpen = m_socketOpen.exchange(false);
    {
        std::lock_guard<std::mutex> lk(m_mu);
        m_snap.linkDetail = reason;
    }
    if (wasOpen) LOG_WARN(m_cfg.name.c_str(), "disconnected: %s", reason.c_str());
    else LOG_DEBUG(m_cfg.name.c_str(), "connect failed: %s", reason.c_str());
    m_wakeCv.notify_all();
}

void Source::onText(const std::string& msg) {
    try {
        const json j = json::parse(msg);
        const std::string type = j.value("type", "");
        if (type == "error") {
            const std::string e = j.value("error", "unknown error");
            LOG_WARN(m_cfg.name.c_str(), "server error: %s", e.c_str());
            std::lock_guard<std::mutex> lk(m_mu);
            m_snap.linkDetail = e;
        } else if (type == "status") {
            if (j.contains("sampleRate") && j["sampleRate"].is_number_integer()) {
                LOG_DEBUG(m_cfg.name.c_str(), "status: %d Hz, mode %s, %.6f MHz",
                          j["sampleRate"].get<int>(), j.value("mode", "?").c_str(),
                          j.value("frequency", 0ULL) / 1e6);
            }
        }
    } catch (const std::exception&) {
        // Not JSON. The server sends only JSON on text frames, so this is a
        // truncation or a proxy injecting something; nothing to act on.
    }
}

void Source::onBinary(const std::string& msg) {
    // The arrival timestamp is taken FIRST, before any parsing or decoding.
    // Everything below — header parsing, Opus, the whole DSP chain — happens
    // after this read, so none of it can add to the number.
    //
    // On CLOCK_MONOTONIC, and converted to REALTIME only where an offset is
    // formed; SampleClock.h has why.
    const double arrival = monotonicNow();

    // A step of the host clock. The fit is immune, but every offset already in
    // the window was measured against the clock as it was, and is now wrong by
    // exactly the step: the median of them would go on serving the old error
    // for up to two minutes. ntpd's slew is at most 500 ppm, ten microseconds
    // between packets, so 5 ms can only be a step.
    const double rtMinusMono = realtimeMinusMonotonic();
    if (m_haveRtMinusMono && std::abs(rtMinusMono - m_lastRtMinusMono) > 0.005) {
        LOG_WARN(m_cfg.name.c_str(), "host clock stepped by %+.1f ms; discarding offsets "
                 "measured against the old clock", (rtMinusMono - m_lastRtMinusMono) * 1000.0);
        std::lock_guard<std::mutex> lk(m_mu);
        m_offsets.clear();
        recomputeOffset();
    }
    m_lastRtMinusMono = rtMinusMono;
    m_haveRtMinusMono = true;

    handleAudio(reinterpret_cast<const std::uint8_t*>(msg.data()), msg.size(), arrival);
}

// ---------------------------------------------------------------------------
// Audio

void Source::handleAudio(const std::uint8_t* pkt, std::size_t len, double arrivalSec) {
    {
        std::lock_guard<std::mutex> lk(m_mu);
        m_snap.packets++;
        m_snap.audioBytes += len;
        m_lastAudioAt = monotonicNow();
    }

    // A packet that cannot be decoded still happened, and still occupied its
    // stretch of the stream. Dropping it without advancing the sample count
    // slips every later sample index 20 ms earlier than the instant it was
    // captured — a slip the SampleClock fit then averages in for five minutes.
    // So a lost packet is replaced by something of the same length: Opus's own
    // concealment where there is a decoder to ask, silence otherwise. Only once
    // a timeline exists — before the first good packet there is nothing to
    // keep in step.
    auto concealLost = [this](int frameSamples) {
        if (m_decoderRate <= 0 || m_samplesWritten <= 0) return;
        const int n = frameSamples > 0 ? frameSamples : m_lastFrameSamples;
        if (n <= 0) return;
        if (m_opus && m_opusRate == m_decoderRate && n <= static_cast<int>(m_opusPcm.size())) {
            const int got = opus_decode(m_opus, nullptr, 0, m_opusPcm.data(), n, 0);
            if (got > 0) {
                feedSamples(m_opusPcm.data(), got, m_decoderRate, 0.0, false);
                return;
            }
        }
        feedSilence(n, m_decoderRate);
    };

    // A session that negotiated Opus still receives lossless frames the moment
    // it tunes to an IQ mode, so the frame itself has to say which it is. The
    // four-byte "PCM4" magic exists for exactly this; an Opus frame has no
    // magic, which is why the test is this way round.
    if (ubersdr::PCMv4StreamDecoder::isV4Frame(pkt, len)) {
        if (!m_pcmv4) m_pcmv4 = std::make_unique<PcmV4Reader>();
        ubersdr::PCMv4Header h;
        std::string err;
        if (!m_pcmv4->dec.decode(pkt, len, h, err)) {
            {
                std::lock_guard<std::mutex> lk(m_mu);
                if (m_snap.decodeErrors++ % 200 == 0)
                    LOG_WARN(m_cfg.name.c_str(), "pcm v4 decode: %s", err.c_str());
            }
            // The header is parsed before the body, so a sample count means
            // the header survived and its timestamp can still be trusted. No
            // count means the header itself failed, and the next timestamp is
            // not to be read as a gap (see trackTimeline).
            if (h.sampleCount > 0 && h.channels == 1 && h.sampleRate == m_decoderRate) {
                trackTimeline(h.timestampNanos, h.sampleRate, h.sampleCount);
                concealLost(h.sampleCount);
            } else {
                m_tsHave = false;
                concealLost(0);
            }
            return;
        }
        {
            std::lock_guard<std::mutex> lk(m_mu);
            m_snap.basebandPowerDb = h.basebandPower;
            m_snap.noiseDb = h.noise;
            m_feedingOpus = 0;
        }
        if (h.channels != 1) {
            // Two channels means an IQ mode, which this is never tuned to.
            // Feeding interleaved I/Q to a decoder expecting mono would look
            // like a signal and decode to nothing.
            return;
        }
        // ensureDecoder before the tracker, as on the Opus path: a rate change
        // restarts the sample count the tracker is measuring against.
        if (!ensureDecoder(h.sampleRate)) return;
        trackTimeline(h.timestampNanos, h.sampleRate, h.sampleCount);
        feedSamples(m_pcmv4->dec.samples(), h.sampleCount, h.sampleRate, arrivalSec);
        return;
    }

    if (ubersdr::PCMv4StreamDecoder::isZstdFrame(pkt, len)) {
        std::lock_guard<std::mutex> lk(m_mu);
        if (m_snap.decodeErrors++ == 0) {
            LOG_ERROR(m_cfg.name.c_str(),
                      "server sent a version 1-3 PCM frame: it is older than 0.1.63 and "
                      "cannot serve protocol version 4");
        }
        return;
    }

    // Opus, then.
    ubersdr::PCMv4Header h;
    std::size_t off = 0;
    std::string err;
    if (!m_opusHeader.decode(pkt, len, h, off, err)) {
        {
            std::lock_guard<std::mutex> lk(m_mu);
            if (m_snap.decodeErrors++ % 200 == 0)
                LOG_WARN(m_cfg.name.c_str(), "opus header: %s", err.c_str());
        }
        // Every delta after this is relative to one never applied, so the
        // timestamps are off by this packet's delta until the next full one.
        m_tsHave = false;
        m_tsWaitResync = true;
        concealLost(0);
        return;
    }
    {
        std::lock_guard<std::mutex> lk(m_mu);
        m_snap.basebandPowerDb = h.basebandPower;
        m_snap.noiseDb = h.noise;
        m_feedingOpus = 1;
    }
    if (h.channels != 1) return;
    if (m_tsWaitResync && m_opusHeader.lastWasResync()) m_tsWaitResync = false;

    // No minimum body length. A one-byte packet is a bare TOC and a valid Opus
    // frame of the length it declares, and a zero-byte body asks the decoder
    // for concealment; both occupy their 20 ms of the stream, and dropping them
    // was a timeline slip.
    const std::size_t bodyLen = len - off;

    if (!m_opus || m_opusRate != h.sampleRate) {
        if (m_opus) opus_decoder_destroy(m_opus);
        int oerr = 0;
        m_opus = opus_decoder_create(h.sampleRate, 1, &oerr);
        if (oerr != OPUS_OK || !m_opus) {
            m_opus = nullptr;
            LOG_ERROR(m_cfg.name.c_str(), "opus_decoder_create(%d Hz): %s",
                      h.sampleRate, opus_strerror(oerr));
            return;
        }
        m_opusRate = h.sampleRate;
        // 120 ms is the longest frame Opus can carry. Sized once against the
        // rate rather than per packet.
        m_opusPcm.resize(static_cast<std::size_t>(h.sampleRate) * 120 / 1000 + 16);
        LOG_INFO(m_cfg.name.c_str(), "opus decoder: %d Hz mono", h.sampleRate);
    }

    // The packet's own declared length, when it declares one, so a lost frame
    // is concealed for as long as it actually was.
    int declared = bodyLen > 0
        ? opus_packet_get_nb_samples(pkt + off, static_cast<opus_int32>(bodyLen), h.sampleRate)
        : 0;
    if (declared <= 0 || declared > static_cast<int>(m_opusPcm.size())) declared = 0;

    // ensureDecoder first, for the timeline: a rate change restarts the sample
    // count, and the gap tracker must see the count it is about to extend.
    if (!ensureDecoder(h.sampleRate)) return;
    if (!m_tsWaitResync) {
        trackTimeline(h.timestampNanos, h.sampleRate,
                      declared > 0 ? declared : m_lastFrameSamples);
    }

    int n;
    if (bodyLen == 0) {
        const int frame = m_lastFrameSamples > 0 ? m_lastFrameSamples : h.sampleRate / 50;
        n = opus_decode(m_opus, nullptr, 0, m_opusPcm.data(), frame, 0);
    } else {
        n = opus_decode(m_opus, pkt + off, static_cast<opus_int32>(bodyLen),
                        m_opusPcm.data(), static_cast<int>(m_opusPcm.size()), 0);
    }
    if (n < 0) {
        {
            std::lock_guard<std::mutex> lk(m_mu);
            if (m_snap.decodeErrors++ % 200 == 0)
                LOG_WARN(m_cfg.name.c_str(), "opus_decode: %s", opus_strerror(n));
        }
        concealLost(declared);
        return;
    }
    feedSamples(m_opusPcm.data(), n, h.sampleRate, arrivalSec);
}

void Source::feedSilence(int count, int rate) {
    // In pieces of at most a second, so a long gap does not allocate its whole
    // length and the decoders see blocks the size they are used to.
    const int chunk = std::max(1, rate);
    if (m_silence.size() < static_cast<std::size_t>(chunk)) m_silence.assign(chunk, 0);
    while (count > 0) {
        const int n = std::min(count, chunk);
        feedSamples(m_silence.data(), n, rate, 0.0, false);
        count -= n;
    }
}

// Keeps m_samplesWritten in step with the stream as the server sent it.
//
// Every frame carries the server's time for its first sample, taken when radiod
// delivered it. `stamp - samples/rate` is then constant for as long as nothing
// goes missing between radiod and this decoder, and steps up by exactly the
// missing duration when something does. Things do go missing where nothing here
// can see them: the server drops a frame when a session's queue is full, drops
// any that is not exactly 20 ms, skips one its encoder refuses, and radiod's
// multicast is UDP. A lost frame that is not replaced slips every later sample
// index one frame early against when it was captured.
//
// The stamp is the server's receive time, so it carries the server's scheduling
// jitter: a Go collector pause makes a run of frames look late and then catch
// up. That delay, like the network's, is bounded below and not above, so the
// test is on the MINIMUM over one-second blocks: a late burst does not move a
// block's minimum, a genuine loss moves every later one. A detected loss is
// filled with silence up to a second after it happened — a second edge in that
// second can be one frame out, which is one sample the offset median discards.
//
// A step down, or a step up too large to be a loss, is something the samples
// cannot be made consistent with: the server's clock was stepped, or a filled
// gap was not one. Then the sample-to-host mapping and anything derived from it
// is dropped and rebuilt rather than trusted; the decoder keeps its lock.
void Source::trackTimeline(std::uint64_t timestampNanos, int rate, int frameSamples) {
    if (rate <= 0 || rate != m_decoderRate) { m_tsHave = false; return; }
    if (frameSamples > 0) m_lastFrameSamples = frameSamples;
    if (m_lastFrameSamples <= 0) m_lastFrameSamples = rate / 50;

    constexpr double kBlockSec = 1.0;
    constexpr double kMaxFillSec = 1.5;

    if (!m_tsHave) {
        m_tsHave = true;
        m_tsOriginNanos = timestampNanos;
        m_tsOriginSample = m_samplesWritten;
        m_tsBaseline = 0.0;
        m_tsBlockHave = false;
    }

    // Relative to an origin so the doubles hold nanoseconds, not 1.7e9 seconds.
    auto driftOf = [&](std::uint64_t ts) {
        const double stamp = static_cast<double>(static_cast<std::int64_t>(ts - m_tsOriginNanos)) * 1e-9;
        return stamp - static_cast<double>(m_samplesWritten - m_tsOriginSample) / rate;
    };

    if (m_tsBlockHave &&
        static_cast<std::int64_t>(timestampNanos - m_tsBlockStartNanos) >= static_cast<std::int64_t>(kBlockSec * 1e9)) {
        const double step = m_tsBlockMin - m_tsBaseline;
        const double frameSec = static_cast<double>(m_lastFrameSamples) / rate;
        if (step > 0.5 * frameSec && step <= kMaxFillSec) {
            // Whole frames: the server only ever loses whole ones, and rounding
            // to them keeps the estimate's jitter out of the fill.
            const long long frames = std::llround(step / frameSec);
            const int fill = static_cast<int>(frames * m_lastFrameSamples);
            if (fill > 0) {
                if (m_gapFills++ % 50 == 0) {
                    LOG_INFO(m_cfg.name.c_str(), "stream lost %.0f ms of audio upstream; "
                             "filled to keep the sample timeline (%llu fills so far)",
                             fill * 1000.0 / rate, static_cast<unsigned long long>(m_gapFills));
                }
                feedSilence(fill, rate);
            }
            m_tsBaseline = m_tsBlockMin - static_cast<double>(fill) / rate;
        } else if (step > kMaxFillSec || step < -0.5 * frameSec) {
            LOG_WARN(m_cfg.name.c_str(), "stream timestamps stepped %+.0f ms against the sample "
                     "count; rebuilding the sample clock", step * 1000.0);
            discardTiming("stream timestamp discontinuity");
            m_tsBaseline = m_tsBlockMin;
        } else {
            // Within half a frame: the ordinary wander of the minimum and the
            // slow drift between radiod's sample clock and the server's.
            m_tsBaseline = m_tsBlockMin;
        }
        m_tsBlockHave = false;
    }

    const double d = driftOf(timestampNanos);
    if (!m_tsBlockHave) {
        m_tsBlockHave = true;
        m_tsBlockStartNanos = timestampNanos;
        m_tsBlockMin = d;
    } else {
        m_tsBlockMin = std::min(m_tsBlockMin, d);
    }
}

void Source::discardTiming(const char* why) {
    // The decoder's own sample indices stay consistent with each other, so its
    // lock survives; what cannot be trusted is their mapping to host time, and
    // the UTC anchor that was composed through it.
    m_clock.reset();
    std::lock_guard<std::mutex> lk(m_mu);
    m_haveAnchor = false;
    m_offsets.clear();
    m_snap.haveOffset = false;
    m_snap.offsetSamples = 0;
    m_snap.clockFitValid = false;
    LOG_DEBUG(m_cfg.name.c_str(), "timing discarded (%s)", why);
}

bool Source::ensureDecoder(int rate) {
    if (m_decoderRate == rate && (m_wwv || m_wwvb)) return true;
    if (m_decoderRate == rate && m_rateRefused) return false;   // said once, not per packet

    // The decoders decimate to a fixed series rate — 200 Hz for WWV/WWVH,
    // 100 Hz for WWVB — so a rate that is not a multiple of it decimates
    // unevenly and drifts, and the result is a decoder that simply never locks.
    // Every UberSDR audio mode clears this (12000 and 24000 both divide by
    // 200); refusing loudly beats decoding wrongly.
    if (rate <= 0 || rate % 200 != 0) {
        LOG_ERROR(m_cfg.name.c_str(),
                  "sample rate %d Hz is not a multiple of 200 Hz and cannot be decimated evenly",
                  rate);
        m_decoderRate = rate;
        m_rateRefused = true;
        m_wwv.reset();
        m_wwvb.reset();
        m_samplesWritten = 0;
        m_tsHave = false;
        m_clock.reset();
        // Everything the old decoder left behind goes with it. A source that
        // was locked at 12 kHz and is now fed an unusable rate would otherwise
        // go on reporting "locked" and a current offset while nothing is being
        // decoded at all — until staleness caught it three minutes later.
        std::lock_guard<std::mutex> lk(m_mu);
        m_haveAnchor = false;
        m_haveFrame = false;
        m_offsets.clear();
        m_leapFrames = 0;
        m_snap.leapPending = false;
        m_snap.haveOffset = false;
        m_snap.offsetSamples = 0;
        m_snap.clockFitValid = false;
        m_snap.sampleRate = rate;
        m_snap.clockState = "stopped";
        return false;
    }
    m_rateRefused = false;

    if (m_decoderRate != 0) {
        LOG_WARN(m_cfg.name.c_str(), "sample rate changed %d -> %d Hz; restarting the decoder "
                 "(a lock takes about four minutes of clean signal to regain)",
                 m_decoderRate, rate);
    }

    m_wwv.reset();
    m_wwvb.reset();
    m_decoderRate = rate;
    m_samplesWritten = 0;
    m_lastFrameSamples = rate / 50;
    m_tsHave = false;
    m_clock.setSampleRate(rate);
    m_clock.reset();
    {
        std::lock_guard<std::mutex> lk(m_mu);
        m_haveAnchor = false;
        m_haveFrame = false;
        m_offsets.clear();
        m_leapFrames = 0;
        m_snap.leapPending = false;
        m_snap.sampleRate = rate;
        // The decoders start in nosignal and onStateChanged only fires on a
        // CHANGE, so without this the snapshot sits at "stopped" while audio is
        // flowing — and the acquisition funnel reads that as "no audio", which
        // sends someone looking at the network for a problem that is on the band.
        m_snap.clockState = "nosignal";
    }

    // WWV/WWVH and WWVB are genuinely different decoders — a 100 Hz BCD
    // subcarrier against pulse-width modulation on the carrier's own amplitude
    // — so this is decided from the dial, not offered as a setting. WWV and
    // WWVH share one decoder and it identifies which it is hearing itself.
    const bool wwvb = m_cfg.dialHz < kWwvbCeilingHz;

    // The plausibility gate. A deep fade zero-biases the same bits in every
    // frame of the voter's window, so the misread is unanimous and no
    // agreement-based metric can catch it; an independent reference clock is
    // the only thing that can veto it. The host clock is that reference — which
    // is circular only in appearance, because the bound is a day and the error
    // being measured is milliseconds.
    auto reference = [] { return hostNowFields(static_cast<long long>(realtimeNow() * 1000.0)); };

    if (wwvb) {
        m_wwvb = std::make_unique<clockdec::WwvbDecoder>(rate);
        m_wwvb->onStateChanged = [this](clockdec::ClockLockState s) { onClockState(s); };
        m_wwvb->onSecond = [this](const clockdec::ClockSecondInfo& i) { onClockSecond(i); };
        m_wwvb->onFrame = [this](const clockdec::ClockFrameInfo& f) { onClockFrame(f); };
        m_wwvb->onTime = [this](const clockdec::ClockTimeInfo& t) { onClockTime(t); };
        m_wwvb->setPlausibility(reference, 24 * 60);
    } else {
        m_wwv = std::make_unique<clockdec::WwvDecoder>(rate);
        m_wwv->onStateChanged = [this](clockdec::ClockLockState s) { onClockState(s); };
        m_wwv->onSecond = [this](const clockdec::ClockSecondInfo& i) { onClockSecond(i); };
        m_wwv->onFrame = [this](const clockdec::ClockFrameInfo& f) { onClockFrame(f); };
        m_wwv->onTime = [this](const clockdec::ClockTimeInfo& t) { onClockTime(t); };
        m_wwv->setPlausibility(reference, 24 * 60);
    }

    LOG_INFO(m_cfg.name.c_str(), "decoder started: %s at %d Hz", wwvb ? "WWVB" : "WWV/WWVH", rate);
    return true;
}

void Source::feedSamples(const std::int16_t* pcm, int count, int rate, double arrivalSec,
                         bool observeArrival) {
    if (!pcm || count <= 0) return;
    if (!ensureDecoder(rate)) return;

    m_samplesWritten += count;

    // The packet's arrival is taken as the time of its LAST sample: the audio
    // was captured before it was sent, so the end of the block is the edge
    // closer to the moment it landed here. Synthesised samples arrived at no
    // particular time and are not observed; the next real packet's arrival is
    // observed against a count that includes them, which is the point.
    if (observeArrival) m_clock.observe(m_samplesWritten, arrivalSec);

    if (m_mono.size() < static_cast<std::size_t>(count)) m_mono.resize(count);
    for (int i = 0; i < count; ++i) m_mono[i] = pcm[i] * (1.0f / 32768.0f);

    if (m_wwv) m_wwv->process(m_mono.data(), static_cast<std::size_t>(count));
    else m_wwvb->process(m_mono.data(), static_cast<std::size_t>(count));

    // Diagnostics are assembled on call from state the decoder already holds,
    // so this costs nothing on the sample path and keeps the status report
    // current without a second timer reaching into the decoder.
    const auto d = m_wwv ? m_wwv->diagnostics() : m_wwvb->diagnostics();
    const auto st = m_wwv ? m_wwv->station() : m_wwvb->station();
    const auto consumed = m_wwv ? m_wwv->samplesConsumed() : m_wwvb->samplesConsumed();

    std::lock_guard<std::mutex> lk(m_mu);
    m_snap.toneSnrDb = d.toneSnrDb;
    m_snap.tickBandRatioDb = d.tickBandRatioDb;
    m_snap.toneDetected = d.toneDetected;
    m_snap.phaseLocked = d.phaseLocked;
    m_snap.delayEstMs = d.delayEstMs;
    m_snap.anchored = d.anchored;
    m_snap.badFrameStreak = d.badFrameStreak;
    m_snap.framesInWindow = d.framesInWindow;
    m_snap.windowSize = d.windowSize;
    m_snap.voteQuality = d.voteQuality;
    m_snap.samplesConsumed = consumed;
    switch (static_cast<clockdec::ClockLockRefusal>(d.refusalReason)) {
        case clockdec::ClockLockRefusal::QualityFloor: m_snap.refusal = "quality_floor"; break;
        case clockdec::ClockLockRefusal::Plausibility: m_snap.refusal = "plausibility"; break;
        case clockdec::ClockLockRefusal::Staleness:    m_snap.refusal = "staleness"; break;
        case clockdec::ClockLockRefusal::Contested:    m_snap.refusal = "contested"; break;
        default: m_snap.refusal = "none"; break;
    }
    switch (st) {
        case clockdec::ClockStation::Wwv:  m_snap.station = "wwv"; break;
        case clockdec::ClockStation::Wwvh: m_snap.station = "wwvh"; break;
        case clockdec::ClockStation::Wwvb: m_snap.station = "wwvb"; break;
        default: m_snap.station = "unknown"; break;
    }

    const ClockFit f = m_clock.fit();
    m_snap.clockFitValid = f.valid;
    m_snap.clockResidualSec = f.residualRms;
    m_snap.clockSpanSec = f.spanSec;
    m_snap.clockPpm = f.valid && m_decoderRate > 0
        ? (f.secPerSample * m_decoderRate - 1.0) * 1e6 : 0.0;
    m_snap.lastExcessDelaySec = m_clock.lastExcessDelay();

    updateDelayModel();
    recomputeOffset();
}

void Source::resetStream(const char* why) {
    m_clock.reset();
    m_opusHeader.reset();
    if (m_pcmv4) m_pcmv4->dec.reset();
    // The Opus decoder's prediction state and its lookahead buffer belong to
    // the last connection's stream. Carried over, the first frames of the new
    // one are reconstructed against audio from before the gap.
    if (m_opus) opus_decoder_ctl(m_opus, OPUS_RESET_STATE);
    m_samplesWritten = 0;
    m_decoderRate = 0;
    m_rateRefused = false;
    m_lastFrameSamples = 0;
    m_tsHave = false;
    m_tsWaitResync = false;
    m_haveRtMinusMono = false;
    m_wwv.reset();
    m_wwvb.reset();

    std::lock_guard<std::mutex> lk(m_mu);
    m_haveAnchor = false;
    m_haveFrame = false;
    m_offsets.clear();
    m_leapFrames = 0;
    m_snap.leapPending = false;
    m_feedingOpus = -1;
    m_snap.haveOffset = false;
    m_snap.offsetSamples = 0;
    m_snap.clockState = "stopped";
    m_snap.clockFitValid = false;
    m_snap.samplesConsumed = 0;
    LOG_DEBUG(m_cfg.name.c_str(), "stream reset (%s)", why);
}

// ---------------------------------------------------------------------------
// Decoder callbacks

void Source::onClockState(clockdec::ClockLockState s) {
    const char* name = "nosignal";
    switch (s) {
        case clockdec::ClockLockState::Locked:    name = "locked"; break;
        case clockdec::ClockLockState::Acquiring: name = "acquiring"; break;
        default: break;
    }
    {
        std::lock_guard<std::mutex> lk(m_mu);
        m_snap.clockState = name;
        if (s != clockdec::ClockLockState::Locked) {
            // An anchor that outlived its lock would keep extending one second
            // at a time through a signal the decoder no longer trusts, quietly
            // manufacturing offsets from nothing.
            m_haveAnchor = false;
        }
    }
    LOG_INFO(m_cfg.name.c_str(), "state -> %s", name);
}

void Source::onClockFrame(const clockdec::ClockFrameInfo& f) {
    std::lock_guard<std::mutex> lk(m_mu);
    // Recorded whether or not anything is emitted: onClockTime composes its
    // timestamp against this frame's second 0, and a `time` event that arrived
    // before any frame would be composed against sample 0 and land minutes out.
    m_frameStartSample = f.frameStartSample;
    m_haveFrame = true;
    m_snap.dut1Tenths = f.dut1Tenths;

    // Three consecutive frames before the warning is believed, and one frame
    // without it to drop it. The bit is a single BCD position decoded afresh
    // each minute, and a fade that flips it would otherwise put LI=1 on every
    // reply for that minute — telling clients to insert a second that is not
    // coming. WWV asserts the real warning for the whole month, so three
    // minutes of latency costs nothing.
    constexpr int kLeapConfirmFrames = 3;
    m_leapFrames = f.leapPending ? std::min(m_leapFrames + 1, kLeapConfirmFrames) : 0;
    m_snap.leapPending = m_leapFrames >= kLeapConfirmFrames;
}

namespace {

// The next UTC midnight after `ms`, when `ms` falls on the last day of a month,
// or -1. A leap second is inserted as 23:59:60 immediately before exactly such
// a midnight, and nowhere else.
long long leapBoundaryAfter(long long ms) {
    if (!isLastDayOfMonth(ms)) return -1;
    return (floorDiv(floorDiv(ms, 1000), 86400) + 1) * 86400LL * 1000LL;
}

} // namespace

void Source::onClockTime(const clockdec::ClockTimeInfo& t) {
    if (t.year2 < 0 || t.doy < 1 || t.hour < 0 || t.minute < 0) return;

    // Composed exactly as the reference front end does: the voted frame's
    // second 0, plus whole seconds to the edge this timestamp is anchored to.
    // The voted frame is the one the minute/hour/doy fields describe, so the
    // elapsed count is measured from its own start.
    const long long baseMs = utcMsFromFields(t.year2, t.doy, t.hour, t.minute);

    std::lock_guard<std::mutex> lk(m_mu);
    if (!m_haveFrame || m_decoderRate <= 0) return;

    // Elapsed whole seconds from the voted frame's second 0 to the edge this
    // timestamp is anchored to. Not lastEdgeSecondOfFrame, which can point into
    // a frame LATER than the one that was voted.
    const long long elapsedSec = std::llround(
        static_cast<double>(t.lastEdgeSample - m_frameStartSample) / m_decoderRate);
    const long long decodedMs = baseMs + elapsedSec * 1000LL;

    // Counting whole seconds from a frame that began before a possible leap
    // second to an edge after it would be a second out if one was inserted.
    // Such an anchor is refused; the next frame begins after the boundary.
    const long long boundary = leapBoundaryAfter(baseMs);
    if (boundary > 0 && decodedMs >= boundary) {
        m_haveAnchor = false;
        return;
    }

    m_anchorEdgeSample = t.lastEdgeSample;
    m_anchorUtcMs = decodedMs;
    m_haveAnchor = true;
    m_anchorSetAt = monotonicNow();
    m_lastTimeAt = m_anchorSetAt;

    m_snap.lastQuality = std::clamp(static_cast<int>(std::lround(t.quality * 100.0)), 0, 100);
    m_snap.lastDecodedUtc = iso8601(decodedMs);
}

void Source::onClockSecond(const clockdec::ClockSecondInfo& i) {
    // Only an edge the decoder actually measured becomes an offset sample.
    // Second 0 carries no pulse, and a weak or coasted second reports where the
    // tracker expects the edge rather than where it was heard: correct, but it
    // is the tracker's opinion counted again, and it would weigh the median
    // towards the estimate instead of the signal.
    if (!i.edgeMeasured) return;

    // The fit is on CLOCK_MONOTONIC; the offset is against CLOCK_REALTIME as it
    // stands NOW, which is what the served time is formed from.
    double hostMono = 0.0;
    if (!m_clock.hostTimeAt(i.edgeSample, hostMono)) return;
    const double hostSec = hostMono + realtimeMinusMonotonic();

    std::lock_guard<std::mutex> lk(m_mu);
    if (!m_haveAnchor) return;
    if (i.edgeSample < m_anchorEdgeSample) return;
    if (monotonicNow() - m_anchorSetAt > kAnchorMaxAgeSec) {
        m_haveAnchor = false;
        return;
    }
    if (m_decoderRate <= 0) return;

    // Extend the anchor: every second edge after the voted one is exactly that
    // many whole seconds later. Rounding is safe because the edges are a second
    // apart by construction and the sample clock is good to milliseconds.
    const double elapsed = static_cast<double>(i.edgeSample - m_anchorEdgeSample) / m_decoderRate;
    const long long wholeSeconds = std::llround(elapsed);
    // If the edge is not within a quarter-second of a whole number of seconds
    // from the anchor, something has resynchronised and the extension is not
    // sound.
    if (std::abs(elapsed - static_cast<double>(wholeSeconds)) > 0.25) return;

    // Every minute is sixty seconds to this count, and the minute before a leap
    // second is sixty-one: past the boundary each extended edge would be dated
    // a second late. Rather than trust a decoded warning bit to say whether
    // this month has one, the anchor is dropped at EVERY month-end midnight —
    // it costs one minute of offsets twelve times a year, and the next `time`
    // event re-anchors on a frame that knows what minute it is. Offsets taken
    // before the boundary were right and are kept. (The 23:59:60 edge itself
    // counts as 00:00:00 here, so it is refused too.)
    const long long edgeMs = m_anchorUtcMs + wholeSeconds * 1000LL;
    const long long boundary = leapBoundaryAfter(m_anchorUtcMs);
    if (boundary > 0 && edgeMs >= boundary) {
        if (m_snap.leapPending) {
            LOG_INFO(m_cfg.name.c_str(), "leap second boundary: dropping the UTC anchor until re-anchored");
        }
        m_haveAnchor = false;
        return;
    }

    const double utcSec = (static_cast<double>(m_anchorUtcMs) / 1000.0) +
                          static_cast<double>(wholeSeconds);

    // offset = the correction to ADD to this host's clock to reach true UTC.
    //
    // hostSec is when the sample carrying this edge was OBSERVED here, so it
    // already contains the whole one-way delay; utcSec is when the edge was
    // TRANSMITTED. Adding the delay back is what makes the difference an offset
    // rather than a measurement of the path.
    const double raw = utcSec - hostSec;
    addOffsetSample(raw + m_snap.delaySec, realtimeNow());
    recomputeOffset();
}

void Source::addOffsetSample(double offsetSec, double atRealtime) {
    m_offsets.push_back({atRealtime, offsetSec});
    while (!m_offsets.empty() && atRealtime - m_offsets.front().at > kOffsetWindowSec) {
        m_offsets.pop_front();
    }
}

void Source::recomputeOffset() {
    if (m_offsets.empty()) {
        m_snap.haveOffset = false;
        m_snap.offsetSamples = 0;
        return;
    }

    const double now = realtimeNow();
    m_snap.offsetAgeSec = now - m_offsets.back().at;
    m_snap.offsetSamples = static_cast<int>(m_offsets.size());

    if (m_snap.offsetAgeSec > kOffsetStaleSec) {
        m_snap.haveOffset = false;
        return;
    }

    // The median, not the mean. A single second edge landing on a fade, or one
    // packet arriving after a stall, produces an outlier of tens of
    // milliseconds; the mean carries it and the median does not. With a hundred
    // or so samples in the window the efficiency cost against a clean mean is
    // irrelevant next to that.
    std::vector<double> v;
    v.reserve(m_offsets.size());
    for (const OffsetSample& s : m_offsets) v.push_back(s.offset);
    std::sort(v.begin(), v.end());
    const std::size_t mid = v.size() / 2;
    const double median = (v.size() % 2) ? v[mid] : 0.5 * (v[mid - 1] + v[mid]);

    // Spread as the median absolute deviation, scaled to be comparable with a
    // standard deviation on normal data. Robust for the same reason the median
    // is: an RMS about the median would be dominated by the outliers it is
    // meant to describe the absence of.
    std::vector<double> dev;
    dev.reserve(v.size());
    for (double x : v) dev.push_back(std::abs(x - median));
    std::sort(dev.begin(), dev.end());
    const double mad = dev.empty() ? 0.0 : dev[dev.size() / 2];
    const double jitter = 1.4826 * mad;

    m_snap.offsetSec = median;
    m_snap.jitterSec = jitter;
    // The raw figure shown is the FILTERED one, recovered by undoing the delay
    // model. Reporting the last unfiltered sample instead put a 15 ms outlier
    // next to a filtered offset, which reads as the delay model being
    // inconsistent rather than as the one noisy second edge it actually was.
    // Individual edges do scatter by a few series samples; that is what the
    // median is here to absorb, and what `jitter` is here to report.
    m_snap.rawOffsetSec = median - m_snap.delaySec;

    // What this source claims to be worth. Three independent terms:
    //   jitter          how much the measurements disagree with each other
    //   clock residual  how well the sample-to-host mapping itself fits
    //   delay           how wrong the delay model could be, which is the term
    //                   nothing can measure and therefore the one that usually
    //                   dominates
    const double delayUncertainty =
        std::max(kDelayUncertaintyFloorSec, m_snap.delaySec * kDelayUncertaintyFraction);
    m_snap.dispersionSec = jitter + m_snap.clockResidualSec + delayUncertainty;

    // Fewer than a handful of measurements is not a filtered value, whatever
    // its spread happens to be.
    m_snap.haveOffset = m_offsets.size() >= 5 && m_snap.clockState == "locked";
}

void Source::recordRtt(double rttSec) {
    if (!(rttSec > 0.0)) return;
    m_rttProbes.push_back(rttSec);
    // A dozen probes at five minutes apart is an hour of history: long enough
    // that a single bad one cannot set the figure, short enough that a genuine
    // routing change works its way in within the hour.
    while (m_rttProbes.size() > 12) m_rttProbes.pop_front();
    double best = m_rttProbes.front();
    for (double v : m_rttProbes) best = std::min(best, v);
    m_snap.httpRttMs = best * 1000.0;
}

void Source::updateDelayModel() {
    if (!m_cfg.autoDelay) {
        m_snap.delaySec = m_cfg.delayMs / 1000.0;
        m_snap.propagationSec = 0.0;
        m_snap.networkSec = 0.0;
        m_snap.codecSec = 0.0;
        m_snap.decoderSec = 0.0;
        m_snap.chainSec = 0.0;
        m_snap.extraSec = m_cfg.delayMs / 1000.0;
        m_snap.pathDescription = "configured delay";
        return;
    }

    // Propagation, from the receiver's published coordinates to whichever
    // transmitter the decoder says it is hearing. Until it says, the dial
    // decides — which is right for WWVB and for the two WWV-only outlets, and
    // is a coin toss on the four WWV and WWVH share. That uncertainty is real
    // and it is worth about 14 ms on a European path, which is why the station
    // tag is used the moment it is available.
    GeoPoint tx;
    if (m_snap.station == "wwvh") tx = wwvhSite();
    else if (m_snap.station == "wwvb" || m_cfg.dialHz < kWwvbCeilingHz) tx = wwvbSite();
    else tx = wwvSite();

    double prop = 0.0;
    if (m_snap.receiverLocation.valid) {
        prop = skywaveDelaySeconds(greatCircleMeters(m_snap.receiverLocation, tx));
        m_snap.pathDescription = describePath(m_snap.receiverLocation, tx);
    } else {
        m_snap.pathDescription = "no receiver coordinates; propagation not modelled";
    }

    // Half the round trip, as the one-way network delay.
    //
    // The MINIMUM of the recent probes, for the same reason SampleClock keeps
    // the minimum of packet arrivals: network delay is bounded below and
    // unbounded above, so a probe that happened to cross a busy moment says
    // nothing about the path and the smallest one says the most.
    const double net = m_snap.httpRttMs > 0.0 ? (m_snap.httpRttMs / 1000.0) * 0.5 : 0.0;
    // The codec the audio actually came through, not the one asked for: a
    // session that negotiated Opus is sent lossless frames in some modes, and
    // charging 8 ms of Opus delay to those biases the offset by 8 ms.
    const bool opus = m_feedingOpus >= 0 ? m_feedingOpus == 1 : m_cfg.format == AudioFormat::Opus;
    const double codec = opus ? kOpusDelaySec : 0.0;

    // The decoder's own edge bias, by which decoder is running -- chosen from
    // the dial exactly as ensureDecoder chooses it. Negative: an edge reported
    // early makes the offset read large, so it is taken back off.
    const double decoder = m_cfg.dialHz < kWwvbCeilingHz ? 0.0 : kWwvDecoderEdgeBiasSec;

    m_snap.propagationSec = prop;
    m_snap.networkSec = net;
    m_snap.codecSec = codec;
    m_snap.decoderSec = decoder;
    m_snap.chainSec = kUberSdrChainDelaySec;
    m_snap.extraSec = m_cfg.extraDelayMs / 1000.0;
    m_snap.delaySec = prop + net + codec + decoder + kUberSdrChainDelaySec + m_snap.extraSec;
}

// ---------------------------------------------------------------------------

SourceSnapshot Source::snapshot() const {
    std::lock_guard<std::mutex> lk(m_mu);
    SourceSnapshot s = m_snap;
    const double now = monotonicNow();
    s.linkAgeSec = now - m_linkSince;
    s.lastAudioAgeSec = m_lastAudioAt > 0.0 ? now - m_lastAudioAt : 1e9;
    s.lastTimeAgeSec = m_lastTimeAt > 0.0 ? now - m_lastTimeAt : 1e9;
    if (!m_offsets.empty()) s.offsetAgeSec = realtimeNow() - m_offsets.back().at;

    // Staleness is decided HERE as well as in recomputeOffset, because
    // recomputeOffset only runs when a packet or a second edge arrives -- and
    // the case that matters is precisely the one where neither does any more.
    // A source whose audio stops while its socket stays open would otherwise
    // keep reporting have_offset with an offset frozen at whatever it last was.
    //
    // The Selector rejects a stale candidate on its own, so this is not what
    // keeps a dead source out of the served time; it is what stops the status
    // page and the JSON showing a confident offset for a source that has gone
    // quiet.
    if (s.haveOffset && s.offsetAgeSec > kOffsetStaleSec) s.haveOffset = false;
    return s;
}

} // namespace ubersdr_ntp
