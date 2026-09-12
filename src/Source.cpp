#include "Source.h"

#include "CivilTime.h"
#include "Log.h"
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

Source::Source(SourceConfig cfg, std::string userAgent)
    : m_cfg(std::move(cfg)),
      m_userAgent(std::move(userAgent)),
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
    m_wakeCv.notify_all();

    // A shared_ptr copy taken under the lock, so the socket cannot be destroyed
    // underneath this call while the supervisor is replacing it. Stopping it
    // here rather than only setting the flag matters because the supervisor may
    // be waiting on a socket that will never say anything again, and a SIGTERM
    // should not take a whole reconnect interval to act on.
    std::shared_ptr<ix::WebSocket> ws;
    {
        std::lock_guard<std::mutex> lk(m_wsMu);
        ws = m_ws;
    }
    if (ws) ws->stop();

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
        ws->setExtraHeaders({{"User-Agent", m_userAgent}});
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
    const std::string payload = body.dump();

    std::string resp, curlErr;
    double rtt = 0.0;
    const long code = httpRequest(m_cfg.url + "/connection", &payload, m_userAgent,
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

void Source::fetchDescription() {
    // Informational, and the source of the receiver's coordinates — which is
    // what makes the propagation delay computable rather than guessed. A
    // failure here is not fatal: the delay model falls back to whatever was
    // configured.
    std::string resp, err;
    double rtt = 0.0;
    const long code = httpRequest(m_cfg.url + "/api/description", nullptr, m_userAgent,
                                  m_cfg.verifyTls, resp, rtt, err);
    if (code != 200) {
        LOG_DEBUG(m_cfg.name.c_str(), "/api/description unavailable (%ld %s)", code, err.c_str());
        return;
    }

    try {
        const json j = json::parse(resp);
        std::lock_guard<std::mutex> lk(m_mu);
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
        LOG_INFO(m_cfg.name.c_str(), "receiver: %s%s, http rtt %.0f ms",
                 m_snap.receiverName.empty() ? "(unnamed)" : m_snap.receiverName.c_str(),
                 m_snap.receiverLocation.valid ? "" : ", no coordinates published",
                 m_snap.httpRttMs);
        if (m_snap.receiverLocation.valid) {
            LOG_INFO(m_cfg.name.c_str(), "path: %s", m_snap.pathDescription.c_str());
        }
    } catch (const std::exception& e) {
        LOG_DEBUG(m_cfg.name.c_str(), "/api/description unparseable: %s", e.what());
    }
}

void Source::probeRtt() {
    std::string resp, err;
    double rtt = 0.0;
    const long code = httpRequest(m_cfg.url + "/health", nullptr, m_userAgent,
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
    // The arrival timestamp is taken FIRST, before any parsing or decoding, and
    // on CLOCK_REALTIME because that is the clock being measured. Everything
    // below — header parsing, Opus, the whole DSP chain — happens after this
    // read, so none of it can add to the number.
    const double arrival = realtimeNow();
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

    // A session that negotiated Opus still receives lossless frames the moment
    // it tunes to an IQ mode, so the frame itself has to say which it is. The
    // four-byte "PCM4" magic exists for exactly this; an Opus frame has no
    // magic, which is why the test is this way round.
    if (ubersdr::PCMv4StreamDecoder::isV4Frame(pkt, len)) {
        if (!m_pcmv4) m_pcmv4 = std::make_unique<PcmV4Reader>();
        ubersdr::PCMv4Header h;
        std::string err;
        if (!m_pcmv4->dec.decode(pkt, len, h, err)) {
            std::lock_guard<std::mutex> lk(m_mu);
            if (m_snap.decodeErrors++ % 200 == 0)
                LOG_WARN(m_cfg.name.c_str(), "pcm v4 decode: %s", err.c_str());
            return;
        }
        {
            std::lock_guard<std::mutex> lk(m_mu);
            m_snap.basebandPowerDb = h.basebandPower;
            m_snap.noiseDb = h.noise;
        }
        if (h.channels != 1) {
            // Two channels means an IQ mode, which this is never tuned to.
            // Feeding interleaved I/Q to a decoder expecting mono would look
            // like a signal and decode to nothing.
            return;
        }
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
        std::lock_guard<std::mutex> lk(m_mu);
        if (m_snap.decodeErrors++ % 200 == 0)
            LOG_WARN(m_cfg.name.c_str(), "opus header: %s", err.c_str());
        return;
    }
    {
        std::lock_guard<std::mutex> lk(m_mu);
        m_snap.basebandPowerDb = h.basebandPower;
        m_snap.noiseDb = h.noise;
    }
    if (h.channels != 1) return;

    const std::size_t bodyLen = len - off;
    if (bodyLen < 3) return;   // a squelched packet carries almost nothing

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

    const int n = opus_decode(m_opus, pkt + off, static_cast<opus_int32>(bodyLen),
                              m_opusPcm.data(), static_cast<int>(m_opusPcm.size()), 0);
    if (n < 0) {
        std::lock_guard<std::mutex> lk(m_mu);
        if (m_snap.decodeErrors++ % 200 == 0)
            LOG_WARN(m_cfg.name.c_str(), "opus_decode: %s", opus_strerror(n));
        return;
    }
    feedSamples(m_opusPcm.data(), n, h.sampleRate, arrivalSec);
}

void Source::ensureDecoder(int rate) {
    if (m_decoderRate == rate && (m_wwv || m_wwvb)) return;

    // The decoders decimate to a fixed series rate — 200 Hz for WWV/WWVH,
    // 100 Hz for WWVB — so a rate that is not a multiple of it decimates
    // unevenly and drifts, and the result is a decoder that simply never locks.
    // Every UberSDR audio mode clears this (12000 and 24000 both divide by
    // 200); refusing loudly beats decoding wrongly.
    if (rate <= 0 || rate % 200 != 0) {
        LOG_ERROR(m_cfg.name.c_str(),
                  "sample rate %d Hz is not a multiple of 200 Hz and cannot be decimated evenly",
                  rate);
        m_decoderRate = rate;   // so this is said once, not once per packet
        m_wwv.reset();
        m_wwvb.reset();
        return;
    }

    if (m_decoderRate != 0) {
        LOG_WARN(m_cfg.name.c_str(), "sample rate changed %d -> %d Hz; restarting the decoder "
                 "(a lock takes about four minutes of clean signal to regain)",
                 m_decoderRate, rate);
    }

    m_wwv.reset();
    m_wwvb.reset();
    m_decoderRate = rate;
    m_samplesWritten = 0;
    m_clock.setSampleRate(rate);
    m_clock.reset();
    {
        std::lock_guard<std::mutex> lk(m_mu);
        m_haveAnchor = false;
        m_haveFrame = false;
        m_offsets.clear();
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
        m_wwvb = std::make_unique<AetherSDR::WwvbDecoder>(rate);
        m_wwvb->onStateChanged = [this](AetherSDR::ClockLockState s) { onClockState(s); };
        m_wwvb->onSecond = [this](const AetherSDR::ClockSecondInfo& i) { onClockSecond(i); };
        m_wwvb->onFrame = [this](const AetherSDR::ClockFrameInfo& f) { onClockFrame(f); };
        m_wwvb->onTime = [this](const AetherSDR::ClockTimeInfo& t) { onClockTime(t); };
        m_wwvb->setPlausibility(reference, 24 * 60);
    } else {
        m_wwv = std::make_unique<AetherSDR::WwvDecoder>(rate);
        m_wwv->onStateChanged = [this](AetherSDR::ClockLockState s) { onClockState(s); };
        m_wwv->onSecond = [this](const AetherSDR::ClockSecondInfo& i) { onClockSecond(i); };
        m_wwv->onFrame = [this](const AetherSDR::ClockFrameInfo& f) { onClockFrame(f); };
        m_wwv->onTime = [this](const AetherSDR::ClockTimeInfo& t) { onClockTime(t); };
        m_wwv->setPlausibility(reference, 24 * 60);
    }

    LOG_INFO(m_cfg.name.c_str(), "decoder started: %s at %d Hz", wwvb ? "WWVB" : "WWV/WWVH", rate);
}

void Source::feedSamples(const std::int16_t* pcm, int count, int rate, double arrivalSec) {
    if (!pcm || count <= 0) return;
    ensureDecoder(rate);
    if (!m_wwv && !m_wwvb) return;

    m_samplesWritten += count;

    // The packet's arrival is taken as the time of its LAST sample: the audio
    // was captured before it was sent, so the end of the block is the edge
    // closer to the moment it landed here.
    m_clock.observe(m_samplesWritten, arrivalSec);

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
    m_snap.toneDetected = d.toneDetected;
    m_snap.phaseLocked = d.phaseLocked;
    m_snap.delayEstMs = d.delayEstMs;
    m_snap.anchored = d.anchored;
    m_snap.badFrameStreak = d.badFrameStreak;
    m_snap.framesInWindow = d.framesInWindow;
    m_snap.windowSize = d.windowSize;
    m_snap.voteQuality = d.voteQuality;
    m_snap.samplesConsumed = consumed;
    switch (static_cast<AetherSDR::ClockLockRefusal>(d.refusalReason)) {
        case AetherSDR::ClockLockRefusal::QualityFloor: m_snap.refusal = "quality_floor"; break;
        case AetherSDR::ClockLockRefusal::Plausibility: m_snap.refusal = "plausibility"; break;
        case AetherSDR::ClockLockRefusal::Staleness:    m_snap.refusal = "staleness"; break;
        case AetherSDR::ClockLockRefusal::Contested:    m_snap.refusal = "contested"; break;
        default: m_snap.refusal = "none"; break;
    }
    switch (st) {
        case AetherSDR::ClockStation::Wwv:  m_snap.station = "wwv"; break;
        case AetherSDR::ClockStation::Wwvh: m_snap.station = "wwvh"; break;
        case AetherSDR::ClockStation::Wwvb: m_snap.station = "wwvb"; break;
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
    m_samplesWritten = 0;
    m_decoderRate = 0;
    m_wwv.reset();
    m_wwvb.reset();

    std::lock_guard<std::mutex> lk(m_mu);
    m_haveAnchor = false;
    m_offsets.clear();
    m_snap.haveOffset = false;
    m_snap.offsetSamples = 0;
    m_snap.clockState = "stopped";
    m_snap.clockFitValid = false;
    m_snap.samplesConsumed = 0;
    LOG_DEBUG(m_cfg.name.c_str(), "stream reset (%s)", why);
}

// ---------------------------------------------------------------------------
// Decoder callbacks

void Source::onClockState(AetherSDR::ClockLockState s) {
    const char* name = "nosignal";
    switch (s) {
        case AetherSDR::ClockLockState::Locked:    name = "locked"; break;
        case AetherSDR::ClockLockState::Acquiring: name = "acquiring"; break;
        default: break;
    }
    {
        std::lock_guard<std::mutex> lk(m_mu);
        m_snap.clockState = name;
        if (s != AetherSDR::ClockLockState::Locked) {
            // An anchor that outlived its lock would keep extending one second
            // at a time through a signal the decoder no longer trusts, quietly
            // manufacturing offsets from nothing.
            m_haveAnchor = false;
        }
    }
    LOG_INFO(m_cfg.name.c_str(), "state -> %s", name);
}

void Source::onClockFrame(const AetherSDR::ClockFrameInfo& f) {
    std::lock_guard<std::mutex> lk(m_mu);
    // Recorded whether or not anything is emitted: onClockTime composes its
    // timestamp against this frame's second 0, and a `time` event that arrived
    // before any frame would be composed against sample 0 and land minutes out.
    m_frameStartSample = f.frameStartSample;
    m_haveFrame = true;
    m_snap.dut1Tenths = f.dut1Tenths;
    m_snap.leapPending = f.leapPending;
}

void Source::onClockTime(const AetherSDR::ClockTimeInfo& t) {
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

    m_anchorEdgeSample = t.lastEdgeSample;
    m_anchorUtcMs = decodedMs;
    m_haveAnchor = true;
    m_anchorSetAt = monotonicNow();
    m_lastTimeAt = m_anchorSetAt;

    m_snap.lastQuality = std::clamp(static_cast<int>(std::lround(t.quality * 100.0)), 0, 100);
    m_snap.lastDecodedUtc = iso8601(decodedMs);
}

void Source::onClockSecond(const AetherSDR::ClockSecondInfo& i) {
    double hostSec = 0.0;
    if (!m_clock.hostTimeAt(i.edgeSample, hostSec)) return;

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
    const double codec = m_cfg.format == AudioFormat::Opus ? kOpusDelaySec : 0.0;

    m_snap.propagationSec = prop;
    m_snap.networkSec = net;
    m_snap.codecSec = codec;
    m_snap.extraSec = m_cfg.extraDelayMs / 1000.0;
    m_snap.delaySec = prop + net + codec + m_snap.extraSec;
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
