#include "Source.h"

#include "CivilTime.h"
#include "Log.h"
#include "Version.h"
#include "../third_party/json.hpp"
#include "../third_party/pcm_v4.hpp"

#include <ixwebsocket/IXNetSystem.h>
#include <ixwebsocket/IXWebSocket.h>

#include <curl/curl.h>
#include <openssl/evp.h>

#include <algorithm>
#include <cmath>
#include <cctype>
#include <cstring>
#include <fstream>
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

// Offsets older than this stop counting as current. A source that locked and
// then faded should stop contributing long before its last reading is useless,
// and the selector needs a definite answer rather than a decaying one.
constexpr double kOffsetStaleSec = 180.0;

// An anchor is extended forwards one second at a time. Past this, the extension
// has run further than a voted timestamp can vouch for and the next `time`
// event should have arrived — WWV sends one a minute.
constexpr double kAnchorMaxAgeSec = 300.0;

// Where the WWV/WWVH decoder puts a second edge relative to the true edge in
// its input: 13.6 ms EARLY, the matched filter's chain delay being taken as 7
// series samples where it is 4.27. Measured, not estimated: tools/decodertest
// generates WWV and WWVH and reports this mean over every edge it checks
// (-13.645 / -13.637 ms, spread about ±1 ms). The WWVB decoder's edges are
// exact to 0.02 ms, so it has no such term. Kept as a correction here rather
// than fixed in the decoder, whose tracker is built around the upstream value.
constexpr double kWwvDecoderEdgeBiasSec = -0.013645;

// What WWV and WWVH need on top of everything else, measured only as a
// residual: +4.7 ms. It is the 4.7 ms the chain constant below carried until
// 2026-09-22, moved here unchanged, so every WWV source's total delay is
// exactly what it was calibrated to.
//
// It belongs to WWV because only WWV needs it. DCF77 read 4.7 ms early against
// a GPS-fed NTP server for forty settled minutes with the chain at 14.1, and
// nothing DCF77-specific can account for that: its AM and PM demodulators are
// independent and agree to under a millisecond, its groundwave path cannot be
// wrong by kilometres enough, and radiod's channel filter delays the iq preset
// exactly as it does usb (filter.c: a linear-phase sinc, (M-1)/2 samples,
// whatever the passband). WWV, calibrated against the NTP class with the same
// chain, came out right. The chain is shared; the difference is not.
//
// Which WWV term is short is not known. The decoder bias above was measured on
// synthetic signals, and a real tick, smeared by multipath and the ionosphere,
// need not sit where a clean one does; or the skywave model's single hop at
// 350 km is shorter than the path the signal takes. And the WWV calibrations
// were all made over Opus, which is no longer used: the sum they fixed held the
// 8 ms then charged for Opus (measured offline as a 5-10 ms bias), so WWV over
// PCM is right only as far as that figure was, and this carries its error too.
// Until a receiver that hears WWV and DCF77 together splits them, this is the
// sum, kept with the decoder term where the status page shows it.
//
// Zero from 2026-09-24, pending exactly that measurement. With capture timing
// nothing between the antenna and the sample index treats WWV differently from
// DCF77 -- the same radiod path, the same capture stamps, no chain term -- so no
// mechanism is known that needs it, and the one WWV-over-PCM reading (K3FEF,
// 2026-09-22, against Cloudflare) put it nearer +2.3 ms than +4.7. The terms it
// could really belong to, the decoder bias on live ticks and the skywave model,
// are to be measured as themselves: WWV against DCF77 on one receiver, both
// capture-timed, over twenty settled minutes by day and by night.
constexpr double kWwvResidualSec = 0.0;

// The delay from RF reaching the SDR to the audio leaving UberSDR's WebSocket:
// radiod's demodulator and filters, its block framing, the server's handling.
// It is a property of the software, the same on every instance, so it is one
// constant rather than something each operator has to calibrate.
//
// First measured live, 2026-09-13, against a north-eastern US receiver hearing
// WWV on 10 and 15 MHz for 14 minutes of lock, from a host disciplined by ntpd
// to about 1.4 ms: 13.6 ms, equal to the decoder bias above, good to perhaps
// ±2.5 ms.
//
// Refined 2026-09-17 to 12.6 ms against the NTP class, on a reading of +0.8 to
// +1.5 ms from two receivers in standby against time.cloudflare.com.
//
// WITHDRAWN 2026-09-20, and back up to 14.1 ms. That refinement was the one
// measurement here ever taken against a reference that was itself wrong: the
// propagation model was a strict lower bound (Propagation.h explains how, and
// it is fixed now), so the comparison it was fitted to carried a one-signed
// deficit of its own. Tuning a constant to cancel a modelling error is how the
// error survives: it moves from a term that describes the path into one that
// does not, and stops being visible on any receiver.
//
// The new figure comes from 11.5 settled hours of class delta on one receiver
// at 2516 km, read off /api/metrics in half-hour buckets rather than watched:
//
//   mean -1.610 ms, median -1.535, sd 0.819
//   local day   -1.395 ms      local night  -1.844 ms
//
// The 0.45 ms day-night split is the ionosphere and belongs to propagation --
// the layer climbs after sunset and a fixed height cannot follow it. What is
// left is flat, present in daylight on a one-hop path where the geometry is
// best known, and the operator sees the same floor on every instance he has
// measured. That is this term. Adding the mean back (the 350 km height already
// returns 0.11 ms of it on this path) puts the constant at 14.1 ms.
//
// Which is where it started. The 2026-09-13 measurement said 13.6 +/- 2.5 ms
// against a host ntpd held to about 1.4 ms, and two routes with nothing in
// common landing inside half a millisecond of each other is worth more than
// either on its own.
//
// It stays degenerate with the decoder-edge constant above: the class delta
// sees only their sum, so charging the difference here is bookkeeping,
// justified by this being the term defined as the residual and the only one
// never measured on its own.
//
// 14.1 ms, and this one is measured rather than glanced at. Sixty-four minutes
// of settled class delta, once the system rate was being borrowed and a second
// upstream was there to check the first:
//
//     mean +0.107 ms, median +0.113, sd 0.491, range -0.91 to +1.07
//     positive in 41 of 65 samples
//
// Centred on zero and scattering both ways, which is what this measurement is
// supposed to look like and never had before tonight.
//
// It briefly went to 13.1 on a SIX-minute window reading +1.06. That window was
// the smoothed delta still climbing out of its own settling transient, and a
// rising curve was read as a plateau. The same figure over an hour is +0.107.
// Three times now this constant has been moved on a window too short to carry
// it -- 13.6 to 12.6, 12.6 to 14.1, 14.1 to 13.1 -- and twice that was wrong.
// The EMA has a five-minute time constant, so anything under about twenty
// minutes of it is still describing the last disturbance rather than the
// crystal. Do not move this again on less.
//
// The 2026-09-13 measurement, made directly against an ntpd-held host and
// owing nothing to any of this, said 13.6 +/- 2.5 ms.
//
// 9.4 ms, 2026-09-22, and this time split rather than moved. Every figure
// above was taken on WWV, where the class delta sees only the SUM of this
// constant, the WWV decoder bias and the skywave model -- so 14.1 was right for
// that sum and said nothing about this term alone. DCF77 is the first source
// with none of the WWV terms in it, and against an NTP server fed directly by a
// GPS receiver, 0.17 ms away, over a 0.05 ms path to the receiver, it read
// +4.78 ms steady for forty settled minutes, timed by PM: its delay was 4.7 ms
// over-counted. What DCF77 and WWV share is this term, so this term is 4.7 ms
// lower, and WWV keeps its total through kWwvResidualSec. WWVB, the other
// source with no WWV terms, moves with DCF77.
constexpr double kUberSdrChainDelaySec = 0.0094;

// With capture timing (CaptureClock, SampleClock.h) the chain is not modelled
// at all: every packet says when the RX888 captured its first sample, net of
// radiod's channel filter delay, so radiod's framing and processing, the
// multicast hop, the server, the WebSocket and the network are all outside the
// measurement. The RX888's own latency -- the A/D pipeline, the FX3's
// buffering, the last USB packet of a transfer in flight -- is taken off in
// radiod (RX888_CAPTURE_LATENCY_NS, ubersdr-radiod's capture-time patch), not
// here: it belongs to the front end, and only radiod knows which front end
// captured the samples. So the stamps are the capture time itself and the
// chain term is zero. Calibrate that constant from the class delta against a
// GPS-fed server, on no less than twenty settled minutes and on DCF77, which
// has no WWV terms; a steady delta of -L ms means L ms more to take off.

// Capture timing: the longest capture-to-arrival interval believed. Real ones
// are tens of milliseconds; past this the stamp is from another stream, or the
// host clock stepped between capture and arrival.
constexpr double kMaxCaptureLagSec = 1.0;
// How long to wait, with audio flowing, for the receiver to say which clock its
// stamps are on before falling back to the arrival fit.
constexpr double kTimingPendingSec = 5.0;
// Capture timing: how close a step in the stamps must come to whole frames to
// be audio lost on the way here rather than radiod re-anchoring. The stamps
// are exact to the nanosecond; whole RX888 transfers are 4.045 ms, which no
// small multiple of brings within this of a 20 ms frame.
constexpr double kCaptureFrameToleranceSec = 50e-6;

// Floor on how well the delay model can be trusted, whatever it computed. The
// receiver's own buffering between radiod and the WebSocket is inside this and
// nothing here can see it.
//
// Set from the worst errors measured against an absolute reference, with
// margin, rather than guessed. Two north-eastern US receivers, ~65 ms of
// modelled delay each: against a PPS-disciplined stratum 1 on 2026-09-17 the
// worst single receiver was 9.0 ms out and the served time 5.3 ms; against
// ntpd on 2026-09-13 the served time was at worst +8.6 ms, about 7.6 ms after
// the chain constant was corrected. With the 1-3 ms each source measures of
// itself on top, a 10 ms floor still covers every one of those by about 1.4x.
// It was 15 ms, which covered them about 2x and left the served dispersion
// near 21 ms for an error that has not been seen past 9.
constexpr double kDelayUncertaintyFloorSec = 0.010;
// And a proportional part, because a long path is a worse-known path: more
// hops, more spread between them, more of the virtual height assumption. At
// 15% it only overtakes the floor past about 67 ms of delay, i.e. on paths
// longer than the ones the floor was measured on.
constexpr double kDelayUncertaintyFraction = 0.15;
// The floor above is the doubt in an HF skywave path timed by packet arrival,
// with UberSDR's buffering inside it. None of that is in a DCF77 source that is
// capture-timed and timed from its phase modulation on a groundwave path:
// radiod stamps the samples at the RX888 (so no chain, network or buffering is
// in the path), the groundwave delay is geometry at a known velocity (the
// night-time skywave off the D/E layer is at most ~90 us later; Propagation.h),
// and the PM correlator's edge is exact to the decoder test's resolution. What
// is left is that skywave bias, the RX888's own latency (50-300 us, not yet
// taken off in radiod) and the decoder's residual. Measured on M9PSY-1 at
// 1061 km against a GPS-fed stratum 1 on 2026-09-24: within 0.03 ms smoothed
// over 30 settled minutes, jitter spikes to 0.2 ms, which the jitter term
// carries on top. 1 ms covers the unmeasured terms together about twice over.
// Only inside the groundwave service area; past it the modes interfere.
constexpr double kDelayUncertaintyFloorLfPmSec = 0.001;
constexpr double kLfGroundwaveServiceM = 2000e3;
// WWV and WWVH, capture-timed, from a receiver whose decoder has named the
// station. The 10 ms floor was measured on arrival-timed WWV, where the chain
// and the receiver's buffering were inside the error; capture timing takes both
// out. What is left is the skywave geometry, bounded from the model itself by
// skywaveModeSpreadSeconds() (Propagation.h), and the decoder's edge on live
// ticks, whose bias was measured on synthetic ones: this allowance is for
// that, not yet measured against a reference. Until the decoder names the
// station the transmitter is a guess worth ~14 ms, and the 10 ms stands.
constexpr double kWwvDecoderAllowanceSec = 0.001;

// The smallest uncertainty a source may claim when it is being weighed against
// the others. See the use site.
constexpr double kWeightDispersionFloorSec = 0.001;

// m_jsonPingSentAt's "do not time the next pong" marker. See the ping in run().
constexpr double kPingUntimed = -1.0;

// The longest JSON ping/pong round trip that counts as a measurement. The
// server answers at once; a pong this late crossed a stall, not the path.
constexpr double kMaxPongSec = 10.0;

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

// A WebSocket close code (RFC 6455 section 7.4, and the IANA registry) in
// words. Says what the code means and nothing about who sent it.
std::string describeClose(unsigned code, bool remote) {
    const char* what = nullptr;
    switch (code) {
        case 1000: what = "closed normally"; break;
        case 1001: what = "the far end is going away: restarting or shutting down"; break;
        case 1002: what = "protocol error"; break;
        case 1003: what = "unsupported data"; break;
        case 1006: what = "connection lost without a close"; break;
        case 1007: what = "invalid data"; break;
        case 1008: what = "refused by the far end's policy"; break;
        case 1009: what = "message too large"; break;
        case 1011: what = "server error"; break;
        case 1012: what = "the far end is restarting"; break;
        case 1013: what = "the far end is overloaded; try again later"; break;
        case 1014: what = "bad gateway"; break;
        default: break;
    }
    std::string out = what ? what : "closed";
    if (code != 0) out += " (code " + std::to_string(code) + ")";
    if (!remote && code != 1006) out += ", by this end";
    return out;
}

} // namespace

// ---------------------------------------------------------------------------

Source::Source(SourceConfig cfg)
    : m_cfg(std::move(cfg)),
      m_broadcast(broadcastFor(m_cfg.carrierHz, m_cfg.dialHz)),
      m_iq(isIqBroadcast(m_broadcast)),
      m_channels(m_iq ? 2 : 1),
      m_sessionId(makeUuidV4()),
      m_clock(12000),
      m_capture(12000) {
    m_snap.name = m_cfg.name;
    m_snap.url = m_cfg.url;
    m_snap.enabled = m_cfg.enabled;
    m_snap.carrierHz = m_cfg.carrierHz;
    m_snap.dialHz = m_cfg.dialHz;
    m_snap.minMarginDb = m_iq ? m_cfg.minMarginDb : 0;
    m_snap.weight = m_cfg.weight;
    m_snap.station = m_broadcast == Broadcast::Dcf77 ? "dcf77"
                   : m_broadcast == Broadcast::Allouis ? "allouis"
                   : m_broadcast == Broadcast::Wwvb ? "wwvb"
                   : wwvOnlyCarrier(m_cfg.carrierHz) ? "wwv" : "unknown";
    m_linkSince = monotonicNow();
}

Source::~Source() {
    stop();
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
    LOG_INFO(tag, "starting: %s dial %.6f MHz (carrier %.6f MHz)",
             m_cfg.url.c_str(), m_cfg.dialHz / 1e6, m_cfg.carrierHz / 1e6);

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
    // A receiver too old to speak protocol version 4 will not start speaking it
    // on the next attempt, and every attempt takes a listener slot on a
    // receiver that is probably not ours. Half an hour is rare enough to cost
    // its operator nothing and still picks up an upgrade the same evening.
    constexpr double kTooOldRetrySec = 1800.0;
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

    // Whether the last pass through this loop found the source held inactive,
    // so the timing is thrown away once on the way down rather than every
    // second for as long as it sits there.
    bool wasInactive = false;

    while (m_running.load()) {
        // Cold standby. Nothing is connected while the Selector says this class
        // is not needed: a long-lived UberSDR session occupies a listener slot
        // on a receiver that may not be ours, and holding one open to decode
        // audio nobody is going to use is the one cost this mode exists to
        // avoid. The timing goes with it -- an offset measured before an
        // arbitrarily long idle period describes a sample clock that no longer
        // exists, and carrying it forward would let a stale figure serve the
        // moment the source came back.
        if (!m_active.load()) {
            if (!wasInactive) {
                wasInactive = true;
                discardTiming("held inactive");
                std::string why;
                {
                    std::lock_guard<std::mutex> lk(m_mu);
                    why = m_activeReason;
                    m_snap.link = LinkState::Idle;
                    m_snap.linkDetail = why.empty() ? "held in cold standby" : why;
                    m_linkSince = monotonicNow();
                }
                LOG_INFO(tag, "idle: %s", why.empty() ? "held in cold standby" : why.c_str());
            }
            std::unique_lock<std::mutex> lk(m_wake);
            m_wakeCv.wait_for(lk, std::chrono::seconds(1),
                              [this] { return !m_running.load() || m_active.load(); });
            continue;
        }
        if (wasInactive) {
            wasInactive = false;
            // A source coming up from cold starts the backoff over: it has not
            // failed at anything, and inheriting a backoff from before it was
            // put away would delay exactly the connection that is now urgent.
            consecutiveFailures = 0;
            backoff = kBackoffMin;
            LOG_INFO(tag, "brought up from cold standby — acquiring from nothing");
        }

        {
            std::lock_guard<std::mutex> lk(m_mu);
            m_snap.link = LinkState::Connecting;
            m_snap.linkDetail.clear();
            m_snap.connectAttempts++;
            // A request that arrived while no connection was up is already
            // satisfied by the one about to be made.
            m_reacquire.store(false);
            // Asked afresh each time: the operator may have upgraded.
            m_serverTooOld.store(false);
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
                case ix::WebSocketMessageType::Close: {
                    // Described in our words, from the close code. The reason
                    // TEXT is whatever terminated the socket chose to say --
                    // the receiver, or any proxy or tunnel in front of it,
                    // which may name its own vendor -- so it is logged as
                    // received and kept out of the status page and events,
                    // where it would read as this daemon's diagnosis.
                    const auto& ci = msg->closeInfo;
                    if (!ci.reason.empty()) {
                        LOG_INFO(m_cfg.name.c_str(), "close frame: code %u, reason \"%s\"",
                                 static_cast<unsigned>(ci.code), ci.reason.c_str());
                    }
                    onClose(describeClose(ci.code, ci.remote));
                    break;
                }
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
                                  [this] { return !m_running.load() || !m_active.load(); });
            }
            if (!m_running.load()) break;
            // Put away while streaming. Breaking here rather than stopping the
            // socket from setActive's thread: ix::WebSocket::stop() joins the
            // socket's own thread with no guard, so only the supervisor may
            // call it -- the same rule the shutdown path follows.
            if (!m_active.load()) {
                LOG_INFO(tag, "dropping the connection: no longer needed");
                break;
            }

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
                    // The JSON keepalive the UberSDR session timer watches, and
                    // also the network round trip, timed from here to the pong.
                    //
                    // It has to be this one and not an RFC 6455 ping. A proxy
                    // that terminates the WebSocket answers control frames
                    // itself -- gorilla does by default, and so do nginx and
                    // Cloudflare -- so a protocol-level pong times the path to
                    // the PROXY. Through tunnel.ubersdr.org that measured
                    // 13.4 ms to a receiver whose JSON pong took 33.7, and the
                    // missing half of the relay's leg read as a source 10 ms
                    // late. The JSON ping is an ordinary message: every proxy
                    // passes it on and only UberSDR itself can answer it. On a
                    // direct connection the two agree (30.5 and 30.6 ms), so
                    // nothing calibrated against the old figure moves.
                    //
                    // The pong carries no echo of its ping, so which ping it
                    // answers is inferred: one outstanding at a time, and if
                    // the last one was never answered, a pong for it may still
                    // be in flight -- the next pong is then left untimed rather
                    // than matched to the wrong ping.
                    {
                        std::lock_guard<std::mutex> lk(m_mu);
                        m_jsonPingSentAt = m_jsonPingSentAt > 0.0 ? kPingUntimed : monotonicNow();
                    }
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

                if (m_serverTooOld.load()) break;   // nothing it sends can be decoded

                if (m_reacquire.exchange(false)) {
                    std::string why;
                    {
                        std::lock_guard<std::mutex> lk(m_mu);
                        why = m_reacquireWhy;
                    }
                    LOG_WARN(tag, "starting over at the consensus's request — %s", why.c_str());
                    break;   // the reconnection below resets the stream and the decoder
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
        // Deactivated: straight back to the top, which parks it as idle. A
        // backoff here would leave it in Backoff for half a minute saying it
        // was retrying something it has been told not to do.
        if (!m_active.load()) continue;

        if (m_serverTooOld.load()) {
            const double delay = kTooOldRetrySec * backoffDelay() / backoff;   // same jitter
            const std::string why = "the receiver runs UberSDR older than 0.1.63, which "
                                    "cannot send protocol version 4; asking again in " +
                                    std::to_string(static_cast<int>(delay / 60.0)) + " minutes";
            {
                std::lock_guard<std::mutex> lk(m_mu);
                m_snap.link = LinkState::Backoff;
                m_snap.linkDetail = why;
                m_linkSince = monotonicNow();
            }
            LOG_WARN(tag, "holding off: %s", why.c_str());
            std::unique_lock<std::mutex> lk(m_wake);
            m_wakeCv.wait_for(lk, std::chrono::duration<double>(delay),
                              [this] { return !m_running.load() || !m_active.load(); });
            continue;
        }

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
        m_descriptionTried.store(true);
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
        m_descriptionTried.store(true);
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
        m_descriptionTried.store(true);
        return false;
    }
}

void Source::onJsonPong() {
    const double now = monotonicNow();
    std::lock_guard<std::mutex> lk(m_mu);
    const double sentAt = m_jsonPingSentAt;
    m_jsonPingSentAt = 0.0;
    if (!(sentAt > 0.0)) return;   // unsolicited, or deliberately untimed
    const double rtt = now - sentAt;
    if (!(rtt > 0.0) || rtt > kMaxPongSec) return;
    recordWsRtt(rtt);
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
      << "?frequency=" << m_cfg.dialHz;
    if (m_iq) {
        // DCF77: the whole 12 kHz of complex baseband, which is exactly the
        // server's own IQ preset, so the mode change moves no filter. Lossless
        // unless min_margin asks for reduced depth, and min_margin=0 says so
        // explicitly rather than relying on the parameter's absence meaning it.
        u << "&mode=iq&bandwidthLow=-6000&bandwidthHigh=6000"
          << "&format=pcm-zstd&min_margin=" << m_cfg.minMarginDb;
    } else {
        u << "&mode=usb"
          // The passband has to reach 2.2 kHz or the WWV/WWVH second tick —
          // which is recovered entirely from its 2000/2200 Hz audio image — is
          // filtered away, and the decoder sits in `acquiring` for ever with
          // nothing to say why. 0-3000 is what the frontend's own clock panel
          // asks for.
          << "&bandwidthLow=0&bandwidthHigh=3000"
          // Lossless at full quality, as IQ is: min_margin=0 says so rather
          // than leaving it to the parameter's absence.
          << "&format=pcm-zstd&min_margin=0";
    }
    u
      // Version 4 for both: the lossless path at version 4 is the predictive
      // codec rather than anything zstd. The query parameter is still spelt
      // "pcm-zstd" for compatibility with older servers.
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
        } else if (type == "pong") {
            onJsonPong();
        } else if (type == "status") {
            onServerClockId(j.value("clockId", std::string()));
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
    // Everything below — header parsing, decoding, the whole DSP chain — happens
    // after this read, so none of it can add to the number.
    //
    // On the daemon clock, which nothing steers and nothing steps (SampleClock.h
    // has why). A step or a slew of the host clock reaches neither the fit nor
    // any offset already measured, so neither needs handling here.
    const double arrival = daemonNow();
    // And how far the host clock is from it right now, for turning a capture
    // stamp on the host clock into the daemon clock (see timeBlock).
    const double dmr = daemonMinusRealtime();

    handleAudio(reinterpret_cast<const std::uint8_t*>(msg.data()), msg.size(), arrival, dmr);
}

// ---------------------------------------------------------------------------
// Audio

void Source::handleAudio(const std::uint8_t* pkt, std::size_t len, double arrivalSec, double dmrSec) {
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
    // So a lost packet is replaced by silence of the same length. Only once a
    // timeline exists — before the first good packet there is nothing to keep
    // in step.
    auto concealLost = [this](int frameSamples) {
        if (m_decoderRate <= 0 || m_samplesWritten <= 0) return;
        const int n = frameSamples > 0 ? frameSamples : m_lastFrameSamples;
        if (n <= 0) return;
        feedSilence(n, m_decoderRate);
    };

    // Every frame asked for is PCM v4, which opens with the four-byte "PCM4"
    // magic.
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
            if (h.sampleCount > 0 && h.channels == m_channels && h.sampleRate == m_decoderRate) {
                trackTimeline(h.timestampNanos, h.sampleRate, h.sampleCount / h.channels);
                concealLost(h.sampleCount / h.channels);
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
        }
        if (h.channels != m_channels) {
            // Interleaved I/Q fed to a decoder expecting mono would look like a
            // signal and decode to nothing, and mono fed to the DCF77 decoder
            // has no phase to decode. Either is the server not tuning the mode
            // asked for, so it is said, once in a while, rather than guessed at.
            std::lock_guard<std::mutex> lk(m_mu);
            if (m_snap.decodeErrors++ % 200 == 0) {
                LOG_ERROR(m_cfg.name.c_str(), "receiver sent %d-channel audio where %d was asked for (%s)",
                          h.channels, m_channels, m_channels == 2 ? "mode=iq" : "mode=usb");
            }
            return;
        }
        // Frames, not int16 values: an IQ frame carries two per sample.
        const int frames = h.sampleCount / h.channels;
        // ensureDecoder before the tracker: a rate change restarts the sample
        // count the tracker is measuring against.
        if (!ensureDecoder(h.sampleRate)) return;
        trackTimeline(h.timestampNanos, h.sampleRate, frames);
        const bool observeArrival = timeBlock(h.timestampNanos, h.sampleRate, arrivalSec, dmrSec);
        feedSamples(m_pcmv4->dec.samples(), frames, h.sampleRate, arrivalSec, observeArrival);
        return;
    }

    if (ubersdr::PCMv4StreamDecoder::isZstdFrame(pkt, len)) {
        std::lock_guard<std::mutex> lk(m_mu);
        if (m_snap.decodeErrors++ == 0) {
            LOG_ERROR(m_cfg.name.c_str(),
                      "server sent a version 1-3 PCM frame: it is older than 0.1.63 and "
                      "cannot serve protocol version 4");
        }
        m_serverTooOld.store(true);
        return;
    }

    // Nothing else is asked for. A frame that is neither is a server speaking
    // some other protocol; it still occupied its stretch of the stream.
    {
        std::lock_guard<std::mutex> lk(m_mu);
        if (m_snap.decodeErrors++ % 200 == 0)
            LOG_WARN(m_cfg.name.c_str(), "receiver sent a %zu-byte frame that is not PCM v4", len);
    }
    m_tsHave = false;
    concealLost(0);
}

void Source::feedSilence(int count, int rate) {
    // In pieces of at most a second, so a long gap does not allocate its whole
    // length and the decoders see blocks the size they are used to.
    const int chunk = std::max(1, rate);
    const std::size_t need = static_cast<std::size_t>(chunk) * m_channels;
    if (m_silence.size() < need) m_silence.assign(need, 0);
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
    // A zero stamp is "no capture time": UberSDR has no usable reference from
    // radiod for a moment (a new channel, a rate change, radiod rebuilding its
    // anchor). It says nothing about the timeline either way, and the samples
    // still count, so the next real stamp lines up against them as usual.
    if (timestampNanos == 0) return;
    if (frameSamples > 0) m_lastFrameSamples = frameSamples;
    if (m_lastFrameSamples <= 0) m_lastFrameSamples = rate / 50;

    constexpr double kBlockSec = 1.0;
    constexpr double kMaxFillSec = 1.5;
    // How far the block minimum may wander BACKWARDS before the mapping is
    // treated as broken rather than noisy.
    //
    // This was half a frame, 10 ms, and that is inside the noise. The figure
    // compared is a MINIMUM over the block, which is the statistic a single
    // early-stamped frame moves the furthest, and a receiver measured here
    // tripped it five times in seven minutes with steps of 11 to 16 ms --
    // every one of them just past the line. Each trip threw away the sample
    // clock and every offset behind it, so the source never held a lock long
    // enough to contribute, while its signal was the best of the three.
    //
    // 100 ms is five frames: far above the jitter that was tripping it, and
    // far below a genuine discontinuity, which means a restarted stream or a
    // changed origin and arrives in tens of milliseconds at least. Absorbing a
    // small backward step costs nothing anyway -- the baseline simply follows
    // it, and the sample-to-host mapping is the arrival fit, not this.
    constexpr double kMaxBackStepSec = 0.1;

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
        const bool reanchor = m_timing == TimingMode::Capture && step > 0.5 * frameSec &&
            std::fabs(step - std::llround(step / frameSec) * frameSec) > kCaptureFrameToleranceSec;
        if (reanchor && step <= kMaxFillSec) {
            // Capture stamps are exact, and audio lost on the way here is whole
            // frames of it; a step that is not is radiod re-anchoring after it
            // lost samples at the USB, before any stream was cut from them.
            // Nothing is missing from this stream -- its samples after the step
            // were simply captured later than the count says, which is what
            // the capture clock is following -- so there is nothing to fill.
            LOG_INFO(m_cfg.name.c_str(), "receiver re-anchored its capture times %+.3f ms "
                     "(samples lost at the SDR, not on the way here)", step * 1000.0);
            m_tsBaseline = m_tsBlockMin;
        } else if (step > 0.5 * frameSec && step <= kMaxFillSec) {
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
        } else if (step > kMaxFillSec || step < -kMaxBackStepSec) {
            LOG_WARN(m_cfg.name.c_str(), "stream timestamps stepped %+.0f ms against the sample "
                     "count; rebuilding the sample clock", step * 1000.0);
            discardTiming("stream timestamp discontinuity");
            m_tsBaseline = m_tsBlockMin;
        } else {
            // Ordinary wander of the minimum, and the slow drift between
            // radiod's sample clock and the server's.
            if (step < -0.5 * frameSec) {
                LOG_DEBUG(m_cfg.name.c_str(), "stream timestamps wandered %+.0f ms; absorbed",
                          step * 1000.0);
            }
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

// ---------------------------------------------------------------------------
// Capture timing

namespace {

// Names the clock this host's timestamps are on, exactly as UberSDR does
// (capture_time.go, hostClockID): the first eight bytes of the SHA-256 of the
// kernel's boot_id, in hex. Every process and container on one host reads the
// same boot_id, and no other host does, so a receiver that sends this value is
// stamping on this host's CLOCK_REALTIME. Empty if the kernel does not say.
const std::string& hostClockId() {
    static const std::string id = [] {
        std::ifstream f("/proc/sys/kernel/random/boot_id");
        std::string s;
        std::getline(f, s);
        while (!s.empty() && std::isspace(static_cast<unsigned char>(s.back()))) s.pop_back();
        std::size_t start = 0;
        while (start < s.size() && std::isspace(static_cast<unsigned char>(s[start]))) ++start;
        s.erase(0, start);
        if (s.empty()) return std::string();
        unsigned char md[EVP_MAX_MD_SIZE];
        unsigned int n = 0;
        if (EVP_Digest(s.data(), s.size(), md, &n, EVP_sha256(), nullptr) != 1 || n < 8) return std::string();
        static const char kHex[] = "0123456789abcdef";
        std::string out;
        for (int i = 0; i < 8; ++i) {
            out += kHex[md[i] >> 4];
            out += kHex[md[i] & 15];
        }
        return out;
    }();
    return id;
}

const char* timingName(int mode) {
    static const char* const kNames[] = {"pending", "capture", "arrival"};
    return kNames[mode];
}

} // namespace

void Source::setTiming(TimingMode mode, const std::string& why) {
    const bool changed = mode != m_timing;
    const bool wasDecided = m_timing != TimingMode::Pending;
    m_timing = mode;
    if (changed && wasDecided) {
        // The two put different terms in the delay model, so offsets measured
        // under one do not continue under the other.
        discardTiming("timing changed");
    }
    if (changed) {
        m_capture.reset();
        m_captureTrusted = true;
        LOG_INFO(m_cfg.name.c_str(), "timing samples by %s: %s",
                 mode == TimingMode::Capture ? "the receiver's capture times" : "their arrival here",
                 why.c_str());
    }
    std::lock_guard<std::mutex> lk(m_mu);
    m_snap.timingMode = timingName(static_cast<int>(mode));
    m_snap.timingWhy = why;
    if (mode != TimingMode::Capture) {
        m_snap.capturePaused = false;
        m_snap.capturePausedWhy.clear();
    }
    updateDelayModel();
}

void Source::onServerClockId(const std::string& clockId) {
    if (!m_cfg.captureTiming) {
        setTiming(TimingMode::Arrival, "capture timing is off in the configuration");
    } else if (clockId.empty()) {
        setTiming(TimingMode::Arrival, "the receiver did not say which clock its timestamps are on");
    } else if (hostClockId().empty()) {
        setTiming(TimingMode::Arrival, "this host has no boot id to compare clocks by");
    } else if (clockId != hostClockId()) {
        setTiming(TimingMode::Arrival, "the receiver is on another host, so its timestamps are on another clock");
    } else {
        setTiming(TimingMode::Capture, "the receiver is on this host and stamps capture times on its clock");
    }
}

bool Source::timeBlock(std::uint64_t stampNanos, int rate, double arrivalSec, double dmrSec) {
    switch (m_timing) {
    case TimingMode::Arrival:
        return true;
    case TimingMode::Pending: {
        // Untimed until the receiver says; a receiver that never does is timed
        // by arrival, which needs nothing from it.
        const double now = monotonicNow();
        if (m_timingPendingSince <= 0.0) {
            m_timingPendingSince = now;
        } else if (now - m_timingPendingSince > kTimingPendingSec) {
            setTiming(TimingMode::Arrival, "the receiver has not said which clock its timestamps are on");
        }
        return false;
    }
    case TimingMode::Capture:
        break;
    }

    m_slew.sample(arrivalSec, dmrSec);
    std::string why;
    const bool trusted = m_slew.steady(arrivalSec, &why);
    if (trusted != m_captureTrusted) {
        m_captureTrusted = trusted;
        if (trusted) LOG_INFO(m_cfg.name.c_str(), "capture times trusted again");
        else LOG_WARN(m_cfg.name.c_str(), "capture times not trusted: %s", why.c_str());
    }

    // The capture instant on the daemon clock: the stamp is on the host clock,
    // and so is dmrSec's other half, read as this packet arrived. Only the
    // capture-to-arrival interval is ever taken on the host clock.
    double capture = 0.0, lag = 0.0;
    bool usable = trusted && stampNanos != 0 && rate > 0;
    if (usable) {
        capture = static_cast<double>(stampNanos / 1000000000ULL) +
                  static_cast<double>(stampNanos % 1000000000ULL) * 1e-9 + dmrSec;
        lag = arrivalSec - capture;
        // Captured after it arrived, or a second before: not this packet's
        // capture time on this clock (a step of the host clock in between).
        usable = lag > 0.0 && lag < kMaxCaptureLagSec;
    }
    if (usable) m_capture.observe(m_samplesWritten, capture);

    std::lock_guard<std::mutex> lk(m_mu);
    m_snap.capturePaused = !trusted;
    m_snap.capturePausedWhy = trusted ? std::string() : why;
    m_snap.hostSlewPpm = m_slew.deviationPpm();
    if (usable) m_snap.captureLagSec = lag;
    else ++m_snap.captureUntimed;
    return false;   // the arrival fit is not used for this source
}

bool Source::hostTimeAt(double sample, double& hostSec) const {
    return m_timing == TimingMode::Capture ? m_capture.hostTimeAt(sample, hostSec)
                                           : m_clock.hostTimeAt(sample, hostSec);
}

void Source::discardTiming(const char* why) {
    // The decoder's own sample indices stay consistent with each other, so its
    // lock survives; what cannot be trusted is their mapping to host time, and
    // the UTC anchor that was composed through it.
    m_clock.reset();
    m_capture.reset();
    std::lock_guard<std::mutex> lk(m_mu);
    m_haveAnchor = false;
    m_offsets.clear();
    m_snap.haveOffset = false;
    m_snap.offsetSamples = 0;
    m_snap.clockFitValid = false;
    LOG_DEBUG(m_cfg.name.c_str(), "timing discarded (%s)", why);
}

// 60 kHz tuned on the carrier is MSF or WWVB, and which is decided by where
// the receiver is: the nearer transmitter. Anthorn serves Europe and Fort
// Collins North America, 7000 km apart, so there is no receiver for which the
// choice is close. Unknown while /api/description has not yet answered; WWVB,
// as 60 kHz always was here, when it answered without coordinates or not at
// all -- and then coordinates that turn up later still decide.
clockdec::ClockStation Source::resolveLf60() {
    GeoPoint loc;
    {
        std::lock_guard<std::mutex> lk(m_mu);
        loc = m_snap.receiverLocation;
    }
    if (loc.valid) {
        const double dm = greatCircleMeters(loc, msfSite());
        const double dw = greatCircleMeters(loc, wwvbSite());
        return dm <= dw ? clockdec::ClockStation::Msf : clockdec::ClockStation::Wwvb;
    }
    if (m_lf60 != clockdec::ClockStation::Unknown) return m_lf60;
    if (!m_descriptionTried.load()) return clockdec::ClockStation::Unknown;
    return clockdec::ClockStation::Wwvb;
}

bool Source::ensureDecoder(int rate) {
    if (m_broadcast == Broadcast::Lf60) {
        // Nothing is decoded until the station is decided -- a few hundred
        // milliseconds after the socket opens, when the description answers.
        const clockdec::ClockStation want = resolveLf60();
        if (want == clockdec::ClockStation::Unknown) return false;
        if (want != m_lf60) {
            GeoPoint loc;
            {
                std::lock_guard<std::mutex> lk(m_mu);
                loc = m_snap.receiverLocation;
                m_snap.station = want == clockdec::ClockStation::Msf ? "msf" : "wwvb";
                updateDelayModel();
            }
            if (loc.valid) {
                LOG_INFO(m_cfg.name.c_str(), "60 kHz: the receiver is %.0f km from MSF and %.0f km from WWVB; "
                         "decoding %s", greatCircleMeters(loc, msfSite()) / 1000.0,
                         greatCircleMeters(loc, wwvbSite()) / 1000.0,
                         want == clockdec::ClockStation::Msf ? "MSF" : "WWVB");
            } else {
                LOG_WARN(m_cfg.name.c_str(), "60 kHz: the receiver publishes no coordinates, so MSF "
                         "cannot be told from WWVB by where it is; decoding WWVB");
            }
            m_lf60Fallback = !loc.valid;
            m_lf60 = want;
            m_decoderRate = 0;   // (re)build below
        }
    }
    if (m_decoderRate == rate && (m_wwv || m_wwvb || m_dcf77 || m_msf || m_allouis)) return true;
    if (m_decoderRate == rate && m_rateRefused) return false;   // said once, not per packet

    // The decoders decimate to a fixed series rate — 200 Hz for WWV/WWVH,
    // 100 Hz for WWVB and DCF77 — so a rate that is not a multiple of it decimates
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
        m_dcf77.reset();
        m_msf.reset();
        m_allouis.reset();
        m_samplesWritten = 0;
        m_tsHave = false;
        m_clock.reset();
        m_capture.reset();
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
    m_dcf77.reset();
    m_msf.reset();
    m_allouis.reset();
    m_wwvbFromIq = false;
    m_decoderRate = rate;
    m_samplesWritten = 0;
    m_lastFrameSamples = rate / 50;
    m_tsHave = false;
    m_clock.setSampleRate(rate);
    m_clock.reset();
    m_capture.setSampleRate(rate);
    m_capture.reset();
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

    // WWV/WWVH, WWVB and DCF77 are genuinely different decoders — a 100 Hz BCD
    // subcarrier, pulse-width modulation on the carrier's own amplitude, and
    // that plus a spread-spectrum phase code on IQ — so this is decided from
    // the tuning (broadcastFor), not offered as a setting. WWV and WWVH share
    // one decoder and it identifies which it is hearing itself.
    const bool wwvb = m_broadcast == Broadcast::Wwvb ||
                      (m_broadcast == Broadcast::Lf60 && m_lf60 == clockdec::ClockStation::Wwvb);

    // The plausibility gate. A deep fade zero-biases the same bits in every
    // frame of the voter's window, so the misread is unanimous and no
    // agreement-based metric can catch it; an independent reference clock is
    // the only thing that can veto it. The host clock is that reference — which
    // is circular only in appearance, because the bound is a day and the error
    // being measured is milliseconds.
    auto reference = [] { return hostNowFields(static_cast<long long>(realtimeNow() * 1000.0)); };

    auto wire = [&](auto& d) {
        d->onStateChanged = [this](clockdec::ClockLockState s) { onClockState(s); };
        d->onSecond = [this](const clockdec::ClockSecondInfo& i) { onClockSecond(i); };
        d->onFrame = [this](const clockdec::ClockFrameInfo& f) { onClockFrame(f); };
        d->onTime = [this](const clockdec::ClockTimeInfo& t) { onClockTime(t); };
        d->setPlausibility(reference, 24 * 60);
    };
    // The carrier's place in the baseband: 0 unless dial_hz moved it.
    const double offsetHz = static_cast<double>(m_cfg.carrierHz) - static_cast<double>(m_cfg.dialHz);

    if (m_broadcast == Broadcast::Allouis) {
        m_allouis = std::make_unique<clockdec::AllouisDecoder>(rate, offsetHz);
        wire(m_allouis);
    } else if (m_broadcast == Broadcast::Lf60 && m_lf60 == clockdec::ClockStation::Msf) {
        m_msf = std::make_unique<clockdec::MsfDecoder>(rate, offsetHz);
        wire(m_msf);
    } else if (m_broadcast == Broadcast::Lf60) {
        // WWVB from IQ: its decoder wants the USB audio of a 59 kHz dial, with
        // the carrier at 1000 Hz. The IQ is low-passed to +/-900 Hz -- WWVB's
        // keying needs tens of hertz -- so that nothing below the carrier folds
        // into the audio, shifted up by 1000 Hz, and its real part taken.
        m_wwvb = std::make_unique<clockdec::WwvbDecoder>(rate);
        wire(m_wwvb);
        m_wwvbFromIq = true;
        const double w0 = 2.0 * 3.14159265358979323846 * 900.0 / rate;
        const double c = std::cos(w0), sn = std::sin(w0);
        // Two Butterworth sections (Q 0.541, 1.307): fourth order, no overshoot to speak of.
        const double qs[2] = {0.54119610014619698, 1.3065629648763766};
        double delay = 0.0;
        for (int sec = 0; sec < 2; ++sec) {
            const double alpha = sn / (2.0 * qs[sec]);
            const double a0 = 1.0 + alpha;
            IqBiquad b;
            b.b0 = ((1.0 - c) / 2.0) / a0; b.b1 = (1.0 - c) / a0; b.b2 = ((1.0 - c) / 2.0) / a0;
            b.a1 = (-2.0 * c) / a0; b.a2 = (1.0 - alpha) / a0;
            m_iqLp[static_cast<std::size_t>(sec)] = b;       // I
            m_iqLp[static_cast<std::size_t>(sec + 2)] = b;   // Q
            // DC group delay, samples: the edges WWVB is timed by are slow.
            delay += 1.0 - (b.a1 + 2.0 * b.a2) / (1.0 + b.a1 + b.a2);
        }
        m_iqToAudioDelaySec = delay / rate;
        m_iqShiftPhase = 0.0;
        m_iqShiftStep = 2.0 * 3.14159265358979323846 * (1000.0 - offsetHz) / rate;
    } else if (m_iq) {
        m_dcf77 = std::make_unique<clockdec::Dcf77Decoder>(rate, offsetHz);
        m_dcf77->onStateChanged = [this](clockdec::ClockLockState s) { onClockState(s); };
        m_dcf77->onSecond = [this](const clockdec::ClockSecondInfo& i) { onClockSecond(i); };
        m_dcf77->onFrame = [this](const clockdec::ClockFrameInfo& f) { onClockFrame(f); };
        m_dcf77->onTime = [this](const clockdec::ClockTimeInfo& t) { onClockTime(t); };
        m_dcf77->setPlausibility(reference, 24 * 60);
    } else if (wwvb) {
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

    // The station tag, before any audio. A WWV-only carrier fixes it. Otherwise
    // the tag the last decoder held carries over: a reconnect or a restart the
    // consensus ordered throws the decoder away, and without this the tag went
    // blank -- and the propagation model fell back to the dial's default, WWV,
    // 14 ms from WWVH on a European path -- for the half-minute and more a new
    // decoder needs to judge it again, when propagation had not changed in the
    // seconds the restart took. A carried tag is still judged, and switched or
    // released on the usual evidence, once the new decoder has heard enough.
    static constexpr double kStationMemorySec = 900.0;
    std::string stationNote;
    if (m_wwv && wwvOnlyCarrier(m_cfg.carrierHz)) {
        m_wwv->pinStation(clockdec::ClockStation::Wwv);
        stationNote = ", station WWV (the only one on this carrier)";
    } else if (m_wwv && m_stationMemory != clockdec::ClockStation::Unknown &&
               monotonicNow() - m_stationMemoryAt < kStationMemorySec) {
        m_wwv->presetStation(m_stationMemory);
        stationNote = std::string(", station tag carried over: ") +
                      (m_stationMemory == clockdec::ClockStation::Wwvh ? "WWVH" : "WWV");
    }

    LOG_INFO(m_cfg.name.c_str(), "decoder started: %s at %d Hz%s",
             m_dcf77 ? "DCF77 (AM + PM, IQ)" : m_allouis ? "Allouis (PM, IQ)" : m_msf ? "MSF (IQ)"
             : m_wwvbFromIq ? "WWVB (from IQ)" : wwvb ? "WWVB" : "WWV/WWVH", rate, stationNote.c_str());
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

    // `count` is frames; an IQ frame is two int16s, I then Q.
    const std::size_t values = static_cast<std::size_t>(count) * m_channels;
    if (m_mono.size() < values) m_mono.resize(values);
    for (std::size_t i = 0; i < values; ++i) m_mono[i] = pcm[i] * (1.0f / 32768.0f);

    const std::size_t n = static_cast<std::size_t>(count);
    if (m_dcf77) m_dcf77->process(m_mono.data(), n);
    else if (m_allouis) m_allouis->process(m_mono.data(), n);
    else if (m_msf) m_msf->process(m_mono.data(), n);
    else if (m_wwv) m_wwv->process(m_mono.data(), n);
    else if (m_wwvbFromIq) {
        if (m_audio.size() < n) m_audio.resize(n);
        auto run = [](IqBiquad& b, double x) {
            const double y = b.b0 * x + b.z1;
            b.z1 = b.b1 * x - b.a1 * y + b.z2;
            b.z2 = b.b2 * x - b.a2 * y;
            return y;
        };
        for (std::size_t i = 0; i < n; ++i) {
            const double I = run(m_iqLp[1], run(m_iqLp[0], m_mono[2 * i]));
            const double Q = run(m_iqLp[3], run(m_iqLp[2], m_mono[2 * i + 1]));
            // Re((I + jQ) e^{+j phi}): the baseband moved up to 1 kHz.
            m_audio[i] = static_cast<float>(I * std::cos(m_iqShiftPhase) - Q * std::sin(m_iqShiftPhase));
            m_iqShiftPhase += m_iqShiftStep;
            if (m_iqShiftPhase > 6.283185307179586) m_iqShiftPhase -= 6.283185307179586;
        }
        m_wwvb->process(m_audio.data(), n);
    } else m_wwvb->process(m_mono.data(), n);

    // Diagnostics are assembled on call from state the decoder already holds,
    // so this costs nothing on the sample path and keeps the status report
    // current without a second timer reaching into the decoder.
    const auto d = m_dcf77 ? m_dcf77->diagnostics() : m_allouis ? m_allouis->diagnostics()
                 : m_msf ? m_msf->diagnostics() : m_wwv ? m_wwv->diagnostics() : m_wwvb->diagnostics();
    const auto st = m_dcf77 ? m_dcf77->station() : m_allouis ? m_allouis->station()
                  : m_msf ? m_msf->station() : m_wwv ? m_wwv->station() : m_wwvb->station();
    const auto consumed = m_dcf77 ? m_dcf77->samplesConsumed() : m_allouis ? m_allouis->samplesConsumed()
                        : m_msf ? m_msf->samplesConsumed()
                        : m_wwv ? m_wwv->samplesConsumed() : m_wwvb->samplesConsumed();

    if (m_wwv) {
        if (st == clockdec::ClockStation::Unknown) {
            m_stationMemory = st;   // the decoder let it go on the evidence; so do we
        } else if (d.phaseLocked) {
            m_stationMemory = st;
            m_stationMemoryAt = monotonicNow();
        }
    }

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
    if (m_allouis || m_msf) {
        // Allouis is timed by its phase alone: the same fields as DCF77's PM.
        m_snap.pmLocked = d.pmLocked;
        m_snap.pmSnrDb = d.pmSnrDb;
        m_snap.timingFromPm = d.timingFromPm;
        m_snap.carrierOffsetHz = d.carrierOffsetHz;
    }
    if (m_dcf77) {
        m_snap.pmLocked = d.pmLocked;
        m_snap.pmSnrDb = d.pmSnrDb;
        m_snap.timingFromPm = d.timingFromPm;
        m_snap.amMinusPmMs = d.amMinusPmMs;
        m_snap.carrierOffsetHz = d.carrierOffsetHz;
        static const char* const kFrom[] = {"", "am", "pm", "both", "conflict"};
        m_snap.frameFrom = kFrom[std::min<int>(d.lastFrameFrom, 4)];
        m_snap.pmRefusedLocks = d.pmRefusedLocks;
        m_snap.pmInterference = d.pmInterference;
        // Which of the two is timing the second is the most useful thing the
        // log can say about a DCF77 source -- once it has held for half a
        // minute. On a marginal path PM comes and goes by the second, and a
        // line each time is noise in a log meant to be read after months.
        const int timing = d.phaseLocked ? static_cast<int>(d.timingFromPm) : -1;
        const double now = monotonicNow();
        if (timing != m_timingSeen) { m_timingSeen = timing; m_timingSeenAt = now; }
        if (timing >= 0 && timing != m_loggedTimingPm && now - m_timingSeenAt >= 30.0) {
            LOG_INFO(m_cfg.name.c_str(), "second edges from %s for the last 30 s%s",
                     timing ? "PM" : "AM", timing ? "" : " (phase modulation not tracking)");
            m_loggedTimingPm = timing;
        }
    }
    switch (static_cast<clockdec::ClockLockRefusal>(d.refusalReason)) {
        case clockdec::ClockLockRefusal::QualityFloor: m_snap.refusal = "quality_floor"; break;
        case clockdec::ClockLockRefusal::Plausibility: m_snap.refusal = "plausibility"; break;
        case clockdec::ClockLockRefusal::Staleness:    m_snap.refusal = "staleness"; break;
        case clockdec::ClockLockRefusal::Contested:    m_snap.refusal = "contested"; break;
        default: m_snap.refusal = "none"; break;
    }
    // DCF77 is the only thing on 77.5 kHz, as WWV is on 20 and 25 MHz: the tag
    // is the carrier's, and does not blink to "unknown" while a new decoder
    // looks for it.
    const char* station = m_dcf77 ? "dcf77" : m_allouis ? "allouis" : m_msf ? "msf" : "unknown";
    switch (st) {
        case clockdec::ClockStation::Wwv:  station = "wwv"; break;
        case clockdec::ClockStation::Wwvh: station = "wwvh"; break;
        case clockdec::ClockStation::Wwvb: station = "wwvb"; break;
        case clockdec::ClockStation::Dcf77: station = "dcf77"; break;
        case clockdec::ClockStation::Msf: station = "msf"; break;
        case clockdec::ClockStation::Allouis: station = "allouis"; break;
        default: break;
    }
    if (m_snap.station != station) {
        // Every change, with the evidence, because the tag moves the delay
        // model by the difference between two transmitter paths.
        if (std::isfinite(d.tickBandRatioDb)) {
            LOG_INFO(m_cfg.name.c_str(), "station tag %s -> %s (tick 2000/2200 Hz %+.1f dB)",
                     m_snap.station.c_str(), station, d.tickBandRatioDb);
        } else {
            LOG_INFO(m_cfg.name.c_str(), "station tag %s -> %s", m_snap.station.c_str(), station);
        }
        m_snap.station = station;
    }

    if (m_timing == TimingMode::Capture) {
        // No fit: each packet times its own samples. See CaptureClock.
        m_snap.clockFitValid = m_capture.marks() > 0;
        m_snap.clockResidualSec = 0.0;
        m_snap.clockSpanSec = m_capture.spanSec();
        m_snap.clockPpm = 0.0;
        m_snap.clockSlopeUncSec = 0.0;
        m_snap.clockSlopeHeld = false;
        m_snap.lastExcessDelaySec = 0.0;
    } else {
        const ClockFit f = m_clock.fit();
        m_snap.clockFitValid = f.valid;
        m_snap.clockResidualSec = f.residualRms;
        m_snap.clockSpanSec = f.spanSec;
        m_snap.clockPpm = f.valid && m_decoderRate > 0
            ? (f.secPerSample * m_decoderRate - 1.0) * 1e6 : 0.0;
        m_snap.clockSlopeUncSec = f.slopeUncertaintySec;
        m_snap.clockSlopeHeld = f.slopeHeld;
        m_snap.lastExcessDelaySec = m_clock.lastExcessDelay();
    }

    updateDelayModel();
    recomputeOffset();
}

void Source::resetStream(const char* why) {
    m_clock.reset();
    m_capture.reset();
    // Decided afresh for every connection, from what the receiver says.
    m_timing = TimingMode::Pending;
    m_timingPendingSince = 0.0;
    m_captureTrusted = true;
    if (m_pcmv4) m_pcmv4->dec.reset();
    m_samplesWritten = 0;
    m_decoderRate = 0;
    m_rateRefused = false;
    m_lastFrameSamples = 0;
    m_tsHave = false;
    m_wwv.reset();
    m_wwvb.reset();
    m_dcf77.reset();
    m_msf.reset();
    m_allouis.reset();
    m_wwvbFromIq = false;

    std::lock_guard<std::mutex> lk(m_mu);
    m_snap.timingMode = "pending";
    m_snap.timingWhy = "waiting for the receiver to say which clock its timestamps are on";
    m_snap.capturePaused = false;
    m_snap.capturePausedWhy.clear();
    m_haveAnchor = false;
    m_haveFrame = false;
    m_offsets.clear();
    m_leapFrames = 0;
    m_snap.leapPending = false;
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
    if (m_snap.leapPending) m_leapWarnAt = daemonNow();
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

    // When the edge this timestamp names was observed here, for the continuity
    // check below. Without it the check cannot run -- and no offset could be
    // formed from the anchor yet either -- so the anchor waits for the next
    // minute rather than going in unchecked.
    double edgeHostSec = 0.0;
    // At the decoder's own resolution, not rounded to a whole sample: see
    // ClockSecondInfo::edgeSampleExact.
    const double edgeAt = std::isfinite(t.lastEdgeSampleExact) ? t.lastEdgeSampleExact
                                                               : static_cast<double>(t.lastEdgeSample);
    if (!hostTimeAt(edgeAt, edgeHostSec)) return;

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

    // UTC does not jump. What this decode says the offset is, against what the
    // source's own history says it must be.
    const double impliedOffset =
        static_cast<double>(decodedMs) / 1000.0 - edgeHostSec + m_snap.delaySec;
    if (!admitDecodedTime(impliedOffset, edgeHostSec, decodedMs)) return;

    m_anchorEdgeSample = t.lastEdgeSample;
    m_anchorUtcMs = decodedMs;
    m_haveAnchor = true;
    m_anchorSetAt = monotonicNow();
    m_lastTimeAt = m_anchorSetAt;

    m_snap.lastQuality = std::clamp(static_cast<int>(std::lround(t.quality * 100.0)), 0, 100);
    m_snap.lastDecodedUtc = iso8601(decodedMs);
}

// Whether a decoded time may anchor this source. The decision is
// TimeContinuity's; this says it -- to the log, the status page and the chart.
bool Source::admitDecodedTime(double impliedOffsetSec, double atDaemon, long long decodedMs) {
    using O = TimeContinuity::Outcome;
    const bool leapWindow = isLastDayOfMonth(decodedMs - 2LL * 3600 * 1000) &&
                            !isLastDayOfMonth(decodedMs) && atDaemon - m_leapWarnAt < 86400.0;
    const bool wasRefusing = m_snap.timeCheck.rfind("refused", 0) == 0;
    const TimeContinuity::Verdict v = m_continuity.judge(impliedOffsetSec, atDaemon, leapWindow);
    const std::string when = iso8601(decodedMs);

    // A refusal is recorded whether or not the time goes on to be taken: it
    // was a jump either way, and the chart is of jumps.
    if (v.outcome == O::Refused || v.outcome == O::Adopted) {
        ++m_snap.timeRejections;
        m_snap.lastRejectedJumpSec = v.jumpSec;
        m_snap.lastRejectedUtc = when;
        if (m_rejectedJumps.size() < 256) m_rejectedJumps.push_back(v.jumpSec);
    }
    if (v.discardFiltered()) m_offsets.clear();

    switch (v.outcome) {
    case O::Accepted:
        if (wasRefusing) {
            LOG_INFO(m_cfg.name.c_str(), "decoded %s agrees with its own history again",
                     when.c_str());
        }
        m_snap.timeCheck.clear();
        return true;
    case O::FirstConfirmed:
        LOG_INFO(m_cfg.name.c_str(), "decoded time %s confirmed by %d readings over %.0f s; "
                 "using it", when.c_str(), v.readings, v.spanSec);
        m_snap.timeCheck.clear();
        return true;
    case O::LeapSecond:
        LOG_WARN(m_cfg.name.c_str(), "decoded %s is %s from its history after an announced "
                 "leap second; taking it", when.c_str(), jumpText(v.jumpSec).c_str());
        m_snap.timeCheck.clear();
        return true;
    case O::Adopted:
        LOG_WARN(m_cfg.name.c_str(), "decoded time has held %s from its old history for %.0f min "
                 "over %d readings; taking it as the time and discarding the old history",
                 jumpText(v.jumpSec).c_str(), v.spanSec / 60.0, v.readings);
        ++m_snap.timeAdoptions;
        m_snap.timeCheck.clear();
        return true;
    case O::Confirming:
        m_snap.timeCheck = "confirming the first decoded time: " + std::to_string(v.readings) +
                           " reading(s) of " + when + " agree, " +
                           std::to_string(TimeContinuity::kFirstReadings) + " over 2 min needed";
        return false;
    case O::Refused: {
        if (v.readings == 1) {
            LOG_WARN(m_cfg.name.c_str(), "decoded %s, %s from where its own history puts it; "
                     "refusing it (time does not jump)", when.c_str(), jumpText(v.jumpSec).c_str());
        }
        const int minutesLeft = static_cast<int>(
            std::ceil(std::max(0.0, TimeContinuity::kAdoptSpanSec - v.spanSec) / 60.0));
        m_snap.timeCheck = "refused decoded " + when + ": " + jumpText(v.jumpSec) +
                           " from its own history, and time does not jump; taken only if it "
                           "holds" + (minutesLeft > 0 ? " for " + std::to_string(minutesLeft) +
                                                            " more min"
                                                      : std::string(" one more reading"));
        return false;
    }
    }
    return false;
}

std::vector<double> Source::takeRejectedJumps() {
    std::lock_guard<std::mutex> lk(m_mu);
    std::vector<double> out;
    out.swap(m_rejectedJumps);
    return out;
}

void Source::onClockSecond(const clockdec::ClockSecondInfo& i) {
    // Only an edge the decoder actually measured becomes an offset sample.
    // Second 0 carries no pulse, and a weak or coasted second reports where the
    // tracker expects the edge rather than where it was heard: correct, but it
    // is the tracker's opinion counted again, and it would weigh the median
    // towards the estimate instead of the signal.
    if (!i.edgeMeasured) return;

    // When the sample carrying this edge was observed here, on the daemon clock
    // the fit is taken on and the offset is measured against.
    double hostSec = 0.0;
    const double edgeAt = std::isfinite(i.edgeSampleExact) ? i.edgeSampleExact
                                                           : static_cast<double>(i.edgeSample);
    if (!hostTimeAt(edgeAt, hostSec)) return;

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
    addOffsetSample(raw + m_snap.delaySec, hostSec);
    recomputeOffset();
}

void Source::addOffsetSample(double offsetSec, double atDaemon) {
    // Dated by the edge, not by when this code ran: the rate is fitted against
    // these instants, and the gap between an edge and its processing is audio
    // buffering that has nothing to do with the crystal.
    m_offsets.add(atDaemon, offsetSec);
}

void Source::recomputeOffset() {
    if (m_offsets.empty()) {
        m_snap.haveOffset = false;
        m_snap.offsetSamples = 0;
        m_snap.offsetHeld = false;
        m_snap.rejoinGapSec = 0.0;
        return;
    }

    // Filtered once per new sample rather than once per packet: the estimator
    // keeps its result until a sample arrives, and what below changes with
    // every packet -- the age and the sample-clock terms -- is cheap.
    // OffsetEstimator.h has the level/rate split and why.
    const OffsetEstimate& e = m_offsets.estimate();
    m_snap.offsetAgeSec = daemonNow() - e.atSec;
    m_snap.offsetSamples = e.samples;

    if (m_snap.offsetAgeSec > kOffsetStaleSec) {
        m_snap.haveOffset = false;
        return;
    }

    m_snap.offsetSec = e.offsetSec;
    m_snap.offsetAtSec = e.atSec;
    m_snap.offsetRate = e.rate;
    m_snap.offsetRateUncertainty = e.rateUncertainty;
    m_snap.offsetRateMeasured = e.rateMeasured;
    m_snap.offsetRateSpanSec = e.rateSpanSec;
    m_snap.rateTermSec = e.rateTermSec;
    m_snap.jitterSec = e.jitterSec;
    m_snap.offsetHeld = e.held;
    m_snap.offsetHeldForSec = e.heldForSec;
    m_snap.liveOffsetSec = e.liveOffsetSec;
    m_snap.rejoinGapSec = e.rejoinGapSec;
    m_snap.jitterBaselineSec = e.jitterBaselineSec;
    m_snap.spikeThresholdSec = e.spikeThresholdSec;
    m_snap.spikeHolds = e.holds;
    m_snap.spikeHoldsTimedOut = e.holdsTimedOut;
    if (e.holds > m_loggedHolds) {
        LOG_WARN(m_cfg.name.c_str(),
                 "jitter %.0f us against a usual %.0f us: holding the offset at its last calm "
                 "level (live level %+.0f us from it)",
                 e.jitterSec * 1e6, e.jitterBaselineSec * 1e6, (e.liveOffsetSec - e.offsetSec) * 1e6);
        m_loggedHolds = e.holds;
        m_loggedHeld = true;
    } else if (m_loggedHeld && !e.held) {
        LOG_INFO(m_cfg.name.c_str(), "%s; slewing out the %+.0f us between the held and live levels",
                 e.holdsTimedOut > m_loggedTimeouts ? "jitter stayed high past the hold limit, taking the live level"
                                                    : "hold ended",
                 e.rejoinGapSec * 1e6);
        m_loggedTimeouts = e.holdsTimedOut;
        m_loggedHeld = false;
    }

    // What this source claims to be worth. Independent terms:
    //   jitter          how much the measurements disagree with each other
    //   clock residual  how well the sample-to-daemon-clock mapping fits
    //   clock slope     how well that mapping's rate is known
    //   rate term       how far the offset's own rate could have carried it
    //   delay           how wrong the delay model could be, which is the term
    //                   nothing can measure and therefore the one that usually
    //                   dominates
    // The tight floor only while every condition that earns it holds: timing
    // falling back to AM, or capture timing pausing, puts the 10 ms back at once.
    const bool lfPmCapture = m_cfg.autoDelay && m_broadcast == Broadcast::Dcf77 &&
                             m_snap.timingMode == "capture" && m_snap.pmLocked && m_snap.timingFromPm &&
                             !m_snap.capturePaused && m_snap.receiverLocation.valid &&
                             greatCircleMeters(m_snap.receiverLocation, dcf77Site()) <= kLfGroundwaveServiceM;
    const bool wwvCapture = m_cfg.autoDelay && m_broadcast == Broadcast::Wwv &&
                            m_snap.timingMode == "capture" && !m_snap.capturePaused &&
                            (m_snap.station == "wwv" || m_snap.station == "wwvh") &&
                            m_snap.receiverLocation.valid;
    // Allouis, likewise: timed by its phase, capture-timed, and inside the
    // groundwave's reach of Allouis.
    const bool allouisCapture = m_cfg.autoDelay && m_broadcast == Broadcast::Allouis &&
                                m_snap.timingMode == "capture" && m_snap.pmLocked && m_snap.timingFromPm &&
                                !m_snap.capturePaused && m_snap.receiverLocation.valid &&
                                greatCircleMeters(m_snap.receiverLocation, allouisSite()) <= kLfGroundwaveServiceM;
    double delayUncertainty;
    if (lfPmCapture || allouisCapture) {
        delayUncertainty = std::max(kDelayUncertaintyFloorLfPmSec, m_snap.delaySec * kDelayUncertaintyFraction);
    } else if (wwvCapture) {
        const GeoPoint tx = m_snap.station == "wwvh" ? wwvhSite() : wwvSite();
        delayUncertainty = skywaveModeSpreadSeconds(greatCircleMeters(m_snap.receiverLocation, tx)) +
                           kWwvDecoderAllowanceSec;
    } else {
        delayUncertainty = std::max(kDelayUncertaintyFloorSec, m_snap.delaySec * kDelayUncertaintyFraction);
    }
    const double own = e.jitterSec + m_snap.clockResidualSec + m_snap.clockSlopeUncSec + e.rateTermSec;
    m_snap.dispersionSec = own + delayUncertainty;

    // And what it is worth against the others. The delay term is deliberately
    // absent: every source runs the same delay model, so including it tells the
    // selector only that all of them share a doubt, while drowning out the
    // things that actually differ. Those are all measured, and a source whose
    // sample clock has just been rebuilt or whose slope was refused says so
    // here rather than waiting to be noticed by hand.
    //
    // The floor keeps a source that reports a suspiciously perfect zero -- a
    // synthetic stream, or a window too short to have scattered yet -- from
    // taking an unbounded share of the weight. A millisecond is about the
    // decoder's own edge resolution, so no honest source is below it.
    m_snap.weightDispersionSec = std::max(kWeightDispersionFloorSec, own);

    // Fewer than a handful of measurements is not a filtered value, whatever
    // its spread happens to be.
    m_snap.haveOffset = e.valid && m_snap.clockState == "locked";

    // The history the next decoded time is judged against. Only a filtered
    // value: a lone first reading is not yet a history.
    if (e.valid) m_continuity.noteFiltered(e.offsetSec, e.rateMeasured ? e.rate : 0.0, e.atSec);
}

void Source::recordWsRtt(double rttSec) {
    if (!(rttSec > 0.0)) return;
    m_wsRttProbes.push_back(rttSec);
    // The same minimum-of-recent rule as the HTTP probe, for the same reason:
    // a pong that crossed a busy moment describes the moment, not the path.
    while (m_wsRttProbes.size() > 12) m_wsRttProbes.pop_front();
    double best = m_wsRttProbes.front();
    for (double v : m_wsRttProbes) best = std::min(best, v);
    m_snap.wsRttMs = best * 1000.0;
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
    bool lf = false;
    bool undecided = false;
    if (m_broadcast == Broadcast::Dcf77) { tx = dcf77Site(); lf = true; }
    else if (m_broadcast == Broadcast::Allouis) { tx = allouisSite(); lf = true; }
    else if (m_snap.station == "msf") { tx = msfSite(); lf = true; }
    else if (m_broadcast == Broadcast::Lf60 && m_snap.station != "wwvb") undecided = true;
    else if (m_snap.station == "wwvh") tx = wwvhSite();
    else if (m_snap.station == "wwvb" || m_broadcast == Broadcast::Wwvb) { tx = wwvbSite(); lf = true; }
    else tx = wwvSite();

    double prop = 0.0;
    if (undecided) {
        m_snap.pathDescription = "60 kHz: MSF or WWVB not yet decided (it waits for the receiver's coordinates)";
    } else if (m_snap.receiverLocation.valid) {
        // 60 and 77.5 kHz get the groundwave, not F-layer hops. See Propagation.h.
        const double d = greatCircleMeters(m_snap.receiverLocation, tx);
        prop = lf ? lfDelaySeconds(d) : skywaveDelaySeconds(d);
        m_snap.pathDescription = lf ? describeLfPath(m_snap.receiverLocation, tx)
                                    : describePath(m_snap.receiverLocation, tx);
    } else {
        m_snap.pathDescription = "no receiver coordinates; propagation not modelled";
    }

    // Half the round trip, as the one-way network delay.
    //
    // The MINIMUM of the recent probes, for the same reason SampleClock keeps
    // the minimum of packet arrivals: network delay is bounded below and
    // unbounded above, so a probe that happened to cross a busy moment says
    // nothing about the path and the smallest one says the most.
    //
    // Measured over the WEBSOCKET, not over an HTTP request, whenever a pong
    // has come back. The two agree on a receiver served directly -- 93 ms
    // against 99 on one measured here -- and disagree by everything that
    // matters on a receiver behind a CDN: a TCP handshake ends at whatever
    // accepted the SYN, so a Cloudflare edge 13 ms away answers for an origin
    // 93 ms away, and the delay model loses 40 ms it cannot get back. The
    // audio comes from the origin either way, and a ping down the audio
    // connection has to reach the origin to be answered. The HTTP figure stays
    // as the fallback for a server too old to pong, where an under-estimate is
    // still better than dropping the largest correctable term entirely.
    // One method for every source, and the WebSocket is the one that is always
    // measuring the right thing. A TCP handshake times the path to whatever
    // accepted the SYN, and most UberSDR instances are reached through a tunnel
    // that terminates TCP near the client -- one measured here answered in 13 ms
    // for an origin 91 ms away. The ping has to be an APPLICATION ping, the JSON
    // one the server answers itself: a proxy that terminates the WebSocket
    // answers RFC 6455 pings on its own, and tunnel.ubersdr.org now does --
    // 13.4 ms by protocol ping against 33.7 by JSON ping, on a receiver where
    // they once agreed to 0.1 ms. See the ping in run().
    //
    // The handshake is the cleaner ruler where it is valid -- the kernel answers
    // a SYN, while a ping waits for the server's event loop, which costs a few
    // milliseconds -- and it is tempting to prefer it when the two agree and
    // fall back to the ping when they do not. That would be a mistake. Using
    // different rulers for different sources makes their errors DIFFER, and a
    // difference between sources is the one thing this daemon cannot calibrate
    // away: it has exactly one absolute reference and it spends it on the terms
    // every source shares. Measuring every source the same way puts the ping's
    // overhead into that shared pile, where the chain constant already lives and
    // where a single calibration removes it. A topology heuristic would move a
    // removable common error into an irremovable per-source one, and it would do
    // it silently, on a threshold nobody can check from the outside.
    //
    // The handshake is still measured and shown as a raw figure, but nothing
    // judges an instance by how it compares -- that would be a per-instance
    // rule by the back door. It does not get a vote.
    const double rttMs = m_snap.wsRttMs > 0.0 ? m_snap.wsRttMs : m_snap.httpRttMs;
    m_snap.rttFromWs = m_snap.wsRttMs > 0.0;
    // With capture timing the samples are timed at the antenna end, so the
    // network is not in the path at all; the round trip is still measured and
    // shown, it just is not charged.
    const bool capture = m_snap.timingMode == "capture";
    const double net = capture ? 0.0 : rttMs > 0.0 ? (rttMs / 1000.0) * 0.5 : 0.0;

    // The decoder's own edge bias, by which decoder is running -- chosen from
    // the dial exactly as ensureDecoder chooses it. Negative: an edge reported
    // early makes the offset read large, so it is taken back off.
    // DCF77's edges are exact to the test's resolution whichever demodulator
    // is timing them (tools/dcf77test), so it has none either.
    // WWV's residual rides here too (kWwvResidualSec), with the term it is most
    // likely to belong to.
    // MSF's and Allouis's decoders put the edge where the station puts the
    // second (MsfDecoder, AllouisDecoder), so neither has one. WWVB taken from
    // IQ is timed through the low-pass that turns it into audio, whose delay
    // is exactly known (ensureDecoder) and taken off here.
    const double decoder = m_broadcast == Broadcast::Wwv ? kWwvDecoderEdgeBiasSec + kWwvResidualSec
                         : m_wwvbFromIq ? m_iqToAudioDelaySec : 0.0;

    m_snap.propagationSec = prop;
    m_snap.networkSec = net;
    m_snap.decoderSec = decoder;
    // The same for an IQ session as a USB one: radiod's channel filter is a
    // linear-phase sinc whose delay is set by the block and overlap, not the
    // passband, so the iq and usb presets are delayed alike.
    m_snap.chainSec = capture ? 0.0 : kUberSdrChainDelaySec;
    m_snap.extraSec = m_cfg.extraDelayMs / 1000.0;
    m_snap.delaySec = prop + net + decoder + m_snap.chainSec + m_snap.extraSec;
}

// ---------------------------------------------------------------------------

// One crystal, one rate: see TimeSource::setSystemRate. Cheap enough to do on
// every pass -- it only writes three doubles -- and the estimator picks it up
// at its next recompute.
void Source::setSystemRate(double rateSec, double uncertaintySec, bool known) {
    std::lock_guard<std::mutex> lk(m_mu);
    m_offsets.setRatePrior(rateSec, uncertaintySec, known);
}

void Source::requestReacquire(const std::string& why) {
    {
        std::lock_guard<std::mutex> lk(m_mu);
        m_reacquireWhy = why;
        ++m_snap.reacquisitions;
    }
    m_reacquire.store(true);
}

void Source::setActive(bool on, const std::string& why) {
    {
        std::lock_guard<std::mutex> lk(m_mu);
        // Kept even when the state is unchanged: the Selector calls this every
        // pass and the REASON moves under it -- "the radio has not had a lock
        // for 4 min" becomes "for 9 min" -- and the status page shows it.
        m_activeReason = why;
    }
    if (m_active.exchange(on) == on) return;

    // The supervisor does the work. It is waiting on m_wakeCv either way --
    // parked at the idle gate, or inside the stream loop -- and it is the only
    // thread allowed to stop the socket, so all this does is wake it.
    { std::lock_guard<std::mutex> lk(m_wake); }
    m_wakeCv.notify_all();
}

SourceSnapshot Source::snapshot() const {
    std::lock_guard<std::mutex> lk(m_mu);
    SourceSnapshot s = m_snap;
    const double now = monotonicNow();
    s.linkAgeSec = now - m_linkSince;
    s.lastAudioAgeSec = m_lastAudioAt > 0.0 ? now - m_lastAudioAt : 1e9;
    s.lastTimeAgeSec = m_lastTimeAt > 0.0 ? now - m_lastTimeAt : 1e9;
    const double nowDaemon = daemonNow();
    if (!m_offsets.empty()) s.offsetAgeSec = nowDaemon - m_offsets.newestAt();

    // The same offset against this host's clock, for people: carried to now
    // along its rate, then moved from the daemon clock onto the host's as the
    // host clock stands at this instant. Nothing is formed from it.
    s.hostOffsetSec = s.offsetSec + s.offsetRate * (nowDaemon - s.offsetAtSec) + daemonMinusRealtime();
    s.rawOffsetSec = s.hostOffsetSec - s.delaySec;

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

    // --- what the Selector reads, in the vocabulary it reads it in ---------
    //
    // The funnel below is the same one the status block renders, said once
    // here so the reason a source is not being used and the reason it is not
    // working are the same sentence. Most specific first: an unlocked decoder
    // also has no offset, and "no offset" would send someone to the wrong
    // place entirely.
    s.kind = SourceKind::Radio;
    s.active = m_active.load();
    s.activeReason = m_activeReason;
    // A locked decoder whose decoded time the continuity check is holding out,
    // with no earlier anchor still extending, has nothing to offer -- and
    // "still filtering" would hide that it is refusing a jump.
    const bool heldOut = !m_snap.timeCheck.empty() && !m_haveAnchor;
    s.ready = s.enabled && s.active && s.clockState == "locked" && !heldOut;
    if (!s.enabled) {
        s.notReadyReason = "disabled in the configuration";
    } else if (!s.active) {
        s.notReadyReason = m_activeReason.empty() ? "held in cold standby" : m_activeReason;
    } else if (s.clockState != "locked") {
        // Name the stage that is actually missing. The most common cause by
        // far -- a passband too narrow to pass the tick image -- is invisible
        // from "not locked" and obvious from "no tick".
        s.notReadyReason =
            s.link != LinkState::Streaming
              ? "no audio: the link is " + std::string(linkStateName(s.link)) +
                    (s.linkDetail.empty() ? "" : " (" + s.linkDetail + ")")
          : !s.toneDetected
              // What "heard" means depends on the decoder: WWV's is the 1 kHz
              // second tick; WWVB and DCF77 are timed off their carriers.
              ? (m_broadcast == Broadcast::Wwv
                     ? std::string("no tick: nothing at 1000 Hz to time a second from")
                     : std::string("no carrier: the transmitter is not heard at ") +
                           (m_broadcast == Broadcast::Dcf77 ? "77.5" : m_broadcast == Broadcast::Allouis ? "162" : "60") + " kHz")
          : !s.phaseLocked                 ? "no edge: the tick is heard but not yet tracked"
          : !s.anchored                    ? "no frame: edges are tracked but no minute has decoded"
          : s.refusal != "none"            ? "frames refused: " + s.refusal
          : s.framesInWindow < 2           ? "voting: " + std::to_string(s.framesInWindow) + " of " +
                                             std::to_string(s.windowSize) + " frames agree so far"
                                           : "decoder not locked yet";
    } else if (heldOut) {
        s.notReadyReason = m_snap.timeCheck;
    } else {
        s.notReadyReason.clear();
    }
    return s;
}

} // namespace ubersdr_ntp
