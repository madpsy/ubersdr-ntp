#include "Status.h"

#include "CivilTime.h"
#include "SampleClock.h"

#include "../third_party/json.hpp"

#include <cmath>
#include <cstdio>
#include <cstring>
#include <sstream>

using nlohmann::json;

namespace ubersdr_ntp {

namespace {

// Every string in these documents that did not come from a literal here --
// a source's link detail above all, which is IXWebSocket's raw HTTP status
// reason and therefore whatever bytes the far server sent -- can hold invalid
// UTF-8. nlohmann's default strict handler throws type_error.316 on that, and
// the callers are detached HTTP threads where an escaped exception is
// std::terminate. U+FFFD in one field is the right outcome; a dead daemon is not.
std::string dumpJson(const nlohmann::json& j, bool pretty) {
    return j.dump(pretty ? 2 : -1, ' ', false, nlohmann::json::error_handler_t::replace);
}

std::string f2(double v, int prec = 2) {
    if (!std::isfinite(v)) return "-";
    char b[48];
    std::snprintf(b, sizeof b, "%.*f", prec, v);
    return b;
}

// The acquisition funnel, as the one word that says where it is stuck.
//
// A time-signal decoder that is not working looks exactly like one working on a
// dead band: it says nothing either way. Naming the first failing stage is the
// single most useful thing this daemon can report, because the most common
// cause by far — a passband too narrow to pass the tick image — is invisible
// otherwise, and the stages after the first failure cannot pass while it is
// down, so only the first is worth acting on.
const char* funnelStage(const SourceSnapshot& s) {
    if (s.clockState == "stopped") return "no audio";
    if (!s.toneDetected) return "no tick";
    if (!s.phaseLocked) return "no edge";
    if (!s.anchored) return "no frame";
    if (s.refusal != "none") return s.refusal.c_str();
    if (s.framesInWindow < 2) return "voting";
    if (s.clockState == "locked") return "locked";
    return "waiting";
}

} // namespace

std::string formatOffsetMs(double seconds) {
    if (!std::isfinite(seconds)) return "-";
    const double ms = seconds * 1000.0;
    char b[48];
    // A sign always, because "your clock is 34 ms slow" and "34 ms fast" are
    // opposite faults and dropping the sign loses which.
    if (std::abs(ms) < 1000.0) std::snprintf(b, sizeof b, "%+.1f ms", ms);
    else if (std::abs(ms) < 60000.0) std::snprintf(b, sizeof b, "%+.3f s", ms / 1000.0);
    else std::snprintf(b, sizeof b, "%+.1f min", ms / 60000.0);
    return b;
}

std::string formatDuration(double seconds) {
    if (!std::isfinite(seconds) || seconds >= 1e8) return "never";
    if (seconds < 0.0) seconds = 0.0;
    char b[48];
    if (seconds < 90.0) std::snprintf(b, sizeof b, "%.0fs", seconds);
    else if (seconds < 5400.0) std::snprintf(b, sizeof b, "%.0fm", seconds / 60.0);
    else if (seconds < 172800.0) std::snprintf(b, sizeof b, "%.1fh", seconds / 3600.0);
    else std::snprintf(b, sizeof b, "%.1fd", seconds / 86400.0);
    return b;
}

std::string renderTimeJson(const Combined& c, double receiveUnixSec,
                           double clientUnixSec, bool pretty) {
    json j;

    // The corrected clock: this host's reading plus what the radio says it is
    // wrong by. Both timestamps are corrected, so a client that trusts this
    // endpoint ends up on UTC rather than on our error.
    const double recv = receiveUnixSec + c.offsetSec;
    const double xmit = realtimeNow() + c.offsetSec;

    j["synchronised"] = c.synchronised;
    j["stratum"] = c.synchronised ? 1 : 16;
    j["refid"] = c.refid;
    j["leap_pending"] = c.leapPending;

    // Present even when unsynchronised, because "here is my best guess and
    // here is how bad it might be" is more useful to a client than a blank —
    // and `synchronised` plus `dispersion_ms` already say not to trust it.
    j["unix"] = xmit;
    j["unix_ms"] = static_cast<long long>(std::llround(xmit * 1000.0));
    j["utc"] = iso8601(static_cast<long long>(std::llround(xmit * 1000.0)));
    j["dispersion_ms"] = c.dispersionSec * 1000.0;
    j["offset_ms"] = c.offsetSec * 1000.0;
    j["reference_age_seconds"] = c.ageSec;
    j["sources_used"] = c.used;
    if (!c.note.empty()) j["note"] = c.note;

    // The round-trip fields, named as RFC 5905 names them so the arithmetic
    // above can be transcribed without translating.
    json rt;
    if (clientUnixSec > 0.0) rt["originate"] = clientUnixSec;
    rt["receive"] = recv;
    rt["transmit"] = xmit;
    // The server's own uncorrected clock, so a client can see what correction
    // was applied rather than having to trust that one was.
    rt["server_raw_receive"] = receiveUnixSec;
    j["roundtrip"] = std::move(rt);

    return dumpJson(j, pretty);
}

std::string renderStatusLine(const StatusInput& in) {
    int locked = 0;
    for (const SourceSnapshot& s : in.sources) if (s.clockState == "locked") ++locked;

    std::ostringstream o;
    if (in.combined.synchronised) {
        o << "synchronised, offset " << formatOffsetMs(in.combined.offsetSec)
          << " +/- " << f2(in.combined.dispersionSec * 1000.0, 1) << " ms";
    } else {
        o << "UNSYNCHRONISED";
        if (!in.combined.note.empty()) o << " (" << in.combined.note << ")";
    }
    o << "; " << locked << "/" << in.sources.size() << " sources locked, "
      << in.combined.used << " in use";
    return o.str();
}

std::string renderStatusBlock(const StatusInput& in) {
    std::ostringstream o;

    // --- the served answer, first, because it is the product --------------
    o << "served time: ";
    if (in.combined.synchronised) {
        o << "stratum 1 (" << in.combined.refid << "), offset "
          << formatOffsetMs(in.combined.offsetSec)
          << ", root dispersion " << f2(in.combined.dispersionSec * 1000.0, 1) << " ms"
          << ", from " << in.combined.used << " of " << in.combined.candidates << " candidate(s)"
          << ", " << formatDuration(in.combined.ageSec) << " old";
    } else {
        o << "UNSYNCHRONISED (stratum 16)";
        if (in.combined.valid) {
            o << ", last known offset " << formatOffsetMs(in.combined.offsetSec)
              << " from " << formatDuration(in.combined.ageSec) << " ago";
        }
    }
    if (!in.combined.note.empty()) o << "\n             " << in.combined.note;
    if (!in.combined.usedNames.empty()) {
        o << "\n             using:";
        for (const std::string& n : in.combined.usedNames) o << ' ' << n;
    }
    o << '\n';

    if (in.combined.residuals.size() >= 2) {
        o << "agreement: ";
        bool first = true;
        for (const SourceResidual& r : in.combined.residuals) {
            if (!first) o << ",  ";
            first = false;
            o << r.name << ' ' << formatOffsetMs(r.averagedSec);
            if (r.refused) o << " REFUSED";
        }
        o << "   (each against its peers — same transmitter, so this is model error)\n";
    }

    o << "ntp :123 -> " << in.ntpPort << ": " << in.ntp.requests << " requests, "
      << in.ntp.answered << " answered, " << in.ntp.ignored << " ignored, "
      << in.ntp.rateLimited << " rate-limited";
    if (in.ntp.unsynchronised) o << ", " << in.ntp.unsynchronised << " while unsynchronised";
    o << "   (up " << formatDuration(in.uptimeSec) << ")\n";
    o << '\n';

    // --- the per-source table ---------------------------------------------
    //
    // Column order follows the chain: what it is, whether the link is up,
    // whether the decoder has a signal, where in the funnel it is, and only
    // then what time it thinks it is. Reading left to right is reading the
    // failure path.
    char head[256];
    std::snprintf(head, sizeof head,
                  "%-14s %-10s %-6s %-9s %-10s %6s %5s %7s %11s %9s %7s",
                  "source", "link", "stn", "state", "stage", "tickdB", "vote",
                  "qual", "offset", "disp", "n");
    o << head << '\n';
    o << std::string(std::strlen(head), '-') << '\n';

    for (const SourceSnapshot& s : in.sources) {
        char row[320];
        std::snprintf(row, sizeof row,
                      "%-14s %-10s %-6s %-9s %-10s %6s %5s %7s %11s %9s %7d",
                      s.name.c_str(),
                      linkStateName(s.link),
                      s.station.c_str(),
                      s.clockState.c_str(),
                      funnelStage(s),
                      s.clockState == "stopped" ? "-" : f2(s.toneSnrDb, 1).c_str(),
                      s.windowSize > 0 ? (std::to_string(s.framesInWindow) + "/" +
                                          std::to_string(s.windowSize)).c_str() : "-",
                      s.lastQuality > 0 ? (std::to_string(s.lastQuality) + "%").c_str() : "-",
                      s.haveOffset ? formatOffsetMs(s.offsetSec).c_str() : "-",
                      s.haveOffset ? (f2(s.dispersionSec * 1000.0, 1) + "ms").c_str() : "-",
                      s.offsetSamples);
        o << row << '\n';
    }

    // --- per-source detail -------------------------------------------------
    //
    // The table answers "which source is working"; this answers "why not", and
    // is where the delay model is exposed, because a source whose offset
    // disagrees with the others by 20 ms is almost always a delay model that
    // is wrong rather than a decoder that is.
    for (const SourceSnapshot& s : in.sources) {
        o << '\n' << s.name << ":  " << s.url << "  dial " << f2(s.dialHz / 1e6, 6)
          << " MHz (carrier " << f2(s.carrierHz / 1e6, 3) << " MHz), "
          << formatName(s.format);
        if (s.sampleRate) o << " @ " << s.sampleRate << " Hz";
        o << '\n';

        if (!s.receiverName.empty()) o << "    receiver: " << s.receiverName << '\n';
        o << "    link: " << linkStateName(s.link);
        if (!s.linkDetail.empty()) o << " (" << s.linkDetail << ")";
        o << " for " << formatDuration(s.linkAgeSec)
          << ", " << s.connectAttempts << " attempt(s)"
          << ", audio " << formatDuration(s.lastAudioAgeSec) << " ago"
          << ", " << s.packets << " packets / " << (s.audioBytes / 1024) << " kiB";
        if (s.decodeErrors) o << ", " << s.decodeErrors << " decode errors";
        o << '\n';

        if (s.basebandPowerDb > -998.0) {
            o << "    signal: " << f2(s.basebandPowerDb, 1) << " dBFS baseband";
            if (s.noiseDb > -998.0) {
                o << ", noise " << f2(s.noiseDb, 1) << " dBFS, SNR "
                  << f2(s.basebandPowerDb - s.noiseDb, 1) << " dB";
            }
            o << '\n';
        }

        o << "    decoder: " << s.clockState << ", stage " << funnelStage(s)
          << ", tick " << f2(s.toneSnrDb, 1) << " dB"
          << (s.toneDetected ? " (detected)" : " (NOT detected)");
        if (std::isfinite(s.tickBandRatioDb)) {
            o << ", 2000/2200 Hz " << f2(s.tickBandRatioDb, 1) << " dB";
        }
        o
          << ", edge " << (s.phaseLocked ? "locked" : "unlocked");
        if (std::isfinite(s.delayEstMs) && s.delayEstMs != 0.0) o << " at " << f2(s.delayEstMs, 1) << " ms";
        o << ", frame " << (s.anchored ? "anchored" : "not anchored");
        if (s.badFrameStreak) o << " (" << s.badFrameStreak << " bad in a row)";
        o << '\n';
        o << "             vote " << s.framesInWindow << "/" << s.windowSize
          << " quality " << f2(s.voteQuality, 2) << ", refusal " << s.refusal
          << ", " << (s.samplesConsumed / (s.sampleRate ? s.sampleRate : 12000)) << " s of audio";
        if (!s.lastDecodedUtc.empty()) {
            o << "\n             last decode " << s.lastDecodedUtc
              << " at quality " << s.lastQuality << "%, "
              << formatDuration(s.lastTimeAgeSec) << " ago";
        }
        if (s.dut1Tenths) o << "\n             DUT1 " << f2(s.dut1Tenths / 10.0, 1) << " s";
        if (s.leapPending) o << "\n             LEAP SECOND PENDING (broadcast warning bit set)";
        o << '\n';

        if (s.haveOffset) {
            o << "    timing: offset " << formatOffsetMs(s.offsetSec)
              << " +/- " << f2(s.dispersionSec * 1000.0, 1) << " ms"
              << " (raw " << formatOffsetMs(s.rawOffsetSec)
              << ", jitter " << f2(s.jitterSec * 1000.0, 1) << " ms"
              << ", " << s.offsetSamples << " samples, newest "
              << formatDuration(s.offsetAgeSec) << " old)"
              << ", worth " << f2(s.weightDispersionSec * 1000.0, 1) << " ms against the others\n";
        } else {
            o << "    timing: no usable offset yet\n";
        }

        if (s.wsRttMs > 0.0 || s.httpRttMs > 0.0) {
            o << "    path:   round trip " << f2((s.rttFromWs ? s.wsRttMs : s.httpRttMs), 1)
              << " ms over " << (s.rttFromWs ? "the audio connection" : "the TCP handshake");
            // Both measured and disagreeing means something is answering for
            // the receiver. Worth saying which figure is being believed.
            if (s.rttFromWs && s.httpRttMs > 0.0 && s.wsRttMs > s.httpRttMs * 1.5) {
                o << " (handshake said " << f2(s.httpRttMs, 1)
                  << " ms — a proxy is terminating it short of the receiver)";
            }
            o << '\n';
        }
        o << "    delay:  " << f2(s.delaySec * 1000.0, 1) << " ms total = "
          << f2(s.propagationSec * 1000.0, 1) << " propagation + "
          << f2(s.networkSec * 1000.0, 1) << " network + "
          << f2(s.codecSec * 1000.0, 1) << " codec + "
          << f2(s.chainSec * 1000.0, 1) << " UberSDR chain + "
          << f2(s.decoderSec * 1000.0, 1) << " decoder bias + "
          << f2(s.extraSec * 1000.0, 1) << " configured\n";
        o << "            " << s.pathDescription << '\n';

        if (s.clockFitValid) {
            o << "    sample clock: fit over " << formatDuration(s.clockSpanSec)
              << ", residual " << f2(s.clockResidualSec * 1000.0, 2) << " ms"
              << ", receiver runs " << f2(s.clockPpm, 1) << " ppm against ours"
              << (s.clockSlopeHeld ? " (slope refused, holding nominal)" : "")
              << ", slope worth " << f2(s.clockSlopeUncSec * 1000.0, 1) << " ms"
              << ", last packet " << f2(s.lastExcessDelaySec * 1000.0, 1) << " ms late\n";
        } else {
            o << "    sample clock: not yet fitted\n";
        }
    }

    std::string out = o.str();
    while (!out.empty() && out.back() == '\n') out.pop_back();
    return out;
}

std::string renderStatusJson(const StatusInput& in, bool pretty) {
    json j;
    j["version"] = in.version;
    j["uptime_seconds"] = in.uptimeSec;

    json served;
    served["synchronised"] = in.combined.synchronised;
    served["valid"] = in.combined.valid;
    served["stratum"] = in.combined.synchronised ? 1 : 16;
    served["refid"] = in.combined.refid;
    served["offset_ms"] = in.combined.offsetSec * 1000.0;
    served["root_dispersion_ms"] = in.combined.dispersionSec * 1000.0;
    {
        json ag = json::array();
        for (const SourceResidual& r : in.combined.residuals) {
            ag.push_back({{"name", r.name},
                          {"residual_ms", r.averagedSec * 1000.0},
                          {"instant_ms", r.instantSec * 1000.0},
                          {"peers", r.peers},
                          {"refused", r.refused}});
        }
        served["agreement"] = std::move(ag);
    }
    served["age_seconds"] = in.combined.ageSec;
    served["sources_used"] = in.combined.used;
    served["sources_candidate"] = in.combined.candidates;
    served["used_names"] = in.combined.usedNames;
    served["rejected_names"] = in.combined.rejectedNames;
    served["leap_pending"] = in.combined.leapPending;
    served["note"] = in.combined.note;
    j["served"] = std::move(served);

    json ntp;
    ntp["port"] = in.ntpPort;
    ntp["requests"] = in.ntp.requests;
    ntp["answered"] = in.ntp.answered;
    ntp["ignored"] = in.ntp.ignored;
    ntp["rate_limited"] = in.ntp.rateLimited;
    ntp["answered_unsynchronised"] = in.ntp.unsynchronised;
    ntp["send_errors"] = in.ntp.sendErrors;
    j["ntp"] = std::move(ntp);

    json arr = json::array();
    for (const SourceSnapshot& s : in.sources) {
        json o;
        o["name"] = s.name;
        o["url"] = s.url;
        o["enabled"] = s.enabled;
        o["receiver_name"] = s.receiverName;
        o["carrier_hz"] = s.carrierHz;
        o["dial_hz"] = s.dialHz;
        o["format"] = formatName(s.format);
        o["weight"] = s.weight;

        json link;
        link["state"] = linkStateName(s.link);
        link["detail"] = s.linkDetail;
        link["age_seconds"] = s.linkAgeSec;
        link["last_audio_age_seconds"] = s.lastAudioAgeSec;
        link["packets"] = s.packets;
        link["audio_bytes"] = s.audioBytes;
        link["decode_errors"] = s.decodeErrors;
        link["connect_attempts"] = s.connectAttempts;
        link["http_rtt_ms"] = s.httpRttMs;
        o["link"] = std::move(link);

        json audio;
        audio["sample_rate"] = s.sampleRate;
        audio["baseband_power_dbfs"] = s.basebandPowerDb;
        audio["noise_dbfs"] = s.noiseDb;
        if (s.basebandPowerDb > -998.0 && s.noiseDb > -998.0)
            audio["snr_db"] = s.basebandPowerDb - s.noiseDb;
        o["audio"] = std::move(audio);

        json dec;
        dec["state"] = s.clockState;
        dec["station"] = s.station;
        dec["stage"] = funnelStage(s);
        dec["tone_snr_db"] = s.toneSnrDb;
        dec["tone_detected"] = s.toneDetected;
        // Which station the tick is really from: + leans WWV, - leans WWVH.
        dec["tick_band_ratio_db"] = std::isfinite(s.tickBandRatioDb)
                                        ? json(s.tickBandRatioDb) : json(nullptr);
        dec["phase_locked"] = s.phaseLocked;
        // NaN has no JSON spelling and the decoder really does report it for a
        // delay estimate that has not settled, so it becomes null rather than
        // producing a document no parser will accept.
        if (std::isfinite(s.delayEstMs)) dec["delay_est_ms"] = s.delayEstMs;
        else dec["delay_est_ms"] = nullptr;
        dec["anchored"] = s.anchored;
        dec["bad_frame_streak"] = s.badFrameStreak;
        dec["frames_in_window"] = s.framesInWindow;
        dec["window_size"] = s.windowSize;
        dec["vote_quality"] = s.voteQuality;
        dec["refusal"] = s.refusal;
        dec["audio_seconds"] = s.sampleRate ? static_cast<double>(s.samplesConsumed) / s.sampleRate : 0.0;
        dec["last_quality"] = s.lastQuality;
        dec["last_decoded_utc"] = s.lastDecodedUtc;
        dec["last_decode_age_seconds"] = s.lastTimeAgeSec;
        dec["leap_pending"] = s.leapPending;
        dec["dut1_seconds"] = s.dut1Tenths / 10.0;
        o["decoder"] = std::move(dec);

        json t;
        t["have_offset"] = s.haveOffset;
        t["offset_ms"] = s.offsetSec * 1000.0;
        t["raw_offset_ms"] = s.rawOffsetSec * 1000.0;
        t["jitter_ms"] = s.jitterSec * 1000.0;
        t["dispersion_ms"] = s.dispersionSec * 1000.0;
        t["weight_dispersion_ms"] = s.weightDispersionSec * 1000.0;
        t["ws_rtt_ms"] = s.wsRttMs;
        t["rtt_from_websocket"] = s.rttFromWs;
        t["age_seconds"] = s.offsetAgeSec;
        t["samples"] = s.offsetSamples;
        o["timing"] = std::move(t);

        json d;
        d["total_ms"] = s.delaySec * 1000.0;
        d["propagation_ms"] = s.propagationSec * 1000.0;
        d["network_ms"] = s.networkSec * 1000.0;
        d["codec_ms"] = s.codecSec * 1000.0;
        d["chain_ms"] = s.chainSec * 1000.0;
        d["decoder_bias_ms"] = s.decoderSec * 1000.0;
        d["configured_ms"] = s.extraSec * 1000.0;
        d["path"] = s.pathDescription;
        if (s.receiverLocation.valid) {
            d["receiver_lat"] = s.receiverLocation.lat;
            d["receiver_lon"] = s.receiverLocation.lon;
        }
        o["delay"] = std::move(d);

        json sc;
        sc["fitted"] = s.clockFitValid;
        sc["residual_ms"] = s.clockResidualSec * 1000.0;
        sc["span_seconds"] = s.clockSpanSec;
        sc["ppm"] = s.clockPpm;
        sc["slope_uncertainty_ms"] = s.clockSlopeUncSec * 1000.0;
        sc["slope_held"] = s.clockSlopeHeld;
        sc["last_excess_delay_ms"] = s.lastExcessDelaySec * 1000.0;
        o["sample_clock"] = std::move(sc);

        arr.push_back(std::move(o));
    }
    j["sources"] = std::move(arr);

    return dumpJson(j, pretty);
}

} // namespace ubersdr_ntp
