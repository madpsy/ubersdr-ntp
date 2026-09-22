#include "Config.h"

#include "../third_party/json.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <set>
#include <sstream>

using nlohmann::json;

namespace ubersdr_ntp {

const char* sourceKindName(SourceKind k) {
    switch (k) {
        case SourceKind::Radio: return "radio";
        case SourceKind::Ntp:   return "ntp";
    }
    return "?";
}

bool parseSourceKind(const std::string& s, SourceKind& out) {
    if (s == "radio" || s == "wwv" || s == "hf") { out = SourceKind::Radio; return true; }
    if (s == "ntp" || s == "network") { out = SourceKind::Ntp; return true; }
    return false;
}

const char* secondaryModeName(SecondaryMode m) {
    switch (m) {
        case SecondaryMode::Always:  return "always";
        case SecondaryMode::Standby: return "standby";
        case SecondaryMode::Cold:    return "cold";
    }
    return "?";
}

bool parseSecondaryMode(const std::string& s, SecondaryMode& out) {
    if (s == "always" || s == "combine") { out = SecondaryMode::Always; return true; }
    if (s == "standby" || s == "warm" || s == "fallback") { out = SecondaryMode::Standby; return true; }
    // "off" is spelt by leaving the class unconfigured, but somebody will type
    // it meaning "do not connect until you must", which is what cold is.
    if (s == "cold" || s == "on_demand" || s == "on-demand" || s == "off") {
        out = SecondaryMode::Cold;
        return true;
    }
    return false;
}

std::uint64_t dialForCarrier(std::uint64_t carrierHz) {
    if (carrierHz == kDcf77CarrierHz) return carrierHz;
    return carrierHz > 1000 ? carrierHz - 1000 : 0;
}

namespace {

// Optional-field readers. A wrong TYPE is an error rather than a silent
// default: a port written as "123" instead of 123 should say so, not listen
// somewhere else.
template <typename T>
bool getOpt(const json& j, const char* key, T& out, std::string& err) {
    auto it = j.find(key);
    if (it == j.end() || it->is_null()) return true;
    try {
        out = it->get<T>();
    } catch (const std::exception& e) {
        err = std::string("field \"") + key + "\": " + e.what();
        return false;
    }
    return true;
}

bool getSource(const json& j, SourceConfig& s, std::string& err) {
    if (!getOpt(j, "name", s.name, err)) return false;
    if (!getOpt(j, "url", s.url, err)) return false;
    if (!getOpt(j, "password", s.password, err)) return false;
    if (!getOpt(j, "enabled", s.enabled, err)) return false;
    if (!getOpt(j, "carrier_hz", s.carrierHz, err)) return false;
    if (!getOpt(j, "dial_hz", s.dialHz, err)) return false;
    if (!getOpt(j, "auto_delay", s.autoDelay, err)) return false;
    if (!getOpt(j, "delay_ms", s.delayMs, err)) return false;
    if (!getOpt(j, "extra_delay_ms", s.extraDelayMs, err)) return false;
    if (!getOpt(j, "weight", s.weight, err)) return false;
    if (!getOpt(j, "verify_tls", s.verifyTls, err)) return false;
    if (auto it = j.find("min_margin"); it != j.end() && !it->is_null()) {
        if (!it->is_number()) { err = "field \"min_margin\" must be a number of dB"; return false; }
        const double m = it->get<double>();
        // The server's own rule, refused here rather than let it be silently
        // bent: anything not above zero is the lossless stream, and a request
        // is clamped to 15-60 dB and rounded to whole dB. So a negative figure,
        // one under 15 or one over 60 would not be what was asked for.
        if (!std::isfinite(m) || (m != 0.0 && (m < 15.0 || m > 60.0))) {
            char v[32];
            std::snprintf(v, sizeof v, "%g", m);
            err = std::string("min_margin ") + v + " is not one UberSDR serves: 0 for the "
                  "lossless stream, or 15 to 60 dB of margin under the noise floor";
            return false;
        }
        s.minMarginDb = static_cast<int>(std::lround(m));
    }

    // "format" was a setting while Opus was the other choice. Every source is
    // PCM v4 now, and a configuration that still names one loads as it did
    // (see the warning in load()).

    // An explicit delay_ms with auto_delay still on is a contradiction that
    // would silently ignore one of them. Say so rather than pick.
    if (j.contains("delay_ms") && !j.contains("auto_delay") && s.delayMs != 0.0) {
        s.autoDelay = false;
    }
    return true;
}

bool getNtpSource(const json& j, NtpSourceConfig& s, std::string& err) {
    if (!getOpt(j, "name", s.name, err)) return false;
    if (!getOpt(j, "server", s.server, err)) return false;
    // "address" and "host" are what people type when the field is called
    // something else in every other program they have configured this week.
    if (!getOpt(j, "address", s.server, err)) return false;
    if (!getOpt(j, "host", s.server, err)) return false;
    if (!getOpt(j, "port", s.port, err)) return false;
    if (!getOpt(j, "enabled", s.enabled, err)) return false;
    if (!getOpt(j, "poll_seconds", s.pollSeconds, err)) return false;
    if (!getOpt(j, "iburst", s.iburst, err)) return false;
    if (!getOpt(j, "weight", s.weight, err)) return false;
    if (!getOpt(j, "max_root_distance_ms", s.maxRootDistanceMs, err)) return false;
    if (!getOpt(j, "max_stratum", s.maxStratum, err)) return false;
    if (!getOpt(j, "extra_delay_ms", s.extraDelayMs, err)) return false;
    return true;
}

} // namespace

bool Config::load(const std::string& path, Config& out, std::string& err) {
    std::ifstream in(path);
    if (!in) {
        err = "cannot open config " + path;
        return false;
    }

    json j;
    try {
        // Comments allowed: a config whose every field needs explaining is one
        // people annotate, and JSON without them invites a parallel README that
        // goes stale.
        j = json::parse(in, nullptr, true, true);
    } catch (const std::exception& e) {
        err = std::string("parse error in ") + path + ": " + e.what();
        return false;
    }
    if (!j.is_object()) { err = "config must be a JSON object"; return false; }

    Config c;

    if (auto it = j.find("log"); it != j.end() && it->is_object()) {
        const json& l = *it;
        if (!getOpt(l, "file", c.log.file, err)) return false;
        if (!getOpt(l, "stderr", c.log.stderrEnabled, err)) return false;
        if (!getOpt(l, "status_interval_seconds", c.log.statusIntervalSeconds, err)) return false;
        std::string lvl;
        if (!getOpt(l, "level", lvl, err)) return false;
        if (!lvl.empty() && !parseLevel(lvl, c.log.level)) {
            err = "unknown log level \"" + lvl + "\" (trace, debug, info, warn, error)";
            return false;
        }
    }

    if (auto it = j.find("ntp"); it != j.end() && it->is_object()) {
        const json& n = *it;
        if (!getOpt(n, "port", c.ntp.port, err)) return false;
        if (!getOpt(n, "coast_seconds", c.ntp.coastSeconds, err)) return false;
        if (!getOpt(n, "coast_drift_ppm", c.ntp.coastDriftPpm, err)) return false;
        if (!getOpt(n, "min_sources", c.ntp.minSources, err)) return false;
        if (auto mit = n.find("min_sources"); mit != n.end() && !mit->is_null()) {
            c.minSourcesGiven = true;
        }
        if (!getOpt(n, "answer_when_unsynchronised", c.ntp.answerWhenUnsynchronised, err)) return false;
        if (!getOpt(n, "honour_leap_warning", c.ntp.honourLeapWarning, err)) return false;
        if (!getOpt(n, "rate_limit_per_client", c.ntp.rateLimitPerClient, err)) return false;
        if (!getOpt(n, "drift_file", c.ntp.driftFile, err)) return false;
        if (auto lit = n.find("listen"); lit != n.end() && !lit->is_null()) {
            if (!getOpt(n, "listen", c.ntp.listen, err)) return false;
        }
    }

    if (auto it = j.find("http"); it != j.end() && it->is_object()) {
        const json& h = *it;
        if (!getOpt(h, "enabled", c.http.enabled, err)) return false;
        if (!getOpt(h, "listen", c.http.listen, err)) return false;
        if (!getOpt(h, "port", c.http.port, err)) return false;
    }

    if (auto it = j.find("mqtt"); it != j.end() && it->is_object()) {
        const json& m = *it;
        if (!getOpt(m, "enabled", c.mqtt.enabled, err)) return false;
        if (!getOpt(m, "ingest_url", c.mqtt.ingestUrl, err)) return false;
    }

    if (auto it = j.find("clock"); it != j.end() && it->is_object()) {
        const json& k = *it;
        std::string v;
        if (!getOpt(k, "primary", v, err)) return false;
        if (!v.empty() && !parseSourceKind(v, c.clock.primary)) {
            err = "unknown clock.primary \"" + v + "\" (expected radio or ntp)";
            return false;
        }
        v.clear();
        if (!getOpt(k, "secondary", v, err)) return false;
        if (!v.empty() && !parseSecondaryMode(v, c.clock.secondary)) {
            err = "unknown clock.secondary \"" + v + "\" (expected always, standby or cold)";
            return false;
        }
        if (!getOpt(k, "failover_after_seconds", c.clock.failoverAfterSec, err)) return false;
        if (!getOpt(k, "failback_after_seconds", c.clock.failbackAfterSec, err)) return false;
        if (!getOpt(k, "min_secondary_sources", c.clock.minSecondarySources, err)) return false;
        if (auto mit = k.find("min_secondary_sources"); mit != k.end() && !mit->is_null()) {
            c.minSecondarySourcesGiven = true;
        }
        if (!getOpt(k, "min_radio_sources", c.clock.minRadioSources, err)) return false;
        if (!getOpt(k, "min_ntp_sources", c.clock.minNtpSources, err)) return false;
    }

    if (auto it = j.find("defaults"); it != j.end() && it->is_object()) {
        if (!getSource(*it, c.defaults, err)) { err = "in \"defaults\": " + err; return false; }
    }
    if (auto it = j.find("ntp_defaults"); it != j.end() && it->is_object()) {
        if (!getNtpSource(*it, c.ntpDefaults, err)) { err = "in \"ntp_defaults\": " + err; return false; }
    }

    // A leftover "format" asking for Opus is not an error -- an unattended
    // install must not stop starting over it -- but it is no longer what
    // happens, so it is said once at startup.
    {
        auto asksOpus = [](const json& o) {
            auto f = o.find("format");
            return f != o.end() && f->is_string() && f->get<std::string>() == "opus";
        };
        bool opus = false;
        if (auto it = j.find("defaults"); it != j.end() && it->is_object()) opus = opus || asksOpus(*it);
        if (auto it = j.find("sources"); it != j.end() && it->is_array())
            for (const json& sj : *it) if (sj.is_object()) opus = opus || asksOpus(sj);
        if (opus)
            c.warnings.push_back("\"format\": \"opus\" is no longer a setting and is ignored: "
                                 "every source is received as lossless PCM v4");
    }

    // Radio sources. No longer required to exist: an install whose primary is
    // NTP and whose secondary is cold may legitimately have none, and the
    // "something must be configured" check belongs in finalise() where both
    // classes are in view.
    if (auto sit = j.find("sources"); sit != j.end() && !sit->is_null()) {
        if (!sit->is_array()) { err = "\"sources\" must be an array"; return false; }
        int idx = 0;
        for (const json& sj : *sit) {
            if (!sj.is_object()) { err = "every entry in \"sources\" must be an object"; return false; }
            SourceConfig s = c.defaults;   // inherit, then override
            s.name.clear();                // never inherited: names must be distinct
            if (!getSource(sj, s, err)) {
                err = "in source #" + std::to_string(idx) + ": " + err;
                return false;
            }
            c.sources.push_back(std::move(s));
            ++idx;
        }
    }

    // Upstream NTP servers.
    if (auto nit = j.find("ntp_sources"); nit != j.end() && !nit->is_null()) {
        if (!nit->is_array()) { err = "\"ntp_sources\" must be an array"; return false; }
        int idx = 0;
        for (const json& sj : *nit) {
            // A bare string is the common case -- a hostname and nothing else --
            // and making people write {"server": "..."} for it would be a tax
            // on the configuration that is most likely to be right.
            NtpSourceConfig s = c.ntpDefaults;
            s.name.clear();
            if (sj.is_string()) {
                s.server = sj.get<std::string>();
            } else if (sj.is_object()) {
                if (!getNtpSource(sj, s, err)) {
                    err = "in ntp_source #" + std::to_string(idx) + ": " + err;
                    return false;
                }
            } else {
                err = "every entry in \"ntp_sources\" must be an object or a hostname string";
                return false;
            }
            c.ntpSources.push_back(std::move(s));
            ++idx;
        }
    }

    out = std::move(c);
    return true;
}

bool Config::finalise(std::string& err) {
    // The environment wins, as it does for the receiver's other addons: a
    // container is configured by its compose file, not by editing a file
    // inside it. Here rather than in load() so a run with no file gets it too.
    if (const char* v = std::getenv("UBERSDR_INGEST_URL"); v && *v) mqtt.ingestUrl = v;
    while (!mqtt.ingestUrl.empty() && mqtt.ingestUrl.back() == '/') mqtt.ingestUrl.pop_back();
    if (mqtt.enabled && mqtt.ingestUrl.rfind("http://", 0) != 0) {
        err = "mqtt.ingest_url must be an http:// URL (got \"" + mqtt.ingestUrl + "\")";
        return false;
    }
    if (ntp.port < 1 || ntp.port > 65535) {
        err = "ntp.port " + std::to_string(ntp.port) + " out of range";
        return false;
    }
    if (http.enabled && (http.port < 1 || http.port > 65535)) {
        err = "http.port " + std::to_string(http.port) + " out of range";
        return false;
    }
    if (http.enabled && http.port == ntp.port) {
        err = "http.port and ntp.port are both " + std::to_string(http.port) +
              "; they are different protocols and cannot share one";
        return false;
    }
    if (ntp.listen.empty()) {
        err = "ntp.listen is empty; nothing would be served";
        return false;
    }
    if (ntp.minSources < 1) ntp.minSources = 1;

    // Numbers that are NaN, infinite or negative where negative means nothing
    // are refused rather than clamped. JSON cannot spell NaN, but the command
    // line can (`--status-interval nan` is a valid strtod), and a NaN passes
    // every `< 0` guard downstream and then poisons whatever it touches: a NaN
    // weight makes every combined offset NaN, a NaN coast never expires.
    auto finiteAtLeast = [&err](double v, double lo, const std::string& what) {
        if (std::isfinite(v) && v >= lo) return true;
        std::ostringstream os;
        os << what << " is " << v << "; it must be a finite number >= " << lo;
        err = os.str();
        return false;
    };
    if (!finiteAtLeast(ntp.coastSeconds, 0.0, "ntp.coast_seconds")) return false;
    if (!finiteAtLeast(ntp.coastDriftPpm, 0.0, "ntp.coast_drift_ppm")) return false;
    if (!finiteAtLeast(ntp.rateLimitPerClient, 0.0, "ntp.rate_limit_per_client")) return false;
    // 0 disables the block. Anything else under a second is not a status
    // interval anybody wants, and a tiny one (1e-12) made the main loop's
    // schedule arithmetic spin: adding it to a monotonic time is a no-op.
    if (!finiteAtLeast(log.statusIntervalSeconds, 0.0, "log.status_interval_seconds")) return false;
    if (log.statusIntervalSeconds > 0.0 && log.statusIntervalSeconds < 1.0) {
        std::ostringstream os;
        os << "log.status_interval_seconds is " << log.statusIntervalSeconds
           << "; use 0 to disable the status block, or at least 1";
        err = os.str();
        return false;
    }

    if (!finiteAtLeast(clock.failoverAfterSec, 0.0, "clock.failover_after_seconds")) return false;
    if (!finiteAtLeast(clock.failbackAfterSec, 0.0, "clock.failback_after_seconds")) return false;
    if (clock.minSecondarySources < 1) clock.minSecondarySources = 1;

    // The minimums, keyed by kind. clock.min_radio_sources and
    // clock.min_ntp_sources are the spelling that means the same thing
    // whichever class is primary; ntp.min_sources and
    // clock.min_secondary_sources are the older role-keyed one, still read.
    // Given both ways for one kind with different figures is refused rather
    // than resolved by a precedence rule nobody would remember.
    {
        const bool radioPrimary = clock.primary == SourceKind::Radio;
        auto resolve = [&](int& byKind, const char* kindKey, int byRole, bool roleGiven,
                           const char* roleKey) {
            if (byKind != 0) {
                if (byKind < 1) {
                    err = std::string("clock.") + kindKey + " is " + std::to_string(byKind) +
                          "; it must be at least 1";
                    return false;
                }
                if (roleGiven && byRole != byKind) {
                    err = std::string("clock.") + kindKey + " is " + std::to_string(byKind) +
                          " but " + roleKey + " is " + std::to_string(byRole) +
                          ", and with clock.primary \"" + sourceKindName(clock.primary) +
                          "\" both set the same figure; give one of them";
                    return false;
                }
            } else {
                byKind = byRole;
            }
            return true;
        };
        if (!resolve(clock.minRadioSources, "min_radio_sources",
                     radioPrimary ? ntp.minSources : clock.minSecondarySources,
                     radioPrimary ? minSourcesGiven : minSecondarySourcesGiven,
                     radioPrimary ? "ntp.min_sources" : "clock.min_secondary_sources")) {
            return false;
        }
        if (!resolve(clock.minNtpSources, "min_ntp_sources",
                     radioPrimary ? clock.minSecondarySources : ntp.minSources,
                     radioPrimary ? minSecondarySourcesGiven : minSourcesGiven,
                     radioPrimary ? "clock.min_secondary_sources" : "ntp.min_sources")) {
            return false;
        }
        ntp.minSources = radioPrimary ? clock.minRadioSources : clock.minNtpSources;
        clock.minSecondarySources = radioPrimary ? clock.minNtpSources : clock.minRadioSources;
    }

    std::set<std::string> names;
    int autoName = 0;
    int enabled = 0;

    for (SourceConfig& s : sources) {
        if (s.url.empty()) { err = "source \"" + s.name + "\" has no url"; return false; }
        if (s.url.rfind("http://", 0) != 0 && s.url.rfind("https://", 0) != 0) {
            err = "source \"" + s.name + "\": url must start with http:// or https://";
            return false;
        }
        // A trailing slash would produce //connection and //ws. Harmless on
        // most servers and confusing in a log either way.
        while (!s.url.empty() && s.url.back() == '/') s.url.pop_back();

        if (s.dialHz == 0) s.dialHz = dialForCarrier(s.carrierHz);
        if (s.dialHz == 0) {
            err = "source \"" + s.name + "\": carrier_hz " + std::to_string(s.carrierHz) +
                  " is too low to derive a dial from";
            return false;
        }

        const Broadcast bc = broadcastFor(s.carrierHz, s.dialHz);
        if (bc == Broadcast::Dcf77) {
            const std::uint64_t off = s.dialHz > s.carrierHz ? s.dialHz - s.carrierHz
                                                             : s.carrierHz - s.dialHz;
            if (off > kDcf77MaxDialOffsetHz) {
                err = "source \"" + (s.name.empty() ? std::string("dcf77") : s.name) +
                      "\": dial_hz " + std::to_string(s.dialHz) + " puts the DCF77 carrier " +
                      std::to_string(off) + " Hz off, outside the 12 kHz IQ passband; "
                      "leave dial_hz out and it goes on the carrier";
                return false;
            }
        }

        if (s.name.empty()) {
            // Named after what it listens to, which is what anyone reading the
            // log wants to see anyway.
            std::ostringstream os;
            os << (bc == Broadcast::Dcf77 ? "dcf77" : bc == Broadcast::Wwvb ? "wwvb" : "wwv");
            if (bc == Broadcast::Wwv) os << (s.carrierHz / 1000000) << "M";
            os << "-" << ++autoName;
            s.name = os.str();
        }
        if (!names.insert(s.name).second) {
            err = "duplicate source name \"" + s.name + "\"";
            return false;
        }

        // !(> 0) rather than <= 0, so NaN is refused too.
        if (!(s.weight > 0.0) || !std::isfinite(s.weight)) {
            err = "source \"" + s.name + "\": weight must be a positive finite number";
            return false;
        }
        const std::string label = "source \"" + s.name + "\": ";
        // extra_delay_ms may be negative -- it is a correction on top of the
        // auto estimate, and that estimate can be too high -- but delay_ms is
        // a total one-way delay, and a signal cannot arrive before it was sent.
        if (!finiteAtLeast(s.delayMs, 0.0, label + "delay_ms")) return false;
        if (!std::isfinite(s.extraDelayMs)) {
            err = label + "extra_delay_ms must be a finite number";
            return false;
        }
        // Reduced depth exists for IQ only; the server sends a demodulated
        // channel lossless whatever it is asked. Not an error -- it may well
        // have come in from "defaults" -- but not what the setting says either.
        if (s.minMarginDb > 0 && broadcastFor(s.carrierHz, s.dialHz) != Broadcast::Dcf77) {
            warnings.push_back(label + "min_margin applies to DCF77's IQ stream only; this source "
                               "is received as audio, which UberSDR always sends lossless");
        }
        if (s.enabled) ++enabled;
    }

    // --- upstream NTP servers ----------------------------------------------
    int ntpEnabled = 0;
    int autoNtpName = 0;
    for (NtpSourceConfig& s : ntpSources) {
        if (s.server.empty()) { err = "ntp_source \"" + s.name + "\" has no server"; return false; }

        // "host:port", which is how everyone writes one. Only when there is
        // exactly one colon, so an IPv6 literal is not cut in half; a v6
        // address with a port is written [::1]:123, as it is everywhere else.
        if (s.server.front() == '[') {
            const std::size_t close = s.server.find(']');
            if (close == std::string::npos) {
                err = "ntp_source \"" + s.name + "\": unterminated [ in \"" + s.server + "\"";
                return false;
            }
            std::string rest = s.server.substr(close + 1);
            s.server = s.server.substr(1, close - 1);
            if (!rest.empty()) {
                if (rest.front() != ':') {
                    err = "ntp_source \"" + s.name + "\": expected :PORT after ] in \"" + s.server + "\"";
                    return false;
                }
                s.port = std::atoi(rest.c_str() + 1);
            }
        } else if (std::count(s.server.begin(), s.server.end(), ':') == 1) {
            const std::size_t colon = s.server.rfind(':');
            s.port = std::atoi(s.server.c_str() + colon + 1);
            s.server = s.server.substr(0, colon);
        }
        if (s.server.empty()) { err = "ntp_source \"" + s.name + "\" has no server"; return false; }
        if (s.port < 1 || s.port > 65535) {
            err = "ntp_source \"" + s.name + "\": port " + std::to_string(s.port) + " out of range";
            return false;
        }

        if (s.name.empty()) {
            // Named after the server, which is what anyone reading the log
            // wants to see. Suffixed only when that would collide, so the
            // common case of one entry per host reads as the host.
            s.name = s.server;
            if (names.count(s.name)) s.name += "-" + std::to_string(++autoNtpName);
        }
        if (!names.insert(s.name).second) {
            // One namespace across both classes: every name that reaches the
            // log, the status page and the Selector's per-source maps is looked
            // up by string, and two sources sharing one would share their
            // agreement history too.
            err = "duplicate source name \"" + s.name + "\"";
            return false;
        }

        const std::string label = "ntp_source \"" + s.name + "\": ";
        if (!(s.weight > 0.0) || !std::isfinite(s.weight)) {
            err = label + "weight must be a positive finite number";
            return false;
        }
        if (!std::isfinite(s.extraDelayMs)) {
            err = label + "extra_delay_ms must be a finite number";
            return false;
        }
        if (!finiteAtLeast(s.maxRootDistanceMs, 0.0, label + "max_root_distance_ms")) return false;
        // 16 is unsynchronised and 0 is a kiss-o'-death; neither is a stratum a
        // reply may usefully carry, so the ceiling is the range in between.
        if (s.maxStratum < 1 || s.maxStratum > 15) {
            err = label + "max_stratum is " + std::to_string(s.maxStratum) +
                  "; it must be between 1 and 15";
            return false;
        }
        // A poll faster than this is not more accurate -- the clock filter and
        // the estimator both work over a span, not a count -- and against a
        // server you do not own it is abuse. The pool's terms of service say
        // the same number.
        if (!finiteAtLeast(s.pollSeconds, 8.0, label + "poll_seconds")) return false;
        if (s.pollSeconds > 4096.0) {
            err = label + "poll_seconds is " + std::to_string(s.pollSeconds) +
                  "; anything over 4096 is longer than the estimator's own windows";
            return false;
        }
        if (s.enabled) ++ntpEnabled;
    }

    // --- what the two classes add up to -------------------------------------
    if (enabled == 0 && ntpEnabled == 0) {
        err = sources.empty() && ntpSources.empty()
                  ? "no sources configured: give \"sources\" (radio), \"ntp_sources\", or both"
                  : "every source is disabled; there would be nothing to synchronise from";
        return false;
    }

    const int primaryEnabled   = clock.primary == SourceKind::Radio ? enabled : ntpEnabled;
    const int secondaryEnabled = clock.primary == SourceKind::Radio ? ntpEnabled : enabled;
    const char* primaryWord   = sourceKindName(clock.primary);
    const char* secondaryWord = sourceKindName(secondaryKind());

    if (primaryEnabled == 0) {
        // Not an error: an install can legitimately run on the secondary class
        // alone while the primary is being set up, and refusing to start is a
        // worse outcome than serving from the class that does exist. But it is
        // never what was meant, so it has to be said out loud -- finalise has
        // no logger, so it is recorded for main() to say once it has one.
        warnings.push_back(std::string("clock.primary is \"") + primaryWord +
                           "\" but no " + primaryWord +
                           " source is enabled; the time will come from the " +
                           secondaryWord + " sources alone");
    }
    if (secondaryEnabled == 0 && clock.secondary != SecondaryMode::Cold) {
        warnings.push_back(std::string("clock.secondary is \"") +
                           secondaryModeName(clock.secondary) + "\" but no " + secondaryWord +
                           " source is enabled; there is nothing to fall back to");
    }

    // A minimum larger than its class can ever supply means that class can
    // never be healthy, which is a configuration that cannot work rather than
    // one that works badly. Named by whichever key set it, so the message
    // points at the line that needs changing.
    auto minKey = [&](SourceKind k) -> std::string {
        const bool primary = k == clock.primary;
        const bool roleGiven = primary ? minSourcesGiven : minSecondarySourcesGiven;
        const std::string roleKey = primary ? "ntp.min_sources" : "clock.min_secondary_sources";
        const std::string kindKey = k == SourceKind::Radio ? "clock.min_radio_sources"
                                                           : "clock.min_ntp_sources";
        return roleGiven ? roleKey + " (the " + sourceKindName(k) + " minimum)" : kindKey;
    };
    if (primaryEnabled > 0 && ntp.minSources > primaryEnabled) {
        err = minKey(clock.primary) + " is " + std::to_string(ntp.minSources) + " but only " +
              std::to_string(primaryEnabled) + " " + primaryWord + " source(s) are enabled";
        return false;
    }
    if (secondaryEnabled > 0 && clock.minSecondarySources > secondaryEnabled) {
        err = minKey(secondaryKind()) + " is " + std::to_string(clock.minSecondarySources) +
              " but only " + std::to_string(secondaryEnabled) + " " + secondaryWord +
              " source(s) are enabled";
        return false;
    }
    return true;
}

} // namespace ubersdr_ntp
