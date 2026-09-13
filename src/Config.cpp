#include "Config.h"

#include "../third_party/json.hpp"

#include <cmath>
#include <fstream>
#include <set>
#include <sstream>

using nlohmann::json;

namespace ubersdr_ntp {

const char* formatName(AudioFormat f) {
    switch (f) {
        case AudioFormat::Opus:  return "opus";
        case AudioFormat::PcmV4: return "pcm-v4";
    }
    return "?";
}

bool parseFormat(const std::string& s, AudioFormat& out) {
    if (s == "opus") { out = AudioFormat::Opus; return true; }
    // Three spellings for one thing: "pcm-zstd" is what the query parameter is
    // called on the wire, "pcm-v4" is what the codec actually is, and "pcm" is
    // what somebody will type. Version 4 carries no zstd; only the parameter
    // name is historical.
    if (s == "pcm-v4" || s == "pcm-zstd" || s == "pcm") { out = AudioFormat::PcmV4; return true; }
    return false;
}

std::uint64_t dialForCarrier(std::uint64_t carrierHz) {
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

    auto it = j.find("format");
    if (it != j.end() && !it->is_null()) {
        std::string f;
        try { f = it->get<std::string>(); }
        catch (const std::exception& e) { err = std::string("field \"format\": ") + e.what(); return false; }
        if (!parseFormat(f, s.format)) {
            err = "unknown format \"" + f + "\" (expected opus or pcm-v4)";
            return false;
        }
    }

    // An explicit delay_ms with auto_delay still on is a contradiction that
    // would silently ignore one of them. Say so rather than pick.
    if (j.contains("delay_ms") && !j.contains("auto_delay") && s.delayMs != 0.0) {
        s.autoDelay = false;
    }
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
        if (!getOpt(n, "answer_when_unsynchronised", c.ntp.answerWhenUnsynchronised, err)) return false;
        if (!getOpt(n, "honour_leap_warning", c.ntp.honourLeapWarning, err)) return false;
        if (!getOpt(n, "rate_limit_per_client", c.ntp.rateLimitPerClient, err)) return false;
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

    if (auto it = j.find("defaults"); it != j.end() && it->is_object()) {
        if (!getSource(*it, c.defaults, err)) { err = "in \"defaults\": " + err; return false; }
    }

    auto sit = j.find("sources");
    if (sit == j.end() || !sit->is_array() || sit->empty()) {
        err = "config needs a non-empty \"sources\" array";
        return false;
    }
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

    out = std::move(c);
    return true;
}

bool Config::finalise(std::string& err) {
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

        if (s.name.empty()) {
            // Named after what it listens to, which is what anyone reading the
            // log wants to see anyway.
            std::ostringstream os;
            os << (s.dialHz < kWwvbCeilingHz ? "wwvb" : "wwv");
            if (s.dialHz >= kWwvbCeilingHz) os << (s.carrierHz / 1000000) << "M";
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
        if (s.enabled) ++enabled;
    }

    if (enabled == 0) {
        err = "every source is disabled; there would be nothing to decode";
        return false;
    }
    if (ntp.minSources > enabled) {
        err = "ntp.min_sources is " + std::to_string(ntp.minSources) +
              " but only " + std::to_string(enabled) + " source(s) are enabled";
        return false;
    }
    return true;
}

} // namespace ubersdr_ntp
