#include "Mqtt.h"

#include "CivilTime.h"
#include "Log.h"
#include "SampleClock.h"
#include "Version.h"

#include <curl/curl.h>

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <set>

namespace ubersdr_ntp {

namespace {

constexpr const char* kTag = "mqtt";

// Shown on this addon's device card in Home Assistant.
constexpr const char* kModel = "WWV/WWVH/WWVB/DCF77 stratum-1 NTP server";

// Well inside the receiver's offline_after_sec (300 s by default), past which
// it marks the addon offline and every entity goes unavailable.
constexpr double kSummaryEverySec = 30.0;
// The closest two summaries come when events prompt them.
constexpr double kSummaryMinGapSec = 5.0;
constexpr double kSourcesMinEverySec = 30.0;
constexpr double kProbeEverySec = 30.0;
// After a 429 or a 503, nothing is sent for this long.
constexpr double kHoldSec = 15.0;
constexpr long kTimeoutSec = 5;

// The ingest port's own limits (addon_mqtt.md, "Reference").
constexpr std::size_t kMaxSubTopic = 64;
constexpr std::size_t kMaxSegmentForSource = kMaxSubTopic - 7;   // "source/"

std::size_t curlWrite(char* ptr, std::size_t size, std::size_t n, void* user) {
    auto* out = static_cast<std::string*>(user);
    // A reply this program reads is a few hundred bytes; anything past that is
    // not worth holding.
    if (out->size() < 16384) out->append(ptr, std::min(size * n, 16384 - out->size()));
    return size * n;
}

// Aborts a transfer in progress when the daemon is shutting down, so stop()
// does not wait out a five-second timeout on a receiver that has gone quiet.
int curlProgress(void* user, curl_off_t, curl_off_t, curl_off_t, curl_off_t) {
    return static_cast<const std::atomic<bool>*>(user)->load() ? 0 : 1;
}

std::string dump(const nlohmann::json& j) {
    // Replace, as everywhere else: strings here include bytes that came off the
    // network unvalidated, and nlohmann's strict default throws on bad UTF-8.
    return j.dump(-1, ' ', false, nlohmann::json::error_handler_t::replace);
}

std::string isoAt(double unix) {
    return iso8601(static_cast<long long>(std::llround(unix * 1000.0)));
}

// UTC now: from the served clock when there is one, as the event log stamps
// its entries, so a host clock that is out does not put a wrong time on the
// one feed that exists to report a right one.
double utcNow(const Combined& c) {
    const double d = daemonNow();
    return c.valid ? c.utcAt(d) : d - daemonMinusRealtime();
}

nlohmann::json entity(const char* key, const char* component, const char* name,
                      const char* valueTemplate) {
    return {{"sub_topic", "summary"}, {"entity_key", key}, {"component", component},
            {"name", name}, {"value_template", valueTemplate}};
}

} // namespace

// ---------------------------------------------------------------------------
// Home Assistant
// ---------------------------------------------------------------------------

// All from the one retained summary, told apart by entity_key, so one publish
// updates every entity and Home Assistant has values the moment it subscribes.
//
// A template that can have nothing to show renders '' rather than a zero:
// Home Assistant skips an empty render and leaves the entity "unknown", where
// a zero would be a reading that was never taken. The served figures are all
// zero until the first measurement, so they are guarded on served.valid.
//
// Per-source entities are deliberately absent. Their number would grow with
// the configuration while the receiver caps an addon at 20; each source's line
// is an attribute of "Sources In Use" instead, and its full record is on its
// own topic for anything that subscribes directly.
std::vector<nlohmann::json> MqttPublisher::entities() {
    std::vector<nlohmann::json> v;
    auto add = [&v](nlohmann::json e) { v.push_back(std::move(e)); return &v.back(); };

    auto* e = add(entity("synchronised", "binary_sensor", "Synchronised",
                         "{{ 'ON' if value_json.served.synchronised else 'OFF' }}"));
    (*e)["icon"] = "mdi:clock-check-outline";
    (*e)["json_attributes_template"] =
        "{{ {'note': value_json.served.note, 'refid': value_json.served.refid} | tojson }}";

    e = add(entity("stratum", "sensor", "Stratum", "{{ value_json.served.stratum }}"));
    (*e)["state_class"] = "measurement";
    (*e)["icon"] = "mdi:layers-triple-outline";

    e = add(entity("serving", "sensor", "Serving", "{{ value_json.clock.serving }}"));
    (*e)["icon"] = "mdi:swap-horizontal";
    (*e)["json_attributes_template"] =
        "{{ {'note': value_json.clock.serving_note, 'primary': value_json.clock.primary, "
        "'secondary': value_json.clock.secondary, 'mode': value_json.clock.secondary_mode} "
        "| tojson }}";

    e = add(entity("failed_over", "binary_sensor", "Failed Over",
                   "{{ 'ON' if value_json.clock.serving == 'secondary' else 'OFF' }}"));
    (*e)["device_class"] = "problem";

    e = add(entity("offset", "sensor", "Host Clock Offset",
                   "{{ value_json.served.offset_ms | round(3) if value_json.served.valid else '' }}"));
    (*e)["unit_of_measurement"] = "ms";
    (*e)["state_class"] = "measurement";
    (*e)["icon"] = "mdi:clock-fast";

    e = add(entity("dispersion", "sensor", "Root Dispersion",
                   "{{ value_json.served.root_dispersion_ms | round(2) "
                   "if value_json.served.valid else '' }}"));
    (*e)["unit_of_measurement"] = "ms";
    (*e)["state_class"] = "measurement";
    (*e)["icon"] = "mdi:plus-minus-variant";

    e = add(entity("sources_used", "sensor", "Sources In Use",
                   "{{ value_json.served.sources_used }}"));
    (*e)["state_class"] = "measurement";
    (*e)["icon"] = "mdi:access-point-network";
    (*e)["json_attributes_template"] = "{{ value_json.sources | tojson }}";

    e = add(entity("receivers_locked", "sensor", "Receivers Locked",
                   "{{ value_json.counts.radio_ready }}"));
    (*e)["state_class"] = "measurement";
    (*e)["icon"] = "mdi:radio-tower";

    e = add(entity("upstreams_usable", "sensor", "Upstream Servers Usable",
                   "{{ value_json.counts.ntp_ready }}"));
    (*e)["state_class"] = "measurement";
    (*e)["icon"] = "mdi:server-network";

    e = add(entity("class_delta", "sensor", "Class Delta",
                   "{{ value_json.clock.class_delta.delta_ms | round(2) "
                   "if value_json.clock.class_delta.valid else '' }}"));
    (*e)["unit_of_measurement"] = "ms";
    (*e)["state_class"] = "measurement";
    (*e)["icon"] = "mdi:delta";

    e = add(entity("ntp_requests", "sensor", "NTP Requests", "{{ value_json.ntp.requests }}"));
    (*e)["state_class"] = "total_increasing";
    (*e)["icon"] = "mdi:counter";

    e = add(entity("ntp_request_rate", "sensor", "NTP Request Rate",
                   "{{ value_json.ntp.requests_per_minute | round(1) "
                   "if value_json.ntp.requests_per_minute is not none else '' }}"));
    (*e)["unit_of_measurement"] = "req/min";
    (*e)["state_class"] = "measurement";
    (*e)["icon"] = "mdi:chart-line";

    // Running totals since startup, from the event counts: the two things a
    // clock meant to run unattended for months should be seen doing rarely.
    // The version is on the device card, so it needs no entity of its own.
    e = add(entity("failovers", "sensor", "Failovers", "{{ value_json.events.counts.failover }}"));
    (*e)["state_class"] = "total_increasing";
    (*e)["icon"] = "mdi:swap-horizontal-bold";

    e = add(entity("time_refusals", "sensor", "Decoded Times Refused",
                   "{{ value_json.events.counts.time_refused }}"));
    (*e)["state_class"] = "total_increasing";
    (*e)["icon"] = "mdi:clock-alert-outline";

    e = add(entity("leap_pending", "binary_sensor", "Leap Second Pending",
                   "{{ 'ON' if value_json.served.leap_pending else 'OFF' }}"));
    (*e)["icon"] = "mdi:timer-alert-outline";

    // A state is capped at 255 characters in Home Assistant; the whole event is
    // an attribute, where there is no such cap.
    e = add(entity("last_event", "sensor", "Last Event",
                   "{{ value_json.last_event.message[:250] if value_json.last_event else '' }}"));
    (*e)["icon"] = "mdi:history";
    (*e)["json_attributes_template"] =
        "{{ value_json.last_event | tojson if value_json.last_event else '{}' }}";

    e = add(entity("reference_age", "sensor", "Reference Age",
                   "{{ value_json.served.age_seconds | round(0) if value_json.served.valid else '' }}"));
    (*e)["unit_of_measurement"] = "s";
    (*e)["device_class"] = "duration";
    (*e)["state_class"] = "measurement";
    (*e)["entity_category"] = "diagnostic";

    e = add(entity("clock_drift", "sensor", "Clock Drift",
                   "{{ value_json.served.clock_rate_ppm | round(3) "
                   "if value_json.served.valid else '' }}"));
    (*e)["unit_of_measurement"] = "ppm";
    (*e)["state_class"] = "measurement";
    (*e)["icon"] = "mdi:speedometer";
    (*e)["entity_category"] = "diagnostic";

    e = add(entity("refid", "sensor", "Reference", "{{ value_json.served.refid }}"));
    (*e)["icon"] = "mdi:tag-outline";
    (*e)["entity_category"] = "diagnostic";

    e = add(entity("started", "sensor", "Started", "{{ value_json.started_utc }}"));
    (*e)["device_class"] = "timestamp";
    (*e)["entity_category"] = "diagnostic";


    return v;
}

std::string MqttPublisher::topicSegment(const std::string& name) {
    std::string out;
    for (unsigned char ch : name) {
        if (std::isalnum(ch)) out += static_cast<char>(std::tolower(ch));
        else if (ch == '_' || ch == '-') out += static_cast<char>(ch);
        else if (!out.empty() && out.back() != '-') out += '-';
    }
    while (!out.empty() && (out.front() == '-' || out.front() == '_')) out.erase(out.begin());
    if (out.size() > kMaxSegmentForSource) out.resize(kMaxSegmentForSource);
    while (!out.empty() && out.back() == '-') out.pop_back();
    if (out.empty()) out = "source";
    return out;
}

// ---------------------------------------------------------------------------
// Lifecycle
// ---------------------------------------------------------------------------

MqttPublisher::MqttPublisher(MqttConfig cfg, StatusProvider provider, const EventLog& events)
    : m_cfg(std::move(cfg)), m_provider(std::move(provider)), m_events(events) {}

MqttPublisher::~MqttPublisher() { stop(); }

void MqttPublisher::start() {
    if (!m_cfg.enabled || m_running.exchange(true)) return;
    LOG_INFO(kTag, "ingest endpoint %s", m_cfg.ingestUrl.c_str());
    m_thread = std::thread([this] { run(); });
}

void MqttPublisher::stop() {
    {
        std::lock_guard<std::mutex> lk(m_mu);
        if (!m_running.exchange(false)) return;
    }
    m_cv.notify_all();
    if (m_thread.joinable()) m_thread.join();
}

void MqttPublisher::run() {
    while (m_running) {
        const double now = monotonicNow();

        if (!m_available && now >= m_nextProbe) {
            m_nextProbe = now + kProbeEverySec;
            if (probe()) {
                m_eventTokens = std::max(4.0, m_rateLimit / 4.0);
                m_eventRefillAt = now;
                m_nextSummary = now;
                m_nextSources = now;
            }
        }

        // Each step only while the receiver is taking publishes: any of them
        // can find it gone (dormant until the next probe) or unable to pass
        // them on (held for a while), and what did not go out goes next time.
        auto open = [this] { return m_available && monotonicNow() >= m_holdUntil; };
        if (open()) publishEvents(now);
        if (open() && now >= m_nextSummary) {
            const StatusInput in = m_provider();
            publishSummary(in, now);
            if (open() && now >= m_nextSources && publishSources(in))
                m_nextSources = now + sourceInterval(in.sources.size());
        }

        std::unique_lock<std::mutex> lk(m_mu);
        m_cv.wait_for(lk, std::chrono::seconds(1), [this] { return !m_running.load(); });
    }
}

// ---------------------------------------------------------------------------
// The ingest port
// ---------------------------------------------------------------------------

long MqttPublisher::request(const char* method, const std::string& path, const std::string* body,
                            std::string& response, std::string& err) {
    CURL* curl = curl_easy_init();
    if (!curl) { err = "curl_easy_init failed"; return -1; }

    struct curl_slist* headers = nullptr;
    headers = curl_slist_append(headers, "Content-Type: application/json");
    const std::string ua = std::string("User-Agent: ") + kUserAgent;
    headers = curl_slist_append(headers, ua.c_str());

    const std::string url = m_cfg.ingestUrl + path;
    curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
    curl_easy_setopt(curl, CURLOPT_CUSTOMREQUEST, method);
    if (body) {
        curl_easy_setopt(curl, CURLOPT_POSTFIELDS, body->data());
        curl_easy_setopt(curl, CURLOPT_POSTFIELDSIZE, static_cast<long>(body->size()));
    }
    curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, curlWrite);
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, &response);
    curl_easy_setopt(curl, CURLOPT_TIMEOUT, kTimeoutSec);
    curl_easy_setopt(curl, CURLOPT_CONNECTTIMEOUT, kTimeoutSec);
    // This is a thread among several; curl's signal-based DNS timeout is not
    // safe in one.
    curl_easy_setopt(curl, CURLOPT_NOSIGNAL, 1L);
    curl_easy_setopt(curl, CURLOPT_NOPROGRESS, 0L);
    curl_easy_setopt(curl, CURLOPT_XFERINFOFUNCTION, curlProgress);
    curl_easy_setopt(curl, CURLOPT_XFERINFODATA, &m_running);

    long code = -1;
    const CURLcode res = curl_easy_perform(curl);
    if (res == CURLE_OK) curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &code);
    else err = curl_easy_strerror(res);

    curl_slist_free_all(headers);
    curl_easy_cleanup(curl);
    return code;
}

void MqttPublisher::unavailable(const std::string& why) {
    m_available = false;
    m_nextProbe = monotonicNow() + kProbeEverySec;
    // A transfer cut short by stop() is not the receiver's doing.
    if (!m_running) return;
    if (!m_warned) {
        m_warned = true;
        LOG_INFO(kTag, "%s -- continuing without MQTT, and asking again every %.0f s",
                 why.c_str(), kProbeEverySec);
    }
}

bool MqttPublisher::probe() {
    std::string resp, err;
    const long code = request("GET", "/health", nullptr, resp, err);
    if (code < 0) { unavailable("ingest port unreachable (" + err + ")"); return false; }
    if (code == 403) {
        unavailable("the receiver does not recognise this machine as an installed addon");
        return false;
    }
    if (code != 200) { unavailable("ingest health answered HTTP " + std::to_string(code)); return false; }

    nlohmann::json h;
    try {
        h = nlohmann::json::parse(resp);
    } catch (const std::exception& e) {
        unavailable(std::string("could not parse the ingest health (") + e.what() + ")");
        return false;
    }
    if (!h.is_object()) { unavailable("ingest health is not an object"); return false; }

    m_addon = h.value("addon", std::string());
    m_haDiscovery = h.value("ha_discovery", false);
    m_rateLimit = std::max(1, h.value("rate_limit", 120));
    m_maxEntities = std::max(0, h.value("max_entities", 20));
    const int offlineAfter = h.value("offline_after_sec", 300);

    m_available = true;
    m_warned = false;
    LOG_INFO(kTag, "publishing as addon \"%s\" (broker %s, Home Assistant discovery %s, "
             "%d publishes/min)",
             m_addon.c_str(), h.value("mqtt_connected", false) ? "connected" : "not connected",
             m_haDiscovery ? "on" : "off", m_rateLimit);
    if (offlineAfter > 0 && offlineAfter < 2 * kSummaryEverySec) {
        LOG_WARN(kTag, "the receiver marks an addon offline after %d s without a publish, "
                 "and this publishes every %.0f s: entities will flap",
                 offlineAfter, kSummaryEverySec);
    }
    if (m_haDiscovery) declareEntities();
    return true;
}

// Idempotent on the receiver's side -- a repeat is an in-place update -- so this
// runs on every (re)connection, which also covers the receiver enabling Home
// Assistant discovery after this started.
void MqttPublisher::declareEntities() {
    const std::vector<nlohmann::json> all = entities();
    const std::size_t n = std::min(all.size(), static_cast<std::size_t>(m_maxEntities));
    std::size_t declared = 0;
    for (std::size_t i = 0; i < n && m_running; ++i) {
        nlohmann::json e = all[i];
        e["addon_version"] = kVersion;
        e["addon_model"] = kModel;
        const std::string body = dump(e);
        std::string resp, err;
        const long code = request("POST", "/discovery", &body, resp, err);
        const std::string key = e.value("entity_key", std::string());
        if (code < 0) { unavailable("declaring " + key + ": " + err); return; }
        if (code == 503) {
            LOG_INFO(kTag, "Home Assistant discovery is off on the receiver; publishing data only");
            return;
        }
        if (code >= 400) {
            while (!resp.empty() && std::isspace(static_cast<unsigned char>(resp.back()))) resp.pop_back();
            LOG_WARN(kTag, "declaring %s refused (HTTP %ld): %s", key.c_str(), code, resp.c_str());
            continue;
        }
        ++declared;
    }
    LOG_INFO(kTag, "declared %zu of %zu Home Assistant entities%s", declared, all.size(),
             n < all.size() ? " (the receiver's limit)" : "");
}

MqttPublisher::Result MqttPublisher::publish(const std::string& subTopic,
                                             const nlohmann::json& payload, bool retain) {
    const std::string body = dump(payload);
    std::string resp, err;
    const long code = request("POST", "/publish/" + subTopic + (retain ? "?retain=true" : ""),
                              &body, resp, err);
    if (code < 0) {
        // The receiver may be restarting. Dormant until the next probe, which
        // re-declares, rather than failing again on every topic.
        unavailable("publishing " + subTopic + " failed (" + err + ")");
        return Result::Unreachable;
    }
    if (code < 300) return Result::Ok;
    if (code == 503) {
        // The receiver's broker, not the receiver: transient, and it says so
        // in its own log. Held rather than retried every second.
        m_holdUntil = monotonicNow() + kHoldSec;
        return Result::BrokerDown;
    }
    if (code == 429) {
        m_holdUntil = monotonicNow() + kHoldSec;
        LOG_WARN(kTag, "rate limited publishing %s; holding for %.0f s",
                 subTopic.c_str(), kHoldSec);
        return Result::RateLimited;
    }
    if (code == 403) {
        unavailable("the receiver no longer recognises this machine as an addon");
        return Result::Refused;
    }
    while (!resp.empty() && std::isspace(static_cast<unsigned char>(resp.back()))) resp.pop_back();
    LOG_WARN(kTag, "publishing %s refused (HTTP %ld): %s", subTopic.c_str(), code, resp.c_str());
    return Result::Refused;
}

// ---------------------------------------------------------------------------
// What is published
// ---------------------------------------------------------------------------

const std::string& MqttPublisher::segmentFor(const std::string& name) {
    if (auto it = m_segments.find(name); it != m_segments.end()) return it->second;
    const std::string base = topicSegment(name);
    std::set<std::string> taken;
    for (const auto& kv : m_segments) taken.insert(kv.second);
    std::string seg = base;
    for (int n = 2; taken.count(seg); ++n) {
        const std::string suffix = "-" + std::to_string(n);
        seg = base.substr(0, kMaxSegmentForSource - suffix.size()) + suffix;
    }
    return m_segments[name] = seg;
}

double MqttPublisher::sourceInterval(std::size_t sources) const {
    // Half the limit, less the summary at its busiest: two a minute on the
    // timer and twelve more if events keep prompting one every five seconds.
    const double budget = std::max(1.0, m_rateLimit / 2.0 - 14.0);
    return std::max(kSourcesMinEverySec, 60.0 * static_cast<double>(sources) / budget);
}

// Every event, once, in order. None is dropped to stay inside the budget: one
// that does not fit waits for the bucket to refill, and one the receiver could
// not pass on (its broker down, or it rate limited) is sent again after the
// hold. Events recorded while the receiver was unreachable are sent when it
// comes back, each carrying the time it happened. The only ones that can be
// lost are those the event log itself overwrote first -- more than a hundred
// behind -- and they are counted in the summary.
void MqttPublisher::publishEvents(double now) {
    const double cap = std::max(4.0, m_rateLimit / 4.0);
    m_eventTokens = std::min(cap, m_eventTokens + (now - m_eventRefillAt) * cap / 60.0);
    m_eventRefillAt = now;

    if (m_events.latestId() <= m_lastEventId) return;
    std::vector<Event> fresh;
    for (const Event& e : m_events.all()) {   // newest first
        if (e.id <= m_lastEventId) break;
        fresh.push_back(e);
    }
    if (!fresh.empty() && fresh.back().id > m_lastEventId + 1) {
        m_eventsLost += fresh.back().id - m_lastEventId - 1;
        m_lastEventId = fresh.back().id - 1;
    }

    bool sent = false;
    for (auto it = fresh.rbegin(); it != fresh.rend() && m_eventTokens >= 1.0; ++it) {
        m_eventTokens -= 1.0;
        const Result r = publish("events", eventJson(*it), false);
        if (r != Result::Ok && r != Result::Refused) break;   // try it again later
        m_lastEventId = it->id;
        sent = true;
    }
    // Let the retained state catch up with what just happened, soon but not
    // once per event of a burst.
    if (sent)
        m_nextSummary = std::min(m_nextSummary, std::max(now, m_lastSummary + kSummaryMinGapSec));
}

void MqttPublisher::publishSummary(const StatusInput& in, double now) {
    nlohmann::json j = statusJson(in);
    j.erase("sources");

    const double utc = utcNow(in.combined);
    j["time_utc"] = isoAt(utc);
    j["started_utc"] = isoAt(utc - in.uptimeSec);

    // Requests a minute over the interval since the last summary.
    if (m_lastRequestsAt >= 0.0 && now - m_lastRequestsAt >= 1.0 && in.ntp.requests >= m_lastRequests)
        j["ntp"]["requests_per_minute"] =
            static_cast<double>(in.ntp.requests - m_lastRequests) / (now - m_lastRequestsAt) * 60.0;
    else
        j["ntp"]["requests_per_minute"] = nullptr;
    m_lastRequests = in.ntp.requests;
    m_lastRequestsAt = now;

    // One line per source, as the status page's diagram has it: its part in the
    // served time (live, standby, down), how far it is from it, and where in
    // acquisition it is.
    nlohmann::json sources = nlohmann::json::object();
    int radio = 0, radioReady = 0, ntp = 0, ntpReady = 0;
    for (const SourceSnapshot& s : in.sources) {
        const nlohmann::json o = sourceJson(s, in.combined);
        const bool inUse = o.value("in_use", false);
        nlohmann::json line;
        line["kind"] = o["kind"];
        line["state"] = inUse ? "live" : (s.ready || s.haveOffset) ? "standby" : "down";
        line["in_use"] = inUse;
        line["ready"] = s.ready;
        line["stage"] = o["decoder"]["stage"];
        line["link"] = o["link"]["state"];
        line["offset_vs_served_ms"] = s.haveOffset && in.combined.valid
            ? nlohmann::json((s.hostOffsetSec - in.combined.hostOffsetSec) * 1000.0)
            : nlohmann::json(nullptr);
        line["not_used_reason"] = o["not_used_reason"];
        line["topic"] = "source/" + segmentFor(s.name);
        sources[s.name] = std::move(line);
        if (s.kind == SourceKind::Ntp) { ++ntp; ntpReady += s.ready; }
        else { ++radio; radioReady += s.ready; }
    }
    j["sources"] = std::move(sources);
    j["counts"] = {{"radio", radio}, {"radio_ready", radioReady},
                   {"ntp", ntp}, {"ntp_ready", ntpReady}};

    const std::vector<Event> all = m_events.all();
    j["last_event"] = all.empty() ? nlohmann::json(nullptr) : eventJson(all.front());
    // Events waiting their turn, and any the event log overwrote before they
    // could be sent (see publishEvents).
    j["mqtt"] = {{"events_pending", m_events.latestId() - std::min(m_lastEventId, m_events.latestId())},
                 {"events_lost", m_eventsLost}};

    const Result r = publish("summary", j, true);
    if (r == Result::Ok || r == Result::Refused) {
        m_lastSummary = now;
        m_nextSummary = now + kSummaryEverySec;
    }
}

// True when every source went out, or was refused outright (retrying a 400
// would not change it); false when the receiver went away or held publishes,
// so the whole set is tried again rather than waiting out the interval.
bool MqttPublisher::publishSources(const StatusInput& in) {
    for (const SourceSnapshot& s : in.sources) {
        if (!m_running) return false;
        const Result r = publish("source/" + segmentFor(s.name), sourceJson(s, in.combined), true);
        if (r != Result::Ok && r != Result::Refused) return false;
    }
    return true;
}

} // namespace ubersdr_ntp
