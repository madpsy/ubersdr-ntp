#pragma once

// Publishing to MQTT through UberSDR's addon ingest port.
//
// UberSDR runs an HTTP listener on the sdr-network (port 6926 by default) that
// lets an addon publish through the receiver's own MQTT connection, and declare
// Home Assistant entities, without holding a broker address or a credential.
// It knows who is calling from the TCP source address alone and places every
// topic under that addon's own namespace -- see addon_mqtt.md in the
// ka9q_ubersdr repository, and mqtt.go in ubersdr_lightning, which this
// follows.
//
// Nothing to configure. With no ingest port to reach -- MQTT off on the
// receiver, or this not running beside one -- or a port that does not recognise
// this machine as an installed addon, it stays dormant, says so once, and asks
// again every thirty seconds, so it comes up by itself when the receiver does.
// No failure here is ever fatal, and nothing here runs on a thread that serves
// time: it is a thread of its own, reading the same StatusInput and EventLog
// the HTTP service reads.
//
// Three topics, under ubersdr/metrics/addons/<name>/ by default:
//
//   summary          retained, every 30 s and soon after any event: the served
//                    time, the two classes, the NTP server's counters, a line
//                    per source, and the newest event. Every Home Assistant
//                    entity reads this one topic.
//   source/<name>    retained, one per source: everything /api/status says
//                    about it. Every 30 s, stretched as sources are added so
//                    that all of them together stay inside the rate limit.
//   events           not retained: every event-log entry, once, in order,
//                    including those recorded while the receiver was away.
//
// Budget. The receiver allows `rate_limit` publishes a minute per addon
// (default 120, read from /health) and answers 429 past it. The summary is at
// most 2 a minute plus one per burst of events, 5 s apart at the closest;
// events have a bucket of their own a quarter of the limit deep, and wait for
// it rather than being dropped; and the source topics get what is left of half
// the limit. Declarations count too, but they are sent once, at connection.

#include "Config.h"
#include "Events.h"
#include "Status.h"

#include "../third_party/json.hpp"

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <map>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace ubersdr_ntp {

class MqttPublisher {
public:
    using StatusProvider = std::function<StatusInput()>;

    MqttPublisher(MqttConfig cfg, StatusProvider provider, const EventLog& events);
    ~MqttPublisher();

    MqttPublisher(const MqttPublisher&) = delete;
    MqttPublisher& operator=(const MqttPublisher&) = delete;

    void start();
    void stop();

    // The Home Assistant entities, in the order they are declared: when the
    // receiver allows fewer than there are, the first ones are kept.
    static std::vector<nlohmann::json> entities();

    // A source name as a topic segment: lowercase [a-z0-9_-], starting with a
    // letter or digit, which is all the ingest port accepts.
    static std::string topicSegment(const std::string& name);

private:
    enum class Result { Ok, Refused, Unreachable, RateLimited, BrokerDown };

    void run();
    bool probe();
    void declareEntities();
    Result publish(const std::string& subTopic, const nlohmann::json& payload, bool retain);
    long request(const char* method, const std::string& path, const std::string* body,
                 std::string& response, std::string& err);
    void unavailable(const std::string& why);

    void publishSummary(const StatusInput& in, double now);
    bool publishSources(const StatusInput& in);
    void publishEvents(double now);
    double sourceInterval(std::size_t sources) const;
    const std::string& segmentFor(const std::string& name);

    MqttConfig m_cfg;
    StatusProvider m_provider;
    const EventLog& m_events;

    std::thread m_thread;
    std::atomic<bool> m_running{false};
    std::mutex m_mu;
    std::condition_variable m_cv;

    // Everything below is the publishing thread's alone.
    bool m_available = false;
    bool m_warned = false;
    bool m_haDiscovery = false;
    int m_rateLimit = 120;           // publishes a minute, from /health
    int m_maxEntities = 20;
    std::string m_addon;

    double m_nextProbe = 0.0;
    double m_nextSummary = 0.0;
    double m_lastSummary = -1e9;
    double m_nextSources = 0.0;
    double m_holdUntil = 0.0;        // after a 429, publish nothing until then

    std::uint64_t m_lastEventId = 0;
    double m_eventTokens = 0.0;
    double m_eventRefillAt = 0.0;
    std::uint64_t m_eventsLost = 0;   // overwritten in the event log before they were sent

    // For the request rate: the counter and when it was read, at the last summary.
    std::uint64_t m_lastRequests = 0;
    double m_lastRequestsAt = -1.0;

    // Source name -> topic segment, fixed at first sight so two names that
    // fold to the same segment keep the suffixes they were given.
    std::map<std::string, std::string> m_segments;
};

} // namespace ubersdr_ntp
