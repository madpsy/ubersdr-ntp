#pragma once

// What has happened, as opposed to what is.
//
// Everything else this daemon reports is a snapshot: the status page says a
// source is locked, and says nothing about the fact that it lost lock twice in
// the last hour, or that the time failed over to the network at 03:12 and came
// back at 06:40. That history is in the log file, among thirty-second status
// blocks, on a machine the person looking at the page may not have a shell on.
// So the transitions worth knowing about are kept here as well, a hundred of
// them, and served to the page.
//
// Worth knowing about means a CHANGE someone would act on or want explained:
// a source locking or losing it, a class becoming healthy or not, a failover,
// the served time going unsynchronised. Not every decoder state or every poll:
// a list that fills with noise in ten minutes is a list nobody reads, and a
// hundred entries has to cover days of a healthy install.
//
// Two parts. EventLog is the ring buffer and knows nothing about sources.
// EventMonitor is fed the same snapshots and combine result the main loop
// already has every 250 ms, remembers the previous pass, and records the
// differences -- one place that decides what counts as an event, rather than a
// call to the log sprinkled through every source and the Selector, each with
// its own idea of what is interesting and none able to see the others.
//
// In memory only. A restart is itself an event, and the log file is the
// durable record.

#include "Config.h"
#include "Selector.h"
#include "SourceSnapshot.h"

#include "../third_party/json.hpp"

#include <cstdint>
#include <deque>
#include <map>
#include <mutex>
#include <string>
#include <vector>

namespace ubersdr_ntp {

enum class EventSeverity { Info, Notice, Warning, Error };
const char* eventSeverityName(EventSeverity s);

enum class EventType {
    DaemonStarted,

    // The served time.
    Synchronised,
    Unsynchronised,
    Coasting,
    StratumChanged,
    LeapPending,
    LeapCleared,

    // The two classes.
    ClassHealthy,
    ClassUnhealthy,
    Failover,
    Failback,
    StandbyUp,
    StandbyDown,

    // One source, either kind.
    SourceReady,
    SourceLost,
    SourceRefused,
    SourceAccepted,
    SourceReacquire,

    // One radio source.
    LinkUp,
    LinkDown,
    StationChanged,
    TimeRefused,
    TimeAdopted,

    // One upstream NTP server.
    UpstreamChanged,
    KissOfDeath,
};

// What the page shows in its filter, and what a reader of the API can learn
// without this header: the name, a label, which group it is in, how serious
// it is, and a sentence on what it means.
struct EventTypeInfo {
    EventType type;
    const char* name;
    const char* label;
    const char* category;   // daemon, clock, class, source, radio, ntp
    EventSeverity severity;
    const char* description;
};
const std::vector<EventTypeInfo>& eventTypes();
const EventTypeInfo& eventTypeInfo(EventType t);

struct Event {
    std::uint64_t id = 0;       // increasing from 1; never reused within a run
    double unix = 0.0;          // UTC, seconds
    double uptimeSec = 0.0;
    EventType type = EventType::DaemonStarted;
    // The source it concerns, or empty for events about the whole daemon; and
    // for a class event the kind of the class, so a filter by "radio" finds it.
    std::string source;
    bool haveKind = false;
    SourceKind kind = SourceKind::Radio;
    std::string message;
};

// One event as /api/eventlog and the MQTT events topic both render it.
nlohmann::json eventJson(const Event& e);

class EventLog {
public:
    static constexpr std::size_t kCapacity = 100;

    // Stamps the id and appends, dropping the oldest past capacity.
    void add(Event e);

    // Newest first.
    std::vector<Event> all() const;
    std::uint64_t latestId() const;
    std::uint64_t totalRecorded() const { return latestId(); }

    // How many of each type since startup, every type present, by name. Not
    // bounded by the capacity: a failover a week ago still counts.
    std::map<std::string, std::uint64_t> countsByType() const;

private:
    mutable std::mutex m_mu;
    std::deque<Event> m_events;
    std::uint64_t m_nextId = 1;
    std::map<EventType, std::uint64_t> m_counts;
};

class EventMonitor {
public:
    EventMonitor(const Config& cfg, EventLog& log);

    struct Pass {
        const std::vector<SourceSnapshot>& sources;
        const Combined& combined;
        bool secondaryActive;
        double unix;       // UTC now
        double uptimeSec;
    };

    // Compares with the previous pass and records what changed. Called from
    // the main loop only; not thread-safe against itself.
    void observe(const Pass& p);

    // Recorded once at startup, before the first pass.
    void started(const std::string& version, double unix);

private:
    struct SourceState {
        bool seen = false;
        bool ready = false;
        LinkState link = LinkState::Idle;
        bool linkFailReported = false;
        std::string station;
        bool refused = false;
        bool rejectingTime = false;     // inside a run of refused decoded times
        int timeAdoptions = 0;
        std::string kiss;
        int stratum = 0;
        std::string refid;
    };

    void emit(EventType t, const Pass& p, std::string message, const std::string& source = {},
              bool haveKind = false, SourceKind kind = SourceKind::Radio);

    const Config& m_cfg;
    EventLog& m_log;

    bool m_first = true;
    std::map<std::string, SourceState> m_sources;

    bool m_synchronised = false;
    bool m_everSynchronised = false;
    ServingClass m_serving = ServingClass::None;
    int m_stratum = 0;
    bool m_leap = false;
    // Which side last served, Primary or Secondary; Both, Coasting and None do
    // not move it, so a primary that coasts and comes back is not a failback.
    ServingClass m_side = ServingClass::None;
    bool m_primaryHealthy = false;
    bool m_secondaryHealthy = false;
    bool m_secondaryActive = true;
};

} // namespace ubersdr_ntp
