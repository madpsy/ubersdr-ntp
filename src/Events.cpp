#include "Events.h"

#include "CivilTime.h"
#include "Log.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdarg>
#include <cstdio>

namespace ubersdr_ntp {

namespace {

constexpr const char* kTag = "event";

std::string fmt(const char* f, ...) {
    char buf[512];
    va_list ap;
    va_start(ap, f);
    std::vsnprintf(buf, sizeof buf, f, ap);
    va_end(ap);
    return buf;
}

std::string upper(std::string s) {
    for (char& c : s) c = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
    return s;
}

std::string offsetText(const SourceSnapshot& s) {
    if (!s.haveOffset) return {};
    return fmt(", offset %+.1f ± %.1f ms", s.offsetSec * 1000.0, s.dispersionSec * 1000.0);
}

std::string usText(double sec, bool sign = false) {
    return fmt(sign ? "%+.0f µs" : "%.0f µs", sec * 1e6);
}

std::string duration(double sec) {
    if (sec < 90.0) return fmt("%.0f s", sec);
    if (sec < 5400.0) return fmt("%.0f min", sec / 60.0);
    if (sec < 172800.0) return fmt("%.1f h", sec / 3600.0);
    return fmt("%.1f days", sec / 86400.0);
}

} // namespace

const char* eventSeverityName(EventSeverity s) {
    switch (s) {
        case EventSeverity::Info:    return "info";
        case EventSeverity::Notice:  return "notice";
        case EventSeverity::Warning: return "warning";
        case EventSeverity::Error:   return "error";
    }
    return "?";
}

const std::vector<EventTypeInfo>& eventTypes() {
    using S = EventSeverity;
    static const std::vector<EventTypeInfo> kTypes = {
        {EventType::DaemonStarted, "daemon_started", "Daemon started", "daemon", S::Info,
         "The daemon started. Everything before this is from a previous run and is gone."},

        {EventType::Synchronised, "synchronised", "Synchronised", "clock", S::Notice,
         "Time is being served from live sources: for the first time, or again after "
         "coasting or being unsynchronised."},
        {EventType::Unsynchronised, "unsynchronised", "Unsynchronised", "clock", S::Error,
         "No time is being served as synchronised: nothing usable, and nothing left to "
         "coast on. Clients are told to look elsewhere."},
        {EventType::Coasting, "coasting", "Coasting", "clock", S::Warning,
         "No source is usable, so the served time runs on this machine's crystal from the "
         "last good offset, with its claimed accuracy decaying, until the coast limit."},
        {EventType::StratumChanged, "stratum_changed", "Stratum changed", "clock", S::Info,
         "The stratum being served changed, usually because the time now comes from a "
         "different kind of source."},
        {EventType::LeapPending, "leap_pending", "Leap second announced", "clock", S::Notice,
         "Every source in use agrees a leap second is scheduled at the end of the month."},
        {EventType::LeapCleared, "leap_cleared", "Leap warning cleared", "clock", S::Info,
         "The leap second warning is no longer being served."},

        {EventType::ClassHealthy, "class_healthy", "Class healthy", "class", S::Notice,
         "A class of source (radio or NTP) has at least its configured minimum of usable "
         "sources."},
        {EventType::ClassUnhealthy, "class_unhealthy", "Class unhealthy", "class", S::Warning,
         "A class of source has fallen below its configured minimum of usable sources. If "
         "it is the primary, the failover countdown starts."},
        {EventType::Failover, "failover", "Failed over", "class", S::Warning,
         "The primary class had nothing usable for the failover interval, and the time now "
         "comes from the secondary."},
        {EventType::Failback, "failback", "Failed back", "class", S::Notice,
         "The primary class stayed healthy for the failback interval and serves the time "
         "again."},
        {EventType::StandbyUp, "standby_up", "Cold standby brought up", "class", S::Info,
         "In cold mode, the secondary sources were connected because the primary lost its "
         "sources."},
        {EventType::StandbyDown, "standby_down", "Cold standby stood down", "class", S::Info,
         "In cold mode, the secondary sources were disconnected because the primary has "
         "been healthy long enough."},

        {EventType::SourceReady, "source_ready", "Source ready", "source", S::Notice,
         "A source became usable: a receiver's decoder locked, or an upstream server became "
         "reachable with a filtered offset."},
        {EventType::SourceLost, "source_lost", "Source lost", "source", S::Warning,
         "A usable source stopped being usable, and why."},
        {EventType::SourceRefused, "source_refused", "Source refused", "source", S::Warning,
         "A source disagrees with the others of its kind by more than any delay model can "
         "explain, and is kept out of the served time."},
        {EventType::SourceAccepted, "source_accepted", "Source accepted again", "source",
         S::Notice, "A refused source agrees with the others again and is used."},
        {EventType::SourceReacquire, "source_reacquire", "Source sent to re-acquire", "source",
         S::Warning,
         "A source refused for long enough was made to drop its lock and acquire from "
         "nothing."},

        {EventType::LinkUp, "link_up", "Receiver connected", "radio", S::Info,
         "Audio started streaming from an UberSDR receiver."},
        {EventType::LinkDown, "link_down", "Receiver disconnected", "radio", S::Warning,
         "The audio stream from a receiver stopped, or a connection to it failed."},
        {EventType::StationChanged, "station_changed", "Station identified", "radio", S::Info,
         "The decoder decided which transmitter it is hearing (WWV, WWVH, WWVB or DCF77), or "
         "changed its mind."},
        {EventType::TimeRefused, "time_refused", "Decoded time refused", "radio", S::Warning,
         "A receiver decoded a time whole seconds or more from where its own history puts it "
         "-- a misread bit in the time code, since UTC does not jump -- and the decode was "
         "not used."},
        {EventType::TimeAdopted, "time_adopted", "New decoded time taken", "radio", S::Warning,
         "A decoded time that contradicted a receiver's history held for long enough that "
         "the history was taken to be the wrong one, and replaced."},

        {EventType::OffsetHeld, "offset_held", "Offset held", "radio", S::Notice,
         "A receiver's jitter jumped far past its usual level, so its offset is held at the "
         "last calm level instead of following the disturbance, until the jitter settles."},
        {EventType::OffsetReleased, "offset_released", "Offset released", "radio", S::Info,
         "A held offset follows the receiver again -- its jitter settled, or the hold reached "
         "its ten-minute limit -- and the difference is slewed out over about a minute."},

        {EventType::UpstreamChanged, "upstream_changed", "Upstream reference changed", "ntp",
         S::Info,
         "An upstream NTP server changed its own stratum or reference: it is now "
         "synchronised to something else."},
        {EventType::KissOfDeath, "kiss_of_death", "Kiss-o'-death", "ntp", S::Error,
         "An upstream server told this daemon to go away (RATE, DENY or RSTR)."},

        {EventType::PpsPulsing, "pps_pulsing", "1PPS output pulsing", "pps", S::Notice,
         "The serial 1PPS output is marking every second of the served time: for the first "
         "time, or again after the daemon synchronised or the port came back."},
        {EventType::PpsStopped, "pps_stopped", "1PPS output stopped", "pps", S::Warning,
         "The serial 1PPS output stopped pulsing: the daemon is not synchronised, or the "
         "port is missing or failing. The message says which."},
    };
    return kTypes;
}

const EventTypeInfo& eventTypeInfo(EventType t) {
    for (const EventTypeInfo& i : eventTypes()) {
        if (i.type == t) return i;
    }
    return eventTypes().front();
}

nlohmann::json eventJson(const Event& e) {
    const EventTypeInfo& info = eventTypeInfo(e.type);
    nlohmann::json o;
    o["id"] = e.id;
    o["unix"] = e.unix;
    o["utc"] = iso8601(static_cast<long long>(std::llround(e.unix * 1000.0)));
    o["uptime_seconds"] = e.uptimeSec;
    o["type"] = info.name;
    o["label"] = info.label;
    o["category"] = info.category;
    o["severity"] = eventSeverityName(info.severity);
    o["source"] = e.source.empty() ? nlohmann::json(nullptr) : nlohmann::json(e.source);
    o["kind"] = e.haveKind ? nlohmann::json(sourceKindName(e.kind)) : nlohmann::json(nullptr);
    o["message"] = e.message;
    return o;
}

// --- EventLog ---------------------------------------------------------------

void EventLog::add(Event e) {
    std::lock_guard<std::mutex> lk(m_mu);
    e.id = m_nextId++;
    ++m_counts[e.type];
    m_events.push_back(std::move(e));
    while (m_events.size() > kCapacity) m_events.pop_front();
}

std::vector<Event> EventLog::all() const {
    std::lock_guard<std::mutex> lk(m_mu);
    return std::vector<Event>(m_events.rbegin(), m_events.rend());
}

std::uint64_t EventLog::latestId() const {
    std::lock_guard<std::mutex> lk(m_mu);
    return m_nextId - 1;
}

std::map<std::string, std::uint64_t> EventLog::countsByType() const {
    std::map<std::string, std::uint64_t> out;
    for (const EventTypeInfo& t : eventTypes()) out[t.name] = 0;
    std::lock_guard<std::mutex> lk(m_mu);
    for (const auto& kv : m_counts) out[eventTypeInfo(kv.first).name] = kv.second;
    return out;
}

// --- EventMonitor -----------------------------------------------------------

EventMonitor::EventMonitor(const Config& cfg, EventLog& log) : m_cfg(cfg), m_log(log) {}

void EventMonitor::emit(EventType t, const Pass& p, std::string message,
                        const std::string& source, bool haveKind, SourceKind kind) {
    Event e;
    e.unix = p.unix;
    e.uptimeSec = p.uptimeSec;
    e.type = t;
    e.source = source;
    e.haveKind = haveKind;
    e.kind = kind;
    e.message = std::move(message);
    // Debug, because nearly every one of these already has a line of its own
    // at the point it happens; this is for matching the page to the log.
    LOG_DEBUG(kTag, "%s%s%s: %s", eventTypeInfo(t).name, source.empty() ? "" : " ",
              source.c_str(), e.message.c_str());
    m_log.add(std::move(e));
}

void EventMonitor::started(const std::string& version, double unix) {
    Event e;
    e.unix = unix;
    e.type = EventType::DaemonStarted;
    int radio = 0, ntp = 0;
    for (const SourceConfig& s : m_cfg.sources) radio += s.enabled ? 1 : 0;
    for (const NtpSourceConfig& s : m_cfg.ntpSources) ntp += s.enabled ? 1 : 0;
    e.message = fmt("version %s; %d radio and %d NTP source(s); primary %s, secondary %s; "
                    "healthy with at least %d radio and %d NTP",
                    version.c_str(), radio, ntp, sourceKindName(m_cfg.clock.primary),
                    secondaryModeName(m_cfg.clock.secondary), m_cfg.clock.minRadioSources,
                    m_cfg.clock.minNtpSources);
    m_log.add(std::move(e));
}

void EventMonitor::observe(const Pass& p) {
    const Combined& c = p.combined;

    // --- each source --------------------------------------------------------
    std::map<std::string, const SourceResidual*> residuals;
    for (const SourceResidual& r : c.residuals) residuals[r.name] = &r;

    for (const SourceSnapshot& s : p.sources) {
        SourceState& st = m_sources[s.name];
        const bool radio = s.kind == SourceKind::Radio;
        auto ev = [&](EventType t, std::string msg) {
            emit(t, p, std::move(msg), s.name, true, s.kind);
        };

        if (!st.seen) {
            // First sight: take the state as it stands. Nothing has changed yet.
            st.seen = true;
            // Not the link: a receiver that is already streaming by the first
            // pass (a local one connects inside a quarter second) would never
            // get its link_up. Idle is what every link starts from anyway.
            st.link = LinkState::Idle;
            st.station = s.station;
            st.kiss = s.ntp.kissCode;
            st.stratum = s.ntp.stratum;
            st.refid = s.ntp.refid;
        }

        // Ready. Only a source that is enabled and meant to be connected can be
        // said to have lost anything: a cold standby being stood down is its
        // own event, and every one of its sources going "not ready" with it
        // would bury that.
        if (s.ready != st.ready) {
            if (s.ready) {
                if (radio) {
                    ev(EventType::SourceReady,
                       "locked" + (s.station != "unknown" ? " on " + upper(s.station) : std::string()) +
                           fmt(" at %.3f MHz", s.carrierHz / 1e6) + offsetText(s));
                } else {
                    ev(EventType::SourceReady,
                       fmt("reachable: stratum %d, refid %s", s.ntp.stratum, s.ntp.refid.c_str()) +
                           offsetText(s));
                }
            } else if (s.enabled && s.active) {
                ev(EventType::SourceLost,
                   (radio ? "lost lock: " : "no longer usable: ") +
                       (s.notReadyReason.empty() ? std::string("not ready") : s.notReadyReason));
            }
            st.ready = s.ready;
        }

        // Radio link. Dropping out of streaming is reported at once; a
        // connection that cannot be made at all is reported once, when it
        // first backs off, and not again on every retry until it has streamed.
        if (radio) {
            if (s.link == LinkState::Streaming && st.link != LinkState::Streaming) {
                ev(EventType::LinkUp,
                   "streaming from " + (s.receiverName.empty() ? s.url : s.receiverName) +
                       (s.connectAttempts > 1 ? fmt(" after %d attempts", s.connectAttempts)
                                              : std::string()));
                st.linkFailReported = false;
            } else if (st.link == LinkState::Streaming && s.link != LinkState::Streaming &&
                       s.link != LinkState::Stopped) {
                ev(EventType::LinkDown, "stream dropped" + (s.linkDetail.empty()
                                                                ? std::string()
                                                                : ": " + s.linkDetail));
                st.linkFailReported = true;
            } else if (s.link == LinkState::Backoff && !st.linkFailReported && s.active) {
                ev(EventType::LinkDown, "cannot connect to " + s.url +
                                            (s.linkDetail.empty() ? std::string()
                                                                  : ": " + s.linkDetail) +
                                            "; retrying");
                st.linkFailReported = true;
            }
            st.link = s.link;

            if (s.station != st.station && s.station != "unknown") {
                ev(EventType::StationChanged,
                   st.station == "unknown" || st.station.empty()
                       ? "hearing " + upper(s.station)
                       : "now hearing " + upper(s.station) + ", was " + upper(st.station));
            }
            // "unknown" does not overwrite a decided station, so a decoder
            // restart that re-decides the same one is not news.
            if (s.station != "unknown") st.station = s.station;

            // Decoded times refused as jumps: once when a run of them starts,
            // not once a minute for as long as it lasts.
            const bool rejecting = s.timeCheck.rfind("refused", 0) == 0;
            if (rejecting && !st.rejectingTime) {
                ev(EventType::TimeRefused,
                   "decoded " + s.lastRejectedUtc + ", " + jumpText(s.lastRejectedJumpSec) +
                       " from its own history; refused");
            }
            st.rejectingTime = rejecting;
            if (s.timeAdoptions > st.timeAdoptions) {
                ev(EventType::TimeAdopted,
                   "a decoded time " + jumpText(s.lastRejectedJumpSec) +
                       " from its old history held long enough to replace it");
            }
            st.timeAdoptions = s.timeAdoptions;

            if (s.offsetHeld && !st.offsetHeld) {
                ev(EventType::OffsetHeld,
                   "jitter " + usText(s.jitterSec) + " against a usual " + usText(s.jitterBaselineSec) +
                       "; offset held at its last calm level");
            } else if (!s.offsetHeld && st.offsetHeld) {
                ev(EventType::OffsetReleased,
                   s.spikeHoldsTimedOut > st.holdsTimedOut
                       ? "jitter still high after the hold limit; following the live level, " +
                             usText(s.rejoinGapSec, true) + " slewed out"
                       : "jitter settled; " + usText(s.rejoinGapSec, true) + " slewed out");
            }
            st.offsetHeld = s.offsetHeld;
            st.holdsTimedOut = s.spikeHoldsTimedOut;
        } else {
            if (!s.ntp.kissCode.empty() && s.ntp.kissCode != st.kiss) {
                ev(EventType::KissOfDeath,
                   "server sent " + s.ntp.kissCode +
                       (s.ntp.stopped ? "; it will not be polled again" : "; backing off"));
            }
            st.kiss = s.ntp.kissCode;

            if (s.ntp.stratum > 0 && s.ntp.stratum < 16) {
                if (st.stratum > 0 && st.stratum < 16 &&
                    (s.ntp.stratum != st.stratum || s.ntp.refid != st.refid)) {
                    ev(EventType::UpstreamChanged,
                       fmt("stratum %d, refid %s (was stratum %d, refid %s)", s.ntp.stratum,
                           s.ntp.refid.c_str(), st.stratum, st.refid.c_str()));
                }
                st.stratum = s.ntp.stratum;
                st.refid = s.ntp.refid;
            }
        }

        // Agreement with the rest of its class. Out of the candidates this
        // pass means no verdict either way, so the state holds.
        if (auto it = residuals.find(s.name); it != residuals.end()) {
            const SourceResidual& r = *it->second;
            if (r.refused && !st.refused) {
                ev(EventType::SourceRefused,
                   fmt("%+.0f ms from the other %s sources", r.averagedSec * 1000.0,
                       sourceKindName(r.kind)));
                st.refused = true;
            } else if (!r.refused && r.judged && st.refused) {
                ev(EventType::SourceAccepted,
                   fmt("agrees with the other %s sources again (%+.1f ms)",
                       sourceKindName(r.kind), r.averagedSec * 1000.0));
                st.refused = false;
            }
        }
    }

    for (const std::string& n : c.reacquireNames) {
        const auto why = c.notUsedReasons.find(n);
        SourceKind kind = SourceKind::Radio;
        for (const SourceSnapshot& s : p.sources) if (s.name == n) kind = s.kind;
        emit(EventType::SourceReacquire, p,
             "refused too long; dropping its lock to acquire from nothing" +
                 (why != c.notUsedReasons.end() ? " (" + why->second + ")" : std::string()),
             n, true, kind);
        // It starts over, and so does its verdict.
        m_sources[n].refused = false;
    }

    // --- the classes --------------------------------------------------------
    const SourceKind pk = m_cfg.clock.primary;
    const SourceKind sk = m_cfg.secondaryKind();
    bool secondaryConfigured = false, primaryConfigured = false;
    for (const SourceSnapshot& s : p.sources) {
        if (!s.enabled) continue;
        (s.primaryClass ? primaryConfigured : secondaryConfigured) = true;
    }

    auto classHealth = [&](bool configured, bool active, int have, int need, SourceKind kind,
                           bool& was) {
        if (!configured) return;
        if (!active) { was = false; return; }   // cold and stood down: not news
        const bool now = have >= need && have > 0;
        if (now == was) return;
        emit(now ? EventType::ClassHealthy : EventType::ClassUnhealthy, p,
             fmt("%s sources %s: %d usable, %d required", sourceKindName(kind),
                 now ? "healthy" : "unhealthy", have, need),
             {}, true, kind);
        was = now;
    };
    classHealth(primaryConfigured, true, c.primaryCandidates, m_cfg.ntp.minSources, pk,
                m_primaryHealthy);
    classHealth(secondaryConfigured, p.secondaryActive, c.secondaryCandidates,
                m_cfg.clock.minSecondarySources, sk, m_secondaryHealthy);

    if (m_cfg.clock.secondary == SecondaryMode::Cold && secondaryConfigured &&
        p.secondaryActive != m_secondaryActive && !m_first) {
        emit(p.secondaryActive ? EventType::StandbyUp : EventType::StandbyDown, p,
             p.secondaryActive
                 ? fmt("connecting the %s sources: the %s sources have nothing usable",
                       sourceKindName(sk), sourceKindName(pk))
                 : fmt("disconnecting the %s sources: the %s sources are healthy",
                       sourceKindName(sk), sourceKindName(pk)),
             {}, true, sk);
    }
    m_secondaryActive = p.secondaryActive;

    if (c.serving == ServingClass::Primary || c.serving == ServingClass::Secondary) {
        if (m_side == ServingClass::Primary && c.serving == ServingClass::Secondary) {
            emit(EventType::Failover, p,
                 fmt("serving from the %s sources: the %s sources had nothing usable for %.0f s",
                     sourceKindName(sk), sourceKindName(pk), m_cfg.clock.failoverAfterSec),
                 {}, true, sk);
        } else if (m_side == ServingClass::Secondary && c.serving == ServingClass::Primary) {
            emit(EventType::Failback, p,
                 fmt("serving from the %s sources again: healthy for %.0f s", sourceKindName(pk),
                     m_cfg.clock.failbackAfterSec),
                 {}, true, pk);
        } else if (m_side == ServingClass::None && c.serving == ServingClass::Secondary) {
            // Never served from the primary this run. Not a failover from
            // anything, but worth saying: the primary is not what is serving.
            emit(EventType::Failover, p,
                 fmt("serving from the %s sources: the %s sources have not been usable since "
                     "startup", sourceKindName(sk), sourceKindName(pk)),
                 {}, true, sk);
        }
        m_side = c.serving;
    } else if (c.serving == ServingClass::Both && m_side == ServingClass::None) {
        m_side = ServingClass::Primary;
    }

    // --- the served time ----------------------------------------------------
    const bool live = c.synchronised && c.serving != ServingClass::Coasting;
    if (c.serving == ServingClass::Coasting && m_serving != ServingClass::Coasting &&
        m_serving != ServingClass::None) {
        emit(EventType::Coasting, p,
             fmt("no usable source; running on this machine's clock for up to %s",
                 duration(m_cfg.ntp.coastSeconds).c_str()));
    }
    if (!c.synchronised && m_synchronised) {
        emit(EventType::Unsynchronised, p,
             c.note.empty() ? std::string("unsynchronised") : c.note);
    }
    const bool wasLive = m_synchronised && m_serving != ServingClass::Coasting;
    if (live && !wasLive) {
        std::string names;
        for (const std::string& n : c.usedNames) names += (names.empty() ? "" : ", ") + n;
        emit(EventType::Synchronised, p,
             fmt("%sstratum %d, refid %s, ± %.1f ms from %s",
                 m_everSynchronised ? "" : "first synchronisation: ", c.stratum, c.refid.c_str(),
                 c.dispersionSec * 1000.0, names.empty() ? "no named source" : names.c_str()));
        m_everSynchronised = true;
    } else if (live && m_stratum > 0 && c.stratum != m_stratum) {
        emit(EventType::StratumChanged, p,
             fmt("stratum %d (was %d), refid %s", c.stratum, m_stratum, c.refid.c_str()));
    }
    if (live) m_stratum = c.stratum;

    if (c.leapPending != m_leap) {
        emit(c.leapPending ? EventType::LeapPending : EventType::LeapCleared, p,
             c.leapPending ? "a leap second is announced for the end of this month"
                           : "the leap second warning is no longer served");
        m_leap = c.leapPending;
    }

    m_synchronised = c.synchronised;
    m_serving = c.serving;
    m_first = false;
}

} // namespace ubersdr_ntp
