// Offline test of the event monitor, and of the metric history beside it.
//
// The monitor turns passes of the main loop into a list of things that
// happened, and the ways it can go wrong are both quiet: it can miss a
// transition someone needed to see, or it can fill a hundred-entry list with
// noise until the transition that mattered has scrolled off the end. So each
// case here feeds a short scripted sequence of snapshots and combine results
// and checks exactly which events came out -- including that the ones that
// should NOT appear did not.
//
// Exit status 0 when every check passes.

#include "Events.h"
#include "Metrics.h"

#include <cmath>

#include <cstdarg>
#include <cstdio>
#include <string>
#include <vector>

using namespace ubersdr_ntp;

namespace {

int g_ok = 0, g_failed = 0;

void check(const std::string& name, bool ok, const char* fmt = "", ...) {
    char detail[1024] = {0};
    va_list ap;
    va_start(ap, fmt);
    std::vsnprintf(detail, sizeof detail, fmt, ap);
    va_end(ap);
    std::printf("  %-5s %s%s%s\n", ok ? "ok" : "FAIL", name.c_str(), detail[0] ? " -- " : "", detail);
    (ok ? g_ok : g_failed) += 1;
}

Config makeConfig(SecondaryMode mode = SecondaryMode::Standby, int minRadio = 1) {
    Config c;
    c.clock.primary = SourceKind::Radio;
    c.clock.secondary = mode;
    c.clock.minRadioSources = minRadio;
    c.clock.minNtpSources = 1;
    c.ntp.minSources = minRadio;
    c.clock.minSecondarySources = 1;
    return c;
}

SourceSnapshot radio(const std::string& name) {
    SourceSnapshot s;
    s.name = name;
    s.kind = SourceKind::Radio;
    s.primaryClass = true;
    s.url = "http://r.example";
    s.carrierHz = 10000000;
    return s;
}

SourceSnapshot peer(const std::string& name) {
    SourceSnapshot s;
    s.name = name;
    s.kind = SourceKind::Ntp;
    s.primaryClass = false;
    return s;
}

// A harness around one monitor: step() runs a pass and returns the names of
// the events it produced, in order.
struct Harness {
    Config cfg;
    EventLog log;
    EventMonitor mon;
    std::uint64_t seen = 0;
    double t = 1000.0;

    explicit Harness(Config c) : cfg(std::move(c)), mon(cfg, log) {}

    std::vector<std::string> step(const std::vector<SourceSnapshot>& snaps, const Combined& c,
                                  bool secondaryActive = true) {
        t += 1.0;
        mon.observe(EventMonitor::Pass{snaps, c, secondaryActive, t, t - 1000.0});
        std::vector<std::string> out;
        const auto all = log.all();
        for (auto it = all.rbegin(); it != all.rend(); ++it) {
            if (it->id > seen) out.push_back(std::string(eventTypeInfo(it->type).name) +
                                             (it->source.empty() ? "" : ":" + it->source));
        }
        seen = log.latestId();
        return out;
    }
};

std::string join(const std::vector<std::string>& v) {
    std::string s;
    for (const auto& x : v) s += (s.empty() ? "" : ", ") + x;
    return s.empty() ? "(none)" : s;
}

bool has(const std::vector<std::string>& v, const std::string& x) {
    for (const auto& y : v) if (y == x) return true;
    return false;
}

Combined serving(ServingClass cls, int primaryCand, int secondaryCand) {
    Combined c;
    c.valid = cls != ServingClass::None;
    c.synchronised = cls != ServingClass::None;
    c.serving = cls;
    c.primaryCandidates = primaryCand;
    c.secondaryCandidates = secondaryCand;
    c.stratum = cls == ServingClass::Secondary ? 3 : 1;
    return c;
}

// ---------------------------------------------------------------------------

void testQuietStartup() {
    std::printf("\nStartup is quiet until something happens\n");
    Harness h(makeConfig());
    auto a = radio("r1"), b = peer("p1");
    a.link = LinkState::Connecting;
    const auto ev = h.step({a, b}, Combined{});
    check("a first pass with nothing ready records nothing", ev.empty(), "%s", join(ev).c_str());
    const auto ev2 = h.step({a, b}, Combined{});
    check("...and nor does a second identical one", ev2.empty(), "%s", join(ev2).c_str());

    Harness fast(makeConfig());
    auto quick = radio("quick");
    quick.link = LinkState::Streaming;
    const auto ev3 = fast.step({quick}, Combined{});
    check("a receiver already streaming on the first pass still gets link_up",
          has(ev3, "link_up:quick"), "%s", join(ev3).c_str());
}

void testLockAndFailover() {
    std::printf("\nA source locks, the class becomes healthy, and fails over and back\n");
    Harness h(makeConfig());
    auto r = radio("r1");
    auto p = peer("p1");
    r.link = LinkState::Connecting;
    h.step({r, p}, Combined{});

    r.link = LinkState::Streaming;
    auto ev = h.step({r, p}, Combined{});
    check("streaming produces link_up", has(ev, "link_up:r1"), "%s", join(ev).c_str());

    r.ready = true;
    r.station = "wwv";
    p.ready = true;
    p.ntp.stratum = 2;
    p.ntp.refid = "GPS";
    ev = h.step({r, p}, serving(ServingClass::Primary, 1, 1));
    check("lock produces source_ready for both", has(ev, "source_ready:r1") &&
          has(ev, "source_ready:p1"), "%s", join(ev).c_str());
    check("...both classes healthy", has(ev, "class_healthy") && ev.size() >= 5, "%s",
          join(ev).c_str());
    check("...the station is identified", has(ev, "station_changed:r1"), "%s", join(ev).c_str());
    check("...and the first synchronisation", has(ev, "synchronised"), "%s", join(ev).c_str());

    ev = h.step({r, p}, serving(ServingClass::Primary, 1, 1));
    check("an unchanged pass records nothing", ev.empty(), "%s", join(ev).c_str());

    r.ready = false;
    r.notReadyReason = "decoder not locked";
    ev = h.step({r, p}, serving(ServingClass::Primary, 0, 1));
    check("losing lock produces source_lost and class_unhealthy",
          has(ev, "source_lost:r1") && has(ev, "class_unhealthy"), "%s", join(ev).c_str());

    ev = h.step({r, p}, serving(ServingClass::Secondary, 0, 1));
    check("serving from the secondary is a failover", has(ev, "failover"), "%s", join(ev).c_str());
    check("...and the stratum change is recorded", has(ev, "stratum_changed"), "%s",
          join(ev).c_str());

    r.ready = true;
    ev = h.step({r, p}, serving(ServingClass::Secondary, 1, 1));
    check("recovering records ready and healthy but not yet a failback",
          has(ev, "source_ready:r1") && has(ev, "class_healthy") && !has(ev, "failback"), "%s",
          join(ev).c_str());
    ev = h.step({r, p}, serving(ServingClass::Primary, 1, 1));
    check("serving from the primary again is a failback", has(ev, "failback"), "%s",
          join(ev).c_str());
}

void testMinimum() {
    std::printf("\nThe class minimum decides healthy, not any one source\n");
    Harness h(makeConfig(SecondaryMode::Standby, 2));
    auto a = radio("a"), b = radio("b");
    a.ready = true;
    auto ev = h.step({a, b}, serving(ServingClass::Coasting, 1, 0));
    check("one of two required is not healthy", !has(ev, "class_healthy"), "%s", join(ev).c_str());
    b.ready = true;
    ev = h.step({a, b}, serving(ServingClass::Primary, 2, 0));
    check("two of two is", has(ev, "class_healthy"), "%s", join(ev).c_str());
}

void testCoastAndUnsync() {
    std::printf("\nCoasting, then unsynchronised, then back\n");
    Harness h(makeConfig());
    auto r = radio("r1");
    r.ready = true;
    h.step({r}, serving(ServingClass::Primary, 1, 0));
    r.ready = false;
    auto ev = h.step({r}, serving(ServingClass::Coasting, 0, 0));
    check("losing every source starts coasting", has(ev, "coasting"), "%s", join(ev).c_str());
    check("...and is not yet unsynchronised", !has(ev, "unsynchronised"), "%s", join(ev).c_str());
    ev = h.step({r}, serving(ServingClass::None, 0, 0));
    check("past the coast limit is unsynchronised", has(ev, "unsynchronised"), "%s",
          join(ev).c_str());
    r.ready = true;
    ev = h.step({r}, serving(ServingClass::Primary, 1, 0));
    check("coming back is synchronised again", has(ev, "synchronised"), "%s", join(ev).c_str());
    check("...and not a failback, which it never left", !has(ev, "failback"), "%s",
          join(ev).c_str());
}

void testLinkFailuresAreNotRepeated() {
    std::printf("\nA receiver that cannot be reached is said once, not every retry\n");
    Harness h(makeConfig());
    auto r = radio("r1");
    r.link = LinkState::Connecting;
    h.step({r}, Combined{});
    int downs = 0;
    for (int i = 0; i < 10; ++i) {
        r.link = i % 2 ? LinkState::Connecting : LinkState::Backoff;
        for (const auto& e : h.step({r}, Combined{})) downs += e == "link_down:r1";
    }
    check("ten retries produce one link_down", downs == 1, "%d", downs);
    r.link = LinkState::Streaming;
    h.step({r}, Combined{});
    r.link = LinkState::Connecting;
    const auto ev = h.step({r}, Combined{});
    check("a stream that then drops is reported again", has(ev, "link_down:r1"), "%s",
          join(ev).c_str());
}

void testRefusal() {
    std::printf("\nRefused and accepted follow the consensus verdict\n");
    Harness h(makeConfig());
    auto r = radio("r1");
    r.ready = true;
    Combined c = serving(ServingClass::Primary, 3, 0);
    SourceResidual res;
    res.name = "r1";
    res.refused = true;
    res.judged = true;
    res.averagedSec = 0.1;
    c.residuals = {res};
    auto ev = h.step({r}, c);
    check("a refusal is recorded", has(ev, "source_refused:r1"), "%s", join(ev).c_str());
    ev = h.step({r}, c);
    check("...once", !has(ev, "source_refused:r1"), "%s", join(ev).c_str());

    c.residuals[0].refused = false;
    c.residuals[0].judged = false;
    ev = h.step({r}, c);
    check("not refused but not judged is not acceptance", !has(ev, "source_accepted:r1"), "%s",
          join(ev).c_str());
    c.residuals[0].judged = true;
    ev = h.step({r}, c);
    check("judged and not refused is", has(ev, "source_accepted:r1"), "%s", join(ev).c_str());
}

void testColdStandby() {
    std::printf("\nCold standby: up and down, and its sources' silence while down\n");
    Harness h(makeConfig(SecondaryMode::Cold));
    auto r = radio("r1");
    auto p = peer("p1");
    r.ready = true;
    p.active = false;
    h.step({r, p}, serving(ServingClass::Primary, 1, 0), false);

    r.ready = false;
    p.active = true;
    auto ev = h.step({r, p}, serving(ServingClass::Coasting, 0, 0), true);
    check("the standby coming up is recorded", has(ev, "standby_up"), "%s", join(ev).c_str());

    p.ready = true;
    ev = h.step({r, p}, serving(ServingClass::Coasting, 0, 1), true);
    check("...and its class becoming healthy", has(ev, "class_healthy"), "%s", join(ev).c_str());

    r.ready = true;
    p.ready = false;
    p.active = false;
    p.notReadyReason = "held in cold standby";
    ev = h.step({r, p}, serving(ServingClass::Primary, 1, 0), false);
    check("standing down is recorded", has(ev, "standby_down"), "%s", join(ev).c_str());
    check("...without a source_lost for the peer it disconnected", !has(ev, "source_lost:p1"),
          "%s", join(ev).c_str());
    check("...or a class_unhealthy for the class it stood down", !has(ev, "class_unhealthy"),
          "%s", join(ev).c_str());
}

void testCapacity() {
    std::printf("\nThe log keeps the newest hundred\n");
    EventLog log;
    for (int i = 0; i < 250; ++i) {
        Event e;
        e.message = std::to_string(i);
        log.add(e);
    }
    const auto all = log.all();
    check("holds exactly its capacity", all.size() == EventLog::kCapacity, "%zu", all.size());
    check("newest first, ids continuing", !all.empty() && all.front().id == 250 &&
          all.back().id == 151, "%llu..%llu",
          all.empty() ? 0ULL : static_cast<unsigned long long>(all.front().id),
          all.empty() ? 0ULL : static_cast<unsigned long long>(all.back().id));
    check("every type has a distinct name", [] {
        std::vector<std::string> names;
        for (const auto& t : eventTypes()) {
            for (const auto& n : names) if (n == t.name) return false;
            names.push_back(t.name);
        }
        return true;
    }());
}

void testMetricHistory() {
    std::printf("\nMetric history is bucketed, windowed and bounded\n");
    const double t0 = 994222.0 * 1800.0;   // on a half-hour boundary
    MetricHistory h;
    const MetricSeriesInfo info{"served.offset_ms", "Clock offset", "ms", "served", ""};
    // Three hours at one sample a second, value = minute number.
    for (int sec = 0; sec < 3 * 3600; ++sec) {
        h.add(info, t0 + sec, std::floor(sec / 60.0));
    }
    const double now = t0 + 3 * 3600 - 1;
    const auto hour = h.snapshot(MetricHistory::Range::Hour, now);
    check("one series", hour.size() == 1, "%zu", hour.size());
    const auto& hp = hour.front().points;
    check("an hour of minute buckets, the filling one included", hp.size() == 60, "%zu", hp.size());
    check("each a minute wide with sixty samples", hp.size() > 1 && hp[1].start - hp[0].start == 60.0 &&
          hp[0].count == 60, "width %.0f count %u", hp.size() > 1 ? hp[1].start - hp[0].start : 0.0,
          hp.empty() ? 0 : hp[0].count);
    check("the newest is the last minute", !hp.empty() && hp.back().mean() == 179.0, "%.1f",
          hp.empty() ? 0.0 : hp.back().mean());

    const auto day = h.snapshot(MetricHistory::Range::Day, now);
    const auto& dp = day.front().points;
    check("three hours make six half-hour buckets", dp.size() == 6, "%zu", dp.size());
    check("...averaging their thirty minutes", dp.size() == 6 && dp[0].mean() == 14.5 &&
          dp[0].min == 0.0 && dp[0].max == 29.0, "mean %.1f min %.0f max %.0f",
          dp.empty() ? 0.0 : dp[0].mean(), dp.empty() ? 0.0 : dp[0].min, dp.empty() ? 0.0 : dp[0].max);

    const auto later = h.snapshot(MetricHistory::Range::Hour, now + 1800);
    check("a series that stops ages out of the window", later.front().points.size() == 30, "%zu",
          later.front().points.size());

    MetricHistory steps;
    steps.add(info, t0 + 120, 1.0);
    steps.add(info, t0 + 30, 3.0);   // the clock stepped back two minutes
    const auto sp = steps.snapshot(MetricHistory::Range::Hour, t0 + 120).front().points;
    check("a backwards step folds into the open bucket", sp.size() == 1 && sp[0].count == 2 &&
          sp[0].mean() == 2.0, "%zu buckets", sp.size());

    steps.add(info, t0 + 200, std::nan(""));
    check("a non-finite value is not recorded",
          steps.snapshot(MetricHistory::Range::Hour, t0 + 200).front().points.back().count == 2);

    MetricHistory many;
    for (int i = 0; i < 1000; ++i) {
        many.add({"s" + std::to_string(i), "", "", "", ""}, t0, 1.0);
    }
    check("series are capped", many.seriesCount() == MetricHistory::kMaxSeries, "%zu",
          many.seriesCount());

    // Days of samples leave the stored buckets at capacity, not growing.
    MetricHistory longRun;
    for (int sec = 0; sec < 5 * 86400; sec += 10) longRun.add(info, t0 + sec, 1.0);
    const double end = t0 + 5 * 86400;
    check("five days hold at most an hour of minutes",
          longRun.snapshot(MetricHistory::Range::Hour, end).front().points.size() <= 61);
    check("...and a day of half hours",
          longRun.snapshot(MetricHistory::Range::Day, end).front().points.size() <= 49);

    // The serving timeline: only changes are kept, and the one in effect at
    // the start of the window comes back with them.
    MetricHistory tl;
    tl.setServing(t0, "primary");
    for (int sec = 1; sec < 7200; ++sec) tl.setServing(t0 + sec, "primary");
    tl.setServing(t0 + 7200, "secondary");
    tl.setServing(t0 + 7500, "primary");
    auto spans = tl.serving(MetricHistory::Range::Hour, t0 + 7800);
    check("an unchanged state is stored once", spans.size() == 3, "%zu spans", spans.size());
    check("...the first says what held at the window's start",
          spans.size() == 3 && spans[0].state == "primary" && spans[0].start == t0 &&
              spans[1].state == "secondary", "%s from %.0f",
          spans.empty() ? "" : spans[0].state.c_str(), spans.empty() ? 0.0 : spans[0].start - t0);
    MetricHistory flap;
    for (int i = 0; i < 20000; ++i) flap.setServing(t0 + i * 0.25, i % 2 ? "primary" : "coasting");
    const auto fs = flap.serving(MetricHistory::Range::Day, t0 + 5000);
    check("a flapping state is capped", fs.size() <= StateTimeline::kMaxChanges, "%zu", fs.size());
}

} // namespace

void testTimeRefusal() {
    std::printf("\nA run of refused decoded times is one event, and taking one is another\n");
    Harness h(makeConfig());
    auto r = radio("r1");
    r.ready = true;
    const Combined c = serving(ServingClass::Primary, 1, 0);
    h.step({r}, c);
    r.timeCheck = "refused decoded 2026-09-17T14:00:00Z: -4 h 0 min from its own history";
    r.timeRejections = 1;
    r.lastRejectedJumpSec = -14400.0;
    r.lastRejectedUtc = "2026-09-17T14:00:00Z";
    auto ev = h.step({r}, c);
    check("a refused decoded time is recorded", has(ev, "time_refused:r1"), "%s", join(ev).c_str());
    int more = 0;
    for (int i = 0; i < 5; ++i) {
        r.timeRejections += 1;
        for (const auto& e : h.step({r}, c)) more += e == "time_refused:r1";
    }
    check("...once for the run, not once a reading", more == 0, "%d more", more);
    r.timeCheck.clear();
    h.step({r}, c);
    r.timeCheck = "refused decoded 2026-09-17T07:04:00Z: -4 min from its own history";
    r.timeRejections += 1;
    ev = h.step({r}, c);
    check("a later run is recorded again", has(ev, "time_refused:r1"), "%s", join(ev).c_str());
    r.timeCheck.clear();
    r.timeAdoptions = 1;
    ev = h.step({r}, c);
    check("a time taken after holding is recorded", has(ev, "time_adopted:r1"), "%s", join(ev).c_str());
    ev = h.step({r}, c);
    check("...once", !has(ev, "time_adopted:r1"), "%s", join(ev).c_str());
}

void testSpikeHold() {
    std::printf("\nAn offset held through a jitter spike is one event, and its release another\n");
    Harness h(makeConfig());
    auto r = radio("r1");
    r.ready = true;
    const Combined c = serving(ServingClass::Primary, 1, 0);
    h.step({r}, c);
    r.offsetHeld = true;
    r.jitterSec = 312e-6;
    r.jitterBaselineSec = 15e-6;
    auto ev = h.step({r}, c);
    check("a hold is recorded", has(ev, "offset_held:r1"), "%s", join(ev).c_str());
    int more = 0;
    for (int i = 0; i < 5; ++i) for (const auto& e : h.step({r}, c)) more += e == "offset_held:r1";
    check("...once, not once a pass", more == 0, "%d more", more);
    r.offsetHeld = false;
    r.rejoinGapSec = -130e-6;
    ev = h.step({r}, c);
    check("its release is recorded", has(ev, "offset_released:r1"), "%s", join(ev).c_str());
    ev = h.step({r}, c);
    check("...once", !has(ev, "offset_released:r1"), "%s", join(ev).c_str());
}

int main() {
    std::printf("ubersdr-ntp event monitor test\n");
    testQuietStartup();
    testSpikeHold();
    testLockAndFailover();
    testMinimum();
    testCoastAndUnsync();
    testLinkFailuresAreNotRepeated();
    testRefusal();
    testTimeRefusal();
    testColdStandby();
    testCapacity();
    testMetricHistory();
    std::printf("\n%d ok, %d failed\n", g_ok, g_failed);
    return g_failed == 0 ? 0 : 1;
}
