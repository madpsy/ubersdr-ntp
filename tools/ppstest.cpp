// Offline test of the 1PPS output (Pps.h).
//
//   1. NMEA 0183: checksums against sentences from real receivers and the
//      standard's own examples, then RMC and ZDA as this program writes them --
//      field counts, lengths, CR LF, position formatting, and RMC with no fix.
//   2. Naming the seconds across a leap second: 23:59:60 whether the served
//      clock steps back before, at or after it; no leap without a warning or
//      on a day that cannot have one; a real step of the clock re-counts.
//   3. The configuration: off by default, and every refusal that stops a
//      half-written block from starting, only when it is enabled.
//   4. The thread, against a port that records what it is asked to do and a
//      served clock with an awkward fraction of a second in it: pulses at the
//      served seconds, the width held, the sentences naming each pulse's
//      second, nothing while unsynchronised, and a failing port given up and
//      reported rather than fought.
//   5. A real serial port's open path, on a pseudo-terminal and on a device
//      that is not there.
//
// Takes about fifteen seconds: the thread is tested in real time.
// Exit status 0 when every check passes.

#include "CivilTime.h"
#include "Config.h"
#include "Pps.h"
#include "SampleClock.h"

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <fstream>
#include <mutex>
#include <string>
#include <thread>
#include <unistd.h>
#include <vector>

using namespace ubersdr_ntp;

namespace {

int g_ok = 0, g_failed = 0;

void check(const std::string& name, bool ok, const char* fmt = "", ...) {
    char detail[512] = {0};
    va_list ap;
    va_start(ap, fmt);
    std::vsnprintf(detail, sizeof detail, fmt, ap);
    va_end(ap);
    std::printf("  %-5s %s%s%s\n", ok ? "ok" : "FAIL", name.c_str(), detail[0] ? " -- " : "", detail);
    (ok ? g_ok : g_failed) += 1;
}

// A sentence's fields between '$' and '*', and whether its checksum and
// ending are right.
struct Parsed {
    bool ok = false;
    std::vector<std::string> f;
};
Parsed parse(const std::string& s) {
    Parsed p;
    if (s.size() < 6 || s[0] != '$' || s.substr(s.size() - 2) != "\r\n") return p;
    const std::size_t star = s.find('*');
    if (star == std::string::npos || star + 5 != s.size()) return p;
    const std::string body = s.substr(1, star - 1);
    if (nmeaChecksum(body) != s.substr(star + 1, 2)) return p;
    std::string cur;
    for (char c : body) {
        if (c == ',') { p.f.push_back(cur); cur.clear(); } else cur += c;
    }
    p.f.push_back(cur);
    p.ok = true;
    return p;
}

void testNmea() {
    std::printf("\nNMEA 0183 sentences\n");
    // Published sentences, checksums and all.
    check("checksum of a real receiver's RMC",
          nmeaChecksum("GPRMC,092750.000,A,5321.6802,N,00630.3372,W,0.02,31.66,280511,,,A") == "43");
    check("checksum of the standard's ZDA example",
          nmeaChecksum("GPZDA,201530.00,04,07,2002,00,00") == "60");
    check("RMC with no time and no fix is byte for byte what a u-blox sends",
          nmeaRmc(std::nullopt, false, {}) == "$GPRMC,,V,,,,,,,,,,N*53\r\n", "%s",
          nmeaRmc(std::nullopt, false, {}).c_str());

    // 2026-09-26T03:23:44Z.
    const PpsLabel t{1790393024, false};
    const std::string zda = nmeaZda(t);
    const Parsed z = parse(zda);
    check("ZDA: checksum and CR LF", z.ok, "%s", zda.c_str());
    check("ZDA: seven fields, time/day/month/year/zone", z.ok && z.f.size() == 7 && z.f[0] == "GPZDA" &&
          z.f[1] == "032344.00" && z.f[2] == "26" && z.f[3] == "09" && z.f[4] == "2026" &&
          z.f[5] == "00" && z.f[6] == "00", "%s", zda.c_str());

    const PpsPosition pos{true, 56.036568, -3.351259};
    const std::string rmc = nmeaRmc(t, true, pos);
    const Parsed r = parse(rmc);
    check("RMC: checksum and CR LF", r.ok, "%s", rmc.c_str());
    check("RMC: thirteen fields, NMEA 2.3 with the mode indicator", r.ok && r.f.size() == 13, "%zu", r.f.size());
    check("RMC: time, status A, date", r.ok && r.f[1] == "032344.00" && r.f[2] == "A" && r.f[9] == "260926");
    check("RMC: position as ddmm.mmmm N, dddmm.mmmm W", r.ok && r.f[3] == "5602.1941" && r.f[4] == "N" &&
          r.f[5] == "00321.0755" && r.f[6] == "W", "%s %s %s %s", r.ok ? r.f[3].c_str() : "",
          r.ok ? r.f[4].c_str() : "", r.ok ? r.f[5].c_str() : "", r.ok ? r.f[6].c_str() : "");
    check("RMC: speed 0.0, course and variation empty, mode M", r.ok && r.f[7] == "0.0" && r.f[8].empty() &&
          r.f[10].empty() && r.f[11].empty() && r.f[12] == "M");
    check("RMC: within NMEA's 82 characters", rmc.size() <= 82, "%zu", rmc.size());

    const std::string far = nmeaRmc(t, true, {true, -33.8688, 151.2093});
    const Parsed fr = parse(far);
    check("RMC: south and east", fr.ok && fr.f[3] == "3352.1280" && fr.f[4] == "S" && fr.f[5] == "15112.5580" &&
          fr.f[6] == "E", "%s", far.c_str());
    const Parsed edge = parse(nmeaRmc(t, true, {true, 0.99999999999, 0.0}));
    check("RMC: a minute that rounds to 60 carries into the degree", edge.ok && edge.f[3] == "0100.0000",
          "%s", edge.ok ? edge.f[3].c_str() : "");
    const Parsed nopos = parse(nmeaRmc(t, true, {}));
    check("RMC: no position leaves its fields empty, still valid", nopos.ok && nopos.f[3].empty() &&
          nopos.f[6].empty() && nopos.f[2] == "A");
    const Parsed v = parse(nmeaRmc(t, false, pos));
    check("RMC while unsynchronised: status V, mode N, no speed", v.ok && v.f[2] == "V" && v.f[12] == "N" &&
          v.f[7].empty());

    const std::string leap = nmeaZda({1483228800, true});
    const Parsed l = parse(leap);
    check("ZDA names the inserted second 23:59:60 of the day before", l.ok && l.f[1] == "235960.00" &&
          l.f[2] == "31" && l.f[3] == "12" && l.f[4] == "2016", "%s", leap.c_str());
    const Parsed lr = parse(nmeaRmc(PpsLabel{1483228800, true}, true, {}));
    check("RMC too", lr.ok && lr.f[1] == "235960.00" && lr.f[9] == "311216");
}

void testLabels() {
    std::printf("\nNaming the seconds across a leap second\n");
    const long long midnight = 1483228800;   // 2017-01-01T00:00:00Z, after 2016-12-31T23:59:60Z
    // `step` is how many pulses after 23:59:59 the served clock steps back one
    // second: 1 steps at the leap second itself, 3 some seconds later.
    auto run = [&](int step, bool warned) {
        PpsLabeler lab;
        std::vector<PpsLabel> out;
        long long served = midnight - 3;
        for (int k = 0; k < 8; ++k) {
            out.push_back(lab.next(served, warned && k < 3, k > 0));
            ++served;
            // Pulse k = 2 is 23:59:59; the pulse `step` after it reads one less.
            if (k + 1 == 2 + step) --served;
        }
        return out;
    };
    for (int step : {1, 2, 5}) {
        const auto o = run(step, true);
        const bool ok = o[2].posix == midnight - 1 && !o[2].leap && o[3].posix == midnight && o[3].leap &&
                        o[4].posix == midnight && !o[4].leap && o[5].posix == midnight + 1 &&
                        o[7].posix == midnight + 3;
        check("23:59:59, 23:59:60, 00:00:00 with the served clock stepping back " +
                  std::string(step == 1 ? "at the leap second" : step == 2 ? "one second after" : "four seconds after"),
              ok);
    }
    {
        const auto o = run(99, false);
        check("no warning, no leap second", !o[3].leap && o[3].posix == midnight && o[4].posix == midnight + 1);
    }
    {
        PpsLabeler lab;
        const long long notLast = 1483142400;   // 2016-12-31T00:00:00Z: not after a month's last day
        lab.next(notLast - 2, true, false);
        lab.next(notLast - 1, true, true);
        const PpsLabel p = lab.next(notLast, true, true);
        check("a warning at a midnight that ends no month is not a leap second", !p.leap && p.posix == notLast);
    }
    {
        PpsLabeler lab;
        lab.next(1000, false, false);
        lab.next(1001, false, true);
        const PpsLabel p = lab.next(1011, false, true);
        const PpsLabel q = lab.next(1012, false, true);
        check("a real step of the served clock re-counts from it", p.posix == 1011 && q.posix == 1012 &&
              lab.resyncs() == 1);
        const PpsLabel g = lab.next(1013, false, false);
        check("a gap re-counts without calling it a step", g.posix == 1013 && lab.resyncs() == 1);
    }

    Combined c;
    c.valid = true;
    c.offsetSec = 1.79e9 + 0.4321;
    c.atSec = 5000.0;
    c.rate = 3.7e-6;
    const double d = daemonAtUtc(c, 1.79e9 + 5200.0);
    check("daemonAtUtc inverts utcAt", std::fabs(c.utcAt(d) - (1.79e9 + 5200.0)) < 1e-6, "%.9f",
          c.utcAt(d) - (1.79e9 + 5200.0));
}

std::string g_tmp;
std::string writeTemp(const std::string& text) {
    static int n = 0;
    const std::string p = g_tmp + "/cfg" + std::to_string(n++) + ".json";
    std::ofstream(p) << text;
    return p;
}
bool loadCfg(const std::string& ppsBlock, Config& c, std::string& err) {
    const std::string p = writeTemp(std::string("{\"sources\":[{\"url\":\"http://x:8080\",\"carrier_hz\":10000000}]") +
                                    (ppsBlock.empty() ? "" : ",\"pps\":" + ppsBlock) + "}");
    c = Config{};
    return Config::load(p, c, err) && c.finalise(err);
}

void testConfig() {
    std::printf("\nConfiguration\n");
    Config c;
    std::string err;
    // Loaded first, checked after: the message is only complete once the load
    // has run, and a check's arguments are evaluated in no fixed order.
    auto loads = [&](const std::string& name, const std::string& block, bool want, bool extra = true) {
        err.clear();
        const bool ok = loadCfg(block, c, err);
        check(name, ok == want && (!ok || extra), "%s", err.c_str());
    };
    loads("no pps block: off", "", true);
    check("...and off", !c.pps.enabled);
    check("defaults: DTR, 100 ms, 4800 baud, no NMEA",
          c.pps.line == "dtr" && c.pps.widthMs == 100 && c.pps.baud == 4800 && c.pps.nmea.empty() && !c.pps.invert);
    loads("a disabled half-written block does not stop the daemon", "{\"enabled\":false,\"line\":\"xyz\",\"baud\":1}", true);
    loads("enabled with no device is refused", "{\"enabled\":true}", false);
    loads("an unknown line is refused", "{\"enabled\":true,\"device\":\"/dev/ttyS0\",\"line\":\"cts\"}", false);
    loads("RTS in capitals is RTS", "{\"enabled\":true,\"device\":\"/dev/ttyS0\",\"line\":\"RTS\"}", true);
    check("...and is rts", c.pps.line == "rts");
    loads("a width over 900 ms is refused", "{\"enabled\":true,\"device\":\"/d\",\"width_ms\":950}", false);
    loads("a nonstandard baud is refused", "{\"enabled\":true,\"device\":\"/d\",\"baud\":1200}", false);
    loads("nmea as a list", "{\"enabled\":true,\"device\":\"/d\",\"nmea\":[\"RMC\",\"zda\"]}", true);
    check("...in its order, lower case", c.pps.nmea == std::vector<std::string>{"rmc", "zda"});
    loads("nmea as one string", "{\"enabled\":true,\"device\":\"/d\",\"nmea\":\"zda\"}", true);
    check("...is a list of one", c.pps.nmea == std::vector<std::string>{"zda"});
    loads("an unknown sentence is refused", "{\"enabled\":true,\"device\":\"/d\",\"nmea\":[\"gga\"]}", false);
    loads("a sentence twice is refused", "{\"enabled\":true,\"device\":\"/d\",\"nmea\":[\"zda\",\"zda\"]}", false);
    loads("latitude without longitude is refused", "{\"enabled\":true,\"device\":\"/d\",\"latitude\":56}", false);
    loads("a position out of range is refused",
          "{\"enabled\":true,\"device\":\"/d\",\"latitude\":91,\"longitude\":0}", false);
    loads("a position is taken", "{\"enabled\":true,\"device\":\"/d\",\"latitude\":56.03,\"longitude\":-3.35}", true);
    check("...as given", c.pps.positionGiven && c.pps.latitude == 56.03 && c.pps.longitude == -3.35);
}

// ---- the thread -------------------------------------------------------------

struct FakePort : PpsPort {
    struct Log {
        std::mutex mu;
        std::vector<std::pair<double, bool>> lines;   // daemon time, level
        std::vector<std::pair<double, std::string>> writes;
        std::atomic<bool> failLine{false};
        std::atomic<int> opens{0};
    };
    std::shared_ptr<Log> log;
    explicit FakePort(std::shared_ptr<Log> l) : log(std::move(l)) {}
    bool setLine(bool on, std::string& err) override {
        if (log->failLine) { err = "the device went away"; return false; }
        std::lock_guard<std::mutex> lk(log->mu);
        log->lines.push_back({daemonNow(), on});
        return true;
    }
    bool write(const std::string& data, std::string& err) override {
        (void)err;
        std::lock_guard<std::mutex> lk(log->mu);
        log->writes.push_back({daemonNow(), data});
        return true;
    }
};

void testThread() {
    std::printf("\nThe output thread, in real time\n");
    auto log = std::make_shared<FakePort::Log>();
    std::mutex cmu;
    Combined clock;
    clock.valid = true;
    clock.synchronised = true;
    // A served clock 1.79e9 s and an awkward 0.61803 s ahead of the daemon's.
    clock.offsetSec = 1.79e9 + 0.61803;
    clock.atSec = daemonNow();
    clock.rate = 0.0;
    auto clockFn = [&] { std::lock_guard<std::mutex> lk(cmu); return clock; };
    auto posFn = [](std::string& from) { from = "config"; return PpsPosition{true, 56.0, -3.0}; };

    PpsConfig cfg;
    cfg.enabled = true;
    cfg.device = "/dev/fake";
    cfg.widthMs = 100;
    cfg.nmea = {"rmc", "zda"};
    PpsOutput out(cfg, clockFn, posFn, nullptr, 0, monotonicNow(),
                  [log](const PpsConfig&, std::string&) {
                      ++log->opens;
                      return std::unique_ptr<PpsPort>(new FakePort(log));
                  });
    out.start();
    std::this_thread::sleep_for(std::chrono::milliseconds(4300));

    std::vector<std::pair<double, bool>> lines;
    std::vector<std::pair<double, std::string>> writes;
    {
        std::lock_guard<std::mutex> lk(log->mu);
        lines = log->lines;
        writes = log->writes;
        log->lines.clear();
        log->writes.clear();
    }
    int rises = 0, onTime = 0, widthOk = 0;
    double worst = 0.0;
    for (std::size_t i = 0; i < lines.size(); ++i) {
        if (!lines[i].second) continue;
        ++rises;
        const double u = clock.utcAt(lines[i].first);
        const double late = u - std::floor(u);
        worst = std::max(worst, late);
        if (late < 0.005) ++onTime;
        if (i + 1 < lines.size() && !lines[i + 1].second) {
            const double w = lines[i + 1].first - lines[i].first;
            if (w > 0.099 && w < 0.120) ++widthOk;
        }
    }
    check("four pulses in four seconds", rises == 4 || rises == 5, "%d", rises);
    check("each at a served second, within 5 ms even unprivileged", onTime == rises, "worst %.3f ms", worst * 1e3);
    check("each held for its width", widthOk >= rises - 1, "%d of %d", widthOk, rises);

    int named = 0;
    for (const auto& w : writes) {
        // An RMC then a ZDA, each naming the second of the pulse just before.
        const std::size_t cut = w.second.find("\r\n") + 2;
        const Parsed r = parse(w.second.substr(0, cut)), z = parse(w.second.substr(cut));
        const long long sec = static_cast<long long>(std::floor(clock.utcAt(w.first)));
        const long long rem = floorMod(sec, 86400);
        char hms[64];
        std::snprintf(hms, sizeof hms, "%02lld%02lld%02lld.00", rem / 3600, rem / 60 % 60, rem % 60);
        if (r.ok && z.ok && r.f[0] == "GPRMC" && z.f[0] == "GPZDA" && r.f[1] == hms && z.f[1] == hms &&
            r.f[2] == "A")
            ++named;
    }
    check("RMC and ZDA after each pulse, naming its second", named == rises && !writes.empty(),
          "%d of %d", named, rises);
    const PpsStats s = out.stats();
    check("stats: pulsing, pulses and sentences counted", s.state == "pulsing" && s.pulses == static_cast<unsigned>(rises) &&
          s.sentences == 2u * rises && s.positionFrom == "config", "%s %llu %llu", s.state.c_str(),
          static_cast<unsigned long long>(s.pulses), static_cast<unsigned long long>(s.sentences));

    // Unsynchronised: the line stays at rest, RMC says V, no ZDA.
    { std::lock_guard<std::mutex> lk(cmu); clock.synchronised = false; clock.note = "testing"; }
    std::this_thread::sleep_for(std::chrono::milliseconds(2600));
    {
        std::lock_guard<std::mutex> lk(log->mu);
        int rises2 = 0;
        for (const auto& l : log->lines) if (l.second) ++rises2;
        bool onlyV = !log->writes.empty();
        for (const auto& w : log->writes) {
            const Parsed r = parse(w.second);
            onlyV = onlyV && r.ok && r.f[0] == "GPRMC" && r.f[2] == "V" && w.second.find("ZDA") == std::string::npos;
        }
        check("unsynchronised: no pulse", rises2 == 0, "%d", rises2);
        check("unsynchronised: RMC with status V, and no ZDA", onlyV, "%zu writes", log->writes.size());
        log->lines.clear();
        log->writes.clear();
    }
    check("stats: waiting, with the reason", out.stats().state == "waiting" &&
          out.stats().detail.find("testing") != std::string::npos, "%s", out.stats().detail.c_str());

    // A port that fails is dropped, reported, and opened again later.
    { std::lock_guard<std::mutex> lk(cmu); clock.synchronised = true; }
    log->failLine = true;
    std::this_thread::sleep_for(std::chrono::milliseconds(1600));
    check("a failing port is reported", out.stats().state == "error" &&
          out.stats().detail.find("went away") != std::string::npos, "%s", out.stats().detail.c_str());
    const int opensBefore = log->opens;
    log->failLine = false;
    // Reopened kReopenSec after the failure, and pulsing from the second after.
    std::this_thread::sleep_for(std::chrono::milliseconds(7500));
    check("...and opened again, and pulsing", log->opens > opensBefore && out.stats().state == "pulsing",
          "%d opens, %s", log->opens.load(), out.stats().state.c_str());
    out.stop();
    check("stops", true);
}

void testSerial() {
    std::printf("\nA real port's open path\n");
    PpsConfig cfg;
    cfg.enabled = true;
    cfg.device = "/dev/ttyDOES-NOT-EXIST";
    std::string err;
    bool refused = !openSerialPps(cfg, err);
    check("a missing device says so", refused && err.find("not present") != std::string::npos, "%s", err.c_str());
    cfg.device = "/dev/null";
    err.clear();
    refused = !openSerialPps(cfg, err);
    check("something that is not a serial port says so",
          refused && err.find("not a serial port") != std::string::npos, "%s", err.c_str());

    const int master = ::posix_openpt(O_RDWR | O_NOCTTY);
    if (master < 0 || ::grantpt(master) != 0 || ::unlockpt(master) != 0) {
        check("pseudo-terminal available", false);
        return;
    }
    cfg.device = ::ptsname(master);
    cfg.nmea = {"zda"};
    auto port = openSerialPps(cfg, err);
    // A pseudo-terminal has no modem-control lines on most kernels. Either it
    // refuses, and says why, or it works like a port and carries the NMEA.
    if (!port) {
        check("a pseudo-terminal without a DTR line is refused with the reason",
              err.find("dtr line") != std::string::npos, "%s", err.c_str());
    } else {
        const std::string s = nmeaZda({1790393024, false});
        const bool wrote = port->write(s, err);
        check("writes to the port", wrote, "%s", err.c_str());
        char buf[128] = {0};
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        const ssize_t n = ::read(master, buf, sizeof buf - 1);
        check("the sentence arrives unchanged: raw mode, no CR/LF translation",
              n == static_cast<ssize_t>(s.size()) && std::string(buf, static_cast<std::size_t>(n)) == s,
              "%zd bytes", n);
        std::string err2;
        const bool second = static_cast<bool>(openSerialPps(cfg, err2));
        check("opened exclusively: a second open is refused", !second, "%s", err2.c_str());
    }
    ::close(master);
}

} // namespace

int main() {
    char tmpl[] = "/tmp/ppstest.XXXXXX";
    const char* d = ::mkdtemp(tmpl);
    if (!d) { std::perror("mkdtemp"); return 1; }
    g_tmp = d;
    testNmea();
    testLabels();
    testConfig();
    testSerial();
    testThread();
    std::printf("\n%d ok, %d failed\n", g_ok, g_failed);
    return g_failed ? 1 : 0;
}
