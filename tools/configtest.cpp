// Offline test of the configuration reader.
//
// Configuration is the one part of this program whose faults are silent. A
// decoder that will not lock says so in the log every thirty seconds; a
// password that was read into the wrong source, or a poll interval that was
// parsed as a string and ignored, produces a daemon that runs and is wrong.
// So the cases here are the ones where a mistake would not announce itself:
//
//   1. Per-source settings really are per source, and inheritance from
//      "defaults" overrides in the right direction. Above all the PASSWORDS:
//      several receivers with different bypass passwords is an ordinary
//      arrangement, and a reader that leaked one source's password into
//      another would work perfectly for whichever receiver happened to match.
//   2. Upstream NTP servers parse in every spelling that is offered -- a bare
//      hostname, an object, host:port, [v6]:port -- and land on the right port.
//   3. The clock block, and the validation that stops a configuration which
//      cannot work from starting rather than from failing later.
//
// Writes its cases to temporary files and reads them back through the real
// Config::load, because the thing being tested is the reader and a test that
// built the structs itself would test nothing.
//
// Exit status 0 when every check passes.

#include "Config.h"

#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <string>
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

std::string g_tmpDir;

// Writes `text` to a temporary file and returns the path. The daemon reads its
// configuration from a path, so the test hands it one.
std::string writeTemp(const std::string& text) {
    static int n = 0;
    const std::string path = g_tmpDir + "/cfg" + std::to_string(++n) + ".json";
    std::ofstream out(path);
    out << text;
    out.close();
    return path;
}

// Loads and finalises, and returns BOTH results together.
//
// Together deliberately. The obvious shape -- `bool load(text, cfg, err)`
// called inside the condition of a check that also prints err -- is undefined
// behaviour: the order in which a call's arguments are evaluated is
// unspecified, so `err.c_str()` may be read before the load has written it,
// and the pointer it returns dangles the moment the load reassigns the string.
// It passed, and printed garbage, which is exactly the failure mode a test is
// supposed to not have.
struct Loaded {
    bool ok = false;
    std::string err;
    Config cfg;

    bool failedWith(const std::string& needle) const {
        return !ok && err.find(needle) != std::string::npos;
    }
    const char* why() const { return err.c_str(); }
};

Loaded load(const std::string& text) {
    Loaded r;
    const std::string path = writeTemp(text);
    if (!Config::load(path, r.cfg, r.err)) return r;
    r.ok = r.cfg.finalise(r.err);
    return r;
}

const SourceConfig* sourceNamed(const Config& c, const std::string& name) {
    for (const SourceConfig& s : c.sources) if (s.name == name) return &s;
    return nullptr;
}

const NtpSourceConfig* peerNamed(const Config& c, const std::string& name) {
    for (const NtpSourceConfig& s : c.ntpSources) if (s.name == name) return &s;
    return nullptr;
}

// ---------------------------------------------------------------------------

void testPerSourcePasswords() {
    std::printf("\nEvery receiver has its own password\n");

    // Three receivers: one with its own password, one inheriting the default,
    // one explicitly overriding the default with an empty string -- which is a
    // real case (a public receiver among private ones) and the one most likely
    // to be got wrong by a reader that treats empty as "not set".
    const Loaded r = load(R"({
      "defaults": { "password": "shared-default", "verify_tls": false },
      "sources": [
        { "name": "a", "url": "http://a.example", "carrier_hz": 10000000,
          "password": "alpha-secret" },
        { "name": "b", "url": "http://b.example", "carrier_hz": 15000000 },
        { "name": "c", "url": "http://c.example", "carrier_hz": 5000000,
          "password": "" }
      ]
    })");
    check("the configuration loads", r.ok, "%s", r.why());
    if (!r.ok) return;

    const SourceConfig* a = sourceNamed(r.cfg, "a");
    const SourceConfig* b = sourceNamed(r.cfg, "b");
    const SourceConfig* d = sourceNamed(r.cfg, "c");
    check("all three sources are present", a && b && d);
    if (!a || !b || !d) return;

    check("a keeps its own password", a->password == "alpha-secret", "\"%s\"", a->password.c_str());
    check("b inherits the default", b->password == "shared-default", "\"%s\"", b->password.c_str());
    check("c overrides the default with none", d->password.empty(), "\"%s\"", d->password.c_str());
    // The failure that would matter: one source's secret reaching another.
    check("no password leaked between sources",
          a->password != b->password && a->password != d->password,
          "a=\"%s\" b=\"%s\" c=\"%s\"",
          a->password.c_str(), b->password.c_str(), d->password.c_str());
    check("other per-source settings inherit too", !a->verifyTls && !b->verifyTls);
    check("...and the URLs did not cross over",
          a->url == "http://a.example" && b->url == "http://b.example");

    // With no defaults block at all, a source without a password has none --
    // rather than picking up the previous source's.
    const Loaded r2 = load(R"({
      "sources": [
        { "name": "a", "url": "http://a.example", "carrier_hz": 10000000, "password": "one" },
        { "name": "b", "url": "http://b.example", "carrier_hz": 15000000 }
      ]
    })");
    const SourceConfig* b2 = r2.ok ? sourceNamed(r2.cfg, "b") : nullptr;
    check("with no defaults, an unset password stays empty",
          r2.ok && b2 && b2->password.empty(), "%s", r2.why());
}

void testNtpSources() {
    std::printf("\nUpstream NTP servers, in every spelling offered\n");

    const Loaded r = load(R"({
      "sources": [ { "name": "r", "url": "http://r.example", "carrier_hz": 10000000 } ],
      "ntp_defaults": { "poll_seconds": 128, "max_stratum": 4 },
      "ntp_sources": [
        "time.example.org",
        "192.0.2.10:1123",
        "[2001:db8::1]:2123",
        { "name": "tuned", "server": "ntp.example.net", "poll_seconds": 16,
          "weight": 3.0, "iburst": false, "max_root_distance_ms": 50,
          "extra_delay_ms": -2.5 }
      ]
    })");
    check("the configuration loads", r.ok, "%s", r.why());
    if (!r.ok) return;

    check("four peers", r.cfg.ntpSources.size() == 4, "%zu", r.cfg.ntpSources.size());

    const NtpSourceConfig* bare = peerNamed(r.cfg, "time.example.org");
    check("a bare hostname becomes a peer named after itself", bare != nullptr);
    if (bare) {
        check("...on the default port", bare->port == 123, "%d", bare->port);
        check("...inheriting ntp_defaults", bare->pollSeconds == 128.0 && bare->maxStratum == 4,
              "poll %.0f, max stratum %d", bare->pollSeconds, bare->maxStratum);
    }

    const NtpSourceConfig* v4 = peerNamed(r.cfg, "192.0.2.10");
    check("host:port splits", v4 != nullptr && v4->port == 1123,
          "%s", v4 ? (v4->server + ":" + std::to_string(v4->port)).c_str() : "missing");

    // The case a naive split gets wrong: an IPv6 literal is full of colons.
    const NtpSourceConfig* v6 = peerNamed(r.cfg, "2001:db8::1");
    check("[v6]:port splits without cutting the address in half",
          v6 != nullptr && v6->port == 2123 && v6->server == "2001:db8::1",
          "%s", v6 ? (v6->server + " port " + std::to_string(v6->port)).c_str() : "missing");

    const NtpSourceConfig* t = peerNamed(r.cfg, "tuned");
    check("an object form overrides every default",
          t && t->pollSeconds == 16.0 && t->weight == 3.0 && !t->iburst &&
              t->maxRootDistanceMs == 50.0 && t->extraDelayMs == -2.5);

    // A v6 literal with no port must not be mistaken for host:port.
    const Loaded r2 = load(R"({
      "ntp_sources": [ "2001:db8::99" ],
      "sources": [ { "url": "http://r.example", "carrier_hz": 10000000 } ]
    })");
    check("a bare IPv6 literal keeps its default port",
          r2.ok && r2.cfg.ntpSources.size() == 1 &&
              r2.cfg.ntpSources[0].server == "2001:db8::99" && r2.cfg.ntpSources[0].port == 123,
          "%s", r2.ok ? r2.cfg.ntpSources[0].server.c_str() : r2.why());
}

void testClockBlock() {
    std::printf("\nThe clock block: which class is primary, and what the other does\n");

    const Loaded def = load(R"({"sources":[{"url":"http://r.example","carrier_hz":10000000}]})");
    check("defaults are radio-primary, secondary on standby",
          def.ok && def.cfg.clock.primary == SourceKind::Radio &&
              def.cfg.clock.secondary == SecondaryMode::Standby,
          "%s", def.why());

    const Loaded r = load(R"({
      "clock": { "primary": "ntp", "secondary": "cold",
                 "failover_after_seconds": 30, "failback_after_seconds": 600,
                 "min_secondary_sources": 2 },
      "sources": [ { "name": "r1", "url": "http://a.example", "carrier_hz": 10000000 },
                   { "name": "r2", "url": "http://b.example", "carrier_hz": 15000000 } ],
      "ntp_sources": [ "time.example.org" ]
    })");
    check("an explicit clock block loads", r.ok, "%s", r.why());
    if (r.ok) {
        const ClockConfig& k = r.cfg.clock;
        check("primary is ntp", k.primary == SourceKind::Ntp);
        check("secondary is cold", k.secondary == SecondaryMode::Cold);
        check("the hold-downs are read",
              k.failoverAfterSec == 30.0 && k.failbackAfterSec == 600.0);
        // The whole point of the two figures being separate: with ntp primary,
        // min_secondary_sources governs the radio class.
        check("min_secondary_sources is read", k.minSecondarySources == 2);
        check("secondaryKind() is the other one", r.cfg.secondaryKind() == SourceKind::Radio);
        check("isPrimary() agrees with it",
              r.cfg.isPrimary(SourceKind::Ntp) && !r.cfg.isPrimary(SourceKind::Radio));
    }

    const Loaded bad1 = load(R"({"clock":{"primary":"gps"},
                                 "sources":[{"url":"http://r.example","carrier_hz":10000000}]})");
    check("an unknown primary is refused, not defaulted", bad1.failedWith("gps"),
          "%s", bad1.why());

    const Loaded bad2 = load(R"({"clock":{"secondary":"sometimes"},
                                 "sources":[{"url":"http://r.example","carrier_hz":10000000}]})");
    check("an unknown secondary mode is refused", bad2.failedWith("sometimes"),
          "%s", bad2.why());

    // The spellings people will actually type.
    const Loaded alt = load(R"({"clock":{"secondary":"fallback"},
                                "sources":[{"url":"http://r.example","carrier_hz":10000000}]})");
    check("\"fallback\" is accepted as standby",
          alt.ok && alt.cfg.clock.secondary == SecondaryMode::Standby, "%s", alt.why());
}

void testValidation() {
    std::printf("\nConfigurations that cannot work are refused before they run\n");

    const Loaded none = load(R"({"ntp":{"port":123}})");
    check("no sources of either kind is refused", none.failedWith("no sources configured"),
          "%s", none.why());

    // One namespace across both classes: the Selector, the log and the status
    // page all key on the name, and a collision would merge two sources'
    // agreement history.
    const Loaded clash = load(R"({
      "sources": [ { "name": "clash", "url": "http://r.example", "carrier_hz": 10000000 } ],
      "ntp_sources": [ { "name": "clash", "server": "time.example.org" } ]
    })");
    check("a name shared by a radio source and a peer is refused",
          clash.failedWith("duplicate source name"), "%s", clash.why());

    const Loaded tooFew = load(R"({
      "ntp": { "min_sources": 3 },
      "sources": [ { "url": "http://r.example", "carrier_hz": 10000000 } ],
      "ntp_sources": [ "a.example", "b.example", "c.example" ]
    })");
    check("min_sources larger than the primary class is refused",
          tooFew.failedWith("min_sources"), "%s", tooFew.why());

    // ...and the same figure is fine when the classes are the other way round,
    // which is the case that proves min_sources governs the primary and not
    // simply the total.
    const Loaded swapped = load(R"({
      "clock": { "primary": "ntp" },
      "ntp": { "min_sources": 3 },
      "sources": [ { "url": "http://r.example", "carrier_hz": 10000000 } ],
      "ntp_sources": [ "a.example", "b.example", "c.example" ]
    })");
    check("...and allowed when that class is the primary", swapped.ok, "%s", swapped.why());

    // The kind-keyed spelling: min_radio_sources counts receivers whichever
    // class is primary, and lands in the role-keyed figure the Selector reads.
    const Loaded twoRadios = load(R"({
      "clock": { "min_radio_sources": 2 },
      "sources": [ { "url": "http://r.example", "carrier_hz": 10000000 },
                   { "url": "http://r.example", "carrier_hz": 15000000 } ],
      "ntp_sources": [ "a.example" ]
    })");
    check("min_radio_sources with radio primary sets the primary minimum",
          twoRadios.ok && twoRadios.cfg.ntp.minSources == 2 &&
              twoRadios.cfg.clock.minSecondarySources == 1,
          "%s primary %d secondary %d", twoRadios.why(), twoRadios.cfg.ntp.minSources,
          twoRadios.cfg.clock.minSecondarySources);

    const Loaded radioSecondary = load(R"({
      "clock": { "primary": "ntp", "min_radio_sources": 2 },
      "sources": [ { "url": "http://r.example", "carrier_hz": 10000000 },
                   { "url": "http://r.example", "carrier_hz": 15000000 } ],
      "ntp_sources": [ "a.example" ]
    })");
    check("...and follows the radio when it is the secondary",
          radioSecondary.ok && radioSecondary.cfg.ntp.minSources == 1 &&
              radioSecondary.cfg.clock.minSecondarySources == 2,
          "%s primary %d secondary %d", radioSecondary.why(),
          radioSecondary.cfg.ntp.minSources, radioSecondary.cfg.clock.minSecondarySources);

    const Loaded legacy = load(R"({
      "clock": { "primary": "ntp", "min_secondary_sources": 2 },
      "sources": [ { "url": "http://r.example", "carrier_hz": 10000000 },
                   { "url": "http://r.example", "carrier_hz": 15000000 } ],
      "ntp_sources": [ "a.example" ]
    })");
    check("the role-keyed spelling still resolves to the right kind",
          legacy.ok && legacy.cfg.clock.minRadioSources == 2 && legacy.cfg.clock.minNtpSources == 1,
          "%s radio %d ntp %d", legacy.why(), legacy.cfg.clock.minRadioSources,
          legacy.cfg.clock.minNtpSources);

    const Loaded both = load(R"({
      "ntp": { "min_sources": 1 },
      "clock": { "min_radio_sources": 2 },
      "sources": [ { "url": "http://r.example", "carrier_hz": 10000000 },
                   { "url": "http://r.example", "carrier_hz": 15000000 } ]
    })");
    check("one kind's minimum given two different ways is refused",
          both.failedWith("min_radio_sources"), "%s", both.why());

    const Loaded tooFewRadios = load(R"({
      "clock": { "min_radio_sources": 3 },
      "sources": [ { "url": "http://r.example", "carrier_hz": 10000000 },
                   { "url": "http://r.example", "carrier_hz": 15000000 } ]
    })");
    check("min_radio_sources larger than the radio sources enabled is refused",
          tooFewRadios.failedWith("clock.min_radio_sources is 3"), "%s", tooFewRadios.why());

    const Loaded rude = load(R"({
      "ntp_sources": [ { "server": "a.example", "poll_seconds": 1 } ],
      "sources": [ { "url": "http://r.example", "carrier_hz": 10000000 } ]
    })");
    check("a poll interval rude to a public server is refused",
          rude.failedWith("poll_seconds"), "%s", rude.why());

    const Loaded strat = load(R"({
      "ntp_sources": [ { "server": "a.example", "max_stratum": 16 } ],
      "sources": [ { "url": "http://r.example", "carrier_hz": 10000000 } ]
    })");
    check("an impossible max_stratum is refused", strat.failedWith("max_stratum"),
          "%s", strat.why());

    // Legal, but never what was meant. It has to start -- refusing would be
    // worse than serving from the class that does exist -- and it has to say so.
    const Loaded empty = load(R"({
      "clock": { "primary": "ntp" },
      "sources": [ { "url": "http://r.example", "carrier_hz": 10000000 } ]
    })");
    check("a primary class with nothing in it still starts", empty.ok, "%s", empty.why());
    check("...but warns that it will serve from the other class",
          empty.ok && !empty.cfg.warnings.empty(),
          "%s", empty.cfg.warnings.empty() ? "(no warning)" : empty.cfg.warnings.front().c_str());

    const Loaded lonely = load(R"({
      "clock": { "secondary": "standby" },
      "sources": [ { "url": "http://r.example", "carrier_hz": 10000000 } ]
    })");
    check("a standby with nothing to fall back to warns as well",
          lonely.ok && !lonely.cfg.warnings.empty(),
          "%s", lonely.cfg.warnings.empty() ? "(no warning)"
                                            : lonely.cfg.warnings.front().c_str());
}

} // namespace

void testMqttBlock() {
    std::printf("\nmqtt\n");
    const char* src = R"("sources": [{ "name": "a", "url": "http://x", "carrier_hz": 10000000 }])";

    ::unsetenv("UBERSDR_INGEST_URL");
    Loaded r = load(std::string("{") + src + "}");
    check("on by default, at the receiver's container name and port",
          r.ok && r.cfg.mqtt.enabled && r.cfg.mqtt.ingestUrl == "http://ubersdr:6926",
          "%s", r.ok ? r.cfg.mqtt.ingestUrl.c_str() : r.why());

    r = load(std::string("{") + src + R"(, "mqtt": { "ingest_url": "http://10.0.0.5:7000/" }})");
    check("an explicit URL is read, its trailing slash dropped",
          r.ok && r.cfg.mqtt.ingestUrl == "http://10.0.0.5:7000",
          "%s", r.ok ? r.cfg.mqtt.ingestUrl.c_str() : r.why());

    r = load(std::string("{") + src + R"(, "mqtt": { "ingest_url": "https://ubersdr:6926" }})");
    check("anything but http:// is refused", r.failedWith("mqtt.ingest_url"), "%s", r.why());

    r = load(std::string("{") + src + R"(, "mqtt": { "enabled": false, "ingest_url": "nonsense" }})");
    check("...unless MQTT is off", r.ok && !r.cfg.mqtt.enabled, "%s", r.why());

    ::setenv("UBERSDR_INGEST_URL", "http://ingest.example:6926", 1);
    r = load(std::string("{") + src + R"(, "mqtt": { "ingest_url": "http://10.0.0.5:7000" }})");
    ::unsetenv("UBERSDR_INGEST_URL");
    check("UBERSDR_INGEST_URL overrides the file",
          r.ok && r.cfg.mqtt.ingestUrl == "http://ingest.example:6926",
          "%s", r.ok ? r.cfg.mqtt.ingestUrl.c_str() : r.why());
}

void testDcf77Tuning() {
    std::printf("\nDCF77: chosen by its carrier and tuned on it; a leftover Opus setting is ignored\n");

    const Loaded r = load(R"({
      "defaults": { "format": "opus" },
      "sources": [ { "url": "http://r.example", "carrier_hz": 77500 },
                   { "url": "http://r.example", "carrier_hz": 60000 } ]
    })");
    const SourceConfig* d = r.ok && r.cfg.sources.size() == 2 ? &r.cfg.sources[0] : nullptr;
    const SourceConfig* b = r.ok && r.cfg.sources.size() == 2 ? &r.cfg.sources[1] : nullptr;
    check("77.5 kHz is DCF77, with the dial ON the carrier",
          d && broadcastFor(d->carrierHz, d->dialHz) == Broadcast::Dcf77 && d->dialHz == 77500,
          "%s", d ? std::to_string(d->dialHz).c_str() : r.why());
    // "format" went with Opus. A configuration that still asks for Opus must
    // keep loading -- an unattended install cannot stop over it -- and say so.
    bool warned = false;
    for (const std::string& w : r.ok ? r.cfg.warnings : std::vector<std::string>{})
        if (w.find("opus") != std::string::npos) warned = true;
    check("a leftover \"format\": \"opus\" still loads, and warns that it is ignored",
          r.ok && warned, "%s", r.why());
    check("and is named after what it listens to",
          d && d->name.rfind("dcf77-", 0) == 0, "%s", d ? d->name.c_str() : "");
    check("60 kHz is still WWVB, 1 kHz below",
          b && broadcastFor(b->carrierHz, b->dialHz) == Broadcast::Wwvb && b->dialHz == 59000 &&
              b->name.rfind("wwvb-", 0) == 0,
          "%s", b ? b->name.c_str() : "");

    const Loaded near = load(R"({"sources":[{"url":"http://r.example","carrier_hz":77500,"dial_hz":76500}]})");
    check("a dial 1 kHz off still leaves the carrier in the IQ passband", near.ok, "%s", near.why());
    const Loaded far = load(R"({"sources":[{"url":"http://r.example","carrier_hz":77500,"dial_hz":70000}]})");
    check("a dial 7.5 kHz off is refused, and says why", !far.ok &&
              std::string(far.why()).find("IQ passband") != std::string::npos, "%s", far.why());
}

void testMinMargin() {
    std::printf("\nmin_margin: UberSDR's reduced-depth IQ, as the server defines it\n");

    const Loaded def = load(R"({"sources":[{"url":"http://r.example","carrier_hz":77500}]})");
    check("absent is 0, the lossless stream",
          def.ok && def.cfg.sources[0].minMarginDb == 0, "%s", def.why());

    const Loaded ok = load(R"({"defaults":{"min_margin":26},
                               "sources":[{"url":"http://r.example","carrier_hz":77500},
                                          {"url":"http://r.example","carrier_hz":77500,"min_margin":0}]})");
    check("26 dB is taken from defaults, and a source can put it back to lossless",
          ok.ok && ok.cfg.sources[0].minMarginDb == 26 && ok.cfg.sources[1].minMarginDb == 0, "%s", ok.why());

    const Loaded round = load(R"({"sources":[{"url":"http://r.example","carrier_hz":77500,"min_margin":26.4}]})");
    check("whole dB, as the server rounds it",
          round.ok && round.cfg.sources[0].minMarginDb == 26, "%s", round.why());

    for (const char* bad : {"-26", "10", "14.9", "61"}) {
        const Loaded r = load(std::string(R"({"sources":[{"url":"http://r.example","carrier_hz":77500,"min_margin":)") +
                              bad + "}]}");
        check((std::string("min_margin ") + bad + " is refused, and says what is served").c_str(),
              !r.ok && std::string(r.why()).find("15 to 60") != std::string::npos, "%s", r.why());
    }

    const Loaded wwv = load(R"({"sources":[{"url":"http://r.example","carrier_hz":10000000,"min_margin":26}]})");
    bool warned = false;
    for (const std::string& w : wwv.ok ? wwv.cfg.warnings : std::vector<std::string>{})
        if (w.find("min_margin") != std::string::npos) warned = true;
    check("on a WWV source it loads, with a warning that audio is always lossless",
          wwv.ok && warned, "%s", wwv.why());
}

int main() {
    std::printf("ubersdr-ntp configuration test\n");

    char tmpl[] = "/tmp/ubersdr-ntp-configtest-XXXXXX";
    const char* dir = ::mkdtemp(tmpl);
    if (!dir) {
        std::fprintf(stderr, "cannot create a temporary directory\n");
        return 2;
    }
    g_tmpDir = dir;

    testPerSourcePasswords();
    testNtpSources();
    testClockBlock();
    testValidation();
    testMqttBlock();
    testDcf77Tuning();
    testMinMargin();

    // Tidy up: the files hold made-up passwords, but leaving a trail of
    // configuration files in /tmp on every build is untidy either way.
    const std::string rm = "rm -rf '" + g_tmpDir + "'";
    if (std::system(rm.c_str()) != 0) {
        std::fprintf(stderr, "note: could not remove %s\n", g_tmpDir.c_str());
    }

    std::printf("\n%d ok, %d failed\n", g_ok, g_failed);
    return g_failed == 0 ? 0 : 1;
}
