// ubersdr-ntp — an NTP server disciplined by WWV/WWVH/WWVB heard over UberSDR.
//
// Connects to one or more UberSDR receivers, tunes each to a time-signal
// frequency, decodes the broadcast time code in process, and serves the result
// as NTP. Several receivers and several frequencies because HF propagation
// makes any single one unreliable: 10 MHz from Fort Collins is excellent in the
// afternoon and gone at 3am, 5 MHz is the other way round, and a receiver on
// another continent hears a different subset again. Redundancy here is about
// the band, not about hardware.
//
// WHAT THIS IS AND IS NOT
//
// It is a stratum-1 radio clock: its reference is not another NTP server. It is
// good to some tens of milliseconds when the delay model is set up, and that
// is the honest figure — the error budget is dominated by things no filter can
// see, chiefly the receiver's own buffering and the ionospheric path. Every
// answer carries that budget in its root dispersion, so a client weighing this
// against a GPS source will correctly prefer the GPS.
//
// It is NOT a substitute for a local GPS reference, and it should not be the
// only source on a machine that has one.
//
// Runs in the foreground and logs to stderr, or with `log.file` set writes the
// same lines plus a periodic detailed status block to a file — which is how you
// see what each source is doing when it is running under systemd with no
// terminal. A read-only HTTP service on port 1234 serves the same facts as
// JSON, as a one-event-per-second stream, and as a page.

#include "Config.h"
#include "HttpApi.h"
#include "Log.h"
#include "NtpServer.h"
#include "SampleClock.h"
#include "Selector.h"
#include "Source.h"
#include "Status.h"
#include "Version.h"

#include <curl/curl.h>
#include <ixwebsocket/IXNetSystem.h>

#include <atomic>
#include <cstdio>
#include <cstring>
#include <csignal>
#include <memory>
#include <string>
#include <thread>
#include <vector>

namespace {

constexpr const char* kTag = "main";

// Signal handlers set these and nothing else: everything they would want to do
// — logging, joining threads, closing sockets — is unsafe in a handler.
volatile std::sig_atomic_t g_quit = 0;
volatile std::sig_atomic_t g_reopen = 0;
volatile std::sig_atomic_t g_dump = 0;

void onSignal(int sig) {
    switch (sig) {
        case SIGINT:
        case SIGTERM: g_quit = 1; break;
        case SIGHUP:  g_reopen = 1; break;   // so logrotate can move the file
        case SIGUSR1: g_dump = 1; break;     // status block now, without waiting
        default: break;
    }
}

void usage() {
    using ubersdr_ntp::kVersion;
    std::printf(
        "ubersdr-ntp %s — an NTP server disciplined by WWV/WWVH/WWVB over UberSDR\n"
        "\n"
        "Usage:\n"
        "  ubersdr-ntp --config FILE [overrides]\n"
        "  ubersdr-ntp --source URL@CARRIER [--source ...] [overrides]\n"
        "\n"
        "Configuration:\n"
        "  -c, --config FILE        JSON configuration file (comments allowed)\n"
        "      --source URL@MHZ     A source, for a quick run without a config file.\n"
        "                           e.g. --source https://sdr.example.org@10\n"
        "                           May be repeated. The dial is derived as 1 kHz below\n"
        "                           the carrier, which is the tuning WWV, WWVH and WWVB\n"
        "                           all want.\n"
        "      --password PW        Bypass password applied to every --source\n"
        "      --format FMT         opus (default) or pcm-v4 (lossless, ~4x the bandwidth,\n"
        "                           and no codec delay to calibrate out)\n"
        "\n"
        "Overrides (these win over the config file):\n"
        "      --port N             NTP port (default 123, which is privileged)\n"
        "      --http-port N        Status/JSON/SSE port, 0 to disable (default 1234)\n"
        "      --http-listen ADDR   Status service bind address (default 127.0.0.1)\n"
        "      --log-file FILE      Also write the log, and the periodic status block, here\n"
        "      --log-level LEVEL    trace, debug, info (default), warn, error\n"
        "      --status-interval N  Seconds between status blocks, 0 to disable (default 30)\n"
        "      --quiet              Do not log to stderr (use with --log-file)\n"
        "\n"
        "Other:\n"
        "      --check              Load the configuration, report it, and exit\n"
        "      --version, --help\n"
        "\n"
        "Signals:\n"
        "  SIGHUP    reopen the log file (for logrotate)\n"
        "  SIGUSR1   write a status block immediately\n"
        "\n"
        "Tuning is automatic and not negotiable: USB at the carrier minus 1 kHz with the\n"
        "passband open to 3 kHz. The WWV/WWVH second tick is recovered entirely from its\n"
        "2000 Hz (WWV) or 2200 Hz (WWVH) audio image, so a narrower filter removes the\n"
        "only thing the decoder can find a second edge in.\n",
        kVersion);
}

bool parseSourceSpec(const std::string& spec, ubersdr_ntp::SourceConfig& out, std::string& err) {
    // URL@CARRIER, where the carrier may be in MHz (10, 9.999) or Hz (10000000).
    // Split on the LAST '@' so a URL with userinfo in it still works.
    const std::size_t at = spec.rfind('@');
    if (at == std::string::npos) {
        err = "expected URL@CARRIER, e.g. https://sdr.example.org@10";
        return false;
    }
    out.url = spec.substr(0, at);
    const std::string freq = spec.substr(at + 1);
    if (freq.empty()) { err = "no carrier frequency after '@'"; return false; }

    char* end = nullptr;
    const double v = std::strtod(freq.c_str(), &end);
    if (end == freq.c_str() || v <= 0.0) { err = "not a frequency: " + freq; return false; }
    // Anything under 1000 is taken as MHz. That covers 10, 9.999 and 0.06 (for
    // WWVB's 60 kHz) and cannot be confused with a figure in Hz, since no
    // time-signal carrier this decodes is below 1 kHz.
    out.carrierHz = static_cast<std::uint64_t>(v < 1000.0 ? v * 1e6 : v);
    return true;
}

} // namespace

int main(int argc, char** argv) {
    using namespace ubersdr_ntp;

    std::string configPath;
    std::vector<std::string> sourceSpecs;
    std::string cliPassword;
    std::string cliFormat;
    int cliPort = -1, cliHttpPort = -1;
    std::string cliHttpListen, cliLogFile, cliLogLevel;
    double cliStatusInterval = -1.0;
    bool cliQuiet = false, checkOnly = false;

    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        auto next = [&](const char* what) -> std::string {
            if (i + 1 >= argc) {
                std::fprintf(stderr, "%s needs a value\n", what);
                std::exit(2);
            }
            return argv[++i];
        };
        if (a == "-h" || a == "--help") { usage(); return 0; }
        else if (a == "--version") { std::printf("ubersdr-ntp %s\n", kVersion); return 0; }
        else if (a == "-c" || a == "--config") configPath = next("--config");
        else if (a == "--source") sourceSpecs.push_back(next("--source"));
        else if (a == "--password") cliPassword = next("--password");
        else if (a == "--format") cliFormat = next("--format");
        else if (a == "--port") cliPort = std::atoi(next("--port").c_str());
        else if (a == "--http-port") cliHttpPort = std::atoi(next("--http-port").c_str());
        else if (a == "--http-listen") cliHttpListen = next("--http-listen");
        else if (a == "--log-file") cliLogFile = next("--log-file");
        else if (a == "--log-level") cliLogLevel = next("--log-level");
        else if (a == "--status-interval") cliStatusInterval = std::atof(next("--status-interval").c_str());
        else if (a == "--quiet") cliQuiet = true;
        else if (a == "--check") checkOnly = true;
        else {
            std::fprintf(stderr, "unknown option: %s\nTry --help.\n", a.c_str());
            return 2;
        }
    }

    // --- configuration ------------------------------------------------------
    Config cfg;
    std::string err;

    if (!configPath.empty()) {
        if (!Config::load(configPath, cfg, err)) {
            std::fprintf(stderr, "%s\n", err.c_str());
            return 1;
        }
    } else if (sourceSpecs.empty()) {
        std::fprintf(stderr,
                     "nothing to do: give --config FILE or at least one --source URL@CARRIER.\n"
                     "Try --help.\n");
        return 2;
    }

    if (!cliFormat.empty()) {
        AudioFormat f;
        if (!parseFormat(cliFormat, f)) {
            std::fprintf(stderr, "unknown --format %s (expected opus or pcm-v4)\n", cliFormat.c_str());
            return 2;
        }
        cfg.defaults.format = f;
        for (SourceConfig& s : cfg.sources) s.format = f;
    }

    for (const std::string& spec : sourceSpecs) {
        SourceConfig s = cfg.defaults;
        s.name.clear();
        if (!parseSourceSpec(spec, s, err)) {
            std::fprintf(stderr, "--source %s: %s\n", spec.c_str(), err.c_str());
            return 2;
        }
        if (!cliPassword.empty()) s.password = cliPassword;
        cfg.sources.push_back(std::move(s));
    }

    if (cliPort > 0) cfg.ntp.port = cliPort;
    if (cliHttpPort == 0) cfg.http.enabled = false;
    else if (cliHttpPort > 0) { cfg.http.enabled = true; cfg.http.port = cliHttpPort; }
    if (!cliHttpListen.empty()) cfg.http.listen = cliHttpListen;
    if (!cliLogFile.empty()) cfg.log.file = cliLogFile;
    if (cliStatusInterval >= 0.0) cfg.log.statusIntervalSeconds = cliStatusInterval;
    if (cliQuiet) cfg.log.stderrEnabled = false;
    if (!cliLogLevel.empty() && !parseLevel(cliLogLevel, cfg.log.level)) {
        std::fprintf(stderr, "unknown --log-level %s\n", cliLogLevel.c_str());
        return 2;
    }

    if (!cfg.finalise(err)) {
        std::fprintf(stderr, "configuration: %s\n", err.c_str());
        return 1;
    }

    // Silent with no file is a daemon nobody can support.
    if (!cfg.log.stderrEnabled && cfg.log.file.empty()) {
        std::fprintf(stderr, "--quiet with no log file would make this silent; "
                             "give --log-file too.\n");
        return 2;
    }

    // --- logging ------------------------------------------------------------
    Log::instance().setLevel(cfg.log.level);
    Log::instance().setStderr(cfg.log.stderrEnabled);
    if (!cfg.log.file.empty() && !Log::instance().setFile(cfg.log.file, err)) {
        std::fprintf(stderr, "log file: %s\n", err.c_str());
        return 1;
    }

    LOG_INFO(kTag, "ubersdr-ntp %s starting (User-Agent: %s)", kVersion, cfg.userAgent.c_str());
    for (const SourceConfig& s : cfg.sources) {
        LOG_INFO(kTag, "source %-14s %s  dial %.6f MHz (carrier %.3f MHz) %s%s%s",
                 s.name.c_str(), s.url.c_str(), s.dialHz / 1e6, s.carrierHz / 1e6,
                 formatName(s.format),
                 s.enabled ? "" : " [DISABLED]",
                 s.autoDelay ? "" : " [fixed delay]");
    }
    LOG_INFO(kTag, "ntp on port %d, coasting up to %.0fs at %.0f ppm, min_sources %d",
             cfg.ntp.port, cfg.ntp.coastSeconds, cfg.ntp.coastDriftPpm, cfg.ntp.minSources);

    if (checkOnly) {
        LOG_INFO(kTag, "configuration is valid (--check); exiting");
        return 0;
    }

    // --- runtime ------------------------------------------------------------
    // SIGPIPE is ignored rather than handled: a client that closes an HTTP or
    // SSE connection mid-write would otherwise kill the process, and every
    // write site already checks its return value.
    std::signal(SIGPIPE, SIG_IGN);
    std::signal(SIGINT, onSignal);
    std::signal(SIGTERM, onSignal);
    std::signal(SIGHUP, onSignal);
    std::signal(SIGUSR1, onSignal);

    curl_global_init(CURL_GLOBAL_DEFAULT);
    ix::initNetSystem();

    const double startedAt = monotonicNow();

    std::vector<std::unique_ptr<Source>> sources;
    for (const SourceConfig& s : cfg.sources) {
        sources.push_back(std::make_unique<Source>(s, cfg.userAgent));
    }

    Selector selector(cfg.ntp.coastSeconds, cfg.ntp.coastDriftPpm, cfg.ntp.minSources);

    auto snapshots = [&sources] {
        std::vector<SourceSnapshot> v;
        v.reserve(sources.size());
        for (const auto& s : sources) v.push_back(s->snapshot());
        return v;
    };

    NtpServer ntp(cfg.ntp, selector);

    auto statusInput = [&](void) -> StatusInput {
        StatusInput in;
        in.sources = snapshots();
        in.combined = selector.current();
        in.ntp = ntp.stats();
        in.uptimeSec = monotonicNow() - startedAt;
        in.version = kVersion;
        in.ntpPort = cfg.ntp.port;
        return in;
    };

    HttpApi http(cfg.http, selector, statusInput);
    if (!http.start(err)) {
        // Not fatal: the status service is how you watch this, but NTP is what
        // it is for, and refusing to serve time because a status page could not
        // bind would be the wrong trade.
        LOG_ERROR(kTag, "%s — continuing without the status service", err.c_str());
    }

    if (!ntp.start(err)) {
        LOG_ERROR(kTag, "%s", err.c_str());
        http.stop();
        curl_global_cleanup();
        return 1;
    }

    for (auto& s : sources) s->start();

    // --- the main loop is the status reporter -------------------------------
    //
    // Everything else runs on its own threads. This one exists to emit the
    // periodic status block, to notice signals, and to keep the Selector's view
    // current — which matters because the NTP server answers from the last
    // combine rather than recomputing per request, so a burst of clients cannot
    // turn into a burst of work.
    double nextStatus = monotonicNow() + std::min(5.0, cfg.log.statusIntervalSeconds > 0
                                                      ? cfg.log.statusIntervalSeconds : 5.0);
    bool lastSync = false;
    bool everSync = false;

    while (!g_quit) {
        std::this_thread::sleep_for(std::chrono::milliseconds(250));

        if (g_reopen) { g_reopen = 0; Log::instance().reopen(); LOG_INFO(kTag, "log file reopened"); }

        const auto snaps = snapshots();
        const Combined c = selector.combine(snaps, realtimeNow());

        // A change in whether time is being served at all is worth a line of
        // its own, immediately, rather than waiting up to thirty seconds for
        // the next block.
        if (c.synchronised != lastSync) {
            lastSync = c.synchronised;
            StatusInput in = statusInput();
            in.combined = c;
            if (c.synchronised) {
                LOG_INFO(kTag, "%s%s", everSync ? "" : "first synchronisation — ",
                         renderStatusLine(in).c_str());
                everSync = true;
            } else {
                LOG_WARN(kTag, "%s", renderStatusLine(in).c_str());
            }
        }

        const double now = monotonicNow();
        const bool due = cfg.log.statusIntervalSeconds > 0 && now >= nextStatus;
        if (due || g_dump) {
            if (g_dump) g_dump = 0;
            if (cfg.log.statusIntervalSeconds > 0) {
                while (nextStatus <= now) nextStatus += cfg.log.statusIntervalSeconds;
            }
            StatusInput in = statusInput();
            in.combined = c;
            Log::instance().block(LogLevel::Info, "status", renderStatusLine(in),
                                  renderStatusBlock(in));
        }
    }

    LOG_INFO(kTag, "shutting down");
    for (auto& s : sources) s->stop();
    ntp.stop();
    http.stop();
    sources.clear();
    ix::uninitNetSystem();
    curl_global_cleanup();
    LOG_INFO(kTag, "stopped");
    return 0;
}
