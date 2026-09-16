// Offline test of the upstream NTP client.
//
// An NtpPeer is pointed at a fake NTP server running in this process on the
// loopback, so the code under test is the real one — the real socket, the real
// packet, the real validation, the real clock filter — and only the thing at
// the far end is made up. A test that built reply packets and called a parsing
// function directly would not exercise the half of this that is a poll loop.
//
// The fake server's clock is deliberately wrong by a known amount. That is what
// makes the central check possible: the peer must report exactly that amount as
// its offset, because that is the whole job.
//
// What is checked, and why each one matters:
//
//   1. IT MEASURES THE OFFSET. Against a server whose clock is a known 250 ms
//      fast, the peer converges on +250 ms. Everything else here is about
//      refusing bad answers; this is the only check that it produces a right one.
//
//   2. THE MINIMUM-DELAY FILTER WORKS. Every other reply is held back 120 ms
//      after its transmit timestamp — the shape of real queueing, which biases
//      the computed offset by half the extra. A mean would land 30 ms out. The
//      filter must pick the clean samples and land on the truth.
//
//   3. IT REFUSES WHAT IT SHOULD. A reply that does not echo the nonce (the
//      whole of off-path spoofing), an unsynchronised server, a stratum past
//      the limit, a root distance past the limit, and a server synchronised to
//      us — which is a loop, and the failure that produces two clocks agreeing
//      perfectly while drifting away together.
//
//   4. IT HANDLES BEING TOLD TO GO AWAY. Kiss-o'-death DENY stops the polling
//      for good; RATE slows it down. Ignoring either is abuse of somebody
//      else's machine.
//
//   5. IT SAYS WHY. Every refusal above has to leave a reason a person can read
//      in the status page, because this daemon runs unattended for months and
//      a peer that is silently doing nothing is indistinguishable from one that
//      is working.
//
//   6. COLD STANDBY. setActive(false) really disconnects and stops polling;
//      setActive(true) bursts and comes back.
//
// Exit status 0 when every check passes.

#include "NtpClient.h"
#include "SampleClock.h"

#include <arpa/inet.h>
#include <atomic>
#include <cmath>
#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <mutex>
#include <netinet/in.h>
#include <poll.h>
#include <string>
#include <sys/socket.h>
#include <thread>
#include <unistd.h>

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

constexpr std::uint64_t kNtpUnixDelta = 2208988800ULL;

void wr32(std::uint8_t* p, std::uint32_t v) {
    p[0] = std::uint8_t(v >> 24); p[1] = std::uint8_t(v >> 16);
    p[2] = std::uint8_t(v >> 8);  p[3] = std::uint8_t(v);
}
void wrTime(std::uint8_t* p, double unixSec) {
    const double ntp = unixSec + double(kNtpUnixDelta);
    const double whole = std::floor(ntp);
    wr32(p, std::uint32_t(std::uint64_t(whole) & 0xFFFFFFFFULL));
    wr32(p + 4, std::uint32_t((ntp - whole) * 4294967296.0));
}
std::uint32_t shortFormat(double sec) {
    if (sec < 0.0) sec = 0.0;
    return std::uint32_t(sec * 65536.0);
}

// ---------------------------------------------------------------------------
// A fake NTP server, on the loopback, with a clock that is wrong on purpose.

class FakeServer {
public:
    struct Policy {
        // The server's clock, as an offset from the daemon clock. This is the
        // quantity the peer has to recover, so it is the answer the test knows
        // and the code does not.
        double clockOffsetSec = 0.0;

        int stratum = 2;
        int leap = 0;
        std::string refid = "1.2.3.4";      // dotted quad, or four characters at stratum 1
        std::uint32_t refidRaw = 0;          // when non-zero, used verbatim (for the loop test)
        double rootDelaySec = 0.002;
        double rootDispersionSec = 0.004;
        std::int8_t precision = -20;

        bool reply = true;                   // false: a black hole
        bool echoOriginate = true;           // false: forge the reply (spoofing)
        std::string kiss;                    // non-empty: stratum 0 with this code

        // Hold every other reply back this long AFTER stamping its transmit
        // timestamp. That is what real queueing does, and it biases the
        // computed offset by half the hold -- which is exactly what the
        // minimum-delay filter exists to throw away.
        double alternateHoldSec = 0.0;
    };

    bool start() {
        m_fd = ::socket(AF_INET, SOCK_DGRAM, 0);
        if (m_fd < 0) return false;
        struct sockaddr_in sa{};
        sa.sin_family = AF_INET;
        sa.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
        sa.sin_port = 0;                      // any free port
        if (::bind(m_fd, reinterpret_cast<struct sockaddr*>(&sa), sizeof sa) != 0) return false;
        socklen_t len = sizeof sa;
        if (::getsockname(m_fd, reinterpret_cast<struct sockaddr*>(&sa), &len) != 0) return false;
        m_port = ntohs(sa.sin_port);
        m_running.store(true);
        m_thread = std::thread([this] { run(); });
        return true;
    }

    void stop() {
        if (!m_running.exchange(false)) return;
        ::shutdown(m_fd, SHUT_RDWR);
        if (m_thread.joinable()) m_thread.join();
        ::close(m_fd);
        m_fd = -1;
    }

    ~FakeServer() { stop(); }

    int port() const { return m_port; }
    std::uint64_t served() const { return m_served.load(); }

    void set(const Policy& p) {
        std::lock_guard<std::mutex> lk(m_mu);
        m_policy = p;
    }
    Policy get() const {
        std::lock_guard<std::mutex> lk(m_mu);
        return m_policy;
    }

private:
    void run() {
        std::uint8_t buf[512];
        bool alternate = false;
        while (m_running.load()) {
            struct pollfd pfd{m_fd, POLLIN, 0};
            if (::poll(&pfd, 1, 200) <= 0) continue;

            struct sockaddr_in from{};
            socklen_t flen = sizeof from;
            const ssize_t n = ::recvfrom(m_fd, buf, sizeof buf, 0,
                                         reinterpret_cast<struct sockaddr*>(&from), &flen);
            if (n < 48) continue;

            const Policy p = get();
            if (!p.reply) continue;

            // The server's own clock: the daemon clock plus the error the test
            // chose. In-process, so both sides read the same oscillator and the
            // difference between them is exactly the planted one.
            const double t2 = daemonNow() + p.clockOffsetSec;

            std::uint8_t out[48];
            std::memset(out, 0, sizeof out);
            const int stratum = p.kiss.empty() ? p.stratum : 0;
            out[0] = std::uint8_t((p.leap << 6) | (4 << 3) | 4);   // version 4, mode 4 (server)
            out[1] = std::uint8_t(stratum);
            out[2] = buf[2];
            out[3] = std::uint8_t(p.precision);
            wr32(out + 4, shortFormat(p.rootDelaySec));
            wr32(out + 8, shortFormat(p.rootDispersionSec));

            if (!p.kiss.empty()) {
                for (int i = 0; i < 4 && i < int(p.kiss.size()); ++i) out[12 + i] = p.kiss[i];
            } else if (p.refidRaw) {
                wr32(out + 12, p.refidRaw);
            } else {
                struct in_addr a{};
                if (::inet_pton(AF_INET, p.refid.c_str(), &a) == 1) {
                    std::memcpy(out + 12, &a.s_addr, 4);
                } else {
                    for (int i = 0; i < 4 && i < int(p.refid.size()); ++i) out[12 + i] = p.refid[i];
                }
            }

            wrTime(out + 16, t2 - 1.0);       // reference timestamp: a second ago
            // Originate: the client's transmit field, echoed. Corrupting it is
            // what an off-path forgery cannot avoid doing, since it cannot know
            // the nonce.
            std::memcpy(out + 24, buf + 40, 8);
            if (!p.echoOriginate) out[24] ^= 0xFF;
            wrTime(out + 32, t2);

            const bool hold = p.alternateHoldSec > 0.0 && alternate;
            alternate = !alternate;

            wrTime(out + 40, daemonNow() + p.clockOffsetSec);
            if (hold) {
                // Stamped, then held. The client's t4 moves out and t3 does
                // not, so this sample's offset is wrong by half the hold and
                // its delay is longer by the whole of it -- which is what the
                // filter sorts on.
                std::this_thread::sleep_for(std::chrono::duration<double>(p.alternateHoldSec));
            }
            ::sendto(m_fd, out, sizeof out, 0,
                     reinterpret_cast<struct sockaddr*>(&from), flen);
            m_served.fetch_add(1);
        }
    }

    int m_fd = -1;
    int m_port = 0;
    std::thread m_thread;
    std::atomic<bool> m_running{false};
    std::atomic<std::uint64_t> m_served{0};
    mutable std::mutex m_mu;
    Policy m_policy;
};

// ---------------------------------------------------------------------------

NtpSourceConfig peerConfig(const std::string& name, int port) {
    NtpSourceConfig c;
    c.name = name;
    c.server = "127.0.0.1";
    c.port = port;
    // The shortest poll the configuration allows, so a test that needs four
    // samples takes seconds rather than minutes. iburst supplies the first six.
    c.pollSeconds = 8.0;
    c.iburst = true;
    return c;
}

// Waits for a condition on the peer's snapshot, up to `limitSec`. Returns the
// last snapshot either way, so a failing check can print what it actually saw.
template <typename F>
SourceSnapshot waitFor(const NtpPeer& peer, F pred, double limitSec = 20.0) {
    const double until = monotonicNow() + limitSec;
    SourceSnapshot s = peer.snapshot();
    while (monotonicNow() < until) {
        s = peer.snapshot();
        if (pred(s)) return s;
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    return s;
}

bool ready(const SourceSnapshot& s) { return s.ready; }
bool answered(const SourceSnapshot& s) { return s.ntp.received > 0; }
bool refused(const SourceSnapshot& s) { return s.ntp.rejected > 0; }

// ---------------------------------------------------------------------------

void testMeasuresOffset() {
    std::printf("\nAgainst a server whose clock is 250 ms fast\n");

    FakeServer srv;
    if (!srv.start()) { check("the fake server starts", false); return; }
    FakeServer::Policy p;
    p.clockOffsetSec = 0.250;
    p.stratum = 2;
    srv.set(p);

    NtpPeer peer(peerConfig("fake", srv.port()), localIpv4Addresses());
    peer.start();

    SourceSnapshot s = waitFor(peer, ready);
    check("the peer becomes ready", s.ready, "%s", s.notReadyReason.c_str());
    check("it answered at least four polls", s.ntp.received >= 4, "%llu",
          (unsigned long long)s.ntp.received);
    check("reach shows the polls answered", s.ntp.reach != 0, "%03o", s.ntp.reach);

    // The whole job: the planted error, recovered.
    const double err = (s.offsetSec - 0.250) * 1000.0;
    check("it recovers the planted offset to within 5 ms", std::abs(err) < 5.0,
          "%+.2f ms measured, %+.2f ms from the truth", s.offsetSec * 1000.0, err);

    // On the loopback the round trip is tens of microseconds, and the peer
    // should say so rather than reporting a figure it invented.
    check("the round trip is measured and small", s.ntp.delaySec > 0.0 && s.ntp.delaySec < 0.010,
          "%.3f ms", s.ntp.delaySec * 1000.0);

    // The server's own account of itself is carried through, not discarded.
    check("the server's stratum is carried through", s.ntp.stratum == 2, "%d", s.ntp.stratum);
    check("...and its root distance, which bounds the answer",
          s.ntp.rootDistanceSec > 0.004 && s.ntp.rootDistanceSec < 0.050,
          "%.1f ms", s.ntp.rootDistanceSec * 1000.0);
    // The asserted dispersion must include the server's own distance: a peer
    // claiming to be tighter than the server it is copying is claiming
    // something nothing can support.
    check("the dispersion it asserts is at least the server's root distance",
          s.dispersionSec >= s.ntp.rootDistanceSec,
          "%.1f ms vs %.1f ms", s.dispersionSec * 1000.0, s.ntp.rootDistanceSec * 1000.0);
    // ...and the figure used to weight it against other sources must NOT,
    // because that part is the server's and not this path's.
    check("...but the weighting figure excludes it",
          s.weightDispersionSec < s.dispersionSec,
          "%.2f ms vs %.1f ms", s.weightDispersionSec * 1000.0, s.dispersionSec * 1000.0);

    peer.stop();
    srv.stop();
}

void testMinimumDelayFilter() {
    std::printf("\nWith every other reply held back 120 ms (the shape of real queueing)\n");

    FakeServer srv;
    if (!srv.start()) { check("the fake server starts", false); return; }
    FakeServer::Policy p;
    p.clockOffsetSec = 0.100;
    p.alternateHoldSec = 0.120;
    srv.set(p);

    NtpPeer peer(peerConfig("queued", srv.port()), localIpv4Addresses());
    peer.start();

    // Enough polls that both kinds are in the eight-deep filter several times.
    SourceSnapshot s = waitFor(peer, [](const SourceSnapshot& x) {
        return x.ready && x.ntp.received >= 6;
    }, 30.0);
    check("the peer becomes ready", s.ready, "%s", s.notReadyReason.c_str());

    // A mean over both kinds would sit 60 ms low (half the hold). The filter
    // keeps the clean ones.
    const double err = (s.offsetSec - 0.100) * 1000.0;
    check("it lands on the truth, not the average of clean and queued",
          std::abs(err) < 10.0,
          "%+.1f ms measured, %+.1f ms from the truth (a mean would be about -60 ms)",
          s.offsetSec * 1000.0, err);
    check("the delay it reports is the clean one, not the held one",
          s.ntp.delaySec < 0.060, "%.1f ms", s.ntp.delaySec * 1000.0);
    // And it says how much the path is moving, which is what the Selector
    // weights it down by.
    check("filter jitter reports the spread it saw", s.ntp.filterJitterSec > 0.010,
          "%.1f ms", s.ntp.filterJitterSec * 1000.0);

    peer.stop();
    srv.stop();
}

void testRefusesForgedReply() {
    std::printf("\nA reply that does not echo the nonce (off-path spoofing)\n");

    FakeServer srv;
    if (!srv.start()) { check("the fake server starts", false); return; }
    FakeServer::Policy p;
    p.clockOffsetSec = 5.0;        // a lie big enough to be obvious if accepted
    p.echoOriginate = false;
    srv.set(p);

    NtpPeer peer(peerConfig("forged", srv.port()), localIpv4Addresses());
    peer.start();

    // It must never become usable. Waiting for the opposite of the condition,
    // so this spends the timeout rather than passing early on a race.
    SourceSnapshot s = waitFor(peer, ready, 12.0);
    check("it never becomes ready", !s.ready, "offset %+.0f ms", s.offsetSec * 1000.0);
    check("no reply was accepted", s.ntp.received == 0, "%llu accepted",
          (unsigned long long)s.ntp.received);
    check("reach stays zero", s.ntp.reach == 0, "%03o", s.ntp.reach);
    check("the server was in fact answering (so this is the check working, "
          "not a dead server)", srv.served() > 0, "%llu replies sent",
          (unsigned long long)srv.served());
    check("and it says why", s.notReadyReason.find("unreachable") != std::string::npos ||
                             s.notReadyReason.find("no poll") != std::string::npos,
          "%s", s.notReadyReason.c_str());

    peer.stop();
    srv.stop();
}

void testRefusesBadServers() {
    std::printf("\nServers that answer, but with something that must not be used\n");

    struct Case {
        const char* name;
        const char* expect;          // a word the refusal has to contain
        FakeServer::Policy (*make)();
    };

    const Case cases[] = {
        {"an unsynchronised server (LI=3)", "unsynchronised", [] {
            FakeServer::Policy p; p.leap = 3; return p; }},
        {"a stratum past max_stratum", "stratum", [] {
            FakeServer::Policy p; p.stratum = 15; return p; }},
        {"a root distance past the limit", "root distance", [] {
            FakeServer::Policy p; p.rootDispersionSec = 3.0; return p; }},
    };

    for (const Case& c : cases) {
        FakeServer srv;
        if (!srv.start()) { check("the fake server starts", false); continue; }
        srv.set(c.make());

        NtpSourceConfig cfg = peerConfig("bad", srv.port());
        cfg.maxStratum = 4;
        cfg.maxRootDistanceMs = 500.0;
        NtpPeer peer(cfg, localIpv4Addresses());
        peer.start();

        SourceSnapshot s = waitFor(peer, refused, 12.0);
        const bool said = s.ntp.lastRejectReason.find(c.expect) != std::string::npos;
        check(std::string(c.name) + " is refused, and says why",
              !s.ready && s.ntp.rejected > 0 && said,
              "%s", s.ntp.lastRejectReason.empty() ? "(no reason given)"
                                                   : s.ntp.lastRejectReason.c_str());
        peer.stop();
        srv.stop();
    }
}

void testLoopDetection() {
    std::printf("\nA server synchronised to this host (a loop)\n");

    const std::vector<std::uint32_t> local = localIpv4Addresses();
    if (local.empty()) {
        check("this host has an IPv4 address to detect a loop with", false);
        return;
    }

    FakeServer srv;
    if (!srv.start()) { check("the fake server starts", false); return; }
    FakeServer::Policy p;
    p.stratum = 2;
    // Its reference is us. In the real world that is a host whose chrony was
    // pointed at this daemon and which was then listed here as a fallback --
    // an easy mistake, and one that leaves two clocks agreeing perfectly while
    // drifting away together with no radio anywhere in the circle.
    p.refidRaw = local.front();
    srv.set(p);

    NtpPeer peer(peerConfig("loop", srv.port()), local);
    peer.start();

    SourceSnapshot s = waitFor(peer, refused, 12.0);
    check("it is refused", !s.ready && s.ntp.rejected > 0);
    check("...and named as a loop, not as some other fault",
          s.ntp.lastRejectReason.find("loop") != std::string::npos,
          "%s", s.ntp.lastRejectReason.empty() ? "(no reason given)"
                                               : s.ntp.lastRejectReason.c_str());

    peer.stop();
    srv.stop();
}

void testKissOfDeath() {
    std::printf("\nKiss-o'-death: the server asking to be left alone\n");

    {
        FakeServer srv;
        if (!srv.start()) { check("the fake server starts", false); return; }
        FakeServer::Policy p;
        p.kiss = "DENY";
        srv.set(p);

        NtpPeer peer(peerConfig("deny", srv.port()), localIpv4Addresses());
        peer.start();

        SourceSnapshot s = waitFor(peer, [](const SourceSnapshot& x) { return x.ntp.stopped; },
                                   12.0);
        check("DENY stops the peer for good", s.ntp.stopped && !s.ready);
        check("...and the status says so in words",
              s.notReadyReason.find("kiss-o'-death") != std::string::npos,
              "%s", s.notReadyReason.c_str());

        // Honouring it means actually stopping, not merely saying so.
        const std::uint64_t before = srv.served();
        std::this_thread::sleep_for(std::chrono::seconds(4));
        const std::uint64_t after = srv.served();
        check("it really does stop polling", after == before,
              "%llu polls in 4 s after DENY", (unsigned long long)(after - before));

        peer.stop();
        srv.stop();
    }

    {
        FakeServer srv;
        if (!srv.start()) { check("the fake server starts", false); return; }
        FakeServer::Policy p;
        p.kiss = "RATE";
        srv.set(p);

        NtpPeer peer(peerConfig("rate", srv.port()), localIpv4Addresses());
        peer.start();

        SourceSnapshot s = waitFor(peer, [](const SourceSnapshot& x) {
            return x.ntp.pollSec > 8.0;
        }, 12.0);
        check("RATE slows the polling down", s.ntp.pollSec > 8.0, "%.0f s", s.ntp.pollSec);
        check("...without stopping it altogether", !s.ntp.stopped);

        peer.stop();
        srv.stop();
    }
}

void testUnreachable() {
    std::printf("\nA server that never answers\n");

    FakeServer srv;
    if (!srv.start()) { check("the fake server starts", false); return; }
    FakeServer::Policy p;
    p.reply = false;
    srv.set(p);

    NtpPeer peer(peerConfig("silent", srv.port()), localIpv4Addresses());
    peer.start();

    SourceSnapshot s = waitFor(peer, [](const SourceSnapshot& x) { return x.ntp.sent >= 2; },
                               20.0);
    check("it keeps trying", s.ntp.sent >= 2, "%llu sent", (unsigned long long)s.ntp.sent);
    check("it is not ready", !s.ready);
    check("reach is zero", s.ntp.reach == 0, "%03o", s.ntp.reach);
    // The point of the memory this daemon is written against: a source that is
    // stuck has to say why on the page, not just fail to appear.
    check("and it says what is wrong rather than going quiet",
          !s.notReadyReason.empty(), "%s", s.notReadyReason.c_str());

    peer.stop();
    srv.stop();
}

void testColdStandby() {
    std::printf("\nCold standby: put away, then brought back\n");

    FakeServer srv;
    if (!srv.start()) { check("the fake server starts", false); return; }
    FakeServer::Policy p;
    p.clockOffsetSec = 0.030;
    srv.set(p);

    NtpPeer peer(peerConfig("standby", srv.port()), localIpv4Addresses());
    peer.start();

    SourceSnapshot s = waitFor(peer, answered, 15.0);
    check("it acquires while active", s.ntp.received > 0, "%llu answered",
          (unsigned long long)s.ntp.received);

    peer.setActive(false, "the primary sources are healthy");
    // Long enough for a poll to have happened if it were going to.
    std::this_thread::sleep_for(std::chrono::seconds(3));
    const std::uint64_t quietFrom = srv.served();
    std::this_thread::sleep_for(std::chrono::seconds(4));

    s = peer.snapshot();
    check("standing it down really stops the polling", srv.served() == quietFrom,
          "%llu polls in 4 s while inactive",
          (unsigned long long)(srv.served() - quietFrom));
    check("it reports itself inactive", !s.active && !s.ready);
    check("...and says which decision put it there, not that it is broken",
          s.notReadyReason.find("primary") != std::string::npos,
          "%s", s.notReadyReason.c_str());
    // The timing is thrown away rather than carried across an idle period of
    // unknown length: an offset measured before it would describe a path that
    // may no longer exist.
    check("the stale offset is discarded", !s.haveOffset);

    peer.setActive(true, "the primary sources have nothing usable");
    s = waitFor(peer, ready, 20.0);
    check("bringing it back up re-acquires", s.ready, "%s", s.notReadyReason.c_str());
    check("...and measures the same offset again",
          std::abs(s.offsetSec - 0.030) < 0.005, "%+.1f ms", s.offsetSec * 1000.0);

    peer.stop();
    srv.stop();
}

} // namespace

int main() {
    std::printf("ubersdr-ntp upstream NTP client test\n");
    // Fix the daemon clock's zero before any thread reads it, as main() does.
    daemonNow();

    testMeasuresOffset();
    testMinimumDelayFilter();
    testRefusesForgedReply();
    testRefusesBadServers();
    testLoopDetection();
    testKissOfDeath();
    testUnreachable();
    testColdStandby();

    std::printf("\n%d ok, %d failed\n", g_ok, g_failed);
    return g_failed == 0 ? 0 : 1;
}
