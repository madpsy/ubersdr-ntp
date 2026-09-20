#include "NtpClient.h"

#include "Log.h"
#include "SampleClock.h"

#include <algorithm>
#include <arpa/inet.h>
#include <arpa/nameser.h>
#include <cerrno>
#include <cmath>
#include <cstring>
#include <ifaddrs.h>
#include <netdb.h>
#include <net/if.h>
#include <netinet/in.h>
#include <poll.h>
#include <random>
#include <resolv.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

namespace ubersdr_ntp {

namespace {

constexpr int kPacketSize = 48;
constexpr std::uint64_t kNtpUnixDelta = 2208988800ULL;

// NTP's PHI: the rate an undisciplined clock is assumed to wander at. Used
// here for the dispersion a sample accumulates between being taken and being
// used, exactly as RFC 5905 uses it.
constexpr double kPhi = 15e-6;

// How long to wait for a reply before calling the poll lost. Long enough for a
// satellite path and a busy server, short enough that an unreachable one does
// not hold the thread past its next poll.
constexpr double kReplyTimeoutSec = 5.0;

// A burst on the first poll and on coming up from cold: six packets two seconds
// apart, which is NTP's own iburst. One sample is not a measurement — the
// filter below wants several to pick a minimum-delay one out of — and a
// fallback that takes six minutes to become usable arrives after the outage it
// was for.
constexpr int kBurstCount = 6;
constexpr double kBurstSpacingSec = 2.0;

// Names are resolved again when their DNS TTL runs out, and never less often
// than this, and sooner after a run of timeouts. A pool name is several
// addresses and the set moves; an address that has stopped answering is the
// case where re-resolving is most likely to help and least likely to cost
// anything. The hour is also what is used when the resolver will not say what
// the TTL is -- a name from /etc/hosts, or a lookup that failed on the second
// query after succeeding on the first.
constexpr double kReresolveSec = 3600.0;
constexpr int kReresolveAfterTimeouts = 4;

// A TTL shorter than this is taken as this. Zero means "do not cache", and
// honouring it literally would put a DNS query in front of every packet of the
// opening burst; half a minute is still far inside any TTL a name's owner means
// to be moved within.
constexpr double kMinResolveSec = 30.0;

// After a lookup fails with a working address in hand, how long before trying
// again. The address is kept meanwhile: a resolver outage is not a reason to
// stop polling a server that answers.
constexpr double kResolveRetrySec = 60.0;

// A poll that came back with nothing usable -- no reply, or a reply dropped as
// a queueing spike -- is tried again this soon, up to kMaxQuickRetries times in
// a row, instead of a whole poll interval later. Without it one lost UDP packet
// costs a minute of silence, and the Selector reads silence as staleness. Not
// done while the server has asked for a slower poll, and not for a server that
// has answered nothing in eight polls: a dead server gets its interval, not
// three packets per interval.
constexpr double kQuickRetrySec = 2.0;
constexpr int kMaxQuickRetries = 2;

// How far a sample's round trip may exceed the best in the register, as a
// multiple of the clean half's own spread above that best. See filterAdd.
constexpr double kSpikeSpreadFactor = 3.0;

// The slow sample that makes this many in a row is taken rather than dropped:
// a path whose round trip has gone up and stayed up has changed, and the
// register's best is then a memory of the old one; see filterAdd. Three is one
// poll and its two quick retries.
constexpr int kSpikeRunLimit = kMaxQuickRetries + 1;

// The Selector refuses a source whose newest measurement is older than this
// many poll intervals (and never less than its own three minutes). Four: the
// quick retries cover a lost packet, and this covers the poll after that.
constexpr double kMaxAgePolls = 4.0;

// Below this, excess round-trip delay is not worth rejecting a sample over:
// half a millisecond of possible bias is far inside everything else in this
// program's error budget, and a gate tighter than that would throw away good
// samples on a LAN where the round trip is tens of microseconds and its
// scatter is most of it. See filterAdd.
constexpr double kSpikeBiasFloorSec = 1e-3;

// The most a kiss-o'-death RATE may stretch the poll interval to. Past this the
// peer is barely a fallback, but the alternative is ignoring a server that has
// asked, in the protocol's own words, to be left alone.
constexpr double kMaxBackoffPollSec = 1024.0;

// Consecutive accepted replies before the poll interval is eased back towards
// the configured one. Eight is one full turn of the reach register: the peer
// has answered everything in its recent memory.
constexpr int kBackoffRecoverAfter = 8;

std::uint32_t read32(const std::uint8_t* p) {
    return (static_cast<std::uint32_t>(p[0]) << 24) | (static_cast<std::uint32_t>(p[1]) << 16) |
           (static_cast<std::uint32_t>(p[2]) << 8) | static_cast<std::uint32_t>(p[3]);
}

void put32(std::uint8_t* p, std::uint32_t v) {
    p[0] = static_cast<std::uint8_t>(v >> 24);
    p[1] = static_cast<std::uint8_t>(v >> 16);
    p[2] = static_cast<std::uint8_t>(v >> 8);
    p[3] = static_cast<std::uint8_t>(v);
}

// A 32-bit short format: 16 bits of seconds, 16 of fraction.
double fromShortFormat(std::uint32_t v) { return static_cast<double>(v) / 65536.0; }

// A 64-bit NTP timestamp as Unix seconds.
//
// The era is resolved against a time we already roughly know, because the wire
// format carries only 32 bits of seconds and wraps in 2036. Taking the era
// whose result lands nearest `approxUnix` is correct for any clock less than 68
// years out, which is every clock that is not deliberately broken — and a host
// further out than that has no reference here to tell it so anyway.
double fromNtpTime(const std::uint8_t* p, double approxUnix) {
    const std::uint32_t sec = read32(p);
    const std::uint32_t frac = read32(p + 4);
    double unix = static_cast<double>(sec) - static_cast<double>(kNtpUnixDelta) +
                  static_cast<double>(frac) / 4294967296.0;
    constexpr double kEra = 4294967296.0;
    while (unix - approxUnix > kEra / 2.0) unix -= kEra;
    while (approxUnix - unix > kEra / 2.0) unix += kEra;
    return unix;
}

// The four reference-identifier bytes as text: four printable ASCII characters
// for a stratum-1 server or a kiss code, a dotted quad otherwise — which is
// what they are, and what every other NTP implementation shows.
std::string refidText(const std::uint8_t* p, int stratum) {
    bool printable = true;
    for (int i = 0; i < 4; ++i) {
        if (p[i] == 0) continue;               // null padding is legal
        if (p[i] < 0x20 || p[i] > 0x7e) { printable = false; break; }
    }
    if (stratum <= 1 && printable) {
        std::string s;
        for (int i = 0; i < 4; ++i) if (p[i] >= 0x20 && p[i] <= 0x7e) s += static_cast<char>(p[i]);
        return s;
    }
    char b[32];
    std::snprintf(b, sizeof b, "%u.%u.%u.%u", p[0], p[1], p[2], p[3]);
    return b;
}

// The four bytes as a kiss code, when stratum 0 says that is what they are.
std::string kissText(const std::uint8_t* p) {
    std::string s;
    for (int i = 0; i < 4; ++i) {
        if (p[i] >= 0x20 && p[i] <= 0x7e) s += static_cast<char>(p[i]);
    }
    return s;
}

std::uint64_t randomNonce() {
    // Seeded once per thread from the system source. The requirement is that an
    // off-path attacker cannot predict the next value, not that it be
    // cryptographic — an attacker who can read our packets to learn the
    // generator's state is on the path, and being on the path defeats any
    // nonce.
    static thread_local std::mt19937_64 rng{std::random_device{}()};
    return rng();
}

double clockPrecisionSec() {
    static const double p = [] {
        const double r = daemonClockResolution();
        return r > 0.0 ? r : 1e-6;
    }();
    return p;
}

// The four reference-identifier bytes naming this peer, for when the served
// time comes from it; see NtpPeerInfo::addressRefid. Read back off the
// connected socket so it is the address actually in use rather than the first
// one getaddrinfo offered.
std::uint32_t refidForPeer(int fd) {
    struct sockaddr_storage sa{};
    socklen_t len = sizeof sa;
    if (::getpeername(fd, reinterpret_cast<struct sockaddr*>(&sa), &len) != 0) return 0;
    if (sa.ss_family == AF_INET) {
        const auto* s = reinterpret_cast<const struct sockaddr_in*>(&sa);
        return ntohl(s->sin_addr.s_addr);
    }
    if (sa.ss_family == AF_INET6) {
        const auto* s = reinterpret_cast<const struct sockaddr_in6*>(&sa);
        // FNV-1a over the sixteen address bytes. Not MD5, which is what RFC
        // 5905 names; see NtpPeerInfo::addressRefid for why that is acceptable
        // and what it costs.
        std::uint32_t h = 2166136261u;
        for (int i = 0; i < 16; ++i) {
            h ^= s->sin6_addr.s6_addr[i];
            h *= 16777619u;
        }
        return h;
    }
    return 0;
}

std::string addressText(const struct sockaddr* sa, socklen_t len, bool withPort = true) {
    char host[NI_MAXHOST] = {0}, serv[NI_MAXSERV] = {0};
    if (getnameinfo(sa, len, host, sizeof host, serv, sizeof serv,
                    NI_NUMERICHOST | NI_NUMERICSERV) != 0) {
        return "?";
    }
    if (!withPort) return host;
    const bool v6 = sa->sa_family == AF_INET6;
    return (v6 ? "[" + std::string(host) + "]" : std::string(host)) + ":" + serv;
}

// True if `host` is a numeric address rather than a name: there is nothing to
// look up, and no TTL to obey.
bool isAddressLiteral(const std::string& host) {
    struct in_addr a4{};
    struct in6_addr a6{};
    return ::inet_pton(AF_INET, host.c_str(), &a4) == 1 ||
           ::inet_pton(AF_INET6, host.c_str(), &a6) == 1;
}

// How long DNS says the answer for `host` may be kept, in seconds, or -1 if it
// will not say.
//
// getaddrinfo(3) resolves the name but throws the TTL away, so this asks the
// resolver again for the same record type directly. The smallest TTL in the
// answer section is the one that counts: a CNAME chain expires when its
// shortest link does. A second query costs one round trip to a resolver that
// has just cached the answer, once per TTL.
int dnsTtl(const std::string& host, int family) {
    struct __res_state st;
    std::memset(&st, 0, sizeof st);
    if (res_ninit(&st) != 0) return -1;
    std::uint8_t answer[4096];
    const int n = res_nsearch(&st, host.c_str(), ns_c_in,
                              family == AF_INET6 ? ns_t_aaaa : ns_t_a,
                              answer, sizeof answer);
    int ttl = -1;
    ns_msg msg;
    if (n > 0 && ns_initparse(answer, n, &msg) == 0) {
        const int count = ns_msg_count(msg, ns_s_an);
        for (int i = 0; i < count; ++i) {
            ns_rr rr;
            if (ns_parserr(&msg, ns_s_an, i, &rr) != 0) break;
            const int t = static_cast<int>(ns_rr_ttl(rr));
            if (ttl < 0 || t < ttl) ttl = t;
        }
    }
    res_nclose(&st);
    return ttl;
}

} // namespace

std::vector<std::uint32_t> localIpv4Addresses() {
    std::vector<std::uint32_t> out;
    struct ifaddrs* ifa = nullptr;
    if (getifaddrs(&ifa) != 0) return out;
    for (struct ifaddrs* p = ifa; p; p = p->ifa_next) {
        if (!p->ifa_addr || p->ifa_addr->sa_family != AF_INET) continue;
        const auto* s = reinterpret_cast<const struct sockaddr_in*>(p->ifa_addr);
        out.push_back(ntohl(s->sin_addr.s_addr));
    }
    freeifaddrs(ifa);
    return out;
}

// ---------------------------------------------------------------------------

NtpPeer::NtpPeer(NtpSourceConfig cfg, std::vector<std::uint32_t> localAddrs)
    : m_cfg(std::move(cfg)),
      m_localAddrs(std::move(localAddrs)),
      m_literal(isAddressLiteral(m_cfg.server)),
      m_offsets(OffsetTuning::forPollInterval(m_cfg.pollSeconds)) {
    m_snap.name = m_cfg.name;
    m_snap.kind = SourceKind::Ntp;
    m_snap.enabled = m_cfg.enabled;
    m_snap.weight = m_cfg.weight;
    m_snap.url = "ntp://" + m_cfg.server + ":" + std::to_string(m_cfg.port);
    m_snap.clockState = "stopped";
    m_snap.station = "ntp";
    m_snap.link = LinkState::Idle;
    m_snap.ntp.server = m_cfg.server;
    m_snap.ntp.port = m_cfg.port;
    m_snap.ntp.pollSec = m_cfg.pollSeconds;
    m_snap.ntp.configuredPollSec = m_cfg.pollSeconds;
    m_snap.ntp.maxRootDistanceSec = m_cfg.maxRootDistanceMs * 1e-3;
    m_snap.ntp.maxStratum = m_cfg.maxStratum;
    m_snap.extraSec = m_cfg.extraDelayMs * 1e-3 / 2.0;
    if (m_cfg.iburst) m_burstLeft = kBurstCount;
}

NtpPeer::~NtpPeer() { stop(); }

void NtpPeer::start() {
    if (!m_cfg.enabled) {
        std::lock_guard<std::mutex> lk(m_mu);
        m_snap.link = LinkState::Stopped;
        m_snap.linkDetail = "disabled in configuration";
        return;
    }
    if (m_running.exchange(true)) return;
    m_thread = std::thread([this] { run(); });
}

void NtpPeer::stop() {
    if (!m_running.exchange(false)) return;
    // The wake mutex is taken, empty, before notifying: without it the flag can
    // flip and the notification fire between the poller testing its predicate
    // and blocking, and a lost wakeup there costs a whole poll interval on
    // SIGTERM.
    { std::lock_guard<std::mutex> lk(m_wake); }
    m_wakeCv.notify_all();
    if (m_thread.joinable()) m_thread.join();
    std::lock_guard<std::mutex> lk(m_mu);
    m_snap.link = LinkState::Stopped;
}

void NtpPeer::requestReacquire(const std::string& why) {
    {
        std::lock_guard<std::mutex> lk(m_mu);
        m_reacquireWhy = why;
        ++m_snap.reacquisitions;
    }
    m_reacquire.store(true);
    { std::lock_guard<std::mutex> lk(m_wake); }
    m_wakeCv.notify_all();
}

void NtpPeer::setActive(bool on, const std::string& why) {
    {
        std::lock_guard<std::mutex> lk(m_mu);
        m_activeReason = why;
    }
    if (m_active.exchange(on) == on) return;
    { std::lock_guard<std::mutex> lk(m_wake); }
    m_wakeCv.notify_all();
}

// ---------------------------------------------------------------------------
// The poll loop

void NtpPeer::run() {
    const char* tag = m_cfg.name.c_str();
    LOG_INFO(tag, "polling %s:%d every %.0fs%s", m_cfg.server.c_str(), m_cfg.port,
             m_cfg.pollSeconds, m_cfg.iburst ? " (bursting to start)" : "");

    bool wasInactive = false;

    while (m_running.load()) {
        if (!m_active.load()) {
            if (!wasInactive) {
                wasInactive = true;
                closeSocket();
                dropTiming("held inactive");
                std::string why;
                {
                    std::lock_guard<std::mutex> lk(m_mu);
                    why = m_activeReason;
                    m_snap.link = LinkState::Idle;
                    m_snap.linkDetail = why.empty() ? "held in cold standby" : why;
                    m_snap.clockState = "stopped";
                    m_snap.ntp.reach = 0;
                }
                LOG_INFO(tag, "idle: %s", why.empty() ? "held in cold standby" : why.c_str());
            }
            std::unique_lock<std::mutex> lk(m_wake);
            m_wakeCv.wait_for(lk, std::chrono::seconds(1),
                              [this] { return !m_running.load() || m_active.load(); });
            continue;
        }
        if (wasInactive) {
            wasInactive = false;
            // Come back bursting. The whole reason a cold peer is worth having
            // is that it can be useful quickly, and one packet a minute is not
            // quickly.
            m_burstLeft = kBurstCount;
            m_backoffPollSec = 0.0;
            LOG_INFO(tag, "brought up from cold standby — bursting to acquire");
        }

        if (m_reacquire.exchange(false)) {
            std::string why;
            {
                std::lock_guard<std::mutex> lk(m_mu);
                why = m_reacquireWhy;
            }
            LOG_WARN(tag, "starting over at the consensus's request — %s", why.c_str());
            closeSocket();              // so the name is resolved again, possibly elsewhere
            dropTiming("re-acquiring");
            m_burstLeft = kBurstCount;
        }

        // Look the name up again when its TTL has run out, or when the peer
        // has gone quiet and may have been moved. Cheap to check, and the case
        // it catches -- one address of a pool going away -- is the common one.
        // The socket stays open through this: if the name still includes the
        // address in use nothing changes, and if the lookup fails the address
        // that was working is kept.
        if (m_fd >= 0 && (m_consecutiveTimeouts >= kReresolveAfterTimeouts ||
                          monotonicNow() >= m_resolveDueAt)) {
            std::string err;
            connectPeer(err);
            m_consecutiveTimeouts = 0;
        }

        std::string err;
        const bool ok = pollOnce(err);

        {
            std::lock_guard<std::mutex> lk(m_mu);
            // The reach register, as NTP's own: one bit per poll, newest at the
            // top. Eight polls of history in a byte, and zero means unreachable.
            m_snap.ntp.reach = static_cast<std::uint8_t>((m_snap.ntp.reach << 1) | (ok ? 1 : 0));
            if (ok) {
                m_snap.link = LinkState::Streaming;
                m_snap.linkDetail = m_resolvedText;
                m_snap.ntp.lastRejectReason.clear();
            } else {
                ++m_snap.ntp.rejected;
                m_snap.ntp.lastRejectReason = err;
                if (m_snap.ntp.reach == 0) {
                    m_snap.link = LinkState::Backoff;
                    m_snap.linkDetail = err;
                }
            }
            recompute();
        }

        if (!ok && m_snap.ntp.reach == 0) {
            // Said once a run rather than once a poll: an upstream that is down
            // for a day would otherwise write 1350 identical lines.
            static thread_local double lastComplaintAt = 0.0;
            const double now = monotonicNow();
            if (now - lastComplaintAt > 300.0) {
                lastComplaintAt = now;
                LOG_WARN(tag, "unreachable: %s", err.c_str());
            }
        }

        bool stopped;
        {
            std::lock_guard<std::mutex> lk(m_mu);
            stopped = m_snap.ntp.stopped;
        }
        if (stopped) {
            // A server that sent DENY or RSTR has told us, in the protocol, not
            // to come back. Honouring that is not optional and there is nothing
            // left for this thread to do; it stays alive only so the status
            // page keeps saying why.
            std::unique_lock<std::mutex> lk(m_wake);
            m_wakeCv.wait_for(lk, std::chrono::seconds(30),
                              [this] { return !m_running.load(); });
            continue;
        }

        double wait;
        std::uint8_t reach;
        {
            std::lock_guard<std::mutex> lk(m_mu);
            reach = m_snap.ntp.reach;
        }
        if (m_burstLeft > 0) {
            --m_burstLeft;
            wait = kBurstSpacingSec;
        } else if (m_retryable && m_backoffPollSec <= 0.0 && reach != 0 &&
                   m_quickRetries < kMaxQuickRetries) {
            ++m_quickRetries;
            wait = kQuickRetrySec;
        } else {
            m_quickRetries = 0;
            wait = m_backoffPollSec > 0.0 ? m_backoffPollSec : m_cfg.pollSeconds;
        }
        {
            std::lock_guard<std::mutex> lk(m_mu);
            m_snap.ntp.pollSec = m_backoffPollSec > 0.0 ? m_backoffPollSec : m_cfg.pollSeconds;
        }
        std::unique_lock<std::mutex> lk(m_wake);
        m_wakeCv.wait_for(lk, std::chrono::duration<double>(wait), [this] {
            return !m_running.load() || !m_active.load() || m_reacquire.load();
        });
    }

    closeSocket();
    std::lock_guard<std::mutex> lk(m_mu);
    m_snap.link = LinkState::Stopped;
    m_snap.clockState = "stopped";
}

bool NtpPeer::ensureSocket(std::string& err) {
    if (m_fd >= 0) return true;
    return connectPeer(err);
}

bool NtpPeer::connectPeer(std::string& err) {
    const char* tag = m_cfg.name.c_str();
    const double now = monotonicNow();

    struct addrinfo hints{};
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_DGRAM;
    hints.ai_protocol = IPPROTO_UDP;

    struct addrinfo* res = nullptr;
    const std::string port = std::to_string(m_cfg.port);
    const int rc = getaddrinfo(m_cfg.server.c_str(), port.c_str(), &hints, &res);
    if (rc != 0 || !res) {
        err = std::string("cannot resolve ") + m_cfg.server + ": " + gai_strerror(rc);
        if (res) freeaddrinfo(res);
        m_resolveDueAt = now + kResolveRetrySec;
        if (m_fd >= 0 && !m_resolveFailing) {
            LOG_WARN(tag, "%s; keeping %s and trying again every %.0fs", err.c_str(),
                     m_resolvedText.c_str(), kResolveRetrySec);
        }
        m_resolveFailing = true;
        return false;
    }
    if (m_resolveFailing && m_fd >= 0) LOG_INFO(tag, "%s resolves again", m_cfg.server.c_str());
    m_resolveFailing = false;

    // The address already in use, if the answer still includes it. A pool
    // name's answers come back in a different order each time; following the
    // order would change servers on every lookup and throw the filter away
    // each time for nothing.
    int family = AF_UNSPEC;
    bool kept = false;
    if (m_fd >= 0) {
        for (struct addrinfo* a = res; a; a = a->ai_next) {
            if (addressText(a->ai_addr, a->ai_addrlen) == m_resolvedText) {
                kept = true;
                family = a->ai_family;
                break;
            }
        }
    }

    // Otherwise the first address that a socket can be made for. connect(2)
    // on a UDP socket rather than sendto: it fixes the peer so a reply from
    // anywhere else is dropped by the kernel before this code sees it, it
    // picks the source address once instead of per packet, and it is what
    // makes ICMP port-unreachable visible as an error on the next call rather
    // than as a silent timeout.
    int fd = -1;
    std::string text, host;
    if (!kept) {
        for (struct addrinfo* a = res; a; a = a->ai_next) {
            fd = ::socket(a->ai_family, SOCK_DGRAM, a->ai_protocol);
            if (fd < 0) continue;
            if (::connect(fd, a->ai_addr, a->ai_addrlen) != 0) {
                ::close(fd);
                fd = -1;
                continue;
            }
            text = addressText(a->ai_addr, a->ai_addrlen);
            host = addressText(a->ai_addr, a->ai_addrlen, false);
            family = a->ai_family;
            break;
        }
    }
    freeaddrinfo(res);

    if (!kept && fd < 0) {
        err = std::string("cannot reach ") + m_cfg.server + ": " + std::strerror(errno);
        m_resolveDueAt = now + kResolveRetrySec;
        return false;   // an open socket, if there is one, is kept
    }

    // When to look again. The TTL of the answer, within [kMinResolveSec,
    // kReresolveSec]; the hour when there is no TTL to be had. An address
    // literal was never looked up and has nothing to expire, so only a run of
    // timeouts brings it back here.
    int ttl = -1;
    double lookAgain = kReresolveSec;
    if (m_literal) {
        lookAgain = 1e18;
    } else {
        ttl = dnsTtl(m_cfg.server, family);
        if (ttl >= 0) lookAgain = std::clamp(static_cast<double>(ttl), kMinResolveSec, kReresolveSec);
    }
    m_resolveDueAt = now + lookAgain;
    {
        std::lock_guard<std::mutex> lk(m_mu);
        m_snap.ntp.dnsTtlSec = ttl;
    }

    if (kept) {
        LOG_DEBUG(tag, "%s still resolves to %s (TTL %d s)", m_cfg.server.c_str(),
                  m_resolvedText.c_str(), ttl);
        return true;
    }

    // Kernel receive timestamps, for the same reason the server asks for them:
    // the gap between a reply arriving and this thread waking to read it is
    // real, it is on our side of the measurement, and counting it as network
    // delay would bias the offset by half of it.
#ifdef SO_TIMESTAMPNS
    int one = 1;
    ::setsockopt(fd, SOL_SOCKET, SO_TIMESTAMPNS, &one, sizeof one);
#endif

    const bool moved = m_fd >= 0;
    const std::string from = m_resolvedText;
    closeSocket();
    m_fd = fd;
    m_resolvedAt = now;
    m_resolvedText = text;
    m_consecutiveTimeouts = 0;
    {
        std::lock_guard<std::mutex> lk(m_mu);
        m_snap.ntp.address = text;
        m_snap.ntp.addressHost = host;
        m_snap.ntp.addressRefid = refidForPeer(fd);
        if (moved) {
            // A different server is a different path, and the register's best
            // round trip belongs to the old one: gating the new server's
            // samples against it would drop every one of them if the new path
            // is longer. The offset history is kept -- it is the same UTC, and
            // the estimator absorbs a step between servers as it would any
            // other.
            for (Sample& f : m_filter) f = Sample{};
            m_spikeRun = 0;
        }
    }
    if (moved) {
        LOG_INFO(tag, "%s now resolves to %s, not %s: moving to it (TTL %d s)",
                 m_cfg.server.c_str(), text.c_str(), from.c_str(), ttl);
    } else {
        LOG_DEBUG(tag, "resolved to %s (TTL %d s)", text.c_str(), ttl);
    }
    return true;
}

void NtpPeer::closeSocket() {
    if (m_fd >= 0) ::close(m_fd);
    m_fd = -1;
}

// One crystal, one rate: see TimeSource::setSystemRate. Cheap enough to do on
// every pass -- it only writes three doubles -- and the estimator picks it up
// at its next recompute.
void NtpPeer::setSystemRate(double rateSec, double uncertaintySec, bool known) {
    std::lock_guard<std::mutex> lk(m_mu);
    m_offsets.setRatePrior(rateSec, uncertaintySec, known);
}

void NtpPeer::dropTiming(const char* why) {
    std::lock_guard<std::mutex> lk(m_mu);
    for (Sample& s : m_filter) s = Sample{};
    m_offsets.clear();
    m_handedOn = 0;
    m_snap.haveOffset = false;
    m_snap.offsetSamples = 0;
    LOG_DEBUG(m_cfg.name.c_str(), "timing discarded (%s)", why);
}

bool NtpPeer::pollOnce(std::string& err) {
    m_retryable = false;
    if (!ensureSocket(err)) return false;

    std::uint8_t out[kPacketSize];
    std::memset(out, 0, sizeof out);
    out[0] = (0 << 6) | (4 << 3) | 3;          // LI 0, version 4, mode 3 (client)
    out[1] = 0;                                 // stratum, unspecified in a request
    {
        const double p = m_backoffPollSec > 0.0 ? m_backoffPollSec : m_cfg.pollSeconds;
        int poll = static_cast<int>(std::lround(std::log2(std::max(1.0, p))));
        out[2] = static_cast<std::uint8_t>(std::clamp(poll, 4, 17));
    }
    out[3] = static_cast<std::uint8_t>(
        std::clamp(static_cast<int>(std::ceil(std::log2(clockPrecisionSec()))), -20, 0));

    // The nonce, in the transmit field. See the header: this is what makes a
    // forged reply require being on the path.
    const std::uint64_t nonce = randomNonce();
    put32(out + 40, static_cast<std::uint32_t>(nonce >> 32));
    put32(out + 44, static_cast<std::uint32_t>(nonce & 0xFFFFFFFFULL));

    // t1 as late as possible before the send, t4 as early as possible after the
    // receive. Everything between them is delay, and whatever this code does in
    // between is delay it has added itself.
    const double t1 = daemonNow();
    if (::send(m_fd, out, sizeof out, 0) != static_cast<ssize_t>(sizeof out)) {
        err = std::string("send: ") + std::strerror(errno);
        closeSocket();
        return false;
    }
    {
        std::lock_guard<std::mutex> lk(m_mu);
        ++m_snap.ntp.sent;
    }

    // Wait for the reply. Anything that is not a well-formed answer to THIS
    // request is discarded and the wait resumes on what is left of the timeout,
    // rather than ending the poll: a late reply to the previous request, or a
    // spoofed packet that arrived first, would otherwise cost this poll.
    const double deadline = monotonicNow() + kReplyTimeoutSec;
    for (;;) {
        const double remain = deadline - monotonicNow();
        if (remain <= 0.0) {
            ++m_consecutiveTimeouts;
            m_retryable = true;
            err = "no reply within " + std::to_string(static_cast<int>(kReplyTimeoutSec)) + "s";
            return false;
        }
        struct pollfd pfd{m_fd, POLLIN, 0};
        const int pr = ::poll(&pfd, 1, static_cast<int>(remain * 1000.0) + 1);
        if (pr < 0) {
            if (errno == EINTR) continue;
            err = std::string("poll: ") + std::strerror(errno);
            closeSocket();
            return false;
        }
        if (pr == 0) continue;

        std::uint8_t in[512];
        struct iovec iov{in, sizeof in};
        alignas(struct cmsghdr) char control[256];
        struct msghdr mh{};
        mh.msg_iov = &iov;
        mh.msg_iovlen = 1;
        mh.msg_control = control;
        mh.msg_controllen = sizeof control;

        const ssize_t n = ::recvmsg(m_fd, &mh, 0);
        if (n < 0) {
            if (errno == EINTR || errno == EAGAIN) continue;
            // ECONNREFUSED here is the ICMP port-unreachable the connect above
            // makes visible: the host is up and nothing is listening, which is
            // a different fault from a timeout and worth saying so.
            err = std::string("recv: ") + std::strerror(errno);
            closeSocket();
            return false;
        }

        double t4 = daemonNow();
#ifdef SO_TIMESTAMPNS
        for (struct cmsghdr* cm = CMSG_FIRSTHDR(&mh); cm; cm = CMSG_NXTHDR(&mh, cm)) {
            if (cm->cmsg_level != SOL_SOCKET || cm->cmsg_type != SO_TIMESTAMPNS) continue;
            struct timespec ts;
            std::memcpy(&ts, CMSG_DATA(cm), sizeof ts);
            // The kernel stamps on the HOST clock; moved onto the daemon clock
            // by the difference as it stands now. A stamp more than a second
            // from the read means the host clock was stepped in between, and
            // the read stands instead.
            const double viaStamp = static_cast<double>(ts.tv_sec) +
                                    static_cast<double>(ts.tv_nsec) * 1e-9 + daemonMinusRealtime();
            if (std::abs(viaStamp - t4) < 1.0) t4 = viaStamp;
        }
#endif

        if (n < kPacketSize) { err = "reply too short"; continue; }

        const int leap = (in[0] >> 6) & 0x03;
        const int version = (in[0] >> 3) & 0x07;
        const int mode = in[0] & 0x07;
        const int stratum = in[1];

        if (mode != 4) { err = "reply was not mode 4"; continue; }
        if (version < 3 || version > 4) { err = "reply version " + std::to_string(version); continue; }

        // The nonce, returned verbatim. Every check below is about whether the
        // server is any good; this one is about whether the packet is from the
        // server at all, so it comes first among them.
        if (read32(in + 24) != static_cast<std::uint32_t>(nonce >> 32) ||
            read32(in + 28) != static_cast<std::uint32_t>(nonce & 0xFFFFFFFFULL)) {
            err = "reply did not echo the request (originate mismatch)";
            continue;   // not ours; keep waiting for one that is
        }

        // --- from here the packet is this server's answer to this request ---
        if (stratum == 0) {
            // Kiss-o'-death. The refid is a four-character code rather than an
            // identifier, and two of them mean stop for good.
            const std::string kiss = kissText(in + 12);
            std::lock_guard<std::mutex> lk(m_mu);
            m_snap.ntp.kissCode = kiss;
            if (kiss == "DENY" || kiss == "RSTR") {
                m_snap.ntp.stopped = true;
                LOG_WARN(m_cfg.name.c_str(),
                         "the server sent kiss-o'-death %s: it does not want to be polled by us. "
                         "Not polling it again.", kiss.c_str());
                err = "the server refused us (kiss-o'-death " + kiss + ")";
            } else if (kiss == "RATE") {
                // The burst is almost always what provoked this -- a pool
                // server rate-limits six packets two seconds apart on sight --
                // so the first thing to do is stop bursting. Doubling the poll
                // while continuing to burst would earn another RATE for each
                // remaining packet and quadruple the interval on the way.
                m_burstLeft = 0;

                // Back off to twice the current interval, or to the poll the
                // server asked for in the packet, whichever is longer. The
                // advertised figure is the server telling us its actual limit;
                // blind doubling either overshoots it or takes several
                // refusals to reach it.
                const double current = m_backoffPollSec > 0.0 ? m_backoffPollSec
                                                              : m_cfg.pollSeconds;
                const double asked = std::pow(2.0, static_cast<double>(
                                                       std::clamp<int>(in[2], 4, 17)));
                m_backoffPollSec = std::min(kMaxBackoffPollSec,
                                            std::max(current * 2.0, asked));
                m_goodRun = 0;
                LOG_WARN(m_cfg.name.c_str(),
                         "the server sent kiss-o'-death RATE: polling too often. "
                         "Backing off to %.0fs.", m_backoffPollSec);
                err = "the server asked for a slower poll (kiss-o'-death RATE)";
            } else {
                err = "kiss-o'-death " + (kiss.empty() ? std::string("(unnamed)") : kiss);
            }
            return false;
        }
        if (leap == 3) { err = "the server says it is unsynchronised (LI=3)"; return false; }
        if (stratum > m_cfg.maxStratum) {
            err = "stratum " + std::to_string(stratum) + " is past max_stratum " +
                  std::to_string(m_cfg.maxStratum);
            return false;
        }

        const double approx = realtimeNow();
        const double t2 = fromNtpTime(in + 32, approx);
        const double t3 = fromNtpTime(in + 40, approx);
        if (read32(in + 40) == 0 && read32(in + 44) == 0) {
            err = "the server's transmit timestamp is zero";
            return false;
        }

        const double rootDelay = fromShortFormat(read32(in + 4));
        const double rootDisp = fromShortFormat(read32(in + 8));
        const std::string refid = refidText(in + 12, stratum);

        // Loop detection. A server whose reference is one of our own addresses
        // is synchronised to THIS daemon, and taking time from it would close
        // a circle with no radio anywhere in it -- the pair would agree
        // perfectly while drifting away together, which is the worst kind of
        // wrong because nothing in the arrangement can see it.
        if (stratum > 1) {
            const std::uint32_t r = read32(in + 12);
            if (std::find(m_localAddrs.begin(), m_localAddrs.end(), r) != m_localAddrs.end()) {
                err = "the server is synchronised to this host (refid " + refid +
                      "): taking time from it would be a loop";
                return false;
            }
        }

        // RFC 5905's offset and delay. The daemon clock on both ends of the
        // round trip, so this is UTC minus the daemon clock -- the same
        // quantity the radio sources measure. See the header.
        double offset = ((t2 - t1) + (t3 - t4)) / 2.0;
        double delay = (t4 - t1) - (t3 - t2);

        // The delay cannot be negative, and a round trip cannot be shorter than
        // the resolution the two clocks were read to. Floored rather than
        // refused: a fast local server genuinely produces a delay of a few
        // microseconds, and refusing it would drop the best peer available.
        const double minDelay = 2.0 * clockPrecisionSec();
        if (delay < minDelay) delay = minDelay;

        // Path asymmetry the halving cannot see. Positive extra_delay_ms means
        // the reply takes longer than the request, which makes the computed
        // offset too small by half the difference.
        offset += m_cfg.extraDelayMs * 1e-3 / 2.0;

        // What this sample is worth: the two clocks' reading resolution plus
        // what an undisciplined clock could have drifted over the round trip,
        // exactly as RFC 5905 computes epsilon.
        const double serverPrecision = std::pow(2.0, static_cast<double>(static_cast<std::int8_t>(in[3])));
        const double sampleDisp = serverPrecision + clockPrecisionSec() + kPhi * (t4 - t1);

        // And what the server itself admits to: its distance from ITS reference,
        // which is the part of the error budget nothing here can improve on.
        const double rootDistance = (rootDelay + delay) / 2.0 + rootDisp + sampleDisp;
        if (rootDistance > m_cfg.maxRootDistanceMs * 1e-3) {
            char b[160];
            std::snprintf(b, sizeof b,
                          "the server's own root distance is %.0f ms (limit %.0f ms)",
                          rootDistance * 1000.0, m_cfg.maxRootDistanceMs);
            err = b;
            return false;
        }

        m_consecutiveTimeouts = 0;
        std::lock_guard<std::mutex> lk(m_mu);
        ++m_snap.ntp.received;

        // Walk the RATE backoff back down after a clean run.
        //
        // Without this a single kiss -- and one on the opening burst is the
        // common case, not the rare one -- leaves the peer at 512 or 1024
        // seconds for the life of the process. This daemon runs for months
        // unattended and every stuck state has to be able to clear itself, so
        // a peer that has been answering happily for eight polls tries the
        // next interval down. If the server objects again it backs off again,
        // which costs one refused poll an hour rather than a fallback that is
        // permanently too slow to be one.
        if (m_backoffPollSec > 0.0 && ++m_goodRun >= kBackoffRecoverAfter) {
            m_goodRun = 0;
            m_backoffPollSec = m_backoffPollSec / 2.0;
            if (m_backoffPollSec <= m_cfg.pollSeconds) {
                m_backoffPollSec = 0.0;
                LOG_INFO(m_cfg.name.c_str(),
                         "back to the configured %.0fs poll after a clean run",
                         m_cfg.pollSeconds);
            } else {
                LOG_INFO(m_cfg.name.c_str(),
                         "easing the poll back to %.0fs after a clean run", m_backoffPollSec);
            }
        }
        m_snap.ntp.stratum = stratum;
        m_snap.ntp.refid = refid;
        m_snap.ntp.leap = leap;
        m_snap.ntp.rootDelaySec = rootDelay;
        m_snap.ntp.rootDispersionSec = rootDisp;
        m_snap.ntp.serverPrecisionSec = serverPrecision;
        m_snap.ntp.lastReplyAgeSec = 0.0;
        m_snap.ntp.kissCode.clear();
        m_snap.lastDecodedUtc.clear();
        // The broadcast leap warning, from the one place that carries it here.
        // Honoured the same way a radio source's is: it is one bit from one
        // machine, and the Selector requires every survivor to agree.
        m_snap.leapPending = (leap == 1 || leap == 2);

        Sample s;
        s.offsetSec = offset;
        s.delaySec = delay;
        s.dispersionSec = sampleDisp;
        s.atSec = t4;
        s.seq = ++m_seq;
        s.valid = true;
        filterAdd(s);
        return true;
    }
}

// ---------------------------------------------------------------------------
// The clock filter

void NtpPeer::filterAdd(const Sample& s) {
    // Shift in at the front, oldest falls off the end.
    for (int i = kFilterSize - 1; i > 0; --i) m_filter[i] = m_filter[i - 1];
    m_filter[0] = s;

    // The winner is the smallest round trip in the register. Delay above the
    // minimum is queueing, queueing is where the asymmetry lives, and the
    // asymmetry is the whole of the error this cannot otherwise see.
    const Sample* best = nullptr;
    for (const Sample& f : m_filter) {
        if (!f.valid) continue;
        if (!best || f.delaySec < best->delaySec) best = &f;
    }
    if (!best) return;

    // Filter jitter: how far the OTHER samples sit from the chosen one. This is
    // the figure that says how much the path is moving, and it is what
    // distinguishes a peer on a quiet link from one behind a loaded router --
    // which is exactly what the Selector needs to weight them by.
    double ss = 0.0;
    int n = 0;
    for (const Sample& f : m_filter) {
        if (!f.valid || f.seq == best->seq) continue;
        const double d = f.offsetSec - best->offsetSec;
        ss += d * d;
        ++n;
    }
    m_snap.ntp.filterJitterSec = n > 0 ? std::sqrt(ss / n) : 0.0;
    m_snap.ntp.delaySec = best->delaySec;
    m_snap.ntp.rootDistanceSec = (m_snap.ntp.rootDelaySec + best->delaySec) / 2.0 +
                                 m_snap.ntp.rootDispersionSec + best->dispersionSec;

    // Hand the NEW sample on, if its delay says it is clean.
    //
    // The obvious thing -- hand on whichever sample currently has the lowest
    // delay -- is wrong, and wrong in a way that only shows up over time: a
    // peer whose first poll happens to be its fastest hands on nothing for the
    // next seven, because the winner has not changed and the same packet must
    // not be counted eight times. At a 64-second poll that is eight minutes of
    // silence, and the estimator downstream needs four samples in its level
    // window before it reports anything at all. Measured against a loopback
    // server answering seven polls in twelve seconds, it had handed on one.
    //
    // So the minimum is used as a GATE rather than as the output. Excess delay
    // above the best in the register is queueing, and queueing on one leg
    // biases this sample's offset by at most half of it -- so a sample is
    // taken when that possible bias is small, and dropped as a spike when it
    // is not. Small is measured against the best round trip itself, because a
    // millisecond means something different on a LAN and over a satellite, with
    // a floor below which nothing in this program's error budget notices.
    //
    // The result is one estimator sample per clean poll and none per queued
    // one, which is what the median and the rate fit downstream want: enough
    // independent readings to be robust, with the ones that are known to be
    // biased left out rather than averaged in.
    //
    // "Small" also widens with how much the path's CLEAN round trips move. An
    // anycast server's delay scatters from reply to reply with nothing queued
    // at all, and one lucky fast reply then sits in the register for eight
    // polls making every ordinary one look like a spike -- measured live
    // against time.cloudflare.com, that starved the estimator until the
    // Selector dropped the peer as stale. The spread is taken from the lower
    // quartile of the register rather than its median, so a path where half
    // the replies really are queued (every other one, in the test) does not
    // widen the gate enough to let them through.
    std::vector<double> delays;
    for (const Sample& f : m_filter) {
        if (f.valid) delays.push_back(f.delaySec);
    }
    std::sort(delays.begin(), delays.end());
    const double cleanSpread = delays[delays.size() / 4] - best->delaySec;

    const Sample& fresh = m_filter[0];
    const double excess = fresh.delaySec - best->delaySec;
    const double allowed = std::max({2.0 * kSpikeBiasFloorSec, best->delaySec,
                                     kSpikeSpreadFactor * cleanSpread});
    if (excess > allowed) {
        // A run of them is not queueing but a path that has changed -- a
        // route that got longer, or an anycast server that moved -- and the
        // register's best is then a memory of the old one that would refuse
        // everything for eight polls. The kSpikeRunLimit-th in a row is taken;
        // the level's median still stands against it if it was a spike after
        // all.
        if (++m_spikeRun < kSpikeRunLimit) {
            ++m_snap.ntp.spikes;
            m_retryable = true;
            return;
        }
        LOG_DEBUG(m_cfg.name.c_str(),
                  "taking a sample %.1f ms slower than the best: %d slow in a row",
                  excess * 1000.0, m_spikeRun);
    }
    m_spikeRun = 0;
    if (fresh.seq > m_handedOn) {
        m_handedOn = fresh.seq;
        m_offsets.add(fresh.atSec, fresh.offsetSec);
    }
}

void NtpPeer::recompute() {
    const OffsetEstimate& e = m_offsets.estimate();
    const double now = daemonNow();

    m_snap.haveOffset = e.valid;
    m_snap.offsetSamples = e.samples;
    m_snap.offsetSec = e.offsetSec;
    m_snap.offsetAtSec = e.atSec;
    m_snap.offsetRate = e.rate;
    m_snap.offsetRateUncertainty = e.rateUncertainty;
    m_snap.offsetRateMeasured = e.rateMeasured;
    m_snap.offsetRateSpanSec = e.rateSpanSec;
    m_snap.rateTermSec = e.rateTermSec;
    m_snap.jitterSec = e.jitterSec;
    m_snap.offsetAgeSec = m_offsets.empty() ? 1e9 : now - m_offsets.newestAt();
    // One sample a poll, so the staleness the Selector allows is counted in
    // polls; its three-minute default is under three of them at 64 s. Against
    // the interval in force, which a RATE backoff stretches.
    m_snap.maxOffsetAgeSec = std::max(180.0, kMaxAgePolls * m_snap.ntp.pollSec);

    // The one-way delay this peer's offset already has taken out of it, for the
    // status report's delay column. Half the round trip is not a measurement of
    // the one-way path, it is the assumption the protocol is built on; the
    // dispersion below is where the doubt about it lives.
    m_snap.networkSec = m_snap.ntp.delaySec / 2.0;
    m_snap.extraSec = m_cfg.extraDelayMs * 1e-3 / 2.0;
    m_snap.delaySec = m_snap.networkSec + m_snap.extraSec;

    // WHAT IT ASSERTS. The server's own distance from its reference, plus this
    // path's share, plus what our own filtering could not remove. Every term is
    // one the server or the measurement actually produced -- nothing here is a
    // guess, which is what makes it comparable with a radio source's dispersion
    // in the Marzullo intersection.
    m_snap.dispersionSec = m_snap.ntp.rootDistanceSec + m_snap.ntp.filterJitterSec +
                           e.jitterSec + e.rateTermSec;

    // WHAT DISTINGUISHES IT FROM THE OTHERS, which is a different number and
    // the one the weighted combine uses. The root distance is the server's
    // account of a path above us that we cannot improve on and did not
    // measure; between two peers behind the same upstream it is nearly the same
    // figure, and dividing by a number they share tells the average nothing.
    // What differs is the path to each: its round trip, and how much that moves.
    m_snap.weightDispersionSec = m_snap.ntp.delaySec / 2.0 + m_snap.ntp.filterJitterSec + e.jitterSec;

    m_snap.hostOffsetSec = m_snap.offsetSec + m_snap.offsetRate * (now - m_snap.offsetAtSec) +
                           daemonMinusRealtime();
    m_snap.rawOffsetSec = m_snap.hostOffsetSec - m_snap.delaySec;

    // "stratum 3 (10.84.8.4)" read as though that address were something we
    // talk to. It is not: above stratum 1 the refid is the server's own
    // UPSTREAM, which for a big operator is a private address on their network
    // that we can neither reach nor check. At stratum 1 the same four bytes are
    // a clock code instead -- GPS, PPS, DCF -- so the two want different words.
    // See refidText().
    const char* addr = m_snap.ntp.address.empty() ? m_cfg.server.c_str()
                                                  : m_snap.ntp.address.c_str();
    char path[256];
    if (m_snap.ntp.stratum <= 1) {
        std::snprintf(path, sizeof path, "%s, stratum %d, reference %s, round trip %.1f ms halved",
                      addr, m_snap.ntp.stratum, m_snap.ntp.refid.c_str(),
                      m_snap.ntp.delaySec * 1000.0);
    } else {
        std::snprintf(path, sizeof path, "%s, stratum %d via its upstream %s, round trip %.1f ms halved",
                      addr, m_snap.ntp.stratum, m_snap.ntp.refid.c_str(),
                      m_snap.ntp.delaySec * 1000.0);
    }
    m_snap.pathDescription = path;
}

SourceSnapshot NtpPeer::snapshot() const {
    std::lock_guard<std::mutex> lk(m_mu);
    SourceSnapshot s = m_snap;
    const double now = monotonicNow();
    s.linkAgeSec = now - m_resolvedAt;
    s.ntp.nextResolveInSec = m_literal || m_resolveDueAt <= 0.0 ? -1.0
                                                               : std::max(0.0, m_resolveDueAt - now);
    if (s.ntp.received > 0) {
        // lastReplyAgeSec is stored as 0 at the moment of the reply and aged
        // here, so a snapshot taken between polls reports the age now rather
        // than the age at the last poll.
        s.ntp.lastReplyAgeSec = m_offsets.empty() ? 1e9 : daemonNow() - m_offsets.newestAt();
    }

    // --- what the Selector reads -------------------------------------------
    s.active = m_active.load();
    s.activeReason = m_activeReason;
    const bool reachable = s.ntp.reach != 0;
    s.ready = s.enabled && s.active && !s.ntp.stopped && reachable && s.haveOffset;

    if (!s.enabled) {
        s.notReadyReason = "disabled in the configuration";
        s.clockState = "stopped";
    } else if (!s.active) {
        s.notReadyReason = m_activeReason.empty() ? "held in cold standby" : m_activeReason;
        s.clockState = "stopped";
    } else if (s.ntp.stopped) {
        s.notReadyReason = "the server sent kiss-o'-death " + s.ntp.kissCode +
                           " and will not be polled again";
        s.clockState = "refused";
    } else if (!reachable) {
        s.notReadyReason = s.ntp.sent == 0
                               ? "no poll has been answered yet"
                               : "unreachable: the last 8 polls went unanswered" +
                                     (s.ntp.lastRejectReason.empty()
                                          ? std::string()
                                          : " (" + s.ntp.lastRejectReason + ")");
        s.clockState = "unreachable";
    } else if (!s.haveOffset) {
        s.notReadyReason = "still filtering: " + std::to_string(s.offsetSamples) +
                           " accepted sample(s) so far";
        s.clockState = "reaching";
    } else {
        s.notReadyReason.clear();
        s.clockState = "locked";
    }
    return s;
}

} // namespace ubersdr_ntp
