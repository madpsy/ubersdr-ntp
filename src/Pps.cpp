#include "Pps.h"

#include "CivilTime.h"
#include "Events.h"
#include "Log.h"
#include "SampleClock.h"

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <ctime>

#include <fcntl.h>
#include <pthread.h>
#include <sched.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

namespace ubersdr_ntp {

namespace {

constexpr const char* kTag = "pps";

// How long before the second the thread stops sleeping and spins: a sleep
// wakes late by up to a timer slack, tens of microseconds at real-time priority
// and more without it, and the spin takes that out.
constexpr double kSpinSec = 0.002;
// A pulse this late is not sent: it would mark the wrong instant, and a
// consumer cannot tell a late pulse from a clock that is wrong.
constexpr double kMaxLateSec = 0.050;
// Between attempts to open a port that is missing, busy or failing.
constexpr double kReopenSec = 5.0;
// After a leap second, how long the served clock may read one second ahead of
// the count while the sources re-anchor (PpsLabeler).
constexpr long long kLeapCatchUpSec = 3600;
// A leap warning counts for the midnight at most this long after it was seen:
// the sources stop announcing it at the event itself.
constexpr long long kLeapArmedSec = 7200;

std::string two(int v) { char b[8]; std::snprintf(b, sizeof b, "%02d", v); return b; }

struct Civil { int y; unsigned mo, d; int hh, mi, ss; };

// The civil fields a label names. 23:59:60 belongs to the day before the
// midnight it precedes.
Civil civilOf(const PpsLabel& t) {
    const long long s = t.leap ? t.posix - 1 : t.posix;
    Civil c{};
    civilFromDays(floorDiv(s, 86400), c.y, c.mo, c.d);
    const long long rem = floorMod(s, 86400);
    c.hh = static_cast<int>(rem / 3600);
    c.mi = static_cast<int>(rem / 60 % 60);
    c.ss = t.leap ? 60 : static_cast<int>(rem % 60);
    return c;
}

std::string hhmmss(const Civil& c) { return two(c.hh) + two(c.mi) + two(c.ss) + ".00"; }

// ddmm.mmmm / dddmm.mmmm: whole degrees, then minutes to four places. Rounded
// once, in ten-thousandths of a minute, so 59.99995' cannot print as 60.0000.
std::string nmeaAngle(double deg, int degDigits) {
    const long long u = std::llround(std::fabs(deg) * 60.0 * 10000.0);
    const long long d = u / 600000, m = u % 600000;
    char b[48];
    std::snprintf(b, sizeof b, "%0*lld%02lld.%04lld", degDigits, d, m / 10000, m % 10000);
    return b;
}

// The lines' TIOCM bit.
int lineBit(const std::string& line) { return line == "rts" ? TIOCM_RTS : TIOCM_DTR; }

speed_t baudConst(int baud) {
    switch (baud) {
        case 9600: return B9600;
        case 19200: return B19200;
        case 38400: return B38400;
        case 57600: return B57600;
        case 115200: return B115200;
        default: return B4800;
    }
}

class SerialPort final : public PpsPort {
public:
    SerialPort(int fd, int bit, bool invert) : m_fd(fd), m_bit(bit), m_invert(invert) {}
    ~SerialPort() override {
        std::string ignored;
        setLine(false, ignored);
        ::close(m_fd);
    }
    bool setLine(bool on, std::string& err) override {
        int bit = m_bit;
        if (::ioctl(m_fd, (on != m_invert) ? TIOCMBIS : TIOCMBIC, &bit) != 0) {
            err = std::string("cannot set the line: ") + std::strerror(errno);
            return false;
        }
        return true;
    }
    bool write(const std::string& data, std::string& err) override {
        // Never blocks: a port whose output has backed up is failing, and the
        // next second's pulse matters more than this second's sentence.
        const ssize_t n = ::write(m_fd, data.data(), data.size());
        if (n == static_cast<ssize_t>(data.size())) return true;
        err = n < 0 ? std::string("cannot write NMEA: ") + std::strerror(errno)
                    : std::string("NMEA output backed up: the port is not sending");
        return false;
    }

private:
    int m_fd;
    int m_bit;
    bool m_invert;
};

} // namespace

// ---- NMEA ---------------------------------------------------------------------

std::string nmeaChecksum(const std::string& body) {
    unsigned char x = 0;
    for (char ch : body) x ^= static_cast<unsigned char>(ch);
    char b[3];
    std::snprintf(b, sizeof b, "%02X", x);
    return b;
}

std::string nmeaSentence(const std::string& body) {
    return "$" + body + "*" + nmeaChecksum(body) + "\r\n";
}

std::string nmeaZda(const PpsLabel& t) {
    const Civil c = civilOf(t);
    char b[64];
    std::snprintf(b, sizeof b, "GPZDA,%s,%02u,%02u,%04d,00,00", hhmmss(c).c_str(), c.d, c.mo, c.y);
    return nmeaSentence(b);
}

std::string nmeaRmc(const std::optional<PpsLabel>& t, bool valid, const PpsPosition& pos) {
    std::string time, date;
    if (t) {
        const Civil c = civilOf(*t);
        time = hhmmss(c);
        date = two(static_cast<int>(c.d)) + two(static_cast<int>(c.mo)) + two(c.y % 100);
    }
    std::string lat, ns, lon, ew;
    if (pos.valid) {
        lat = nmeaAngle(pos.lat, 2);
        ns = pos.lat < 0.0 ? "S" : "N";
        lon = nmeaAngle(pos.lon, 3);
        ew = pos.lon < 0.0 ? "W" : "E";
    }
    // time, status, lat, N/S, lon, E/W, speed, course, date, magvar, E/W, mode.
    const std::string body = "GPRMC," + time + "," + (valid ? "A" : "V") + "," + lat + "," + ns + "," +
                             lon + "," + ew + "," + (valid ? "0.0" : "") + ",," + date + ",,," +
                             (valid ? "M" : "N");
    return nmeaSentence(body);
}

// ---- labels -------------------------------------------------------------------

PpsLabel PpsLabeler::next(long long served, bool leapPending, bool consecutive) {
    if (leapPending) m_leapAnnouncedAt = served;
    if (!m_have || !consecutive) {
        m_have = true;
        m_last = {served, false};
        return m_last;
    }
    const long long expected = m_last.leap ? m_last.posix : m_last.posix + 1;
    const bool armed = m_leapAnnouncedAt >= 0 && expected - m_leapAnnouncedAt <= kLeapArmedSec &&
                       floorMod(expected, 86400) == 0 && isLastDayOfMonth((expected - 1) * 1000);
    // The inserted second, whether the served clock still reads midnight here
    // or has already stepped back to 23:59:59.
    if (!m_last.leap && armed && (served == expected || served == expected - 1)) {
        m_last = {expected, true};
        m_leapLabelAt = expected;
        return m_last;
    }
    if (served == expected ||
        (m_leapLabelAt >= 0 && expected - m_leapLabelAt < kLeapCatchUpSec && served == expected + 1)) {
        m_last = {expected, false};
        return m_last;
    }
    ++m_resyncs;
    m_last = {served, false};
    return m_last;
}

double daemonAtUtc(const Combined& c, double utc) {
    return (utc - c.offsetSec + c.rate * c.atSec) / (1.0 + c.rate);
}

// ---- the port -------------------------------------------------------------------

std::unique_ptr<PpsPort> openSerialPps(const PpsConfig& cfg, std::string& err) {
    const int fd = ::open(cfg.device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
    if (fd < 0) {
        const int e = errno;
        err = cfg.device + ": " + std::strerror(e);
        if (e == ENOENT || e == ENODEV || e == ENXIO) {
            err = cfg.device + " is not present" +
                  " (in Docker, run ./restart.sh after plugging it in: the device is mapped when the"
                  " container is created)";
        } else if (e == EACCES || e == EPERM) {
            err = cfg.device + ": permission denied (the daemon's user needs the device's group, "
                               "usually dialout)";
        } else if (e == EBUSY) {
            err = cfg.device + " is in use by another program";
        }
        return nullptr;
    }
    if (!::isatty(fd)) {
        // EIO is a port the kernel lists with no UART behind it -- ttyS1..31
        // on most PCs -- which is a different mistake from naming a file.
        const int e = errno;
        ::close(fd);
        err = e == EIO ? cfg.device + " has no serial hardware behind it (Input/output error): "
                                      "the kernel lists the port but nothing answers"
                       : cfg.device + " is not a serial port";
        return nullptr;
    }
    // Ours alone: a second program opening it would get EBUSY rather than
    // interleave its own output with the NMEA or fight over the line.
    if (::ioctl(fd, TIOCEXCL) != 0) {
        err = cfg.device + ": cannot take it exclusively: " + std::strerror(errno);
        ::close(fd);
        return nullptr;
    }
    struct termios t{};
    if (::tcgetattr(fd, &t) != 0) {
        err = cfg.device + ": " + std::strerror(errno);
        ::close(fd);
        return nullptr;
    }
    ::cfmakeraw(&t);
    t.c_cflag &= ~(CSIZE | PARENB | CSTOPB | CRTSCTS | HUPCL);
    t.c_cflag |= CS8 | CLOCAL | CREAD;
    ::cfsetispeed(&t, baudConst(cfg.baud));
    ::cfsetospeed(&t, baudConst(cfg.baud));
    if (::tcsetattr(fd, TCSANOW, &t) != 0) {
        err = cfg.device + ": cannot set " + std::to_string(cfg.baud) + " 8N1: " + std::strerror(errno);
        ::close(fd);
        return nullptr;
    }
    auto port = std::make_unique<SerialPort>(fd, lineBit(cfg.line), cfg.invert);
    // At rest from the start: opening raised DTR and RTS both.
    if (!port->setLine(false, err)) {
        err = cfg.device + ": " + err + " (is it a real serial port? a pseudo-terminal has no " +
              cfg.line + " line)";
        return nullptr;
    }
    return port;
}

// ---- the output -------------------------------------------------------------------

PpsOutput::PpsOutput(PpsConfig cfg, ClockFn clock, PositionFn position, EventLog* events,
                     int realtimePriority, double startedAt, PortFn openPort)
    : m_cfg(std::move(cfg)), m_clock(std::move(clock)), m_position(std::move(position)),
      m_events(events), m_rtPriority(realtimePriority), m_startedAt(startedAt),
      m_openPort(std::move(openPort)) {
    m_stats.enabled = m_cfg.enabled;
    m_stats.device = m_cfg.device;
    m_stats.line = m_cfg.line;
    m_stats.widthMs = m_cfg.widthMs;
    m_stats.invert = m_cfg.invert;
    m_stats.baud = m_cfg.baud;
    m_stats.nmea = m_cfg.nmea;
}

PpsOutput::~PpsOutput() { stop(); }

void PpsOutput::start() {
    if (!m_cfg.enabled || m_running.exchange(true)) return;
    LOG_INFO(kTag, "1PPS output on %s (%s, %d ms%s), NMEA %s", m_cfg.device.c_str(),
             m_cfg.line == "rts" ? "RTS" : "DTR", m_cfg.widthMs, m_cfg.invert ? ", inverted" : "",
             m_cfg.nmea.empty() ? "off" : ([&] {
                 std::string s;
                 for (const auto& n : m_cfg.nmea) s += (s.empty() ? "" : "+") + n;
                 return s + " at " + std::to_string(m_cfg.baud) + " baud";
             }()).c_str());
    m_thread = std::thread([this] { run(); });
}

void PpsOutput::stop() {
    if (!m_running.exchange(false)) return;
    if (m_thread.joinable()) m_thread.join();
}

PpsStats PpsOutput::stats() const {
    std::lock_guard<std::mutex> lk(m_mu);
    return m_stats;
}

void PpsOutput::setState(const std::string& state, const std::string& detail, double unix) {
    std::string was;
    {
        std::lock_guard<std::mutex> lk(m_mu);
        if (m_stats.state == state && m_stats.detail == detail) return;
        was = m_stats.state;
        m_stats.state = state;
        m_stats.detail = detail;
    }
    // A log line and an event on a change of state only. The reason can change
    // every second while waiting (the Selector's note), and the page shows it.
    if (was == state) return;
    if (state == "pulsing") {
        LOG_INFO(kTag, "pulsing on %s", m_cfg.device.c_str());
    } else {
        LOG_WARN(kTag, "not pulsing: %s", detail.c_str());
    }
    if (!m_events) return;
    if (state == "pulsing" || was == "pulsing") {
        Event e;
        e.unix = unix;
        e.uptimeSec = monotonicNow() - m_startedAt;
        e.type = state == "pulsing" ? EventType::PpsPulsing : EventType::PpsStopped;
        e.source = "pps";
        e.message = state == "pulsing" ? "pulsing on " + m_cfg.device : detail;
        m_events->add(std::move(e));
    }
}

bool PpsOutput::sleepUntil(double daemonAt) {
    // In steps of at most a fifth of a second, so stop() is prompt.
    for (;;) {
        if (!m_running.load()) return false;
        const double left = daemonAt - daemonNow();
        if (left <= 0.0) return true;
        const double step = std::min(left, 0.2);
        const double monoAt = monotonicNow() + step;
        struct timespec ts{};
        ts.tv_sec = static_cast<time_t>(std::floor(monoAt));
        ts.tv_nsec = static_cast<long>((monoAt - std::floor(monoAt)) * 1e9);
        while (::clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, nullptr) == EINTR) {}
    }
}

void PpsOutput::run() {
    if (m_rtPriority > 0) {
        struct sched_param sp{};
        sp.sched_priority = m_rtPriority;
        const int rc = ::pthread_setschedparam(::pthread_self(), SCHED_FIFO, &sp);
        if (rc == 0) {
            LOG_INFO(kTag, "timing the pulse at real-time priority %d", m_rtPriority);
        } else {
            LOG_WARN(kTag, "cannot time the pulse at real-time priority %d (%s): it runs at normal "
                     "priority and will be later and less even under load",
                     m_rtPriority, std::strerror(rc));
        }
    }

    std::unique_ptr<PpsPort> port;
    double nextOpenAt = 0.0;
    double lastSlot = -1e9;        // daemon time of the last second acted on
    PpsLabeler labels;
    const bool wantRmc = std::find(m_cfg.nmea.begin(), m_cfg.nmea.end(), "rmc") != m_cfg.nmea.end();

    auto unixNow = [&](const Combined& c) {
        const double d = daemonNow();
        return c.valid ? c.utcAt(d) : d - daemonMinusRealtime();
    };

    while (m_running.load()) {
        const double now = daemonNow();
        Combined c = m_clock();

        if (!port && now >= nextOpenAt) {
            std::string err;
            port = m_openPort(m_cfg, err);
            if (!port) {
                nextOpenAt = now + kReopenSec;
                setState(err.find("not present") != std::string::npos ? "no device" : "error", err, unixNow(c));
            }
        }

        // The next second to act on: of the served clock when there is one,
        // of this host's clock otherwise (then only for an RMC saying V).
        long long second = 0;
        double at = 0.0;
        if (c.valid) {
            second = static_cast<long long>(std::floor(c.utcAt(std::max(now, lastSlot + 0.5)))) + 1;
            at = daemonAtUtc(c, static_cast<double>(second));
        } else {
            const double host = now - daemonMinusRealtime();
            at = now + (std::floor(host) + 1.0 - host);
        }

        if (!sleepUntil(at - kSpinSec)) break;
        // The clock may have moved while asleep: the instant again, from now.
        if (c.valid) {
            const Combined c2 = m_clock();
            if (c2.valid) { c = c2; at = daemonAtUtc(c, static_cast<double>(second)); }
        }
        while (m_running.load() && daemonNow() < at) {}
        if (!m_running.load()) break;

        const bool sync = c.valid && c.synchronised;
        bool pulsed = false;
        double latency = 0.0;
        if (port && sync) {
            if (daemonNow() - at > kMaxLateSec) {
                std::lock_guard<std::mutex> lk(m_mu);
                ++m_stats.skipped;
            } else {
                std::string err;
                if (port->setLine(true, err)) {
                    latency = daemonNow() - at;
                    pulsed = true;
                } else {
                    port.reset();
                    nextOpenAt = daemonNow() + kReopenSec;
                    setState("error", m_cfg.device + ": " + err, unixNow(c));
                }
            }
        }

        std::optional<PpsLabel> label;
        if (c.valid) {
            label = labels.next(second, c.leapPending, std::fabs(at - lastSlot - 1.0) < 0.1);
        } else {
            labels.reset();
        }
        lastSlot = at;

        if (port && !m_cfg.nmea.empty()) {
            PpsPosition pos;
            std::string from;
            if (wantRmc) pos = m_position(from);
            std::string out;
            for (const std::string& n : m_cfg.nmea) {
                if (n == "rmc") out += nmeaRmc(label, sync, pos);
                else if (n == "zda" && sync && label) out += nmeaZda(*label);
            }
            std::string err;
            if (!out.empty() && !port->write(out, err)) {
                port.reset();
                nextOpenAt = daemonNow() + kReopenSec;
                setState("error", m_cfg.device + ": " + err, unixNow(c));
            } else if (!out.empty()) {
                std::lock_guard<std::mutex> lk(m_mu);
                m_stats.sentences += static_cast<std::uint64_t>(std::count(out.begin(), out.end(), '$'));
                m_stats.positionFrom = pos.valid ? from : std::string();
            }
        }

        {
            std::lock_guard<std::mutex> lk(m_mu);
            m_stats.labelResyncs = labels.resyncs();
            if (pulsed) {
                ++m_stats.pulses;
                if (label) { m_stats.lastPulseUtc = static_cast<double>(label->posix); m_stats.lastWasLeap = label->leap; }
                m_latency[m_latencyNext] = latency * 1e6;
                m_latencyNext = (m_latencyNext + 1) % m_latency.size();
                m_latencyN = std::min(m_latencyN + 1, m_latency.size());
                double sum = 0.0, mx = 0.0;
                for (std::size_t i = 0; i < m_latencyN; ++i) { sum += m_latency[i]; mx = std::max(mx, m_latency[i]); }
                m_stats.lastLatencyUs = latency * 1e6;
                m_stats.meanLatencyUs = sum / static_cast<double>(m_latencyN);
                m_stats.maxLatencyUs = mx;
            }
        }
        if (pulsed) {
            setState("pulsing", "", unixNow(c));
        } else if (port && !sync) {
            setState("waiting", c.note.empty() ? "the daemon is not synchronised" : "not synchronised: " + c.note,
                     unixNow(c));
        }

        // The end of the pulse.
        if (pulsed) {
            if (!sleepUntil(at + m_cfg.widthMs / 1000.0)) { port.reset(); break; }
            std::string err;
            if (port && !port->setLine(false, err)) {
                port.reset();
                nextOpenAt = daemonNow() + kReopenSec;
                setState("error", m_cfg.device + ": " + err, unixNow(c));
            }
        }
    }
    port.reset();   // at rest, and closed
}

} // namespace ubersdr_ntp
