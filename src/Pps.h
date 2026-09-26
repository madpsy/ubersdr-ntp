// Pps.h — an optional one-pulse-per-second output on a serial port, with NMEA.
//
// Off unless the configuration turns it on (PpsConfig). When on, a thread of its
// own raises a modem-control line -- DTR by default, or RTS -- at the start of
// every UTC second of the SERVED time, holds it for width_ms and drops it. With
// NMEA asked for, it also writes the sentences naming that second on the port's
// TX, straight after the pulse: the pairing chrony, gpsd, ntpd's NMEA driver and
// most lab equipment expect from a GPS receiver.
//
// It pulses only while the daemon is synchronised, which is when NTP answers as
// synchronised too. A PPS that is confidently wrong is worse than none, so there
// is no holdover of its own: while unsynchronised the line stays at rest, RMC is
// sent with status V as a GPS receiver without a fix sends it, and ZDA -- which
// has no validity field to say so -- is not sent at all.
//
// NMEA 0183: 8N1 at 4800 baud unless set otherwise, "$GP" talker, the checksum
// and CR LF on every sentence, RMC in its v2.3 form with the mode indicator.
// A leap second is sent as 23:59:60 (PpsLabeler).
//
// The pulse is timed in software: the thread sleeps to within two milliseconds
// of the second and spins the rest, at real-time priority where the host allows
// it. Each pulse's lateness is measured -- from the instant due to the return of
// the ioctl that set the line -- and reported, because it depends on the port:
// tens of microseconds on a motherboard or PCIe UART, up to a millisecond on USB.
#pragma once

#include "Config.h"
#include "Selector.h"
#include "SourceSnapshot.h"

#include <array>
#include <atomic>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

namespace ubersdr_ntp {

class EventLog;

// ---- NMEA 0183 --------------------------------------------------------------

// The XOR of every character between '$' and '*', as two upper-case hex digits.
std::string nmeaChecksum(const std::string& body);
// "$" + body + "*" + checksum + "\r\n". `body` starts with the talker and type.
std::string nmeaSentence(const std::string& body);

// A second to be named: the UTC second a pulse starts, as POSIX seconds, or the
// inserted 23:59:60 before POSIX second `posix` (a midnight) when leap is set.
struct PpsLabel {
    long long posix = 0;
    bool leap = false;
};

// Where the station is, for RMC. Empty fields when not known.
struct PpsPosition {
    bool valid = false;
    double lat = 0.0, lon = 0.0;
};

// $GPZDA: time, day, month, year, and a local zone of 00,00.
std::string nmeaZda(const PpsLabel& t);
// $GPRMC, v2.3: status A with mode M (a fixed position, entered rather than
// measured) when `valid`, status V with mode N when not; speed 0.0, course and
// magnetic variation empty. With no label at all (`t` absent), every field that
// needs the time is empty too, as a receiver that has never had a fix sends it.
std::string nmeaRmc(const std::optional<PpsLabel>& t, bool valid, const PpsPosition& pos);

// ---- naming the seconds ------------------------------------------------------

// Names each pulse's second by counting on from the last, and checks the count
// against the served clock rather than trusting either alone.
//
// Counting is what makes 23:59:60 possible: the served time is POSIX-like, with
// no second to spare at midnight, so the second after 23:59:59 on the last day
// of a month with a leap second announced is named 23:59:60 however the served
// clock reads it. After it, the served clock is allowed to read one second
// ahead of the count for up to an hour, while the sources re-anchor and the
// served offset steps back; any other disagreement is taken to be a real step
// of the clock and the count starts again from what the clock says.
class PpsLabeler {
public:
    // `served` is the served clock's own reading of the pulse's second;
    // `leapPending` whether a leap second is announced now; `consecutive`
    // whether this pulse is one second after the last one labelled.
    PpsLabel next(long long served, bool leapPending, bool consecutive);
    void reset() { m_have = false; }
    int resyncs() const { return m_resyncs; }

private:
    bool m_have = false;
    PpsLabel m_last;
    long long m_leapAnnouncedAt = -1;   // served second a warning was last seen
    long long m_leapLabelAt = -1;       // the midnight a 23:59:60 was sent before
    int m_resyncs = 0;
};

// The daemon-clock instant at which the served clock `c` reads UTC `utc`.
double daemonAtUtc(const Combined& c, double utc);

// ---- the port ---------------------------------------------------------------

// What the output needs of a serial port: open, set the pulse line, write.
// Abstract so the tests can drive the thread without hardware.
class PpsPort {
public:
    virtual ~PpsPort() = default;
    // Sets the pulse line's level: true is asserted (before `invert`).
    virtual bool setLine(bool on, std::string& err) = 0;
    virtual bool write(const std::string& data, std::string& err) = 0;
};

// A real serial port: opened exclusively, raw 8N1 at the configured baud, no
// flow control (the pulse line is ours), and the line put at rest at once --
// Linux raises DTR and RTS on open.
std::unique_ptr<PpsPort> openSerialPps(const PpsConfig& cfg, std::string& err);

// ---- the output ---------------------------------------------------------------

struct PpsStats {
    bool enabled = false;
    std::string device;
    std::string line;           // "dtr" or "rts"
    int widthMs = 0;
    bool invert = false;
    int baud = 0;
    std::vector<std::string> nmea;

    // "off", "pulsing", "waiting" (not synchronised), "no device" or "error".
    std::string state = "off";
    std::string detail;         // why, in words, when not pulsing
    std::uint64_t pulses = 0;
    std::uint64_t sentences = 0;
    std::uint64_t skipped = 0;  // seconds the thread woke too late to pulse on time
    int labelResyncs = 0;
    double lastPulseUtc = 0.0;  // the second named, POSIX; 0 before the first
    bool lastWasLeap = false;
    // Lateness of the line, over the last minute of pulses.
    double lastLatencyUs = 0.0, meanLatencyUs = 0.0, maxLatencyUs = 0.0;
    std::string positionFrom;   // "config", "receiver <name>", or empty
};

class PpsOutput {
public:
    using ClockFn = std::function<Combined()>;
    using PositionFn = std::function<PpsPosition(std::string& from)>;
    using PortFn = std::function<std::unique_ptr<PpsPort>(const PpsConfig&, std::string& err)>;

    // `events` may be null. `realtimePriority` 0 runs at normal priority.
    // `startedAt` is the daemon's start on monotonicNow(), for event uptimes.
    PpsOutput(PpsConfig cfg, ClockFn clock, PositionFn position, EventLog* events,
              int realtimePriority, double startedAt, PortFn openPort = openSerialPps);
    ~PpsOutput();

    void start();
    void stop();
    PpsStats stats() const;

private:
    void run();
    void setState(const std::string& state, const std::string& detail, double unix);
    bool sleepUntil(double daemonAt);   // false when stopping

    PpsConfig m_cfg;
    ClockFn m_clock;
    PositionFn m_position;
    EventLog* m_events;
    int m_rtPriority;
    double m_startedAt;
    PortFn m_openPort;

    std::atomic<bool> m_running{false};
    std::thread m_thread;

    mutable std::mutex m_mu;
    PpsStats m_stats;
    std::array<double, 60> m_latency{};
    std::size_t m_latencyN = 0, m_latencyNext = 0;
};

} // namespace ubersdr_ntp
