// NmeaServer.h — NMEA 0183 over TCP, for gpsd and anything else that reads a GPS.
//
// The same sentences the serial 1PPS output writes (Pps.h), sent to every
// connected client at the start of each second of the SERVED time: RMC and ZDA
// by default, naming that second. It is what a networked GPS receiver offers,
// and gpsd takes it as one -- "gpsd tcp://host:10110" -- and hands the time on
// to chrony or ntpd through its usual SHM or socket, on whatever host gpsd
// runs. In Docker that makes it the one way to reach a local time daemon as a
// reference clock without sharing the host's IPC namespace or a socket
// directory: a published port is enough.
//
// Only the arrival of a sentence carries the instant, so each is sent as close
// to the second as the thread can make it -- at real-time priority where the
// host allows it, with the timer slack taken out -- and the lateness is
// measured and reported. It is microseconds on the loopback and a LAN's own
// jitter across one; gpsd treats NMEA time without a PPS line as coarse
// either way, and a constant `offset` in chrony.conf takes out the rest.
//
// The rules are the serial output's: while the daemon is unsynchronised RMC is
// sent with status V, as a GPS receiver without a fix sends it, and ZDA -- which
// has no validity field to say so -- is not sent at all.
//
// Read-only: anything a client sends is read and discarded. A client that
// stops reading is dropped once its small send buffer fills, rather than let
// its backlog grow; the cap on clients is for the same reason.
#pragma once

#include "Config.h"
#include "Pps.h"
#include "Selector.h"

#include <array>
#include <atomic>
#include <cstdint>
#include <functional>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace ubersdr_ntp {

struct NmeaClientInfo {
    std::string address;
    double connectedSec = 0.0;   // how long ago, at the time of the snapshot
};

struct NmeaTcpStats {
    bool enabled = false;
    int port = 0;
    std::vector<std::string> listen;       // the addresses actually bound
    std::vector<std::string> sentences;
    int maxClients = 0;

    // "serving" (synchronised, status A), "waiting" (unsynchronised: RMC V
    // only), "error" (nothing could be bound) or "off".
    std::string state = "off";
    std::string detail;
    std::vector<NmeaClientInfo> clients;
    std::uint64_t connections = 0;   // accepted since start
    std::uint64_t refused = 0;       // turned away at the cap
    std::uint64_t dropped = 0;       // closed for not reading
    std::uint64_t sent = 0;          // sentences delivered, over all clients
    // Lateness of the first send each second, over the last minute of them.
    double lastLateUs = 0.0, meanLateUs = 0.0, maxLateUs = 0.0;
    std::string positionFrom;
};

class NmeaServer {
public:
    using ClockFn = std::function<Combined()>;
    using PositionFn = PpsOutput::PositionFn;

    // `realtimePriority` 0 runs at normal priority.
    NmeaServer(NmeaTcpConfig cfg, ClockFn clock, PositionFn position, int realtimePriority);
    ~NmeaServer();

    NmeaServer(const NmeaServer&) = delete;
    NmeaServer& operator=(const NmeaServer&) = delete;

    // Binds every configured address and starts the thread. False with `err`
    // set when none could be bound; the caller carries on without it, as it
    // does without the status page.
    bool start(std::string& err);
    void stop();
    NmeaTcpStats stats() const;
    // The port actually bound: the configured one, or the kernel's choice for 0.
    int boundPort() const { return m_boundPort; }

private:
    struct Client {
        int fd;
        std::string address;
        double since;   // monotonicNow()
    };

    void run();
    void acceptFrom(int listenFd);
    void closeClient(std::size_t i);

    NmeaTcpConfig m_cfg;
    ClockFn m_clock;
    PositionFn m_position;
    int m_rtPriority;
    int m_boundPort = 0;

    std::vector<int> m_listenFds;
    std::vector<Client> m_clients;   // the thread's own; copied into m_stats
    std::atomic<bool> m_running{false};
    std::thread m_thread;

    mutable std::mutex m_mu;
    NmeaTcpStats m_stats;
    std::array<double, 60> m_late{};
    std::size_t m_lateN = 0, m_lateNext = 0;
};

} // namespace ubersdr_ntp
