#pragma once

// The read-only HTTP service: a JSON API, a server-sent-event stream, and a
// page that renders both.
//
// Three audiences.
//
// A CLIENT that wants the time but does not speak NTP — a microcontroller, a
// script, a browser — asks GET /api/time. NTP over UDP remains the primary and
// the better interface: it is what disciplines a system clock, and TCP plus TLS
// make HTTP asymmetric in a way that costs accuracy. But an HTTP answer
// carrying the same receive/transmit pair lets such a client do the same
// round-trip correction, and one that merely wants to display the time does not
// care.
//
// A CLIENT THAT WANTS TO FOLLOW ALONG subscribes to GET /api/events, which
// emits one event per second on the CORRECTED second boundary — the instant the
// broadcast's own second rolls over, not this host's — carrying the time and a
// summary of every source. A display driven from that ticks with WWV rather
// than with the machine it is running on.
//
// A PERSON who wants to know why a source is not locking asks GET /api/status,
// or opens the page.
//
// A PERSON who wants to know what HAPPENED -- when it lost lock, when it
// failed over -- asks GET /api/eventlog: the last hundred transitions worth
// knowing about, and the list of types they come in. GET /api/metrics is the
// recent history of the figures the page draws charts for: an hour of minute
// averages, or a day of half-hour ones. Not /api/events, which
// was already the one-second stream; each tick carries `events_latest_id` so
// a follower knows when to ask again.
//
// Read-only without qualification: no route changes anything, no route takes a
// body, and anything but GET or HEAD is refused. That is what makes it safe to
// leave running, and why there is no authentication — there is nothing to
// authorise. It still binds to localhost by default, because the receiver
// passwords in this daemon's configuration are not served but its existence
// advertises which receivers it uses.
//
// WHY THIS IS NOT BUILT ON IXWebSocket's HttpServer
//
// That server's callback returns one complete HttpResponse and the connection
// is then closed, which SSE cannot be expressed in. A stream needs to hold the
// socket and keep writing to it for as long as the client is listening. So
// there is a small HTTP/1.1 server here instead — a few hundred lines for GET
// and HEAD on a fixed set of paths, which is the whole requirement.

#include "Events.h"
#include "Metrics.h"
#include "NtpServer.h"
#include "Selector.h"
#include "Source.h"
#include "Status.h"

#include <atomic>
#include <functional>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <thread>
#include <vector>

namespace ubersdr_ntp {

class HttpApi {
public:
    // Called per request rather than cached, so the page and the stream show
    // the state now and not the state at the last status tick.
    using StatusProvider = std::function<StatusInput()>;

    // `events` and `metrics` may be null, and /api/eventlog and /api/metrics
    // then serve empty lists.
    HttpApi(HttpConfig cfg, Selector& selector, StatusProvider provider,
            const EventLog* events = nullptr, const MetricHistory* metrics = nullptr);
    ~HttpApi();

    HttpApi(const HttpApi&) = delete;
    HttpApi& operator=(const HttpApi&) = delete;

    bool start(std::string& err);
    void stop();

    int streamClients() const { return m_streams.load(); }

private:
    void accept();
    void serveConnection(int fd);
    void serveEvents(int fd);

    HttpConfig m_cfg;
    Selector& m_selector;
    StatusProvider m_provider;
    const EventLog* m_events;
    const MetricHistory* m_metrics;

    int m_listenFd = -1;
    std::thread m_acceptor;
    std::atomic<bool> m_running{false};
    std::atomic<int> m_streams{0};
    std::atomic<int> m_connections{0};

    // Connection threads are detached -- one per request, and a request can be
    // an SSE stream that lives for hours, so joining them individually is not
    // the shape of the problem. But they call m_provider() and m_selector, and
    // this object is destroyed when the daemon shuts down: a stream still
    // inside serveEvents would then be reading a Selector that no longer
    // exists. So their sockets are tracked, stop() shuts every one of them
    // down to break them out of poll/send, and then waits for the count to
    // reach zero before returning.
    mutable std::mutex m_fdMu;
    std::set<int> m_liveFds;
};

} // namespace ubersdr_ntp
