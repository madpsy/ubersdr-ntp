#pragma once

// What the daemon says about itself, in two renderings of one set of facts.
//
// The periodic log block is the primary interface: this runs in the background
// with no terminal, so the log file is how anyone sees what each source is
// doing. It is written as a fixed-width table rather than one line per field
// because the whole point is comparing sources against each other — which one
// has the tick, which one is anchored, which one disagrees — and a column does
// that in a way a paragraph cannot.
//
// The JSON is the same facts for the HTTP API and the page that renders it, and
// is generated from the same snapshots in the same pass so the two can never
// disagree about what the state was.

#include "NtpServer.h"
#include "Selector.h"
#include "Source.h"

#include <string>
#include <vector>

namespace ubersdr_ntp {

struct StatusInput {
    std::vector<SourceSnapshot> sources;
    Combined combined;
    NtpStats ntp;
    double uptimeSec = 0.0;
    std::string version;
    int ntpPort = 123;

    // The clock arrangement, so the report can say which class is primary and
    // what the other is doing without a second route to the configuration.
    // `secondaryKind` is derivable from `primaryKind` and carried anyway,
    // because every site that wants it wants the word rather than the
    // conditional.
    SourceKind primaryKind = SourceKind::Radio;
    SourceKind secondaryKind = SourceKind::Ntp;
    SecondaryMode secondaryMode = SecondaryMode::Standby;
    bool secondaryActive = true;
    std::string secondaryActiveReason;
};

// The multi-line block for the log. No trailing newline; Log::block indents it.
std::string renderStatusBlock(const StatusInput& in);

// The time itself, for clients that would rather ask over HTTP than speak NTP.
//
// Shaped so a client can do the same round-trip correction NTP does: it sends
// the time it asked (`clientUnixSec`, 0 if it did not say), and gets back when
// the request arrived and when the answer was sent, both on the corrected
// clock. offset = ((recv - orig) + (xmit - dest)) / 2 and delay =
// (dest - orig) - (xmit - recv), exactly as in RFC 5905 — so an HTTP client can
// reach the same answer as an NTP one, minus whatever TCP and TLS add to the
// asymmetry. A client that just wants the time reads `unix` and ignores the rest.
//
// `receiveDaemonSec` is when the request arrived, on the daemon clock
// (daemonNow()), taken before any of this ran.
std::string renderTimeJson(const Combined& c, double receiveDaemonSec,
                           double clientUnixSec, bool pretty);

// A one-line summary, for the state-change log lines and for a quick glance.
std::string renderStatusLine(const StatusInput& in);

// The JSON the HTTP API serves. Pretty-printed: it is read by people at least
// as often as by programs, and a few hundred bytes of whitespace is nothing
// against how much easier it is to read in a terminal.
std::string renderStatusJson(const StatusInput& in, bool pretty);

// Human-readable formatters, shared so the log and the page agree on how a
// duration or an offset reads.
std::string formatOffsetMs(double seconds);
std::string formatDuration(double seconds);

} // namespace ubersdr_ntp
