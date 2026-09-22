#pragma once

// What one source reports about itself, whichever kind it is.
//
// One struct for both kinds rather than two, because everything above a source
// — the Selector, the status block, the JSON, the page — wants to lay them side
// by side and ask the same questions of each: is it usable, what does it think
// the offset is, how well does it know that, and if it is not usable, why not.
// A variant would make every one of those sites branch on the kind to ask a
// question that has one answer.
//
// Most of the fields ARE kind-specific. The radio ones are inline, because
// they were here first and every reader of them already branches on having a
// receiver; the upstream-peer ones are gathered in `ntp`, which is meaningful
// only when `kind` says so and is left at its defaults otherwise.
//
// The fields that matter most are the ones BOTH kinds fill in: `ready` and
// `notReadyReason`, `offsetSec` and the uncertainties around it. Everything the
// Selector reads is in that set, which is what keeps it from having to know
// what a tick or a reach register is.

#include "Config.h"
#include "Propagation.h"

#include <cstdint>
#include <limits>
#include <string>

namespace ubersdr_ntp {

enum class LinkState { Idle, Connecting, Streaming, Backoff, Stopped };
const char* linkStateName(LinkState s);

// What only an upstream NTP peer has. Meaningful when kind == SourceKind::Ntp.
//
// Most of it comes straight out of the reply packet, and is carried through
// rather than folded away because it is the only account the server gives of
// its own quality — a client that reads the offset and discards the stratum and
// the root distance is trusting a machine it has never checked.
struct NtpPeerInfo {
    std::string server;          // as configured
    std::string address;         // what it resolved to, as text, with the port
    std::string addressHost;     // ...and without it, which is what a refid is
    int port = 123;

    // How long the name's answer may be kept, as DNS said when it was last
    // looked up, and how long until it is looked up again. -1 when the server
    // is an address literal (there is nothing to look up) or the resolver
    // gave no TTL (the hourly fallback is used). See NtpPeer::connectPeer.
    int dnsTtlSec = -1;
    double nextResolveInSec = -1.0;

    // Straight from the last accepted reply.
    int stratum = 0;
    std::string refid;           // four ASCII characters, or a dotted quad
    int leap = 0;                // 0 none, 1 add, 2 delete, 3 unsynchronised
    double rootDelaySec = 0.0;
    double rootDispersionSec = 0.0;
    double serverPrecisionSec = 0.0;

    // Measured here.
    double pollSec = 0.0;        // what it is being polled at NOW, which backoff changes
    double delaySec = 0.0;       // round trip of the sample the filter chose
    double filterJitterSec = 0.0;// spread of the filter's other samples about it
    double rootDistanceSec = 0.0;// (rootDelay + delay)/2 + rootDispersion: the honest total

    // Reachability, as NTP's own eight-bit shift register: one bit per poll,
    // newest at the top, set when a reply came back. 0 is unreachable, 0xFF is
    // eight for eight. Shown as an octal byte the way every other NTP
    // implementation shows it, because that is what people know how to read.
    std::uint8_t reach = 0;
    double lastReplyAgeSec = 1e9;

    std::uint64_t sent = 0;
    std::uint64_t received = 0;
    std::uint64_t rejected = 0;
    // Replies that were valid but arrived too late behind the best round trip
    // in the filter to be worth timing from. Not a fault: it is the queueing
    // the filter exists to drop, and the count is how busy the path is.
    std::uint64_t spikes = 0;
    std::string lastRejectReason;    // why the newest refused reply was refused

    // What this peer is being judged against, carried alongside what it
    // measured so a reader can see a figure and its limit together rather than
    // having to hold the configuration in their head.
    double maxRootDistanceSec = 0.0;
    int maxStratum = 0;
    double configuredPollSec = 0.0;   // before any kiss-o'-death backoff

    // The four bytes to put in THIS server's reference identifier when the
    // served time comes from this peer. RFC 5905 requires the upstream's IPv4
    // address there above stratum 1, which is also how every implementation's
    // loop detection recognises itself; for an IPv6 upstream the RFC specifies
    // the first four octets of an MD5 of the address, and this uses a plain
    // 32-bit hash instead. Nothing interoperable depends on the exact function
    // -- it is a 32-bit value that identifies a peer and is not an IPv4 address
    // -- but it does mean a v6 upstream will not recognise its own refid coming
    // back, which is a limitation shared with most implementations and is why
    // the loop check here also compares against our own addresses directly.
    std::uint32_t addressRefid = 0;

    // A kiss-o'-death code the server sent. Non-empty means it asked to be left
    // alone, and DENY or RSTR means permanently: see NtpClient.cpp.
    std::string kissCode;
    bool stopped = false;            // ...and this peer will not be polled again
};

// Everything one source knows about itself, copied out under lock.
struct SourceSnapshot {
    std::string name;
    std::string url;
    bool enabled = true;

    // Which implementation is behind this, and therefore which of the two
    // groups below means anything.
    SourceKind kind = SourceKind::Radio;

    // Whether this source is in the class that is trusted first. Set by main()
    // from clock.primary, not by the source, which has no opinion about it.
    bool primaryClass = true;

    // Whether it is connected at all. False in cold secondary mode while the
    // primary is healthy; `activeReason` says which decision put it there, so
    // an idle source does not look like a broken one.
    bool active = true;
    std::string activeReason;

    // Whether this source's measurements are worth considering, and if not,
    // why not in words meant for someone reading the status page. The whole of
    // what the Selector needs to decide candidacy: a radio source sets it from
    // the decoder's lock, a peer from its reach register, and neither of those
    // concepts has to leave the source.
    bool ready = false;
    std::string notReadyReason;

    // --- radio ------------------------------------------------------------
    std::uint64_t carrierHz = 0;
    std::uint64_t dialHz = 0;
    AudioFormat format = AudioFormat::Opus;

    // link
    LinkState link = LinkState::Idle;
    std::string linkDetail;
    double linkAgeSec = 0.0;        // how long in this link state
    double lastAudioAgeSec = 1e9;   // since the last audio packet
    std::uint64_t packets = 0;
    std::uint64_t audioBytes = 0;
    std::uint64_t decodeErrors = 0;
    int connectAttempts = 0;
    int reacquisitions = 0;         // times the consensus sent it back to start over
    double httpRttMs = 0.0;
    std::string receiverName;       // from /api/description
    GeoPoint receiverLocation;

    // audio
    int sampleRate = 0;
    double basebandPowerDb = -999.0;
    double noiseDb = -999.0;

    // decoder
    std::string clockState = "stopped";
    std::string station = "unknown";
    double toneSnrDb = 0.0;
    double delayEstMs = 0.0;
    // WWV/WWVH: tick energy in the 2000 Hz band over the 2200 Hz band, in dB --
    // what the station tag is decided from. Positive leans WWV, negative WWVH;
    // NaN when there is none (WWVB, or no tick yet).
    double tickBandRatioDb = std::numeric_limits<double>::quiet_NaN();
    bool toneDetected = false;
    bool phaseLocked = false;
    bool anchored = false;
    int badFrameStreak = 0;
    int framesInWindow = 0;
    int windowSize = 0;
    double voteQuality = 0.0;
    std::string refusal = "none";
    std::int64_t samplesConsumed = 0;
    int lastQuality = 0;            // voter confidence of the last `time`, 0..100
    std::string lastDecodedUtc;
    double lastTimeAgeSec = 1e9;
    bool leapPending = false;
    int dut1Tenths = 0;

    // DCF77's two demodulators (Dcf77Decoder.h); false / NaN / empty for WWV
    // and WWVB. timingFromPm says whose edges the offset is being taken from,
    // amMinusPmMs is the one check the AM timing has, and frameFrom which of
    // the two the last minute's time was read from: "am", "pm", "both", or
    // "conflict" when both read a valid time and they were not the same.
    bool pmLocked = false;
    double pmSnrDb = std::numeric_limits<double>::quiet_NaN();
    bool timingFromPm = false;
    double amMinusPmMs = std::numeric_limits<double>::quiet_NaN();
    double carrierOffsetHz = std::numeric_limits<double>::quiet_NaN();
    std::string frameFrom;
    int pmRefusedLocks = 0;
    bool pmInterference = false;

    // The continuity check on decoded times (Source.cpp, admitDecodedTime):
    // UTC does not jump, so a decoded time that does has been misread. Empty
    // while every decode agrees with the source's own history; otherwise what
    // the check is holding out and until when, in words for the status page.
    std::string timeCheck;
    std::uint64_t timeRejections = 0;   // decoded times refused, ever
    double lastRejectedJumpSec = 0.0;   // ...the newest, against the history
    std::string lastRejectedUtc;        // ...and the time it claimed
    int timeAdoptions = 0;              // a new time held long enough to take over

    // timing
    bool haveOffset = false;
    // UTC minus the DAEMON clock (SampleClock.h) at offsetAtSec, delay model
    // applied, moving at offsetRate. What the selector combines.
    double offsetSec = 0.0;
    double offsetAtSec = 0.0;
    double offsetRate = 0.0;             // d(offsetSec)/d(daemon time); 1e-6 is 1 ppm
    double offsetRateUncertainty = 0.0;
    bool offsetRateMeasured = false;     // false: zero assumed, uncertainty is the bound
    double offsetRateSpanSec = 0.0;
    double rateTermSec = 0.0;            // what the rate's doubt could move the offset by
    // The same offset against THIS HOST's clock, now, and before the delay model.
    // For people and the status page; nothing is formed from them.
    double hostOffsetSec = 0.0;
    double rawOffsetSec = 0.0;
    double jitterSec = 0.0;         // spread of the measurements in the window
    double dispersionSec = 0.0;     // jitter + fit residual + delay uncertainty
    // What this source is worth RELATIVE TO THE OTHERS, which is not the same
    // number. dispersionSec is dominated by the delay model's uncertainty, and
    // every source shares that model: two receivers on similar paths both carry
    // about 16 ms of it, so a source with seventeen times another's jitter still
    // ends up only a third wider overall and keeps a third of the vote. The
    // common term belongs in what the server ADVERTISES -- it is real, and the
    // answer really is that uncertain -- but not in deciding which source to
    // believe. This is the part that actually distinguishes them.
    double weightDispersionSec = 0.0;
    double offsetAgeSec = 1e9;
    // The oldest offsetAgeSec the Selector will still take, or 0 for its own
    // default. A radio source measures every second and leaves this alone; an
    // NTP peer measures once a poll, and a limit that is under three polls
    // would drop it for two lost packets. See NtpPeer::recompute.
    double maxOffsetAgeSec = 0.0;
    int offsetSamples = 0;

    // delay model
    double delaySec = 0.0;
    double propagationSec = 0.0;
    double networkSec = 0.0;
    double codecSec = 0.0;
    double decoderSec = 0.0;   // the running decoder's edge bias (negative: early)
    double chainSec = 0.0;     // UberSDR's fixed RF-to-WebSocket delay
    double extraSec = 0.0;
    std::string pathDescription;

    // sample clock
    bool clockFitValid = false;
    double clockResidualSec = 0.0;
    double clockSpanSec = 0.0;
    double clockPpm = 0.0;          // receiver sample clock against ours
    double wsRttMs = 0.0;           // round trip over the audio connection
    bool rttFromWs = false;         // ...and whether the delay model used it
    double clockSlopeUncSec = 0.0;  // bias the slope could be putting on an edge
    bool clockSlopeHeld = false;    // fitted slope refused as implausible
    double lastExcessDelaySec = 0.0;

    // --- upstream NTP peer ------------------------------------------------
    NtpPeerInfo ntp;

    double weight = 1.0;
};


} // namespace ubersdr_ntp
