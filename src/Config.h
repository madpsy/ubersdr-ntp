#pragma once

// Configuration: a JSON file, with a handful of command-line overrides for the
// things you change while trying something out.
//
// THE DELAY MODEL
//
// Everything this daemon can do well is decided here, so it is worth setting
// out. The decoder says what UTC the transmitter was sending. NTP needs to know
// what UTC it is HERE, now. Between the two sits a one-way delay that nothing
// in the stream measures:
//
//   propagation   the RF path from Fort Collins or Kauai. 10-30 ms, and it is
//                 longer than the great-circle distance because the signal
//                 goes up to the ionosphere and back down, once per hop.
//   receiver      radiod's buffering and the multicast hop inside the server.
//   codec         Opus adds a measured, constant ~10 ms at 12 kHz. Lossless
//                 PCM v4 adds none.
//   network       the one-way trip from the UberSDR server to this host.
//
// All four are biases, not noise, and the filtering in SampleClock cannot see
// any of them — a constant delay is indistinguishable from a clock that is
// simply wrong, which is precisely the thing being measured. So they are
// modelled explicitly and the residual uncertainty is reported as dispersion.
//
// Auto-estimation covers what is measurable: propagation from the receiver's
// own coordinates (the server publishes them at /api/description) against the
// transmitter the decoder reports, the codec's known constant, and half the
// HTTP round-trip to the server. The receiver's internal delay is the same on
// every UberSDR instance, so it is a built-in constant (measured live), as is
// the WWV decoder's own edge bias; see Source.cpp. What is left, if anything,
// is `extra_delay_ms` — for a source with a reason of its own to disagree with
// a reference clock. It defaults to 0.
//
// Getting this wrong does not break anything; it biases the served time by
// exactly the amount you got it wrong by. Leaving it at zero biases the served
// time by the whole delay, which on a typical path is 50-150 ms.
//
// TWO CLASSES OF SOURCE
//
// There are now two kinds of reference: the radio sources above, and upstream
// NTP servers. They are not interchangeable and the difference is not about
// quality -- a public NTP server is very likely more accurate than this
// daemon's own radio estimate. It is about what they FAIL TO. A radio source
// fails when the band is dead, which is a nightly event and correlated across
// receivers hearing the same transmitter. An NTP source fails when the network
// is down, which is a different event with a different cause. Neither covers
// the other, and a clock meant to run unattended for months wants both.
//
// So one class is the PRIMARY and the other the SECONDARY, and the secondary's
// mode says what it does while the primary is healthy: contribute alongside it,
// stand by warm, or stay cold. See ClockConfig.

#include <cstdint>
#include <string>
#include <vector>

#include "Log.h"
#include "Version.h"

namespace ubersdr_ntp {

enum class AudioFormat {
    Opus,      // format=opus  — the default. Lossy, ~24 kbps, adds a constant delay.
    PcmV4,     // format=pcm-zstd&version=4 — the predictive lossless codec.
               // The query parameter is still spelt "pcm-zstd" for compatibility;
               // version 4 carries no zstd at all.
};

const char* formatName(AudioFormat f);
bool parseFormat(const std::string& s, AudioFormat& out);

struct SourceConfig {
    std::string name;          // what it is called in the log; must be unique
    std::string url;           // http:// or https:// base, e.g. https://sdr.example.org
    std::string password;      // bypass password, if the receiver has one
    bool enabled = true;

    // The transmitter's carrier. The dial is derived as carrier - 1 kHz, which
    // is the tuning WWV/WWVH and WWVB all want: it puts the carrier at 1000 Hz
    // audio in USB.
    std::uint64_t carrierHz = 10000000;
    // An explicit dial, for a receiver that needs one. 0 means derive it.
    std::uint64_t dialHz = 0;

    AudioFormat format = AudioFormat::Opus;

    // Delay model — see the file comment.
    bool autoDelay = true;
    double delayMs = 0.0;        // explicit total; used verbatim when autoDelay is false
    double extraDelayMs = 0.0;   // added on top of the auto estimate

    // Sources whose signal is reliably worse can be kept but weighted down, or
    // held as a standby that only counts when nothing better has a lock.
    double weight = 1.0;

    bool verifyTls = true;
};

// Which kind of reference a source is. The two classes fail to different
// things, which is the entire reason for having both; see the file comment.
enum class SourceKind { Radio, Ntp };
const char* sourceKindName(SourceKind k);
bool parseSourceKind(const std::string& s, SourceKind& out);

// One upstream NTP server, polled as a client.
//
// This is an ordinary NTPv4 client: mode 3 out, mode 4 back, offset and delay
// from the four timestamps exactly as RFC 5905 defines them. Two details are
// not ordinary and both matter.
//
// The transmit timestamp is a RANDOM NONCE rather than the clock, and the reply
// is refused unless its originate field returns those eight bytes verbatim. A
// real client's transmit timestamp is a value an off-path attacker can guess to
// within the poll interval, and guessing it is the whole of the work in forging
// a reply; sixty-four random bits are not guessable, so a forgery has to be on
// the path. chrony does the same thing for the same reason. The cost is that
// the reply's originate field is no longer this host's clock at transmission --
// which nothing here needed, because t1 is recorded locally.
//
// And the timestamps are taken on the DAEMON clock (SampleClock.h), not the
// host clock, so the offset this yields is "UTC minus the daemon clock" -- the
// same quantity a radio source measures, in the same units, against the same
// oscillator. That is what lets one Selector combine them without a conversion
// step that could go stale between the two reads.
struct NtpSourceConfig {
    std::string name;          // what it is called in the log; must be unique
    std::string server;        // hostname or literal address
    int port = 123;
    bool enabled = true;

    // Seconds between polls. 64 is NTP's own default and what a public server
    // expects; anything under 16 is rude to a server you do not own, and the
    // pool's terms of service say so. The estimator windows scale off this, so
    // changing it does not need anything else changed with it.
    double pollSeconds = 64.0;

    // Send a short burst on the first poll, and whenever the peer is brought up
    // from cold. One sample is not a measurement -- the clock filter wants a
    // few to pick a minimum-delay one out of -- and a fallback source that
    // takes six minutes to become usable is a fallback that arrives after the
    // outage it was for.
    bool iburst = true;

    // Relative confidence in the combine, as for a radio source.
    double weight = 1.0;

    // Refuse a server whose own root distance is worse than this: it is telling
    // you, in the packet, how well it knows the time, and past some figure the
    // answer is not worth having. 500 ms is loose enough for a busy pool server
    // on a long path and tight enough to drop one that has lost its own sync.
    double maxRootDistanceMs = 500.0;

    // The furthest up the tree this will follow. Stratum 16 is unsynchronised
    // and always refused; the limit here is about how much accumulated path to
    // accept, not about validity.
    int maxStratum = 15;

    // Anything the round-trip halving cannot see -- a path whose two directions
    // are known to be of different lengths. Positive means the reply takes
    // LONGER than the request, which is what a satellite downlink or an
    // asymmetric DSL line does. Leave at 0 unless you have measured it.
    double extraDelayMs = 0.0;
};

struct NtpConfig {
    std::vector<std::string> listen{"0.0.0.0", "::"};
    int port = 123;

    // How long to keep answering after the last source loses lock, and how
    // fast the claimed accuracy decays while doing so. 15 ppm is NTP's own PHI
    // — the rate at which an undisciplined clock is assumed to wander.
    double coastSeconds = 3600.0;
    double coastDriftPpm = 15.0;

    // Refuse to answer at all until this many sources agree. 1 is right for a
    // single-receiver install; 2 buys protection against one receiver decoding
    // a coherent misread, which is the failure mode the clock decoder's own
    // plausibility gate exists for.
    int minSources = 1;

    // Answer with LI=3 / stratum 0 when unsynchronised, rather than falling
    // silent. A client learns to look elsewhere immediately from the first and
    // only on timeout from the second.
    bool answerWhenUnsynchronised = true;

    // Ignore the broadcast leap-second warning bit. It is decoded correctly,
    // but a single misread frame asserting it would tell every client on the
    // network that a leap second is coming.
    bool honourLeapWarning = true;

    // Per-client responses per second, 0 to disable. NTP on UDP is a reflection
    // amplifier if left open; the response is the same size as the request so
    // the gain is 1, but a limit still caps what this can be aimed at.
    double rateLimitPerClient = 10.0;
};

// What the secondary class does while the primary is healthy.
//
// The three differ in two independent things -- whether the secondary is
// CONNECTED, and whether it CONTRIBUTES -- and only three of the four
// combinations are useful (contributing while disconnected is not a thing).
//
//   Always    connected, and contributing. Both classes go into one
//             intersection and one weighted mean. The most accurate
//             arrangement, and the one that makes a disagreement between the
//             two visible as it happens rather than at the moment of failover.
//
//   Standby   connected, not contributing. The served time comes from the
//             primary alone, but the secondary is measured continuously: it is
//             warm, so failover costs nothing, it is being checked, so a broken
//             standby is known about BEFORE it is needed, and the difference
//             between the two classes is measured all the while -- which is the
//             only absolute reference this daemon has ever had for the constant
//             every radio source shares.
//
//   Cold      neither. Nothing is connected until the primary fails. This is
//             the only mode that costs a remote receiver nothing while it is
//             not needed -- public UberSDR receivers cap concurrent sessions
//             per address, usually at two -- and the only one where failover is
//             slow, because a radio source needs about five minutes to acquire
//             a lock from nothing.
enum class SecondaryMode { Always, Standby, Cold };
const char* secondaryModeName(SecondaryMode m);
bool parseSecondaryMode(const std::string& s, SecondaryMode& out);

// Which class is trusted first, and what the other one does about it.
struct ClockConfig {
    // Radio is the default because this is a radio clock: a stratum-1 reference
    // whose whole point is not depending on the network. Set it to Ntp on a
    // machine where the network is the reliable part and the radio is the
    // interesting part.
    SourceKind primary = SourceKind::Radio;
    SecondaryMode secondary = SecondaryMode::Standby;

    // How long the primary must have had nothing usable before the secondary
    // takes over, and how long it must be healthy again before it takes back.
    //
    // The two are deliberately different. Failing over is cheap and failing
    // over late is expensive -- a minute is about how long a fade the daemon
    // can coast through without anyone caring. Failing BACK is the other way
    // round: it costs a step in the served time for no gain if the primary is
    // about to drop out again, and a decoder coming out of a fade re-locks and
    // loses it repeatedly for several minutes. So it has to stay healthy for a
    // while before it is believed.
    double failoverAfterSec = 60.0;
    double failbackAfterSec = 300.0;

    // Sources that must agree within the SECONDARY class before it may serve.
    // Separate from ntp.min_sources, which governs the primary: an install with
    // two receivers and one NTP server wants 2 and 1, and one number cannot be
    // both.
    int minSecondarySources = 1;
};

// The read-only status service: a JSON API and a small page that renders it.
//
// Read-only without qualification — there is no route that changes anything
// and no authentication because there is nothing to authorise. It binds every
// interface by default, like the NTP service itself; no route serves the
// receiver passwords out of the configuration, but it does name the receivers
// this daemon uses, so set "127.0.0.1" if that should stay on this machine.
struct HttpConfig {
    bool enabled = true;
    std::string listen = "0.0.0.0";
    int port = 1234;
};

struct LogConfig {
    std::string file;                  // empty: no file
    LogLevel level = LogLevel::Info;
    bool stderrEnabled = true;
    double statusIntervalSeconds = 30.0;  // 0 disables the periodic status block
};

struct Config {
    LogConfig log;
    NtpConfig ntp;
    HttpConfig http;
    ClockConfig clock;
    std::vector<SourceConfig> sources;
    std::vector<NtpSourceConfig> ntpSources;

    // Defaults applied to any source that does not set them itself.
    SourceConfig defaults;
    NtpSourceConfig ntpDefaults;

    // Which class each source belongs to, resolved from clock.primary. Asking
    // the config rather than recomputing `kind == clock.primary` at each of the
    // dozen sites that need it, because getting one of them backwards would
    // invert the failover and nothing would say so.
    bool isPrimary(SourceKind k) const { return k == clock.primary; }
    SourceKind secondaryKind() const {
        return clock.primary == SourceKind::Radio ? SourceKind::Ntp : SourceKind::Radio;
    }

    // Things that are legal but almost certainly not meant -- a primary class
    // with no enabled source in it, a secondary with nothing to fall back to.
    // Collected rather than logged because finalise() runs before the log has
    // been opened, and a warning written to a terminal nobody is watching is a
    // warning nobody sees. main() says them once the log exists.
    std::vector<std::string> warnings;

    // Loads and validates. Returns false with `err` set; never half-applies.
    static bool load(const std::string& path, Config& out, std::string& err);

    // Applies defaults, derives dial frequencies, and rejects anything that
    // cannot work — a duplicate name, a dial outside what the decoder handles,
    // a sample rate the decimation cannot divide.
    bool finalise(std::string& err);
};

// The dial for a carrier: 1 kHz below it, which is what puts the carrier at
// 1000 Hz audio in USB for WWV, WWVH and WWVB alike.
std::uint64_t dialForCarrier(std::uint64_t carrierHz);

// Below this the dial is taken to be WWVB, which is a different decoder rather
// than a setting. Matches wwvbCeilingHz in ka9q_ubersdr's clock extension and
// WWVB_CEILING_HZ in the frontend panel.
inline constexpr std::uint64_t kWwvbCeilingHz = 1000000;

// Carriers only WWV transmits on. WWVH shares 2.5, 5, 10 and 15 MHz with it,
// so only on these two does the dial alone say which station is heard.
inline bool wwvOnlyCarrier(std::uint64_t carrierHz) {
    return carrierHz == 20000000 || carrierHz == 25000000;
}

} // namespace ubersdr_ntp
