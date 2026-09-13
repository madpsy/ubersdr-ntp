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

// The read-only status service: a JSON API and a small page that renders it.
//
// Read-only without qualification — there is no route that changes anything,
// no authentication because there is nothing to authorise, and it binds to
// localhost by default because a receiver password lives in this daemon's
// configuration and nothing served here should tempt anyone into exposing it.
struct HttpConfig {
    bool enabled = true;
    std::string listen = "127.0.0.1";
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
    std::vector<SourceConfig> sources;

    // Defaults applied to any source that does not set them itself.
    SourceConfig defaults;

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

} // namespace ubersdr_ntp
