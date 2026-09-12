// ubersdr-clock — standalone WWV/WWVH/WWVB time-code decoder.
//
// Reads raw mono int16 little-endian PCM on stdin, writes newline-delimited
// JSON events on stdout. Same shape as UberSDR's other external decoder
// binaries (cw-decoder, ubersdr-drm, freedv-ka9q): no framing, no handshake,
// close stdin to stop.
//
// The DSP is AetherSDR's AetherClock chain, copied verbatim — see PROVENANCE
// in the README. This file is the only part written for UberSDR: it replaces
// the Qt engine (AetherClockEngine) that fed the decoders there, and it does
// only what that engine did on the receive side — hold the sample<->host
// anchor, compose a UTC timestamp from the voted frame, and report the offset.
//
// Timing anchor
// ─────────────
// A pipe carries samples, not timestamps, so the host anchor here is simply
// the wall clock read when the first sample arrives, advanced by the sample
// count. That is exactly what AetherClockEngine does, and it is good to a few
// tens of ms — dominated by the buffering between the receiver and this
// process, which this process cannot see.
//
// UberSDR can do better, and should: every AudioSample reaching an audio
// extension carries a GPS-synchronised GPSTimeNs. Every event below therefore
// reports the raw sample indices (`edge_sample`, `frame_start_sample`,
// `last_edge_sample`) the offset was derived from, so the Go wrapper can
// recompute it against the GPS timestamps and ignore `offset_ms` entirely.
// `offset_ms` is for standalone use.

#include "WwvDecoder.h"
#include "WwvbDecoder.h"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <string_view>
#include <vector>

namespace {

constexpr const char* kVersion = "1.0.0";

// ---------------------------------------------------------------------------
// Calendar arithmetic (Howard Hinnant's civil-date algorithms).
//
// Deliberately not timegm()/gmtime_r(): both are POSIX rather than ISO C, and
// whether they are visible under -std=c++20 depends on feature-test macros. A
// decoder whose timestamps depend on how the compiler was invoked is not one
// to debug at 3am. These are exact for the whole proleptic Gregorian range.

long long daysFromCivil(int y, unsigned m, unsigned d) {
    y -= m <= 2;
    const long long era = (y >= 0 ? y : y - 399) / 400;
    const unsigned yoe = static_cast<unsigned>(y - era * 400);             // [0, 399]
    const unsigned doy = (153u * (m + (m > 2 ? -3u : 9u)) + 2u) / 5u + d - 1u;
    const unsigned doe = yoe * 365u + yoe / 4u - yoe / 100u + doy;         // [0, 146096]
    return era * 146097LL + static_cast<long long>(doe) - 719468LL;
}

void civilFromDays(long long z, int& y, unsigned& m, unsigned& d) {
    z += 719468;
    const long long era = (z >= 0 ? z : z - 146096) / 146097;
    const unsigned doe = static_cast<unsigned>(z - era * 146097);          // [0, 146096]
    const unsigned yoe = (doe - doe / 1460u + doe / 36524u - doe / 146096u) / 365u;
    const long long yy = static_cast<long long>(yoe) + era * 400;
    const unsigned doy = doe - (365u * yoe + yoe / 4u - yoe / 100u);       // [0, 365]
    const unsigned mp = (5u * doy + 2u) / 153u;                            // [0, 11]
    d = doy - (153u * mp + 2u) / 5u + 1u;
    m = mp + (mp < 10u ? 3u : -9u);
    y = static_cast<int>(yy + (m <= 2u));
}

// Floor division/modulo — plain / and % truncate toward zero, which is wrong
// for pre-1970 epochs. Not reachable from a valid decode, but the fallback
// paths below hand these whatever the voter produced.
long long floorDiv(long long a, long long b) {
    const long long q = a / b;
    return (a % b != 0 && ((a < 0) != (b < 0))) ? q - 1 : q;
}
long long floorMod(long long a, long long b) { return a - floorDiv(a, b) * b; }

// UTC milliseconds for a decoded WWV timestamp. doy is 1-based (NIST BCD day
// field), year2 the two-digit year in the 20xx century, seconds always 0 —
// a time-code frame names the minute, and the second comes from the edge.
long long utcMsFromFields(int year2, int doy, int hour, int minute) {
    const long long days = daysFromCivil(2000 + year2, 1, 1) + (doy - 1);
    return (days * 86400LL + hour * 3600LL + minute * 60LL) * 1000LL;
}

std::string iso8601(long long ms) {
    const long long secs = floorDiv(ms, 1000);
    const long long days = floorDiv(secs, 86400);
    long long rem = floorMod(secs, 86400);
    int y = 0; unsigned mo = 0, d = 0;
    civilFromDays(days, y, mo, d);
    const int hh = static_cast<int>(rem / 3600); rem %= 3600;
    const int mi = static_cast<int>(rem / 60);
    const int ss = static_cast<int>(rem % 60);
    char buf[40];
    std::snprintf(buf, sizeof buf, "%04d-%02u-%02uT%02d:%02d:%02dZ", y, mo, d, hh, mi, ss);
    return buf;
}

long long hostNowMs() {
    using namespace std::chrono;
    return duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
}

// The host clock as the voter's plausibility reference. Fires on this thread
// inside process(), so it must stay cheap — it is four integer divisions.
AetherSDR::TimeFields hostNowFields() {
    const long long secs = floorDiv(hostNowMs(), 1000);
    const long long days = floorDiv(secs, 86400);
    const long long rem = floorMod(secs, 86400);
    int y = 0; unsigned mo = 0, d = 0;
    civilFromDays(days, y, mo, d);
    AetherSDR::TimeFields tf;
    tf.minute = static_cast<int>((rem / 60) % 60);
    tf.hour = static_cast<int>(rem / 3600);
    tf.doy = static_cast<int>(days - daysFromCivil(y, 1, 1)) + 1;
    tf.year2 = y % 100;
    return tf;
}

// ---------------------------------------------------------------------------
// Minimal JSON line writer. One object per line, no nesting beyond a flat
// array, which is all any event below needs — so this stays a string builder
// rather than a dependency.

class Json {
public:
    Json() : m_s("{") {}

    Json& str(const char* k, std::string_view v) {
        key(k);
        m_s += '"';
        for (char c : v) {
            switch (c) {
                case '"':  m_s += "\\\""; break;
                case '\\': m_s += "\\\\"; break;
                case '\n': m_s += "\\n"; break;
                case '\r': m_s += "\\r"; break;
                case '\t': m_s += "\\t"; break;
                default:
                    if (static_cast<unsigned char>(c) < 0x20) {
                        char esc[8];
                        std::snprintf(esc, sizeof esc, "\\u%04x", c);
                        m_s += esc;
                    } else {
                        m_s += c;
                    }
            }
        }
        m_s += '"';
        return *this;
    }

    Json& i(const char* k, long long v) {
        key(k);
        char buf[24];
        std::snprintf(buf, sizeof buf, "%lld", v);
        m_s += buf;
        return *this;
    }

    Json& b(const char* k, bool v) { key(k); m_s += v ? "true" : "false"; return *this; }

    // Non-finite becomes null: JSON has no NaN, and diagnostics() really does
    // report NaN for a delay estimate that has not settled.
    Json& f(const char* k, double v, int prec = 4) {
        key(k);
        m_s += fmt(v, prec);
        return *this;
    }

    Json& arr(const char* k, const std::vector<float>& v, int prec = 4) {
        key(k);
        m_s += '[';
        for (std::size_t n = 0; n < v.size(); ++n) {
            if (n) m_s += ',';
            m_s += fmt(v[n], prec);
        }
        m_s += ']';
        return *this;
    }

    // Writes the line and flushes. stdout on a pipe is fully buffered, so
    // without the flush the reading process sees nothing until the buffer
    // fills — minutes of silence for a decoder that emits one line a second.
    void emit() {
        m_s += "}\n";
        std::fwrite(m_s.data(), 1, m_s.size(), stdout);
        std::fflush(stdout);
    }

private:
    void key(const char* k) {
        if (!m_first) m_s += ',';
        m_first = false;
        m_s += '"';
        m_s += k;
        m_s += "\":";
    }

    static std::string fmt(double v, int prec) {
        if (!std::isfinite(v)) return "null";
        char buf[64];
        std::snprintf(buf, sizeof buf, "%.*f", prec, v);
        return buf;
    }

    std::string m_s;
    bool m_first = true;
};

// ---------------------------------------------------------------------------

const char* stationName(AetherSDR::ClockStation s) {
    using AetherSDR::ClockStation;
    switch (s) {
        case ClockStation::Wwv:  return "wwv";
        case ClockStation::Wwvh: return "wwvh";
        case ClockStation::Wwvb: return "wwvb";
        default:                 return "unknown";
    }
}

const char* stateName(AetherSDR::ClockLockState s) {
    using AetherSDR::ClockLockState;
    switch (s) {
        case ClockLockState::Locked:    return "locked";
        case ClockLockState::Acquiring: return "acquiring";
        default:                        return "nosignal";
    }
}

const char* refusalName(std::uint8_t r) {
    using AetherSDR::ClockLockRefusal;
    switch (static_cast<ClockLockRefusal>(r)) {
        case ClockLockRefusal::QualityFloor: return "quality_floor";
        case ClockLockRefusal::Plausibility: return "plausibility";
        case ClockLockRefusal::Staleness:    return "staleness";
        case ClockLockRefusal::Contested:    return "contested";
        default:                             return "none";
    }
}

struct Options {
    int sampleRate = 12000;
    bool wwvb = false;
    bool seconds = true;
    bool envelope = false;
    int diagSeconds = 10;
    int plausibilityMinutes = 24 * 60;   // AetherClockEngine's kPlausibilityBoundMinutes
};

// Turns decoder callbacks into JSON lines, and holds the two pieces of state
// the decoders do not carry themselves: the sample<->host anchor, and the
// frame start the voted timestamp is composed against.
class Emitter {
public:
    Emitter(const Options& o, int sampleRate) : m_o(o), m_rate(sampleRate) {}

    void setAnchor(long long hostMs) { m_anchorMs = hostMs; m_haveAnchor = true; }

    double hostMsAtSample(std::int64_t n) const {
        return static_cast<double>(m_anchorMs) + 1000.0 * static_cast<double>(n) / m_rate;
    }

    void onState(AetherSDR::ClockLockState s, AetherSDR::ClockStation st) {
        Json j;
        j.str("type", "state").str("state", stateName(s)).str("station", stationName(st));
        j.emit();
    }

    void onSecond(const AetherSDR::ClockSecondInfo& i, AetherSDR::ClockStation st) {
        if (!m_o.seconds) return;
        Json j;
        j.str("type", "second")
         .i("edge_sample", i.edgeSample)
         .i("symbol", static_cast<long long>(i.symbol))
         .f("confidence", i.confidence)
         .i("second_of_frame", i.secondOfFrame)
         .i("series_rate", i.seriesRateHz)
         .i("window_shift", i.windowShift)
         .str("station", stationName(st));
        if (m_o.envelope) {
            j.arr("envelope", i.envelope, 3);
            j.arr("expected", i.expected, 3);
        }
        j.emit();
    }

    void onFrame(const AetherSDR::ClockFrameInfo& f) {
        // Recorded whether or not the frame is emitted: onTime composes
        // against it, and a raw frame decode is never suppressed anyway.
        m_frameStartSample = f.frameStartSample;
        m_haveFrame = true;

        Json j;
        j.str("type", "frame")
         .i("minute", f.minute).i("hour", f.hour).i("doy", f.doy).i("year2", f.year2)
         .i("dut1_tenths", f.dut1Tenths)
         .b("dst1", f.dst1).b("dst2", f.dst2)
         .b("leap_pending", f.leapPending).b("leap_year", f.leapYear)
         .f("confidence", f.frameConfidence)
         .i("frame_start_sample", f.frameStartSample)
         .str("station", stationName(f.station));
        j.emit();
    }

    void onTime(const AetherSDR::ClockTimeInfo& t) {
        // onFrame always precedes a vote, but a decoder that locked on a
        // backlog replay could in principle reach here first; composing
        // against a frame start of 0 would put the timestamp minutes out.
        if (!m_haveFrame || !m_haveAnchor) return;
        if (t.year2 < 0 || t.doy < 1 || t.hour < 0 || t.minute < 0) return;

        // Composed exactly as AetherClockEngine::handleTime does: the voted
        // frame's second 0, plus the elapsed samples to the last edge. Using
        // lastEdgeSecondOfFrame directly would be wrong — it can point into a
        // frame later than the one that was voted.
        const long long baseMs = utcMsFromFields(t.year2, t.doy, t.hour, t.minute);
        const long long elapsedSec = std::llround(
            static_cast<double>(t.lastEdgeSample - m_frameStartSample) / m_rate);
        const long long decodedMs = baseMs + elapsedSec * 1000LL;

        const double offsetMs = static_cast<double>(decodedMs) - hostMsAtSample(t.lastEdgeSample);

        Json j;
        j.str("type", "time")
         .str("utc", iso8601(decodedMs))
         .i("utc_ms", decodedMs)
         .i("minute", t.minute).i("hour", t.hour).i("doy", t.doy).i("year2", t.year2)
         .i("quality", std::clamp(static_cast<int>(std::lround(t.quality * 100.0)), 0, 100))
         .f("offset_ms", offsetMs, 1)
         .i("last_edge_sample", t.lastEdgeSample)
         .i("frame_start_sample", m_frameStartSample)
         .i("host_anchor_ms", m_anchorMs)
         .str("station", stationName(t.station));
        j.emit();
    }

    void onDiag(const AetherSDR::ClockDecoderDiagnostics& g,
                AetherSDR::ClockLockState s, AetherSDR::ClockStation st,
                std::int64_t samples) {
        Json j;
        j.str("type", "diag")
         .str("state", stateName(s)).str("station", stationName(st))
         .f("tone_snr_db", g.toneSnrDb, 2)
         .f("pwm_contrast", g.pwmContrast, 3)
         .b("tone_detected", g.toneDetected)
         .b("phase_locked", g.phaseLocked)
         .f("delay_est_ms", g.delayEstMs, 2)
         .b("anchored", g.anchored)
         .i("bad_frame_streak", g.badFrameStreak)
         .i("frames_in_window", g.framesInWindow)
         .i("window_size", g.windowSize)
         .f("vote_quality", g.voteQuality, 3)
         .str("refusal", refusalName(g.refusalReason))
         .i("samples_consumed", samples);
        j.emit();
    }

private:
    const Options& m_o;
    int m_rate;
    long long m_anchorMs = 0;
    bool m_haveAnchor = false;
    std::int64_t m_frameStartSample = 0;
    bool m_haveFrame = false;
};

void emitError(std::string_view message) {
    Json j;
    j.str("type", "error").str("message", message);
    j.emit();
}

// ---------------------------------------------------------------------------

template <typename Decoder>
int run(const Options& o) {
    Decoder decoder(o.sampleRate);
    Emitter em(o, o.sampleRate);

    decoder.onStateChanged = [&](AetherSDR::ClockLockState s) { em.onState(s, decoder.station()); };
    decoder.onSecond = [&](const AetherSDR::ClockSecondInfo& i) { em.onSecond(i, decoder.station()); };
    decoder.onFrame = [&](const AetherSDR::ClockFrameInfo& f) { em.onFrame(f); };
    decoder.onTime = [&](const AetherSDR::ClockTimeInfo& t) { em.onTime(t); };

    if (o.plausibilityMinutes > 0)
        decoder.setPlausibility(hostNowFields, o.plausibilityMinutes);

    constexpr std::size_t kChunkSamples = 4096;
    std::vector<unsigned char> raw(kChunkSamples * 2);
    std::vector<float> mono(kChunkSamples);

    bool anchored = false;
    unsigned char oddByte = 0;
    bool haveOddByte = false;
    const std::int64_t diagEvery =
        o.diagSeconds > 0 ? static_cast<std::int64_t>(o.diagSeconds) * o.sampleRate : 0;
    std::int64_t nextDiag = diagEvery;

    for (;;) {
        std::size_t offset = 0;
        if (haveOddByte) { raw[0] = oddByte; offset = 1; haveOddByte = false; }

        const std::size_t got = std::fread(raw.data() + offset, 1, raw.size() - offset, stdin);
        if (got == 0) {
            if (std::ferror(stdin)) {
                emitError("stdin read error");
                return 1;
            }
            break;   // clean EOF — the caller closed the pipe
        }

        if (!anchored) {
            // The wall clock as the first sample arrives. Everything before
            // this point (receiver buffering, pipe latency) is invisible here
            // and shows up as a constant term in offset_ms; see the note at
            // the top of this file.
            em.setAnchor(hostNowMs());
            anchored = true;
        }

        std::size_t avail = offset + got;
        const std::size_t nSamples = avail / 2;
        if (avail & 1u) { oddByte = raw[avail - 1]; haveOddByte = true; }

        for (std::size_t n = 0; n < nSamples; ++n) {
            const auto lo = static_cast<std::uint16_t>(raw[2 * n]);
            const auto hi = static_cast<std::uint16_t>(raw[2 * n + 1]);
            const auto v = static_cast<std::int16_t>(static_cast<std::uint16_t>(lo | (hi << 8)));
            mono[n] = static_cast<float>(v) * (1.0f / 32768.0f);
        }

        decoder.process(mono.data(), nSamples);

        if (diagEvery > 0 && decoder.samplesConsumed() >= nextDiag) {
            em.onDiag(decoder.diagnostics(), decoder.state(), decoder.station(),
                      decoder.samplesConsumed());
            // Advance past the current point rather than by one step: a large
            // read must not queue up a burst of backdated diagnostics.
            while (nextDiag <= decoder.samplesConsumed()) nextDiag += diagEvery;
        }
    }

    return 0;
}

void usage() {
    std::printf(
        "ubersdr-clock %s — WWV/WWVH/WWVB time-code decoder\n"
        "\n"
        "Reads raw mono int16 little-endian PCM on stdin, writes newline-delimited\n"
        "JSON events on stdout. Close stdin to stop.\n"
        "\n"
        "Options:\n"
        "  --sample-rate HZ           Input PCM rate (default: 12000)\n"
        "  --station NAME             wwv, wwvh or wwvb (default: wwv)\n"
        "  --no-seconds               Suppress the per-second classification events\n"
        "  --envelope                 Include the 1 s alignment arrays in second events\n"
        "  --diag-seconds N           Diagnostics event every N seconds, 0 = off (default: 10)\n"
        "  --plausibility-minutes N   Refuse a lock more than N minutes from the host\n"
        "                             clock, 0 = disarm (default: 1440)\n"
        "  --version                  Print the version and exit\n"
        "  --help                     Print this and exit\n"
        "\n"
        "Tuning:\n"
        "  WWV/WWVH  USB at (carrier - 1 kHz), e.g. 9.999 MHz for the 10 MHz outlet.\n"
        "  WWVB      USB at 0.059 MHz.\n"
        "  The passband must reach 2.2 kHz: the WWV/WWVH second tick is recovered\n"
        "  from its 2000 Hz (WWV) / 2200 Hz (WWVH) image, and without it the\n"
        "  decoder never gets a second edge to classify against.\n",
        kVersion);
}

// Parses an integer argument, or reports which option was wrong and why.
bool parseInt(const char* opt, const char* text, int& out) {
    if (text == nullptr) {
        std::fprintf(stderr, "ubersdr-clock: %s requires a value\n", opt);
        return false;
    }
    char* end = nullptr;
    const long v = std::strtol(text, &end, 10);
    if (end == text || *end != '\0' || v < 0 || v > 100000000L) {
        std::fprintf(stderr, "ubersdr-clock: %s: not a valid number: %s\n", opt, text);
        return false;
    }
    out = static_cast<int>(v);
    return true;
}

}  // namespace

int main(int argc, char** argv) {
    Options o;

    for (int i = 1; i < argc; ++i) {
        const std::string_view a = argv[i];
        const char* next = (i + 1 < argc) ? argv[i + 1] : nullptr;

        if (a == "--help" || a == "-h") { usage(); return 0; }
        if (a == "--version") { std::printf("%s\n", kVersion); return 0; }

        if (a == "--sample-rate") {
            if (!parseInt("--sample-rate", next, o.sampleRate)) return 2;
            ++i;
        } else if (a == "--station") {
            if (next == nullptr) {
                std::fprintf(stderr, "ubersdr-clock: --station requires a value\n");
                return 2;
            }
            const std::string_view s = next;
            if (s == "wwvb") {
                o.wwvb = true;
            } else if (s == "wwv" || s == "wwvh") {
                // One decoder covers both: it tags the station itself from
                // which tick band folds to an impulse.
                o.wwvb = false;
            } else {
                std::fprintf(stderr, "ubersdr-clock: unknown station: %s "
                                     "(expected wwv, wwvh or wwvb)\n", next);
                return 2;
            }
            ++i;
        } else if (a == "--no-seconds") {
            o.seconds = false;
        } else if (a == "--envelope") {
            o.envelope = true;
            o.seconds = true;   // the arrays live on the second event
        } else if (a == "--diag-seconds") {
            if (!parseInt("--diag-seconds", next, o.diagSeconds)) return 2;
            ++i;
        } else if (a == "--plausibility-minutes") {
            if (!parseInt("--plausibility-minutes", next, o.plausibilityMinutes)) return 2;
            ++i;
        } else {
            std::fprintf(stderr, "ubersdr-clock: unknown option: %s "
                                 "(try --help)\n", argv[i]);
            return 2;
        }
    }

    // The WWV chain needs the 2000/2200 Hz tick images, so Nyquist has to clear
    // 2.2 kHz with room for the tick bandpass skirts; WWVB only needs its
    // ~1 kHz tone. Both decimate to a fixed series rate (200 Hz / 100 Hz), so
    // a rate that is not a multiple of it decimates unevenly and drifts.
    const int seriesRate = o.wwvb ? 100 : 200;
    if (o.sampleRate < (o.wwvb ? 4000 : 8000)) {
        std::fprintf(stderr, "ubersdr-clock: --sample-rate %d is too low for %s "
                             "(need at least %d Hz)\n",
                     o.sampleRate, o.wwvb ? "WWVB" : "WWV/WWVH", o.wwvb ? 4000 : 8000);
        return 2;
    }
    if (o.sampleRate % seriesRate != 0) {
        std::fprintf(stderr, "ubersdr-clock: --sample-rate %d is not a multiple of %d Hz "
                             "(the %s series rate); decimation would drift\n",
                     o.sampleRate, seriesRate, o.wwvb ? "WWVB" : "WWV/WWVH");
        return 2;
    }

    return o.wwvb ? run<AetherSDR::WwvbDecoder>(o) : run<AetherSDR::WwvDecoder>(o);
}
