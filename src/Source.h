#pragma once

// One UberSDR receiver, tuned to one time-signal frequency.
//
// Owns everything from the TCP socket to a filtered clock offset: the HTTP
// session handshake, the WebSocket, the audio codec, an in-process
// WWV/WWVH/WWVB decoder, and the statistics the selector and the status report
// read. Sources are entirely independent of each other — that is the point of
// having several — and share nothing but the log.
//
// THREADS
//
// Two, per source. A supervisor thread runs the connect / stream / reconnect
// lifecycle and the 30-second JSON keepalive. IXWebSocket delivers messages on
// its own thread, and that is where audio is decoded and the clock decoder
// runs, because copying 20 ms of audio onto a queue to be processed 20 ms later
// by another thread would add a scheduling delay to the one quantity this
// program exists to measure. Everything the outside world reads goes through
// m_mu.
//
// WHY THE OFFSET IS SAMPLED EVERY SECOND
//
// A `time` event — a voted, trustworthy timestamp — arrives once per minute on
// WWV, which is a thin diet for a filter. But a `time` event anchors the UTC of
// one sample index, and after it, every `second` event is exactly one second
// later than the last. So the anchor is extended forwards: each second edge
// yields an independent measurement of the same offset, differing only by the
// noise in that edge's estimate. Sixty measurements a minute instead of one,
// from the same voted, plausibility-checked timestamp — the vote is not being
// bypassed, it is being re-used.

#include "Config.h"
#include "OpusV4Header.h"
#include "Propagation.h"
#include "SampleClock.h"
#include "clock/WwvDecoder.h"
#include "clock/WwvbDecoder.h"

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

struct OpusDecoder;

namespace ix { class WebSocket; }

namespace ubersdr_ntp {

enum class LinkState { Idle, Connecting, Streaming, Backoff, Stopped };
const char* linkStateName(LinkState s);

// Everything one source knows about itself, copied out under lock.
struct SourceSnapshot {
    std::string name;
    std::string url;
    bool enabled = true;
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

    // timing
    bool haveOffset = false;
    double offsetSec = 0.0;         // corrected: raw + total delay
    double rawOffsetSec = 0.0;      // before the delay model, for diagnosis
    double jitterSec = 0.0;         // spread of the measurements in the window
    double dispersionSec = 0.0;     // jitter + fit residual + delay uncertainty
    double offsetAgeSec = 1e9;
    int offsetSamples = 0;

    // delay model
    double delaySec = 0.0;
    double propagationSec = 0.0;
    double networkSec = 0.0;
    double codecSec = 0.0;
    double extraSec = 0.0;
    std::string pathDescription;

    // sample clock
    bool clockFitValid = false;
    double clockResidualSec = 0.0;
    double clockSpanSec = 0.0;
    double clockPpm = 0.0;          // receiver sample clock against ours
    double lastExcessDelaySec = 0.0;

    double weight = 1.0;
};

class Source {
public:
    Source(SourceConfig cfg, std::string userAgent);
    ~Source();

    Source(const Source&) = delete;
    Source& operator=(const Source&) = delete;

    void start();
    void stop();

    const std::string& name() const { return m_cfg.name; }
    SourceSnapshot snapshot() const;

private:
    void supervise();
    bool sessionHandshake(std::string& err);     // POST /connection
    void fetchDescription();                     // GET /api/description
    std::string buildWsUrl() const;

    void onOpen();
    void onClose(const std::string& reason);
    void onBinary(const std::string& msg);
    void onText(const std::string& msg);

    // Audio path, all on the WebSocket thread.
    void handleAudio(const std::uint8_t* pkt, std::size_t len, double arrivalSec);
    void feedSamples(const std::int16_t* pcm, int count, int rate, double arrivalSec);
    void ensureDecoder(int rate);
    void resetStream(const char* why);

    // Decoder callbacks.
    void onClockState(AetherSDR::ClockLockState s);
    void onClockSecond(const AetherSDR::ClockSecondInfo& i);
    void onClockFrame(const AetherSDR::ClockFrameInfo& f);
    void onClockTime(const AetherSDR::ClockTimeInfo& t);

    void addOffsetSample(double offsetSec, double atRealtime);
    void recomputeOffset();     // caller holds m_mu
    void updateDelayModel();    // caller holds m_mu
    void recordRtt(double rttSec);  // caller holds m_mu
    void probeRtt();

    SourceConfig m_cfg;
    std::string m_userAgent;
    std::string m_sessionId;

    std::thread m_thread;
    std::atomic<bool> m_running{false};
    std::mutex m_wake;
    std::condition_variable m_wakeCv;

    // Guards the pointer, not the socket. ix::WebSocket::stop() is safe to
    // call from another thread; resetting the unique_ptr that holds it while
    // that other thread is dereferencing it is not, and stop() and the
    // supervisor loop both do exactly that.
    mutable std::mutex m_wsMu;
    std::shared_ptr<ix::WebSocket> m_ws;

    // --- state, all under m_mu ---
    mutable std::mutex m_mu;
    SourceSnapshot m_snap;
    double m_linkSince = 0.0;
    double m_lastAudioAt = 0.0;
    double m_lastTimeAt = 0.0;
    std::atomic<bool> m_socketOpen{false};

    // audio decode
    OpusV4HeaderDecoder m_opusHeader;
    OpusDecoder* m_opus = nullptr;
    int m_opusRate = 0;
    std::vector<std::int16_t> m_opusPcm;
    std::unique_ptr<class PcmV4Reader> m_pcmv4;   // pimpl: keeps pcm_v4.hpp out of this header
    std::vector<float> m_mono;

    // clock decoder
    std::unique_ptr<AetherSDR::WwvDecoder> m_wwv;
    std::unique_ptr<AetherSDR::WwvbDecoder> m_wwvb;
    int m_decoderRate = 0;
    std::int64_t m_samplesWritten = 0;

    SampleClock m_clock;

    // The UTC anchor a `time` event leaves behind, extended one second per
    // `second` event. Reset whenever lock is lost, because an anchor that
    // survives a resync would date the new stream from the old one.
    bool m_haveAnchor = false;
    bool m_haveFrame = false;
    std::int64_t m_frameStartSample = 0;
    std::int64_t m_anchorEdgeSample = 0;
    long long m_anchorUtcMs = 0;
    double m_anchorSetAt = 0.0;

    struct OffsetSample { double at; double offset; };
    std::deque<OffsetSample> m_offsets;
    std::deque<double> m_rttProbes;
};

} // namespace ubersdr_ntp
