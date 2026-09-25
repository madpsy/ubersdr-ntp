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
#include "OffsetEstimator.h"
#include "TimeSource.h"
#include "Propagation.h"
#include "SampleClock.h"
#include "TimeContinuity.h"
#include "clock/WwvDecoder.h"
#include "clock/WwvbDecoder.h"
#include "clock/Dcf77Decoder.h"
#include "clock/MsfDecoder.h"
#include "clock/AllouisDecoder.h"

#include <array>
#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace ix { class WebSocket; }

namespace ubersdr_ntp {

class Source final : public TimeSource {
public:
    explicit Source(SourceConfig cfg);
    ~Source() override;

    Source(const Source&) = delete;
    Source& operator=(const Source&) = delete;

    void start() override;
    void stop() override;

    const std::string& name() const override { return m_cfg.name; }
    SourceKind kind() const override { return SourceKind::Radio; }
    SourceSnapshot snapshot() const override;
    std::vector<double> takeRejectedJumps() override;

    // Drop the connection and acquire from nothing, at the Selector's word
    // that this source has disagreed with the others for too long. Safe from
    // any thread; acted on by the supervisor within a second, and only while a
    // connection is up -- a source that is already reconnecting is already
    // starting over.
    void requestReacquire(const std::string& why) override;

    // Hold the receiver session open, or drop it and stay disconnected. Cold
    // secondary mode (see ClockConfig): a public UberSDR receiver caps
    // concurrent sessions per address, so a standby nobody is using should not
    // be holding one of them. Coming back costs a full acquisition -- about
    // five minutes of clean signal -- which is the price of the mode.
    void setActive(bool on, const std::string& why) override;
    void setSystemRate(double rateSec, double uncertaintySec, bool known) override;
    bool active() const override { return m_active.load(); }

private:
    void supervise();
    bool sessionHandshake(std::string& err);     // POST /connection
    bool fetchDescription();                     // GET /api/description; true once parsed
    std::string buildWsUrl() const;

    void onOpen();
    void onClose(const std::string& reason);
    void onBinary(const std::string& msg);
    void onText(const std::string& msg);

    // Audio path, all on the WebSocket thread.
    void handleAudio(const std::uint8_t* pkt, std::size_t len, double arrivalSec, double dmrSec);
    // arrivalSec is the daemon clock. observeArrival=false for samples this
    // program synthesised (concealment, gap fill), which advance the timeline
    // but did not arrive at any particular instant.
    void feedSamples(const std::int16_t* pcm, int count, int rate, double arrivalSec,
                     bool observeArrival = true);
    void feedSilence(int count, int rate);
    bool ensureDecoder(int rate);   // false: no decoder can run at this rate
    void trackTimeline(std::uint64_t timestampNanos, int rate, int frameSamples);
    void discardTiming(const char* why);   // caller does NOT hold m_mu

    // Capture timing (CaptureClock in SampleClock.h). WebSocket thread.
    enum class TimingMode { Pending, Capture, Arrival };
    void setTiming(TimingMode mode, const std::string& why);
    void onServerClockId(const std::string& clockId);
    // Times the block about to be fed. With capture timing it records the
    // block's capture mark itself; otherwise it says whether the arrival fit
    // should observe the block's arrival, which is what it returns.
    bool timeBlock(std::uint64_t stampNanos, int rate, double arrivalSec, double dmrSec);
    // Daemon time at a sample index, by whichever clock the mode uses.
    bool hostTimeAt(double sample, double& hostSec) const;
    void resetStream(const char* why);

    // Decoder callbacks.
    void onClockState(clockdec::ClockLockState s);
    void onClockSecond(const clockdec::ClockSecondInfo& i);
    void onClockFrame(const clockdec::ClockFrameInfo& f);
    void onClockTime(const clockdec::ClockTimeInfo& t);

    // Whether a decoded time may anchor this source (TimeContinuity.h).
    bool admitDecodedTime(double impliedOffsetSec, double atDaemon, long long decodedMs); // caller holds m_mu
    void addOffsetSample(double offsetSec, double atDaemon);   // caller holds m_mu
    void recomputeOffset();     // caller holds m_mu
    void updateDelayModel();    // caller holds m_mu
    void recordRtt(double rttSec);  // caller holds m_mu
    void probeRtt();
    void onJsonPong();
    void recordWsRtt(double rttSec);

    SourceConfig m_cfg;
    // What is being listened to, from the tuning (Config.h, broadcastFor). The
    // LF stations are taken as IQ -- two channels a frame (isIqBroadcast) --
    // and everything else as mono audio.
    const Broadcast m_broadcast;
    const bool m_iq;
    const int m_channels;
    std::string m_sessionId;

    std::thread m_thread;
    std::atomic<bool> m_running{false};
    // Whether the supervisor should be holding a connection at all. Separate
    // from m_running, which is about the thread's life: an inactive source
    // still has its supervisor thread, waiting to be told to come back.
    std::atomic<bool> m_active{true};
    std::string m_activeReason;     // under m_mu
    std::mutex m_wake;
    std::condition_variable m_wakeCv;
    std::atomic<bool> m_reacquire{false};
    // The receiver sent a version 1-3 frame: it predates protocol version 4 and
    // nothing this daemon can ask for will change that. Set by the audio path,
    // acted on by the supervisor (see run()).
    std::atomic<bool> m_serverTooOld{false};
    std::string m_reacquireWhy;     // under m_mu

    // Guards the pointer, not the socket. Only the supervisor ever calls
    // ix::WebSocket::stop(): it joins the socket's thread with no guard of its
    // own, so two threads stopping one socket is a double join — undefined
    // behaviour, and in practice std::terminate on SIGTERM. The pointer is
    // shared so onOpen can send on it without racing the supervisor's reset.
    mutable std::mutex m_wsMu;
    std::shared_ptr<ix::WebSocket> m_ws;

    // --- state, all under m_mu ---
    mutable std::mutex m_mu;
    SourceSnapshot m_snap;
    double m_linkSince = 0.0;
    double m_lastAudioAt = 0.0;
    double m_lastTimeAt = 0.0;
    std::atomic<bool> m_socketOpen{false};
    bool m_haveDescription = false;   // supervisor thread only
    // /api/description has answered, or failed, at least once: what a 60 kHz
    // source waits for before choosing MSF or WWVB (resolveLf60).
    std::atomic<bool> m_descriptionTried{false};

    // The leap warning bit is taken from single frames, so one misread frame
    // would otherwise announce a leap second to every client.
    int m_leapFrames = 0;

    // audio decode
    std::unique_ptr<class PcmV4Reader> m_pcmv4;   // pimpl: keeps pcm_v4.hpp out of this header
    std::vector<float> m_mono;      // mono samples, or interleaved I/Q for DCF77
    std::vector<std::int16_t> m_silence;

    // clock decoder
    std::unique_ptr<clockdec::WwvDecoder> m_wwv;
    std::unique_ptr<clockdec::WwvbDecoder> m_wwvb;
    std::unique_ptr<clockdec::Dcf77Decoder> m_dcf77;
    std::unique_ptr<clockdec::MsfDecoder> m_msf;
    std::unique_ptr<clockdec::AllouisDecoder> m_allouis;
    // 60 kHz tuned on the carrier (Broadcast::Lf60): which station the
    // receiver's location made it, Unknown until decided, and whether that was
    // the no-coordinates fallback (so coordinates that turn up later can undo
    // it). WebSocket thread.
    clockdec::ClockStation m_lf60 = clockdec::ClockStation::Unknown;
    bool m_lf60Fallback = false;
    // WWVB from IQ (Lf60 resolved to WWVB): the IQ turned into the USB audio
    // its decoder was written for -- low-passed, shifted up 1 kHz, the real
    // part -- and the low-pass's delay, which the delay model takes back off.
    bool m_wwvbFromIq = false;
    struct IqBiquad { double b0 = 1, b1 = 0, b2 = 0, a1 = 0, a2 = 0, z1 = 0, z2 = 0; };
    std::array<IqBiquad, 4> m_iqLp{};   // I, I, Q, Q: two sections each
    double m_iqShiftPhase = 0.0, m_iqShiftStep = 0.0;
    double m_iqToAudioDelaySec = 0.0;
    std::vector<float> m_audio;
    clockdec::ClockStation resolveLf60();
    // DCF77: the timing source last logged, and the one being seen and since
    // when, so a change is logged once it has held (WebSocket thread).
    int m_loggedTimingPm = -1;
    int m_timingSeen = -1;
    double m_timingSeenAt = 0.0;
    int m_decoderRate = 0;
    bool m_rateRefused = false;
    // The last station tag a WWV/WWVH decoder held, so the next one can start
    // from it (see ensureDecoder). Refreshed only while the tick is heard, so a
    // tag nothing has backed for a while is not carried forward. WebSocket
    // thread only.
    clockdec::ClockStation m_stationMemory = clockdec::ClockStation::Unknown;
    double m_stationMemoryAt = 0.0;
    std::int64_t m_samplesWritten = 0;
    int m_lastFrameSamples = 0;     // the length a lost packet is assumed to have

    // Timeline integrity, all on the WebSocket thread. The server stamps each
    // frame with the time its first sample reached it; the stamp minus this
    // program's own sample count is constant while no audio goes missing, and
    // steps by exactly the missing duration when some does. See trackTimeline.
    bool m_tsHave = false;
    std::uint64_t m_tsOriginNanos = 0;
    std::int64_t m_tsOriginSample = 0;
    double m_tsBaseline = 0.0;
    double m_tsBlockMin = 0.0;
    std::uint64_t m_tsBlockStartNanos = 0;
    bool m_tsBlockHave = false;
    std::uint64_t m_gapFills = 0;

    SampleClock m_clock;

    // Capture timing, WebSocket thread. m_timing is decided per connection
    // from the receiver's clock id; see onServerClockId.
    TimingMode m_timing = TimingMode::Pending;
    double m_timingPendingSince = 0.0;   // monotonic, first audio while pending
    CaptureClock m_capture;
    HostSlewGuard m_slew;                // host-wide in truth, but cheap per source
    bool m_captureTrusted = true;        // for logging the transitions

    // The UTC anchor a `time` event leaves behind, extended one second per
    // `second` event. Reset whenever lock is lost, because an anchor that
    // survives a resync would date the new stream from the old one.
    bool m_haveAnchor = false;
    bool m_haveFrame = false;
    std::int64_t m_frameStartSample = 0;
    std::int64_t m_anchorEdgeSample = 0;
    long long m_anchorUtcMs = 0;
    double m_anchorSetAt = 0.0;

    // Under m_mu. A radio source holds its level through a jitter spike; see
    // OffsetEstimator.h.
    OffsetEstimator m_offsets{[] { OffsetTuning t; t.holdJitterSpikes = true; return t; }()};
    int m_loggedHolds = 0;          // holds already logged as started
    bool m_loggedHeld = false;      // ...and whether one is in progress
    int m_loggedTimeouts = 0;

    // The continuity check on decoded times (TimeContinuity.h), with this
    // source's own filtered offset as its history. Deliberately NOT reset by a
    // reconnect, a decoder restart or a re-acquisition -- which is where a
    // misread lock is most likely -- because it describes UTC against the
    // daemon clock, not the stream. Under m_mu.
    TimeContinuity m_continuity;
    std::vector<double> m_rejectedJumps;   // under m_mu, drained by takeRejectedJumps
    double m_leapWarnAt = -1e18;          // daemon clock: last frame with the warning believed
    std::deque<double> m_rttProbes;
    std::deque<double> m_wsRttProbes;
    // When the outstanding JSON ping went out (monotonic seconds), 0 when none
    // is outstanding, or kPingUntimed when the next pong must not be timed.
    // Under m_mu. See the ping in run().
    double m_jsonPingSentAt = 0.0;
};

} // namespace ubersdr_ntp
