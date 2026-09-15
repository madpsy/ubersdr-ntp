#pragma once

// Maps a decoder sample index onto the daemon's clock (see daemonNow below).
//
// This is the whole problem. The decoder says "the broadcast second edge was
// at input sample 5027880"; NTP needs to know what time it was HERE when that
// sample was captured. Nothing in the audio stream says so directly.
//
// What we have is a stream at a known rate arriving over TCP. Sample n was
// captured at
//
//     t(n) = A + n / (rate * (1 + eps))
//
// where A is the unknown anchor (host time of sample 0, including every fixed
// delay in the chain) and eps the fractional difference between the receiver's
// sample clock and ours. Each packet gives one noisy observation of t(): the
// local time it arrived, which is t(last sample) PLUS a transport delay.
//
// The transport delay is the reason a naive fit is wrong. It is not zero-mean
// noise: it is bounded below and unbounded above — a packet can be arbitrarily
// late but never early. Least squares through the raw arrivals therefore fits
// the middle of the delay distribution, which moves with load and drags the
// anchor with it. The minimum is the estimator that matters, exactly as it is
// in NTP's clock filter and in PTP.
//
// So: bucket the arrivals into short windows, keep the MINIMUM residual in
// each (one point per window, at the cleanest packet in it), and least-squares
// a line through those low-envelope points. The line's intercept is A and its
// slope gives eps. Bucketing rather than a plain running minimum matters
// because a single unusually fast packet would otherwise pin the estimate for
// the whole window length; one point per bucket lets the envelope follow real
// drift.
//
// What this does NOT remove is the CONSTANT part of the delay — radiod's
// buffering, the multicast hop, the server's queueing, the codec, the one-way
// network latency, and the propagation delay from the transmitter. Those are a
// fixed bias that no amount of filtering can see, because nothing in the
// stream is a reference for them. They are handled as a per-source calibration
// constant, and their residual uncertainty is reported as dispersion rather
// than pretended away.

#include <cstdint>
#include <deque>
#include <mutex>

namespace ubersdr_ntp {

struct ClockFit {
    bool valid = false;
    double anchorSec = 0.0;   // daemon clock (seconds) at sample 0
    double secPerSample = 0.0; // 1 / effective rate, absorbing eps
    double residualRms = 0.0;  // spread of the low-envelope points about the fit (s)
    double spanSec = 0.0;      // how much stream the fit covers
    int points = 0;
    // How wrong the SLOPE could make a timestamp at the working end of the
    // window. The residual above says how well the points sit on the line; this
    // says how well the line's direction is known, which is a different thing
    // and the one that hurts: a rate error is a bias that grows with distance
    // from the window's centre, so a poorly determined slope quietly biases
    // every edge rather than scattering them. Standard error of the OLS slope
    // times the distance worked to -- or, when the slope was refused as
    // implausible, how far the refused slope wanted to pull that timestamp,
    // because data that implies an impossible rate is data that disagrees with
    // itself by that much.
    double slopeUncertaintySec = 0.0;
    bool slopeHeld = false;    // the fitted slope was refused; nominal is in use
};

class SampleClock {
public:
    // bucketSec   how long a low-envelope bucket is. Long enough that a busy
    //             moment does not fill it entirely with late packets, short
    //             enough that several fit inside the regression window.
    // windowSec   how far back the regression reaches. Long enough to separate
    //             a rate error from noise; short enough to track a real one.
    SampleClock(int sampleRate, double bucketSec = 4.0, double windowSec = 300.0);

    void setSampleRate(int rate);
    int sampleRate() const { return m_rate; }

    // Records that the block ENDING at sample index `endSample` arrived at
    // daemon time `hostSec` (daemonNow(), seconds).
    //
    // The fit spans five minutes of arrivals and assumes a straight line, so it
    // must be taken on a clock nothing steers. CLOCK_MONOTONIC is not one: it is
    // immune to steps, but ntpd or chrony slewing the host changes its RATE, by
    // hundreds of ppm for a minute at a time while they correct an offset --
    // measured on a host whose chrony took this daemon as its source, the rate
    // swung from +177 to -500 ppm within a minute. The fit read that as a
    // receiver running impossibly fast, refused the slope, and put 40 ms of
    // slope uncertainty on every edge. The daemon clock is the raw oscillator,
    // so the line through these points is a line.
    //
    // The end of the block, not the start: its audio was captured before it was
    // sent, so the last sample is the edge closest to the moment it landed.
    void observe(std::int64_t endSample, double hostSec);

    // Host time at a sample index, or valid=false before there is enough to fit.
    ClockFit fit() const;
    bool hostTimeAt(std::int64_t sample, double& hostSec) const;

    // Drops everything. For a reconnect: sample indices restart and the old
    // envelope describes a stream that no longer exists.
    void reset();

    // The most recent raw arrival delay above the fitted line, in seconds.
    // Not used for timing — it is the live "how jittery is this path" figure
    // the status report shows.
    double lastExcessDelay() const;

private:
    struct Point {
        std::int64_t sample;
        double hostSec;
        double residual;   // hostSec - sample/rate, the quantity being minimised
    };

    void refit();

    mutable std::mutex m_mu;
    int m_rate;
    double m_bucketSec;
    double m_windowSec;

    // The bucket being filled: the best (smallest-residual) observation so far.
    bool m_haveBucket = false;
    Point m_bucketBest{};
    double m_bucketStartSec = 0.0;

    std::deque<Point> m_envelope;
    ClockFit m_fit{};
    double m_lastExcess = 0.0;
};

// The clocks this daemon reads, as doubles. They are not interchangeable.
//
// daemonNow() is the daemon's own clock, and the one every timing measurement
// and every served timestamp is taken on: CLOCK_MONOTONIC_RAW -- the machine's
// oscillator with no correction applied by anyone -- offset once, at the first
// call, so that it reads close to Unix time. The radio decides what time it
// is; this only has to count between broadcast seconds, and a clock that only
// counts must not be a clock something else is steering. The host's clock is
// exactly that: whatever disciplines the host slews it, and a host whose NTP
// client uses this daemon as its source slews it by what this daemon serves,
// which closes a loop through the measurement. Measured and served on the raw
// oscillator, the answer is the radio's and the oscillator's alone. Its rate
// error against UTC is a crystal's, steady and measured (see OffsetEstimator).
// It stops while the machine is suspended, as MONOTONIC does.
double daemonNow();

// CLOCK_REALTIME: the host's own clock. Read only to REPORT how far it is from
// the served time, and for the decoder's day-scale plausibility reference,
// never to form a timestamp.
double realtimeNow();

// CLOCK_MONOTONIC: timeouts and intervals, which want neither a step of the
// host clock nor anything else from this file.
double monotonicNow();

// daemonNow() minus realtimeNow(), now: add it to a host-clock instant to get
// the daemon-clock instant. Read daemon / real / daemon and averaged, so a
// preemption between the reads is halved rather than counted whole.
double daemonMinusRealtime();

// The resolution the daemon clock is read to, in seconds.
double daemonClockResolution();

} // namespace ubersdr_ntp
