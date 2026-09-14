#include "WwvDecoder.h"

#include "CivilTime.h"

// WWV/WWVH 100 Hz-subcarrier BCD time-code decoder — streaming implementation.
//
// Ports the MATH of the gate-passed reference chain
// (research/wwv_decode_proto.py) to a sample-streaming front-end: the
// prototype's whole-file FFT stages become biquad cascades + running mixers +
// decimated per-second work, per the NIST WWV/WWVH time-code table (NIST
// SP 432) and the reference chain documented in WwvDecoder.h.
//
// Chain (identical intent to the prototype, streaming realization):
//   analytic bandpass 700-1300 Hz (biquad cascade) -> rectify+LPF envelope
//   -> coherent 100 Hz demod (running quadrature mixer, 25 Hz LPF both rails,
//      magnitude) -> decimate to a 200 Hz amplitude series a[]
//   -> tick rail: bandpass 2000 Hz (WWV) / 2200 Hz (WWVH), envelope, decimate,
//      fold mod 1 s for tick phase + station tag
//   -> per-second matched-filter classify (zero-mean 170/470/770 ms templates
//      at +30 ms; confidence = best correlation minus runner-up)
//   -> marker frame sync (P markers at 9/19/29/39/49/59), mod-10 degeneracy
//      resolved by the s0 minute-mark subcarrier hole + minute-increment
//      scoring -> NIST BCD field map -> TimeFrameVoter.

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <deque>
#include <limits>
#include <vector>

namespace clockdec {

namespace {

// ---- NIST WWV/WWVH BCD field map (second, weight) — per NIST SP 432. -------
// LSB-first within each field; a field's value is the sum of the weights of
// the seconds decoded as binary One.
const ClockFieldMap kMin = {{10, 1}, {11, 2}, {12, 4}, {13, 8},
                            {15, 10}, {16, 20}, {17, 40}};
const ClockFieldMap kHr  = {{20, 1}, {21, 2}, {22, 4}, {23, 8}, {25, 10}, {26, 20}};
const ClockFieldMap kDoy = {{30, 1},  {31, 2},  {32, 4},  {33, 8},  {35, 10},
                            {36, 20}, {37, 40}, {38, 80}, {40, 100}, {41, 200}};
const ClockFieldMap kYr  = {{4, 1},  {5, 2},  {6, 4},  {7, 8},
                            {51, 10}, {52, 20}, {53, 40}, {54, 80}};

constexpr int kSeriesRate = 200;                 // decimated series rate (Hz)
constexpr int kSecLen     = kSeriesRate;          // samples per broadcast second
constexpr int kFrameSecs  = 60;

// Nominal biquad-chain group delay at 200 Hz (~35 ms) — the matched-filter
// shift a clean, drift-free stream settles at. The tracked delay estimate
// starts here, and the reported second edge subtracts it back out so the edge
// label follows REAL stream drift, not the fixed chain delay.
constexpr int kNominalDelaySamples = 7;

// Matched-filter shift search ceiling (series samples, 200 ms). Wide (WS-4.5)
// so slow sample-clock drift is ABSORBED by the tracked delay estimate instead
// of accumulating into systematic misreads; drift beyond the rails triggers a
// soft resynchronization.
constexpr int kMaxShift = 40;

// Leaky tick-fold decay per hit (each phase bin is hit once per second, so
// this is τ ≈ 100 s). The fold must FORGET a stale phase: after a sample
// discontinuity the re-seed has to find the CURRENT tick phase, not history.
constexpr double kFoldDecay = 0.99;

// Station tag (see onSeriesSample): one band's folded tick excess must be this
// many times the other's for a verdict. A single station's own tick leaks into
// the other band at roughly a third of its excess (3.2x clean, never below
// 1.77x through Opus with a programme tone in offline measurement), so 1.5x
// separates the stations with margin while staying undecided when both are
// heard at similar strength.
constexpr double kStationExcessRatio = 1.5;
// What still SUPPORTS a tag once it is established: the tagged band ahead by
// this much, below the 1.5x needed to adopt one. A single ratio for both let a
// signal hovering near 1.5x -- both stations heard, one a little stronger --
// fail to support its own tag for two minutes, release it, then re-adopt it,
// over and over. Still above 1.0, so a band that has genuinely fallen level
// with the other stops supporting it.
constexpr double kStationHoldRatio = 1.2;
// Station-tag timing, all in once-per-second verdicts (see onSeriesSample):
// fold age past tick lock before any verdict counts; identical verdicts to
// adopt a first tag; consecutive contrary verdicts to switch an established
// one (longer than a fade, far shorter than a propagation change); seconds
// without a supporting verdict before a tag is released to Unknown.
constexpr int kStationWarmSecs    = 20;
constexpr int kStationConfirmSecs = 10;
constexpr int kStationSwitchSecs  = 30;
constexpr int kStationReleaseSecs = 120;

// Reported second edge. Each second's own best matched-filter shift is a
// noisy, 5 ms-quantised measurement (it moves +/-3 series samples = +/-15 ms
// under noise); reporting it directly put that jitter on every edge. The
// reported edge instead follows a separate smoothed, sub-sample estimate of
// the same shift: the first kEdgeDelayWarm timed seconds form a running mean
// (a fast start), then an EMA with this weight (tau ~16 s, far faster than any
// sample-clock drift the tracked delay has to follow). Past the warm-up a
// shift more than kEdgeDelayGate series samples from the estimate is a misfit,
// not drift, and is ignored.
constexpr double kEdgeDelayAlpha = 1.0 / 16.0;
constexpr int    kEdgeDelayWarm  = 8;
constexpr double kEdgeDelayGate  = 3.0;

// A second only contributes timing if it actually carried a pulse: its mean
// amplitude must reach this fraction of the running peak. A 170 ms binary-0
// pulse averages ~0.17 of the peak over the second; the minute hole, with no
// subcarrier at all, averages near zero.
constexpr double kPulseEnergyFrac = 0.05;

// Minimum matched-filter margin for a single second to count as evidence that
// the frame alignment has slipped. A clean marker clears it with ease; a
// coin-flip read under noise does not throw away a good lock.
constexpr float kStructConf = 0.10f;

// Markers (P1..P5, P0) land at seconds 9/19/29/39/49/59 — i.e. (s % 10 == 9).
inline bool isMarkerSec(int s) { return (s % 10) == 9; }

// ---- Transposed Direct-Form-II biquad (RBJ cookbook coefficients). ---------
// MSVC <cmath> does not define M_PI without _USE_MATH_DEFINES; repo core
// convention is a local constant (Biquad/ClientEq/ClientPhaseRotator).
constexpr double kPi = 3.14159265358979323846;

struct Biquad {
    double b0 = 1.0, b1 = 0.0, b2 = 0.0, a1 = 0.0, a2 = 0.0;
    double z1 = 0.0, z2 = 0.0;
    inline double process(double x) {
        double y = b0 * x + z1;
        z1 = b1 * x - a1 * y + z2;
        z2 = b2 * x - a2 * y;
        return y;
    }
    void reset() { z1 = z2 = 0.0; }
};

Biquad designBandpass(double f0, double q, double fs) {
    // RBJ constant-0dB-peak bandpass.
    double w0 = 2.0 * kPi * f0 / fs;
    double c = std::cos(w0), s = std::sin(w0);
    double alpha = s / (2.0 * q);
    double a0 = 1.0 + alpha;
    Biquad bq;
    bq.b0 = alpha / a0;
    bq.b1 = 0.0;
    bq.b2 = -alpha / a0;
    bq.a1 = (-2.0 * c) / a0;
    bq.a2 = (1.0 - alpha) / a0;
    return bq;
}

Biquad designLowpass(double fc, double q, double fs) {
    double w0 = 2.0 * kPi * fc / fs;
    double c = std::cos(w0), s = std::sin(w0);
    double alpha = s / (2.0 * q);
    double a0 = 1.0 + alpha;
    Biquad bq;
    bq.b0 = ((1.0 - c) / 2.0) / a0;
    bq.b1 = (1.0 - c) / a0;
    bq.b2 = ((1.0 - c) / 2.0) / a0;
    bq.a1 = (-2.0 * c) / a0;
    bq.a2 = (1.0 - alpha) / a0;
    return bq;
}

} // namespace

// ---------------------------------------------------------------------------

struct WwvDecoder::Impl {
    explicit Impl(int sampleRateHz);

    // Public-surface state.
    int fs;
    int decim;                       // input samples per 200 Hz series sample
    ClockLockState state = ClockLockState::NoSignal;
    ClockStation station = ClockStation::Unknown;
    std::int64_t samplesConsumed = 0;

    // Callbacks (owned by the outer WwvDecoder; copied pointers here).
    WwvDecoder* owner = nullptr;

    // --- Streaming front-end filters (preallocated, no per-sample alloc). ---
    std::array<Biquad, 2> bpMain;    // analytic bandpass 700-1300 Hz
    std::array<Biquad, 2> lpEnv;     // envelope smoothing
    std::array<Biquad, 2> lpI;       // 25 Hz LPF, in-phase rail
    std::array<Biquad, 2> lpQ;       // 25 Hz LPF, quadrature rail
    std::array<Biquad, 2> bpTickV;   // WWV  tick band (2000 Hz)
    std::array<Biquad, 2> bpTickH;   // WWVH tick band (2200 Hz)

    // 100 Hz quadrature oscillator (running rotation — no per-sample trig).
    double oscC = 1.0, oscS = 0.0, rotC = 1.0, rotS = 0.0;
    int oscRenorm = 0;

    // Decimation accumulators.
    double accA = 0.0, accTickV = 0.0, accTickH = 0.0;
    int decCount = 0;
    std::int64_t n200 = 0;           // count of 200 Hz series samples emitted

    // Tick-phase fold (mod 1 s). Station is tagged by folded IMPULSIVENESS
    // (peak-to-mean of each tick band), not total energy: WWV voice
    // announcements dump broadband energy that lands in the 2200 Hz WWVH band
    // and would out-power a plain energy comparison, but voice is phase-
    // incoherent and folds flat, whereas the real 5 ms tick folds to a sharp
    // peak at a fixed phase.
    std::array<double, kSecLen> foldV{};
    std::array<double, kSecLen> foldH{};
    bool tickLocked = false;
    int tickPhase = 0;               // series-sample index of the second edge
    std::int64_t tickLockJ = 0;      // series index at which the tick locked
    bool stationPinned = false;      // fixed by the carrier; never judged (pinStation)
    ClockStation pendingStation = ClockStation::Unknown;  // verdict awaiting confirmation
    int pendingCount = 0;            // consecutive identical verdicts for pendingStation
    int stationContrary = 0;         // consecutive decisive verdicts against the tag
    int stationUnsupported = 0;      // consecutive verdicts not supporting the tag
    double tickExcessRatio = 0.0;    // latest 2000 Hz / 2200 Hz tick excess

    // Slowly-adapted matched-filter delay estimate: nominal chain group delay
    // plus any accumulated stream drift. Tracked slowly instead of re-searched
    // freely every second (which flaps under fading/noise); reaching the
    // search rails means drift exceeded what the window can absorb -> soft
    // resynchronization.
    double delayEst = kNominalDelaySamples;   // series samples at 200 Hz
    bool delayLocked = false;
    int delayCount = 0;

    // Smoothed sub-sample matched-filter shift the reported edge is built on
    // (see kEdgeDelayAlpha). Separate from delayEst, which steers the search
    // band and must keep its own dynamics.
    double edgeDelayEst = kNominalDelaySamples;
    int edgeDelayCount = 0;
    int edgeDelayRejects = 0;        // consecutive timed seconds gated out

    // Consecutive structurally-invalid frames (marker skeleton broken).
    int badFrameStreak = 0;

    // Slip / leap-second handling.
    std::int64_t suspectSec = -1;     // latest secIndex whose symbol contradicted the frame
    std::int64_t lastRealignK = -1;   // frame start produced by the last +/-1 s realignment
    bool haveLastFields = false;      // lastFields holds the last completed minute
    TimeFields lastFields;
    bool lastLeapWarn = false;        // previous frame's leap-warning bit

    // Per-second windowing at 200 Hz.
    std::array<float, kSecLen> curSec{};
    int curFill = 0;
    bool secStarted = false;
    std::int64_t secStartJ = 0;      // 200 Hz index of current window start
    double aScale = 1e-6;            // running peak of a[] for display normalize

    // Matched-filter templates (zero-mean) + norms.
    std::array<std::array<float, kSecLen>, 3> tpl{};
    std::array<float, 3> tplNorm{};

    // --- Per-second record ring for frame anchoring. -----------------------
    struct Rec {
        std::int64_t secIndex;
        std::int64_t edgeSample;
        int8_t sym;                  // 0 Zero / 1 One / 2 Marker
        float conf;
        float energy;                // mean a[] over the second (s0 hole => ~0)
    };
    static constexpr std::size_t kRecCap = 12 * kFrameSecs;
    std::deque<Rec> recs;
    std::int64_t recBase = 0;        // secIndex of recs.front()
    std::int64_t secIndex = 0;       // next second index to assign

    // Frame anchor / assembly.
    bool anchored = false;
    std::int64_t anchorSec0 = 0;     // secIndex whose second-of-frame == 0
    std::int64_t nextFrameStartK = 0;
    std::int64_t lastEdgeSample = 0;
    int lastEdgeSecondOfFrame = -1;

    // Cross-frame voter.
    TimeFrameVoter voter;

    // --- Methods. ----------------------------------------------------------
    void designFilters();
    void buildTemplates();
    void setState(ClockLockState s);
    void processSample(float x);
    void onSeriesSample(double a, double tickV, double tickH);
    void processSecond(std::int64_t startJ, const std::array<float, kSecLen>& w);
    void classify(const std::array<float, kSecLen>& w, int8_t& sym, float& conf,
                  int& winShift, double& fracShift) const;
    bool leapSecondPossible() const;
    void tryAnchor();
    void feedPendingFrames();
    void softReacquire();
    ClockFrameInfo decodeFrame(const std::array<int8_t, kFrameSecs>& sym,
                               const std::array<float, kFrameSecs>& conf,
                               std::int64_t frameStartSample) const;
    void reset();

    static TimeFrameVoter::Config makeVoterConfig();
};

TimeFrameVoter::Config WwvDecoder::Impl::makeVoterConfig() {
    TimeFrameVoter::Config c;
    c.fields[TimeFrameVoter::FieldMinutes] = kMin;
    c.fields[TimeFrameVoter::FieldHours]   = kHr;
    c.fields[TimeFrameVoter::FieldDoy]     = kDoy;
    c.fields[TimeFrameVoter::FieldYear]    = kYr;
    c.markerSeconds = {9, 19, 29, 39, 49, 59};
    c.window = 8;
    c.minFramesForLock = 2;
    c.agingFactor = 0.9f;
    // WS-4.5 honesty floors: noise-grade margins (clean-signal bits run
    // >= ~0.12 even at the pinned 20 dB SNR floor) must not vote — a deep fade
    // zero-biases the SAME bits in every frame, and unanimity of noise-grade
    // reads is how the 2026-07-20 receiver certified 2006-01-01 at q100. The
    // lock-quality floor sits BELOW the dirty-but-correct band measured on the
    // 2026-07-19 live corpus (q ~0.11-0.16) and far above unanimous-fade
    // garbage (~0.00).
    c.minBitConfidence = 0.05f;
    c.minLockQuality   = 0.05f;
    return c;
}

WwvDecoder::Impl::Impl(int sampleRateHz)
    : fs(sampleRateHz > 0 ? sampleRateHz : 24000),
      decim(std::max(1, (sampleRateHz > 0 ? sampleRateHz : 24000) / kSeriesRate)),
      voter(makeVoterConfig()) {
    designFilters();
    buildTemplates();
}

void WwvDecoder::Impl::designFilters() {
    double f = static_cast<double>(fs);
    // Analytic bandpass 700-1300 Hz: pass the 1000 Hz carrier + 900/1100 Hz
    // subcarrier sidebands, reject the 2000/2200 Hz tick image and out-of-band
    // noise. Two wide sections keep the sidebands while limiting noise.
    for (auto& b : bpMain) b = designBandpass(1000.0, 1.0, f);
    // Envelope smoothing: pass the 100 Hz subcarrier modulation, kill the
    // rectified-carrier ripple (2000 Hz and up).
    for (auto& b : lpEnv) b = designLowpass(180.0, 0.70710678, f);
    // Coherent-demod baseband LPF (25 Hz) — matches the prototype's 25 Hz mask.
    for (auto& b : lpI) b = designLowpass(25.0, 0.70710678, f);
    for (auto& b : lpQ) b = designLowpass(25.0, 0.70710678, f);
    // Tick bands: narrow so 2000 Hz (WWV) and 2200 Hz (WWVH) separate cleanly.
    for (auto& b : bpTickV) b = designBandpass(2000.0, 12.0, f);
    for (auto& b : bpTickH) b = designBandpass(2200.0, 12.0, f);

    double dth = 2.0 * kPi * 100.0 / f;   // 100 Hz demod rotation per sample
    rotC = std::cos(dth);
    rotS = std::sin(dth);
}

void WwvDecoder::Impl::buildTemplates() {
    // Zero-mean matched templates: pulse rises +30 ms into the second and lasts
    // 170 ms (binary 0), 470 ms (binary 1) or 770 ms (marker) — NIST SP 432.
    const int s0 = static_cast<int>(std::lround(0.030 * kSeriesRate));
    const int durMs[3] = {170, 470, 770};
    for (int k = 0; k < 3; ++k) {
        auto& t = tpl[k];
        t.fill(0.0f);
        int len = static_cast<int>(std::lround(durMs[k] / 1000.0 * kSeriesRate));
        for (int i = s0; i < s0 + len && i < kSecLen; ++i) t[i] = 1.0f;
        double mean = 0.0;
        for (float v : t) mean += v;
        mean /= kSecLen;
        double nrm = 0.0;
        for (float& v : t) { v -= static_cast<float>(mean); nrm += double(v) * v; }
        tplNorm[k] = static_cast<float>(std::sqrt(nrm));
    }
}

void WwvDecoder::Impl::setState(ClockLockState s) {
    if (s == state) return;
    state = s;
    if (owner && owner->onStateChanged) owner->onStateChanged(s);
}

void WwvDecoder::Impl::processSample(float x) {
    ++samplesConsumed;

    // 1) Analytic bandpass 700-1300 Hz, then rectify + LPF -> AM envelope. The
    //    envelope carries the 100 Hz subcarrier as a DC + 100 Hz component.
    double bp = x;
    for (auto& b : bpMain) bp = b.process(bp);
    double env = std::fabs(bp);
    for (auto& b : lpEnv) env = b.process(env);

    // 2) Coherent 100 Hz demod: mix envelope down by the running quadrature
    //    oscillator, LPF both rails at 25 Hz, take the magnitude.
    double i = env * oscC;
    double q = env * oscS;
    for (auto& b : lpI) i = b.process(i);
    for (auto& b : lpQ) q = b.process(q);
    double a = std::sqrt(i * i + q * q);

    // advance + periodically renormalize the oscillator
    double nc = oscC * rotC - oscS * rotS;
    double ns = oscS * rotC + oscC * rotS;
    oscC = nc; oscS = ns;
    if (++oscRenorm >= 1024) {
        oscRenorm = 0;
        double m = std::sqrt(oscC * oscC + oscS * oscS);
        if (m > 0.0) { oscC /= m; oscS /= m; }
    }

    // 3) Tick rails: bandpass around each tick image, rectify.
    double tv = x, th = x;
    for (auto& b : bpTickV) tv = b.process(tv);
    for (auto& b : bpTickH) th = b.process(th);
    double etv = std::fabs(tv), eth = std::fabs(th);

    // 4) Decimate by block-average to the 200 Hz series (matches prototype).
    accA += a; accTickV += etv; accTickH += eth;
    if (++decCount >= decim) {
        double invd = 1.0 / decim;
        onSeriesSample(accA * invd, accTickV * invd, accTickH * invd);
        accA = accTickV = accTickH = 0.0;
        decCount = 0;
    }
}

void WwvDecoder::Impl::onSeriesSample(double a, double tickV, double tickH) {
    const std::int64_t j = n200++;

    // Fold each tick band's envelope mod 1 s — leaky, so a stale phase decays
    // and a post-discontinuity re-seed finds the CURRENT phase (WS-4.5).
    int phase = static_cast<int>(j % kSecLen);
    foldV[phase] = foldV[phase] * kFoldDecay + tickV;
    foldH[phase] = foldH[phase] * kFoldDecay + tickH;

    // Folded impulsiveness of a band: peak-to-mean ratio + argmax phase.
    auto stats = [](const std::array<double, kSecLen>& fold, double& ratio, int& arg) {
        double peak = 0.0, sum = 0.0; arg = 0;
        for (int p = 0; p < kSecLen; ++p) {
            sum += fold[p];
            if (fold[p] > peak) { peak = fold[p]; arg = p; }
        }
        double mean = sum / kSecLen;
        ratio = (mean > 0.0) ? peak / mean : 0.0;
    };

    // Tick EXCESS of a band: what its folded peak (argmax bin and both
    // neighbours, so a tick straddling two 5 ms bins counts whole) holds above
    // the band's own background (median bin).
    //
    // Station is decided by comparing excesses, not the bands' peak-to-mean
    // ratios. The 5 ms tick at 2000 Hz leaks into the 2200 Hz filter at about
    // -10 dB, at the same phase, so a WWV-only signal shows a real impulse in
    // BOTH bands; peak-to-mean is scale-free per band, so which band "wins"
    // was decided by the two bands' backgrounds rather than by where the tick
    // is. Measured offline on synthetic WWV only: clean, the old margin was
    // 1.47x against a 1.3x gate; with WWV's own 600 Hz programme tone it was
    // 1.36x; through Opus at 16 kbps plus that tone the 2200 Hz band came out
    // MORE impulsive than 2000 Hz at some tick phases (30.4 vs 28.6), and at
    // 17 of 48 phase/codec/tone combinations no tag ever formed. The excess
    // ratio stayed >= 1.77x for WWV in every one of those cases.
    auto tickExcess = [](const std::array<double, kSecLen>& fold, int arg) {
        std::array<double, kSecLen> tmp = fold;
        std::nth_element(tmp.begin(), tmp.begin() + kSecLen / 2, tmp.end());
        const double median = tmp[kSecLen / 2];
        double e = 0.0;
        for (int d = -1; d <= 1; ++d) e += fold[(arg + d + kSecLen) % kSecLen] - median;
        return std::max(0.0, e);
    };
    auto tickVerdict = [&](double eV, double eH) {
        if (eV >= kStationExcessRatio * eH && eV > 0.0) return ClockStation::Wwv;
        if (eH >= kStationExcessRatio * eV && eH > 0.0) return ClockStation::Wwvh;
        return ClockStation::Unknown;
    };

    if (!tickLocked && j >= 5 * kSecLen) {
        double rV, rH; int argV, argH;
        stats(foldV, rV, argV);
        stats(foldH, rH, argH);
        // Lock once either band folds to a genuine impulse (a flat fold from
        // noise or voice never clears the ratio gate). The tick phase comes from
        // the band that actually carries the tick -- the larger excess -- not
        // from whichever band looks peakier, which can be the leakage band.
        if (std::max(rV, rH) > 2.5) {
            const double eV = tickExcess(foldV, argV);
            const double eH = tickExcess(foldH, argH);
            tickLocked = true;
            tickLockJ = j;
            tickPhase = (eV >= eH) ? argV : argH;
            // No station tag yet: five hits is enough to find the tick's phase,
            // not to judge which band holds it (see the per-second block).
            setState(ClockLockState::Acquiring);
        }
    }

    // Judge the station once per second for as long as the stream runs, once
    // the fold has integrated kStationWarmSecs past tick lock. The fixed-phase
    // tick keeps sharpening its band's fold while phase-incoherent voice
    // averages flat, and propagation can hand a shared frequency from one
    // station to the other. The tag used to be set once, from the five-second
    // fold at tick lock, and never revisited: offline, one Opus-coded WWV run
    // read WWVH (-1.8 dB) at five seconds, then settled at +0.3..+1.4 dB -- WWV,
    // but never decisively -- and kept the wrong tag for the whole stream.
    //   - Unknown -> a station: kStationConfirmSecs consecutive identical
    //     decisive verdicts.
    //   - a station -> the other: kStationSwitchSecs consecutive decisive
    //     contrary verdicts, so a fade does not flap the propagation model.
    //   - a station -> Unknown: kStationReleaseSecs with no second supporting
    //     it, where support is the tagged band ahead by kStationHoldRatio. The
    //     engine models Unknown on a shared frequency as WWV, the dial's
    //     default, which is better than holding a tag nothing backs.
    // A preset tag (presetStation) enters as established and is held to the
    // same rules; a pinned one is not judged at all.
    if (tickLocked && phase == 0) {
        double rV, rH; int argV, argH;
        stats(foldV, rV, argV);
        stats(foldH, rH, argH);
        const double eV = tickExcess(foldV, argV);
        const double eH = tickExcess(foldH, argH);
        tickExcessRatio = (eH > 0.0) ? eV / eH : std::numeric_limits<double>::infinity();
        const ClockStation v = tickVerdict(eV, eH);
        const bool supported =
            (station == ClockStation::Wwv && eV > 0.0 && eV >= kStationHoldRatio * eH) ||
            (station == ClockStation::Wwvh && eH > 0.0 && eH >= kStationHoldRatio * eV);
        if (stationPinned || j - tickLockJ < kStationWarmSecs * kSecLen) {
            // Fixed by the carrier, or the fold is still too young to judge.
        } else if (station == ClockStation::Unknown) {
            if (v != ClockStation::Unknown && v == pendingStation) {
                if (++pendingCount >= kStationConfirmSecs) {
                    station = v;
                    stationContrary = stationUnsupported = 0;
                    pendingCount = 0;
                }
            } else {
                pendingStation = v;
                pendingCount = (v != ClockStation::Unknown) ? 1 : 0;
            }
        } else if (supported) {
            stationContrary = stationUnsupported = 0;
        } else {
            ++stationUnsupported;
            stationContrary = (v != ClockStation::Unknown) ? stationContrary + 1 : 0;
            if (stationContrary >= kStationSwitchSecs) {
                station = v;
                stationContrary = stationUnsupported = 0;
            } else if (stationUnsupported >= kStationReleaseSecs) {
                station = ClockStation::Unknown;
                stationContrary = stationUnsupported = 0;
                pendingStation = ClockStation::Unknown;
                pendingCount = 0;
            }
        }
    }

    if (a > aScale) aScale = a;
    aScale *= 0.99999;               // slow decay so display normalization adapts

    if (!tickLocked) return;

    // Cut a[] into 1 s windows aligned to the tick phase.
    bool boundary = (((j - tickPhase) % kSecLen) == 0) && (j >= tickPhase);
    if (boundary) {
        if (secStarted && curFill == kSecLen) processSecond(secStartJ, curSec);
        curFill = 0;
        secStarted = true;
        secStartJ = j;
    }
    if (secStarted && curFill < kSecLen) curSec[curFill++] = static_cast<float>(a);
}

void WwvDecoder::Impl::classify(const std::array<float, kSecLen>& w,
                                int8_t& sym, float& conf, int& winShift,
                                double& fracShift) const {
    // Normalized correlation against each zero-mean template; symbol = best,
    // confidence = best minus runner-up (the prototype's classify() margin).
    double mean = 0.0;
    for (float v : w) mean += v;
    mean /= kSecLen;
    double vnorm = 0.0;
    std::array<double, kSecLen> v{};
    for (int n = 0; n < kSecLen; ++n) { v[n] = w[n] - mean; vnorm += v[n] * v[n]; }
    vnorm = std::sqrt(vnorm);
    double invn = 1.0 / (vnorm + 1e-12);

    // Streaming biquads add a FIXED group delay the prototype's zero-phase FFT
    // filters did not, so the received pulse sits a few samples late — and any
    // sample-clock drift between the DAX stream and true UTC seconds shifts it
    // further. All three 170/470/770 ms templates are correlated at a COMMON
    // start-shift each second (a fair duration comparison — a longer template
    // never wins on a short pulse), and the shift is picked to best explain the
    // second. Once the delay estimate has settled, the search is constrained to
    // a narrow band around it so the alignment can't flap second-to-second
    // under fading, while the estimate itself keeps tracking slow drift.
    int lo = 0, hi = kMaxShift;
    if (delayLocked) {
        int c = static_cast<int>(std::lround(delayEst));
        lo = std::max(0, c - 3);
        hi = std::min(kMaxShift, c + 3);
    }

    auto score = [&](int k, int d) {
        double dot = 0.0;
        for (int n = d; n < kSecLen; ++n) dot += v[n] * tpl[k][n - d];
        return dot * invn / (tplNorm[k] + 1e-12);
    };

    double bestScore = -1e30, scStar[3] = {0, 0, 0};
    winShift = lo;
    for (int d = lo; d <= hi; ++d) {
        double sc[3];
        for (int k = 0; k < 3; ++k) sc[k] = score(k, d);
        double m = std::max({sc[0], sc[1], sc[2]});
        if (m > bestScore) {
            bestScore = m; winShift = d;
            scStar[0] = sc[0]; scStar[1] = sc[1]; scStar[2] = sc[2];
        }
    }

    int best = 0;
    for (int k = 1; k < 3; ++k) if (scStar[k] > scStar[best]) best = k;
    double runner = -1e30;
    for (int k = 0; k < 3; ++k) if (k != best && scStar[k] > runner) runner = scStar[k];
    sym = static_cast<int8_t>(best);
    conf = static_cast<float>(std::max(0.0, scStar[best] - runner));

    // Sub-sample position of the correlation peak: a parabola through the
    // winning template's score at the best shift and its two neighbours. Used
    // only for the reported edge -- the 5 ms series grid is coarser than the
    // timing this decoder is asked for, and the peak's shape carries the rest.
    // Neighbours are evaluated even outside the constrained search band, which
    // bounds where the peak may be looked for, not where its slope is sampled.
    fracShift = 0.0;
    if (winShift > 0 && winShift < kSecLen - 1) {
        const double sm = score(best, winShift - 1);
        const double sp = score(best, winShift + 1);
        const double den = sm - 2.0 * scStar[best] + sp;
        if (den < 0.0) fracShift = std::clamp(0.5 * (sm - sp) / den, -0.5, 0.5);
    }
}

bool WwvDecoder::Impl::leapSecondPossible() const {
    // Leap seconds are only ever inserted as 23:59:60 UTC on the last day of a
    // month, so the only minute a leap second can follow is 23:59 on such a day.
    if (!haveLastFields) return false;
    const TimeFields& f = lastFields;
    if (f.minute != 59 || f.hour != 23) return false;
    if (f.doy < 1 || f.doy > 366 || f.year2 < 0 || f.year2 > 99) return false;
    return ubersdr_ntp::isLastDayOfMonth(
        ubersdr_ntp::utcMsFromFields(f.year2, f.doy, f.hour, f.minute));
}

void WwvDecoder::Impl::processSecond(std::int64_t startJ,
                                     const std::array<float, kSecLen>& w) {
    int8_t sym; float conf; int winShift = 0; double fracShift = 0.0;
    classify(w, sym, conf, winShift, fracShift);

    // Slowly adapt the delay estimate toward the alignment that confident
    // seconds actually used; constrain the search band once it settles. The
    // estimate keeps moving after settling — that is what absorbs slow
    // sample-clock drift (WS-4.5).
    if (conf > 0.12f) {
        delayEst = 0.85 * delayEst + 0.15 * winShift;
        if (++delayCount >= 4) delayLocked = true;
    }

    // Drift beyond the search rails cannot be absorbed — the window itself is
    // wrong. Resynchronize instead of degrading into systematic misreads (the
    // 2026-07-20 misalignment failure mode).
    if (delayLocked && (delayEst < 1.5 || delayEst > kMaxShift - 1.5)) {
        softReacquire();
        return;
    }

    double emean = 0.0;
    for (float v : w) emean += v;
    emean /= kSecLen;

    const std::int64_t k = secIndex;
    int secOfFrame = anchored
        ? static_cast<int>(((k - anchorSec0) % kFrameSecs + kFrameSecs) % kFrameSecs)
        : -1;

    // Only a second that carried a pulse, and read confidently, measures the
    // edge. The minute hole has nothing to align to: its "best" shift is
    // whatever the noise favoured.
    const bool holeSecond = anchored && secOfFrame == 0;
    const bool timed = conf > 0.12f && !holeSecond && emean > kPulseEnergyFrac * aScale;
    if (timed) {
        const double shift = winShift + fracShift;
        if (edgeDelayCount < kEdgeDelayWarm ||
            std::fabs(shift - edgeDelayEst) <= kEdgeDelayGate) {
            edgeDelayRejects = 0;
            ++edgeDelayCount;
            const double alpha = std::max(1.0 / edgeDelayCount, kEdgeDelayAlpha);
            edgeDelayEst += alpha * (shift - edgeDelayEst);
        } else if (++edgeDelayRejects >= kEdgeDelayWarm) {
            // Not a misfit but a real move the search band has already
            // followed: gating it out forever would freeze the reported edge on
            // a delay the stream no longer has. Start the estimate over.
            edgeDelayRejects = 0;
            edgeDelayCount = 1;
            edgeDelayEst = shift;
        }
    }

    // The second-edge label subtracts the nominal chain delay back out of the
    // smoothed matched shift, so it tracks REAL stream drift: on a clean
    // drift-free stream the shift settles at kNominalDelaySamples and this
    // reduces to the window start, unchanged from pre-WS-4.5 behavior. Same
    // calibration as the old per-second (startJ + winShift - nominal) form --
    // the estimate converges on the mean of those shifts -- minus their jitter
    // and 5 ms quantisation.
    const double reportDelay = edgeDelayCount > 0 ? edgeDelayEst : delayEst;
    const std::int64_t edgeSample = static_cast<std::int64_t>(std::llround(
        (static_cast<double>(startJ) + reportDelay - kNominalDelaySamples) * decim));

    // Slip detection, BEFORE this second is emitted. The per-frame skeleton
    // check below only runs once a minute, and every second until then would
    // be labelled from a second count that is no longer right. A confident
    // marker one second after its slot (with the slot itself not a marker), or
    // one second before it, is a slipped count -- an unannounced leap second,
    // or a second lost or duplicated in the stream. Straight after a possible
    // leap-second minute, the minute hole landing on s1 instead of s0 (s1 is
    // always a pulsed binary 0) says the same thing within one second instead
    // of ten. Either way stop certifying time now and let the frame decide.
    if (anchored && secOfFrame > 0 && !recs.empty() && recs.back().secIndex == k - 1) {
        const Rec& prev = recs.back();
        const bool late = conf >= kStructConf && sym == 2 && secOfFrame % 10 == 0 &&
                          prev.sym != 2;
        const bool early = conf >= kStructConf && sym != 2 && isMarkerSec(secOfFrame) &&
                           prev.sym == 2 && prev.conf >= kStructConf;
        const bool leapHole = secOfFrame == 1 && leapSecondPossible() &&
                              emean < 0.5 * prev.energy;
        if (late || early || leapHole) {
            suspectSec = k;
            if (state == ClockLockState::Locked) setState(ClockLockState::Acquiring);
        }
    }

    lastEdgeSample = edgeSample;
    lastEdgeSecondOfFrame = secOfFrame;

    // Emit the classified second (drives the alignment display).
    if (owner && owner->onSecond) {
        ClockSecondInfo info;
        info.edgeSample = edgeSample;
        info.edgeMeasured = timed;
        info.symbol = static_cast<ClockSymbol>(sym);
        info.confidence = conf;
        info.secondOfFrame = secOfFrame;
        info.seriesRateHz = kSeriesRate;
        info.windowShift = winShift - kNominalDelaySamples;
        info.envelope.resize(kSecLen);
        double s = (aScale > 1e-9) ? (1.0 / aScale) : 0.0;
        for (int n = 0; n < kSecLen; ++n)
            info.envelope[n] = static_cast<float>(std::min(1.5, w[n] * s));
        info.expected.assign(tpl[sym].begin(), tpl[sym].end());
        owner->onSecond(info);
    }

    // Record for frame anchoring.
    recs.push_back(Rec{k, edgeSample, sym, conf, static_cast<float>(emean)});
    if (recs.size() > kRecCap) { recs.pop_front(); ++recBase; }
    ++secIndex;

    if (!anchored) tryAnchor();
    feedPendingFrames();
}

void WwvDecoder::Impl::tryAnchor() {
    // Marker-only anchoring is degenerate mod 10 s; disambiguate with the s0
    // subcarrier hole (second 0 has NO subcarrier -> ~0 energy) and minute-
    // increment scoring across frames. Gate on structure so noise never anchors.
    const int M = static_cast<int>(recs.size());
    if (M < 2 * kFrameSecs) return;

    int bestScore = -1 << 30, bestOff = -1, bestMarker = 0, bestInc = 0, bestHole = 0;

    for (int off = 0; off < kFrameSecs; ++off) {
        int nf = (M - off) / kFrameSecs;
        if (nf < 2) continue;

        int markerScore = 0, holeScore = 0;
        std::vector<int> minutes;
        minutes.reserve(nf);

        for (int t = 0; t < nf; ++t) {
            int base = off + kFrameSecs * t;
            // marker agreement
            for (int s = 0; s < kFrameSecs; ++s) {
                if (recs[base + s].sym == 2) markerScore += isMarkerSec(s) ? 2 : -1;
            }
            // s0 hole: energy[0] a clear low outlier vs the frame's pulse energy
            double e0 = recs[base + 0].energy;
            double meanE = 0.0, minE = 1e30;
            for (int s = 1; s < kFrameSecs; ++s) {
                double e = recs[base + s].energy;
                meanE += e;
                if (e < minE) minE = e;
            }
            meanE /= (kFrameSecs - 1);
            if (e0 < 0.5 * meanE && e0 <= minE + 1e-9) ++holeScore;
            // per-frame minute decode
            int mv = 0;
            for (const auto& bw : kMin)
                if (recs[base + bw.second].sym == 1) mv += bw.weight;
            minutes.push_back(mv);
        }
        int inc = 0;
        for (int t = 0; t + 1 < nf; ++t)
            if (minutes[t + 1] - minutes[t] == 1) ++inc;

        int score = markerScore + 4 * inc + 3 * holeScore;
        if (score > bestScore) {
            bestScore = score; bestOff = off;
            bestMarker = markerScore; bestInc = inc; bestHole = holeScore;
        }
    }

    // Structural gate: real marker agreement, at least one minute increment, and
    // a detected s0 hole. A 10 s-shifted anchor lands a pulse on second 0 (no
    // hole) and misdecodes minutes (no increment), so it loses this gate.
    if (bestOff >= 0 && bestMarker > 0 && bestInc >= 1 && bestHole >= 1) {
        anchored = true;
        anchorSec0 = recBase + bestOff;
        nextFrameStartK = anchorSec0;
    }
}

void WwvDecoder::Impl::feedPendingFrames() {
    if (!anchored) return;
    const std::int64_t maxComplete = secIndex - 1;   // last processed secIndex

    while (nextFrameStartK + (kFrameSecs - 1) <= maxComplete) {
        if (nextFrameStartK < recBase) { nextFrameStartK += kFrameSecs; continue; }

        std::array<int8_t, kFrameSecs> sym{};
        std::array<float, kFrameSecs> conf{};
        std::array<ClockSymbol, kFrameSecs> vsym{};
        std::int64_t base = nextFrameStartK - recBase;
        for (int s = 0; s < kFrameSecs; ++s) {
            const Rec& r = recs[static_cast<std::size_t>(base + s)];
            sym[s] = r.sym;
            conf[s] = r.conf;
            vsym[s] = static_cast<ClockSymbol>(r.sym);
        }

        int mkOk = 0, mkFalse = 0, mkLate = 0, mkEarly = 0;
        for (int s = 0; s < kFrameSecs; ++s) {
            if (sym[s] != 2) continue;
            (isMarkerSec(s) ? mkOk : mkFalse) += 1;
            if (s % 10 == 0) ++mkLate;     // content one second later than labelled
            if (s % 10 == 8) ++mkEarly;    // content one second earlier
        }

        // A whole skeleton one second off is a slipped second count, not a
        // noisy frame: a leap second (NIST SP 432 sends 23:59:60 as a binary 0,
        // so s59 -> s60 -> hole pushes every later marker one slot late), or a
        // second lost or duplicated upstream. Realign the anchor by that second
        // and assemble the frame again from there, without emitting or voting
        // the misaligned one -- its minute is the same minute, read correctly
        // once aligned. A frame that still looks slipped right after a
        // realignment means the evidence is not a clean slip: start over.
        if ((mkLate >= 4 || mkEarly >= 4) && mkOk <= 1) {
            if (nextFrameStartK == lastRealignK) {
                softReacquire();
                return;
            }
            const int shift = (mkLate >= mkEarly) ? 1 : -1;
            anchorSec0 += shift;
            nextFrameStartK += shift;
            lastRealignK = nextFrameStartK;
            suspectSec = -1;   // the slip it flagged is now accounted for
            if (state == ClockLockState::Locked) setState(ClockLockState::Acquiring);
            continue;
        }

        // A second inside this frame contradicted the alignment (see
        // processSecond). The frame may be half one count and half the other,
        // so neither its decode nor a timestamp composed from its second 0 can
        // be trusted.
        const bool suspect = suspectSec >= nextFrameStartK &&
                             suspectSec < nextFrameStartK + kFrameSecs;

        std::int64_t frameStartSample = recs[static_cast<std::size_t>(base)].edgeSample;
        ClockFrameInfo frame = decodeFrame(sym, conf, frameStartSample);
        if (!suspect && owner && owner->onFrame) owner->onFrame(frame);

        // Structural re-validation (WS-4.5): the marker skeleton is the frame's
        // ground truth — a healthy frame has its 6 P markers on the 9s and
        // nowhere else. A STREAK of broken-skeleton frames means the current
        // window/anchor is systematically wrong (sample discontinuity,
        // accumulated drift) — resynchronize rather than decode garbage
        // forever. Individual noisy frames still VOTE below: their bits carry
        // usable signal and pruning them thins the window the fade rescue
        // needs (measured on the 2026-07-19 live corpus — the voter's own
        // range/staleness/trust gates absorb per-frame noise).
        const bool skeletonOk = !(mkOk < 4 || mkFalse > 5);
        if (!skeletonOk) {
            if (++badFrameStreak >= 3) {
                softReacquire();
                return;
            }
        } else {
            badFrameStreak = 0;
        }

        // Feed the cross-frame voter (markers excluded internally). A suspect
        // frame still takes its window slot -- the voter extrapolates by slot
        // age, so skipping one would shift every older frame's minute -- but
        // as all-Unknown: it votes nothing and counts as range-invalid.
        if (suspect) {
            std::array<ClockSymbol, kFrameSecs> blank{};
            blank.fill(ClockSymbol::Unknown);
            std::array<float, kFrameSecs> zero{};
            voter.addFrame(blank, zero);
        } else {
            voter.addFrame(vsym, conf);
        }
        const bool certified = voter.locked();

        // The minute this frame names (voted when the window certifies one), for
        // judging whether a leap second can follow it.
        haveLastFields = !suspect;
        if (!suspect) {
            lastFields = certified
                ? TimeFields{voter.votedField(TimeFrameVoter::FieldMinutes),
                             voter.votedField(TimeFrameVoter::FieldHours),
                             voter.votedField(TimeFrameVoter::FieldDoy),
                             voter.votedField(TimeFrameVoter::FieldYear)}
                : TimeFields{frame.minute, frame.hour, frame.doy, frame.year2};
        }
        // 23:59 on the last day of a month with the leap warning up (this
        // frame's bit or the previous one's, in case this one faded): the next
        // second is probably 23:59:60. The engine extends this frame's timestamp
        // by whole seconds of samples, so that second would be labelled
        // 00:00:00 and every one after it a second early. Do not issue a
        // timestamp from this frame and stop certifying before the leap second
        // is emitted; the slip checks realign the next minute, which then
        // re-locks. If no leap second comes after all, it re-locks just the same.
        const bool leapNext = !suspect && (frame.leapPending || lastLeapWarn) &&
                              leapSecondPossible();
        if (!suspect) lastLeapWarn = frame.leapPending;

        if (certified && (suspect || leapNext)) {
            if (state == ClockLockState::Locked) setState(ClockLockState::Acquiring);
        } else if (certified && !skeletonOk) {
            // Voter still certifies, but this frame's own alignment is not
            // confirmed: hold the state and let the previous timestamp keep
            // extending rather than compose a fresh one against a second 0 the
            // skeleton does not vouch for.
        } else if (certified) {
            setState(ClockLockState::Locked);
            if (owner && owner->onTime) {
                ClockTimeInfo t;
                t.minute = voter.votedField(TimeFrameVoter::FieldMinutes);
                t.hour   = voter.votedField(TimeFrameVoter::FieldHours);
                t.doy    = voter.votedField(TimeFrameVoter::FieldDoy);
                t.year2  = voter.votedField(TimeFrameVoter::FieldYear);
                t.quality = voter.lockConfidence();
                t.lastEdgeSample = lastEdgeSample;
                t.lastEdgeSecondOfFrame = lastEdgeSecondOfFrame;
                t.station = station;
                owner->onTime(t);
            }
        } else if (state == ClockLockState::Locked) {
            // The voter no longer certifies a timestamp: demote instead of
            // pinning a stale Locked (the decoder is the lock authority the
            // engine's state resync trusts — WS-4.5).
            setState(ClockLockState::Acquiring);
        }

        nextFrameStartK += kFrameSecs;
    }
}

void WwvDecoder::Impl::softReacquire() {
    // Forget every timing estimate; keep the warm filters, the leaky tick fold
    // (already integrating the CURRENT phase, which is what makes re-lock
    // fast), the station tag, and the sample counter. Called when structure
    // proves the current window/anchor wrong.
    tickLocked = false;
    curFill = 0;
    secStarted = false;
    secStartJ = 0;
    delayEst = kNominalDelaySamples;
    delayLocked = false;
    delayCount = 0;
    edgeDelayEst = kNominalDelaySamples;
    edgeDelayCount = 0;
    edgeDelayRejects = 0;
    recs.clear();
    recBase = secIndex;
    anchored = false;
    anchorSec0 = 0;
    nextFrameStartK = 0;
    badFrameStreak = 0;
    suspectSec = -1;
    lastRealignK = -1;
    haveLastFields = false;
    lastLeapWarn = false;
    voter.reset();
    if (state != ClockLockState::NoSignal) setState(ClockLockState::Acquiring);
}

ClockFrameInfo WwvDecoder::Impl::decodeFrame(
    const std::array<int8_t, kFrameSecs>& sym,
    const std::array<float, kFrameSecs>& conf,
    std::int64_t frameStartSample) const {
    auto bit = [&](int s) { return sym[s] == 1; };
    auto sumField = [&](const ClockFieldMap& m) {
        int v = 0;
        for (const auto& bw : m) if (bit(bw.second)) v += bw.weight;
        return v;
    };

    ClockFrameInfo f;
    f.minute = sumField(kMin);
    f.hour   = sumField(kHr);
    f.doy    = sumField(kDoy);
    f.year2  = sumField(kYr);

    // DUT1: sign at s50 (1 = positive); magnitude in tenths at s56/57/58.
    int magTenths = (bit(56) ? 1 : 0) + (bit(57) ? 2 : 0) + (bit(58) ? 4 : 0);
    f.dut1Tenths = (bit(50) ? magTenths : -magTenths);

    f.dst1 = bit(2);          // DST at 00:00Z today
    f.dst2 = bit(55);         // DST at 24:00Z today
    f.leapPending = bit(3);   // leap-second warning
    f.leapYear = false;       // WWV/WWVH carry no leap-year bit

    // Frame confidence: mean per-second margin over the non-marker seconds.
    double sumc = 0.0; int cnt = 0;
    for (int s = 0; s < kFrameSecs; ++s)
        if (!isMarkerSec(s)) { sumc += conf[s]; ++cnt; }
    f.frameConfidence = cnt ? static_cast<float>(std::min(1.0, sumc / cnt)) : 0.0f;

    f.frameStartSample = frameStartSample;
    f.station = station;
    return f;
}

void WwvDecoder::Impl::reset() {
    for (auto& b : bpMain) b.reset();
    for (auto& b : lpEnv) b.reset();
    for (auto& b : lpI) b.reset();
    for (auto& b : lpQ) b.reset();
    for (auto& b : bpTickV) b.reset();
    for (auto& b : bpTickH) b.reset();
    oscC = 1.0; oscS = 0.0; oscRenorm = 0;
    accA = accTickV = accTickH = 0.0; decCount = 0; n200 = 0;
    foldV.fill(0.0); foldH.fill(0.0);
    tickLocked = false; tickPhase = 0;
    tickLockJ = 0; pendingStation = ClockStation::Unknown; pendingCount = 0;
    stationContrary = 0; stationUnsupported = 0; tickExcessRatio = 0.0;
    delayEst = kNominalDelaySamples; delayLocked = false; delayCount = 0;
    edgeDelayEst = kNominalDelaySamples; edgeDelayCount = 0; edgeDelayRejects = 0;
    badFrameStreak = 0;
    suspectSec = -1; lastRealignK = -1; haveLastFields = false; lastLeapWarn = false;
    curFill = 0; secStarted = false; secStartJ = 0; aScale = 1e-6;
    recs.clear(); recBase = 0; secIndex = 0;
    anchored = false; anchorSec0 = 0; nextFrameStartK = 0;
    lastEdgeSample = 0; lastEdgeSecondOfFrame = -1;
    if (!stationPinned) station = ClockStation::Unknown;
    samplesConsumed = 0;
    voter.reset();
    setState(ClockLockState::NoSignal);
}

// ---------------------------------------------------------------------------
// Public surface.

WwvDecoder::WwvDecoder(int sampleRateHz)
    : m_impl(std::make_unique<Impl>(sampleRateHz)) {
    m_impl->owner = this;
}

WwvDecoder::~WwvDecoder() = default;

void WwvDecoder::process(const float* mono, std::size_t n) {
    if (!mono) return;
    Impl* d = m_impl.get();
    for (std::size_t k = 0; k < n; ++k) d->processSample(mono[k]);
}

void WwvDecoder::reset() { m_impl->reset(); }

void WwvDecoder::setPlausibility(std::function<TimeFields()> referenceNow,
                                 int boundMinutes) {
    m_impl->voter.setPlausibility(std::move(referenceNow), boundMinutes);
}

void WwvDecoder::presetStation(ClockStation s) {
    Impl& d = *m_impl;
    if (d.stationPinned || (s != ClockStation::Wwv && s != ClockStation::Wwvh)) return;
    d.station = s;
    d.pendingStation = ClockStation::Unknown;
    d.pendingCount = d.stationContrary = d.stationUnsupported = 0;
}

void WwvDecoder::pinStation(ClockStation s) {
    Impl& d = *m_impl;
    d.station = s;
    d.stationPinned = true;
    d.pendingStation = ClockStation::Unknown;
    d.pendingCount = d.stationContrary = d.stationUnsupported = 0;
}

ClockLockState WwvDecoder::state() const { return m_impl->state; }
ClockStation WwvDecoder::station() const { return m_impl->station; }
std::int64_t WwvDecoder::samplesConsumed() const { return m_impl->samplesConsumed; }

ClockDecoderDiagnostics WwvDecoder::diagnostics() const {
    const Impl& d = *m_impl;
    ClockDecoderDiagnostics g;

    // Stage 1: folded tick-band impulsiveness — the same peak-to-mean statistic
    // the tick lock gates on, recomputed here from the leaky folds (read-only).
    const auto foldRatio = [](const std::array<double, kSecLen>& fold) {
        double peak = 0.0, sum = 0.0;
        for (int p = 0; p < kSecLen; ++p) {
            sum += fold[p];
            if (fold[p] > peak) peak = fold[p];
        }
        const double mean = sum / kSecLen;
        return (mean > 0.0) ? peak / mean : 0.0;
    };
    const double ratio = std::max(foldRatio(d.foldV), foldRatio(d.foldH));
    g.toneSnrDb = (ratio > 0.0)
        ? static_cast<float>(10.0 * std::log10(ratio)) : 0.0f;
    g.pwmContrast = 0.0f;  // WWVB-only metric
    g.tickBandRatioDb = (d.tickLocked && d.tickExcessRatio > 0.0 && std::isfinite(d.tickExcessRatio))
        ? static_cast<float>(10.0 * std::log10(d.tickExcessRatio))
        : std::numeric_limits<float>::quiet_NaN();
    g.toneDetected = d.tickLocked;

    g.phaseLocked = d.tickLocked;
    g.delayEstMs = d.delayLocked
        ? static_cast<float>(d.delayEst * 1000.0 / kSeriesRate)
        : std::numeric_limits<float>::quiet_NaN();

    g.anchored = d.anchored;
    g.badFrameStreak = d.badFrameStreak;

    g.framesInWindow = d.voter.frameCount();
    g.windowSize = d.voter.windowSize();
    g.voteQuality = d.voter.lockConfidence();
    g.refusalReason = static_cast<std::uint8_t>(d.voter.lockRefusal());
    return g;
}

} // namespace clockdec
