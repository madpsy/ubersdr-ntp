// WwvbDecoder.cpp — implementation translation unit (no include guard needed).

#include "WwvbDecoder.h"

#include "CivilTime.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <initializer_list>
#include <limits>
#include <utility>
#include <vector>

// WWVB 60 kHz legacy-AM/PWM decoder. All format facts per the NIST WWVB
// time-code description (NIST SP 250-67): one bit per second; carrier power
// reduced 17 dB at each second start for 0.2 s (binary 0), 0.5 s (binary 1)
// or 0.8 s (marker); markers at seconds 0, 9, 19, 29, 39, 49, 59; two
// consecutive markers (s59 -> s0) mark the minute boundary. BCD field weights
// are MSB-first (opposite of WWV). Amplitude-only by design: the post-2012
// BPSK phase layer (a 180 deg flip +100 ms after each second) leaves the AM
// envelope unchanged in steady state (|-z| == |z|), and the flip TRANSIENT --
// the discontinuity briefly rings the I/Q low-pass into a ~1-sample magnitude
// notch -- is rejected by the ~10 ms boxcar decimation averaging it flat plus
// the sustained-low edge gate (several consecutive low samples required), so
// this decoder ignores the BPSK layer entirely; no edge is phase-derived.
//
// Chain: one-shot tone search 800-1200 Hz -> complex mix to baseband ->
// biquad LPF on I/Q -> magnitude -> boxcar-decimate to a 100 Hz envelope ->
// AM-drop second-edge tracking -> per-second zero-mean matched-filter
// classify (0.2/0.5/0.8 s low durations; confidence = correlation margin) ->
// double-marker minute sync -> NIST WWVB BCD map -> TimeFrameVoter.

namespace AetherSDR {

namespace {

constexpr double kPi          = 3.14159265358979323846;
constexpr int    kEnvRateHz   = 100;    // decoded envelope series rate
constexpr int    kEnvCap      = 1024;   // envelope ring capacity (>10 s)
constexpr int    kPctWin      = 300;    // adaptive-threshold window (3 s)
constexpr int    kEdgeTol     = 12;     // +/- env samples searched per second
// Envelope history an edge candidate needs on either side: the sustained-low
// check and the low-level estimate look up to 10 samples past it, the high-
// level estimate 8 before. Classification waits for this much lookahead so a
// candidate near the end of the search band is judged on real samples, not on
// a truncated window that silently fails it.
constexpr int    kEdgeLook    = 12;
constexpr int    kWarmEnv     = 200;    // env samples before seeding phase
constexpr double kMinContrast = 1.4;    // p90 >= 1.4*p10 => a real AM drop exists
constexpr double kLpfCutHz    = 150.0;  // I/Q low-pass corner

// Acquisition tone search.
constexpr double kAcqSeconds  = 2.0;
constexpr double kToneLoHz    = 800.0;
constexpr double kToneHiHz    = 1200.0;
constexpr double kToneStepHz  = 2.0;
constexpr double kToneGate    = 12.0;   // peak/median power ratio for "tone present"

constexpr std::array<int, 7> kMarkerSeconds = {0, 9, 19, 29, 39, 49, 59};

// Low-duration (in 100 Hz env samples) of each symbol: 0.2/0.5/0.8 s.
constexpr std::array<int, 3> kSymbolLowLen = {20, 50, 80};

// Minimum matched-filter margin for a single second to count as structural
// evidence that the frame alignment is wrong (a marker where none belongs, or
// a non-marker where one must be). Low enough that a clean marker always
// clears it, high enough that a coin-flip read under noise does not throw
// away a good lock.
constexpr float kStructConf = 0.10f;

// Edge tracker gains (see trackEdge): phase weight ~1/8 (tau ~8 s) once warm,
// period weight near critical damping for that alpha, first kTrkWarm measured
// seconds a plain running mean.
constexpr double kTrkAlpha = 1.0 / 8.0;
constexpr double kTrkBeta  = 1.0 / 128.0;
constexpr int    kTrkWarm  = 8;

inline bool isWwvbMarkerSecond(int s) {
    for (int m : kMarkerSeconds) if (m == s) return true;
    return false;
}

// Transposed-direct-form-II biquad, runtime-designed Butterworth low-pass.
struct Biquad {
    double b0 = 1.0, b1 = 0.0, b2 = 0.0, a1 = 0.0, a2 = 0.0;
    double z1 = 0.0, z2 = 0.0;

    double process(double x) {
        double y = b0 * x + z1;
        z1 = b1 * x - a1 * y + z2;
        z2 = b2 * x - a2 * y;
        return y;
    }
    void reset() { z1 = z2 = 0.0; }

    static Biquad lowpass(double fs, double fc) {
        const double w0 = 2.0 * kPi * fc / fs;
        const double c = std::cos(w0);
        const double s = std::sin(w0);
        const double q = 0.70710678118654752440;   // Butterworth Q
        const double alpha = s / (2.0 * q);
        const double a0 = 1.0 + alpha;
        Biquad bq;
        bq.b0 = ((1.0 - c) / 2.0) / a0;
        bq.b1 = (1.0 - c) / a0;
        bq.b2 = ((1.0 - c) / 2.0) / a0;
        bq.a1 = (-2.0 * c) / a0;
        bq.a2 = (1.0 - alpha) / a0;
        return bq;
    }
};

} // namespace

// ---------------------------------------------------------------------------

struct WwvbDecoder::Impl {
    explicit Impl(WwvbDecoder* o, int sampleRateHz)
        : owner(o),
          sr(sampleRateHz > 0 ? sampleRateHz : 24000),
          decim(std::max(1, static_cast<int>(std::lround(sr / 100.0)))),
          voter(buildVoterConfig()) {
        acqTarget = static_cast<int64_t>(std::llround(kAcqSeconds * sr));
        for (double f = kToneLoHz; f <= kToneHiHz + 1e-9; f += kToneStepHz) {
            gFreq.push_back(f);
            gCoeff.push_back(2.0 * std::cos(2.0 * kPi * f / sr));
        }
        gS1.assign(gFreq.size(), 0.0);
        gS2.assign(gFreq.size(), 0.0);
        pctScratch.reserve(kPctWin);
        buildTemplates();
    }

    // ---- fixed configuration -------------------------------------------

    // NIST WWVB BCD field maps (MSB-first weights) for the shared voter.
    static TimeFrameVoter::Config buildVoterConfig() {
        TimeFrameVoter::Config c;
        c.fields[TimeFrameVoter::FieldMinutes] =
            {{1, 40}, {2, 20}, {3, 10}, {5, 8}, {6, 4}, {7, 2}, {8, 1}};
        c.fields[TimeFrameVoter::FieldHours] =
            {{12, 20}, {13, 10}, {15, 8}, {16, 4}, {17, 2}, {18, 1}};
        c.fields[TimeFrameVoter::FieldDoy] =
            {{22, 200}, {23, 100}, {25, 80}, {26, 40}, {27, 20},
             {28, 10}, {30, 8}, {31, 4}, {32, 2}, {33, 1}};
        c.fields[TimeFrameVoter::FieldYear] =
            {{45, 80}, {46, 40}, {47, 20}, {48, 10}, {50, 8}, {51, 4}, {52, 2}, {53, 1}};
        c.markerSeconds = {0, 9, 19, 29, 39, 49, 59};
        // WS-4.5 honesty floors (shared rationale with WwvDecoder): noise-grade
        // margins must not vote, and a window whose trust collapsed must not
        // lock — unanimity of systematic misreads is not evidence. Floors match
        // WwvDecoder's (calibrated on the 2026-07-19 live WWV corpus).
        c.minBitConfidence = 0.05f;
        c.minLockQuality   = 0.05f;
        return c;
    }

    void buildTemplates() {
        // Zero-mean expected envelope per symbol: 0 during the reduced-carrier
        // (low) portion, 1 during the full-carrier (high) portion. The winning
        // template's low->high transition point identifies the pulse duration.
        for (int s = 0; s < 3; ++s) {
            const int low = kSymbolLowLen[s];
            double mean = 0.0;
            for (int k = 0; k < kEnvRateHz; ++k) {
                float e = (k < low) ? 0.0f : 1.0f;
                tpl[s][k] = e;
                mean += e;
            }
            mean /= kEnvRateHz;
            double nrm = 0.0;
            for (int k = 0; k < kEnvRateHz; ++k) {
                tpl[s][k] -= static_cast<float>(mean);
                nrm += static_cast<double>(tpl[s][k]) * tpl[s][k];
            }
            tplNorm[s] = std::sqrt(nrm) + 1e-12;
        }
    }

    // ---- driving ---------------------------------------------------------

    void process(const float* mono, std::size_t n) {
        for (std::size_t i = 0; i < n; ++i) {
            const double x = static_cast<double>(mono[i]);
            ++samplesConsumed;
            if (phase == Phase::Searching)
                feedAcquisition(x);
            else
                feedSteady(x);
        }
    }

    void reset() {
        phase = Phase::Searching;
        acqCount = 0;
        lastToneSnrDb = 0.0f;
        std::fill(gS1.begin(), gS1.end(), 0.0);
        std::fill(gS2.begin(), gS2.end(), 0.0);
        oscRe = 1.0; oscIm = 0.0; oscRenorm = 0;
        lpI.reset(); lpQ.reset();
        magAccum = 0.0; magCount = 0;
        env.fill(0.0f); envCount = 0; envBaseSample = 0;
        pP10 = 0.0f; pP90 = 0.0f;
        phaseKnown = false; curStart = 0; scanPos = 0;
        curEdge = 0.0; curEdgeMeasured = false;
        trkValid = false; trkEdge = 0.0; trkPeriod = 0.0; trkCount = 0; trkOutliers = 0;
        anchored = false; sofNext = 0; prevSym = ClockSymbol::Unknown; havePrev = false;
        prevPrevSym = ClockSymbol::Unknown;
        frFilledCount = 0; frStartSample = 0;
        haveLastFrame = false; lastFrameS0Env = 0;
        leapInserted = false; haveLastFields = false; lastFields = TimeFields{};
        lastLeapWarn = false;
        haveVoted = false;
        votedMinute = votedHour = votedDoy = votedYear = -1; votedQuality = 0.0f;
        voter.reset();
        samplesConsumed = 0;
        setLockState(ClockLockState::NoSignal);
    }

    // ---- acquisition (one-shot tone search) ------------------------------

    void feedAcquisition(double x) {
        for (std::size_t i = 0; i < gFreq.size(); ++i) {
            double s = x + gCoeff[i] * gS1[i] - gS2[i];
            gS2[i] = gS1[i];
            gS1[i] = s;
        }
        if (++acqCount >= acqTarget)
            finalizeAcquisition();
    }

    void finalizeAcquisition() {
        int peak = 0;
        double peakP = -1.0;
        std::vector<double> pw(gFreq.size());
        for (std::size_t i = 0; i < gFreq.size(); ++i) {
            double p = gS1[i] * gS1[i] + gS2[i] * gS2[i] - gCoeff[i] * gS1[i] * gS2[i];
            pw[i] = p;
            if (p > peakP) { peakP = p; peak = static_cast<int>(i); }
        }
        std::vector<double> med = pw;
        std::nth_element(med.begin(), med.begin() + med.size() / 2, med.end());
        double median = med[med.size() / 2];

        // WS-7 telemetry: remember what the tone search measured (read-only
        // store — the gate decision below is untouched). Updated every 2 s
        // search window until the tone gate passes, so the funnel's stage-1
        // readout is live even when nothing is there.
        if (median > 0.0)
            lastToneSnrDb = static_cast<float>(10.0 * std::log10(peakP / median));

        if (peakP > kToneGate * median && median > 0.0) {
            f0 = gFreq[peak];
            startSteady();
        } else {
            // No tone: keep searching over the next acquisition window.
            std::fill(gS1.begin(), gS1.end(), 0.0);
            std::fill(gS2.begin(), gS2.end(), 0.0);
            acqCount = 0;
        }
    }

    void startSteady() {
        phase = Phase::Running;
        envBaseSample = samplesConsumed;    // env index 0 begins here
        const double w0 = 2.0 * kPi * f0 / sr;
        stepRe = std::cos(w0);
        stepIm = -std::sin(w0);             // e^{-j w0}
        oscRe = 1.0; oscIm = 0.0; oscRenorm = 0;
        lpI = Biquad::lowpass(sr, kLpfCutHz);
        lpQ = Biquad::lowpass(sr, kLpfCutHz);
        // DC group delay of that low-pass in input samples (~1.5 ms at 150 Hz):
        // the first moment of its impulse response, which is exactly how late a
        // step comes out of it. For b(1 + 2z^-1 + z^-2) / (1 + a1 z^-1 + a2 z^-2)
        // the symmetric numerator contributes 1 sample and the denominator
        // -(a1 + 2 a2) / (1 + a1 + a2). Derived from the coefficients rather
        // than calibrated, so a changed corner keeps edges honest.
        lpfDelay = 1.0 - (lpI.a1 + 2.0 * lpI.a2) / (1.0 + lpI.a1 + lpI.a2);
        magAccum = 0.0; magCount = 0;
        setLockState(ClockLockState::Acquiring);
    }

    // ---- steady state (mix -> LPF -> magnitude -> decimate) --------------

    void feedSteady(double x) {
        double I = lpI.process(x * oscRe);
        double Q = lpQ.process(x * oscIm);
        magAccum += std::sqrt(I * I + Q * Q);
        if (++magCount >= decim) {
            pushEnvelope(static_cast<float>(magAccum / magCount));
            magAccum = 0.0;
            magCount = 0;
        }
        // Advance the baseband phasor (rotation recurrence + periodic renorm).
        double nr = oscRe * stepRe - oscIm * stepIm;
        double ni = oscRe * stepIm + oscIm * stepRe;
        oscRe = nr; oscIm = ni;
        if (++oscRenorm >= 1024) {
            double m = std::sqrt(oscRe * oscRe + oscIm * oscIm);
            if (m > 1e-9) { oscRe /= m; oscIm /= m; }
            oscRenorm = 0;
        }
    }

    void pushEnvelope(float v) {
        env[static_cast<std::size_t>(envCount % kEnvCap)] = v;
        ++envCount;
        updatePercentiles();
        processSeconds();
    }

    float envAt(int64_t idx) const { return env[static_cast<std::size_t>(idx % kEnvCap)]; }

    void updatePercentiles() {
        int64_t n = std::min<int64_t>(kPctWin, envCount);
        if (n < 8) { pP10 = pP90 = 0.0f; return; }
        pctScratch.clear();
        for (int64_t i = envCount - n; i < envCount; ++i)
            pctScratch.push_back(envAt(i));
        std::size_t k10 = static_cast<std::size_t>(0.10 * n);
        std::size_t k90 = static_cast<std::size_t>(0.90 * n);
        std::nth_element(pctScratch.begin(), pctScratch.begin() + k10, pctScratch.end());
        pP10 = pctScratch[k10];
        std::nth_element(pctScratch.begin(), pctScratch.begin() + k90, pctScratch.end());
        pP90 = pctScratch[k90];
    }

    bool haveContrast() const {
        return pP90 >= kMinContrast * std::max(pP10, 1e-6f);
    }

    // A genuine second edge stays low for several env samples; the BPSK
    // +100 ms phase-flip transient is only ~1 sample wide and is rejected.
    bool sustainedLow(int64_t i, float thrLo) const {
        int checked = 0, low = 0;
        for (int d = 1; d <= 7; ++d) {
            if (i + d >= envCount) break;
            ++checked;
            if (envAt(i + d) < thrLo) ++low;
        }
        return checked >= 5 && low >= checked - 1;
    }

    // ---- second segmentation & classification ----------------------------

    void processSeconds() {
        if (!phaseKnown) {
            trySeed();
            if (!phaseKnown) return;
        }
        while (phaseKnown && envCount >= curStart + kEnvRateHz + kEdgeTol + kEdgeLook)
            classifyAndAdvance();
    }

    // Is env[i] the first envelope sample below the 50% level of a real carrier
    // drop? The drop only has to cross the MIDPOINT between env[i-1] and env[i]:
    // an earlier form demanded env[i-1] >= 60% and env[i] < 40%, and when the
    // drop landed 40-60% of the way into a 10 ms averaging block that block sat
    // between the two thresholds, so no second ever qualified and the decoder
    // never seeded (the reviewer's 0-seconds-in-6-minutes run). A full-carrier
    // sample must still appear within the three before the crossing, and the
    // sustained-low gate still rejects the ~1-sample BPSK flip notch.
    bool isFallingEdge(int64_t i, float thrHi, float thrMid, float thrLo) const {
        if (!(envAt(i - 1) >= thrMid && envAt(i) < thrMid)) return false;
        bool high = false;
        for (int d = 1; d <= 3 && !high; ++d) high = envAt(i - d) >= thrHi;
        return high && sustainedLow(i, thrLo);
    }

    // Sub-block position of the drop that crosses at env[i], as a (fractional)
    // input-sample index. The 10 ms boxcar that makes the envelope is also what
    // lets the edge be recovered finely: a block holding a step at fraction f
    // averages to L + (1 - f)(H - L), so the step's position is in the AREA of
    // the transition, sum((env - L) / (H - L)) over blocks spanning it, not in
    // which block happens to cross a threshold. That sum measures the FILTERED
    // step; for the unity-DC-gain I/Q low-pass it is the true step delayed by
    // exactly the filter's DC group delay, which is subtracted analytically.
    // H and L are local means either side, so fading and the adaptive
    // percentiles' lag do not bias it.
    double edgeSampleAt(int64_t i) const {
        double hi = 0.0, lo = 0.0;
        for (int d = 4; d <= 8; ++d) hi += envAt(i - d);
        for (int d = 4; d <= 10; ++d) lo += envAt(i + d);
        hi /= 5.0;
        lo /= 7.0;
        double pos = static_cast<double>(i);
        const double span = hi - lo;
        if (span > 1e-9) {
            // Blocks i-2 .. i+3 cover the step wherever the crossing rule can
            // place it (within a block either side of i) plus the filter's
            // few-ms settling tail.
            double area = 0.0;
            for (int64_t k = i - 2; k <= i + 3; ++k) area += (envAt(k) - lo) / span;
            pos = std::clamp(static_cast<double>(i - 2) + area, i - 1.5, i + 1.5);
        }
        return static_cast<double>(envBaseSample) + pos * decim - lpfDelay;
    }

    // Second edges are exactly one broadcast second apart, so a single
    // second's measurement is not the best estimate of its own edge: under
    // noise the drop's area moves +/-3 ms second to second at 10 dB. An
    // alpha-beta tracker (phase + period, so a receiver sample rate a few ppm
    // off nominal is followed without lag) reports the smoothed edge instead.
    // It starts as a running mean so the first seconds are not dragged from
    // zero, coasts on its own prediction through seconds with no drop found,
    // and re-seeds only after several consecutive measurements disagree by a
    // whole envelope block (a real step, not noise).
    void trackEdge(bool measured, double raw) {
        if (!trkValid) {
            trkEdge = raw;
            trkPeriod = sr;
            trkValid = measured;
            trkCount = measured ? 1 : 0;
            trkOutliers = 0;
            return;
        }
        const double pred = trkEdge + trkPeriod;
        if (!measured) { trkEdge = pred; return; }
        const double r = raw - pred;
        if (std::fabs(r) > decim) {
            if (++trkOutliers >= 3) {
                trkEdge = raw;
                trkPeriod = sr;
                trkCount = 1;
                trkOutliers = 0;
            } else {
                trkEdge = pred;
            }
            return;
        }
        trkOutliers = 0;
        ++trkCount;
        const double alpha = std::max(1.0 / trkCount, kTrkAlpha);
        trkEdge = pred + alpha * r;
        if (trkCount > kTrkWarm) {
            // +/-200 ppm is far outside any real receiver; the clamp only stops
            // a pathological residual run from winding the period away.
            trkPeriod = std::clamp(trkPeriod + kTrkBeta * r, sr * (1.0 - 2e-4), sr * (1.0 + 2e-4));
        }
    }

    void edgeThresholds(float& thrHi, float& thrMid, float& thrLo) const {
        thrHi  = pP10 + 0.6f * (pP90 - pP10);
        thrMid = pP10 + 0.5f * (pP90 - pP10);
        thrLo  = pP10 + 0.4f * (pP90 - pP10);
    }

    void trySeed() {
        if (envCount < kWarmEnv || !haveContrast()) return;
        float thrHi, thrMid, thrLo;
        edgeThresholds(thrHi, thrMid, thrLo);
        int64_t lo = std::max<int64_t>(std::max<int64_t>(scanPos, 9), envCount - kEnvCap + 9);
        for (int64_t i = lo; i + kEdgeLook < envCount; ++i) {
            scanPos = i;
            if (isFallingEdge(i, thrHi, thrMid, thrLo)) {
                curStart = i;
                curEdge = edgeSampleAt(i);
                curEdgeMeasured = true;
                phaseKnown = true;
                return;
            }
        }
    }

    // Look for the next second's drop near its predicted env index. On success
    // sets startOut to the crossing sample (the classifier's window start) and
    // edgeOut to the sub-block edge; otherwise startOut is the prediction and
    // the caller coasts.
    bool findFallingEdgeNear(int64_t pred, int tol, int64_t& startOut, double& edgeOut) {
        startOut = pred;
        if (!haveContrast()) return false;   // coast on predicted cadence
        float thrHi, thrMid, thrLo;
        edgeThresholds(thrHi, thrMid, thrLo);
        int64_t lo = std::max<int64_t>(pred - tol, envCount - kEnvCap + 9);
        int64_t hi = std::min<int64_t>(pred + tol, envCount - 1 - kEdgeLook);
        int64_t bestEdge = pred, bestDist = tol + 1;
        for (int64_t i = lo; i <= hi; ++i) {
            if (i < 9) continue;
            if (isFallingEdge(i, thrHi, thrMid, thrLo)) {
                int64_t d = std::llabs(i - pred);
                if (d < bestDist) { bestDist = d; bestEdge = i; }
            }
        }
        if (bestDist > tol) return false;
        startOut = bestEdge;
        edgeOut = edgeSampleAt(bestEdge);
        return true;
    }

    void classifyAndAdvance() {
        // Window = one full second from the drop edge (100 env samples).
        std::array<float, kEnvRateHz> w;
        double mean = 0.0;
        for (int k = 0; k < kEnvRateHz; ++k) { w[k] = envAt(curStart + k); mean += w[k]; }
        mean /= kEnvRateHz;

        std::array<float, kEnvRateHz> v;
        double vnorm = 0.0;
        for (int k = 0; k < kEnvRateHz; ++k) {
            v[k] = w[k] - static_cast<float>(mean);
            vnorm += static_cast<double>(v[k]) * v[k];
        }
        vnorm = std::sqrt(vnorm) + 1e-12;

        double corr[3];
        for (int s = 0; s < 3; ++s) {
            double dot = 0.0;
            for (int k = 0; k < kEnvRateHz; ++k)
                dot += static_cast<double>(v[k]) * tpl[s][k];
            corr[s] = dot / (vnorm * tplNorm[s]);
        }
        int best = 0;
        for (int s = 1; s < 3; ++s) if (corr[s] > corr[best]) best = s;
        double runner = -1e18;
        for (int s = 0; s < 3; ++s) if (s != best && corr[s] > runner) runner = corr[s];
        float conf = static_cast<float>(std::max(0.0, corr[best] - runner));

        ClockSymbol sym = (best == 0) ? ClockSymbol::Zero
                        : (best == 1) ? ClockSymbol::One
                                      : ClockSymbol::Marker;
        // Reported at input-sample resolution from the edge tracker, fed the
        // sub-block estimate made when this second was located; curStart only
        // chooses which envelope samples the classifier sees.
        trackEdge(curEdgeMeasured, curEdge);
        int64_t edgeSample = static_cast<int64_t>(std::llround(trkEdge));

        int sof = -1;
        updateSync(sym, conf, edgeSample, sof);
        emitSecond(edgeSample, curEdgeMeasured, sym, conf, sof, w, best);

        if (anchored && sof >= 0)
            recordFrameSecond(sof, sym, conf);

        if (lockState == ClockLockState::Locked && haveVoted && owner->onTime) {
            ClockTimeInfo ti;
            ti.minute = votedMinute; ti.hour = votedHour;
            ti.doy = votedDoy; ti.year2 = votedYear;
            ti.quality = votedQuality;
            ti.lastEdgeSample = edgeSample;
            ti.lastEdgeSecondOfFrame = sof;
            ti.station = ClockStation::Wwvb;
            owner->onTime(ti);
        }

        int64_t pred = curStart + kEnvRateHz;
        double edge = 0.0;
        curEdgeMeasured = findFallingEdgeNear(pred, kEdgeTol, curStart, edge);
        // Coasting keeps the last edge's sub-block phase instead of snapping to
        // the envelope grid, so a missed drop does not add a quantisation step.
        curEdge = curEdgeMeasured ? edge : curEdge + sr;
    }

    // ---- frame sync (double marker) --------------------------------------

    // Leap seconds are only ever inserted as 23:59:60 UTC on the last day of a
    // month, so a third consecutive marker is a leap second only straight after
    // such a minute. With no decoded minute to judge by (still acquiring) it
    // cannot be ruled out, and treating it as one costs nothing: the frame it
    // starts is validated at s59 like any other.
    bool leapSecondPossible() const {
        if (!haveLastFields) return true;
        const TimeFields& f = lastFields;
        if (f.minute != 59 || f.hour != 23) return false;
        if (f.doy < 1 || f.doy > 366 || f.year2 < 0 || f.year2 > 99) return false;
        return ubersdr_ntp::isLastDayOfMonth(
            ubersdr_ntp::utcMsFromFields(f.year2, f.doy, f.hour, f.minute));
    }

    void updateSync(ClockSymbol sym, float conf, int64_t edgeSample, int& sofOut) {
        const bool marker = (sym == ClockSymbol::Marker);
        if (!anchored) {
            // Two consecutive markers = s59 -> s0; this second is s0.
            if (havePrev && prevSym == ClockSymbol::Marker && marker) {
                anchored = true;
                sofOut = 0;
                sofNext = 1;
                beginFrame(edgeSample);
            } else {
                sofOut = -1;
            }
        } else if (sofNext == 1 && marker && prevSym == ClockSymbol::Marker &&
                   prevPrevSym == ClockSymbol::Marker && leapSecondPossible()) {
            // Three markers in a row -- s59, s60, s0 -- is how NIST SP 250-67
            // sends a leap second. The second just labelled s0 was 23:59:60 and
            // this one is the real s0, so the minute restarts here and the frame
            // that just finished spans 61 s. Labelling on without this puts
            // every second of the next minute one second late. If the lock was
            // still up (the warning bit was missed), drop it now, BEFORE this
            // second is emitted: the leap second itself has already gone out
            // labelled as the next minute's s0 and nothing can recall that, but
            // no later one is extended from it.
            if (lockState == ClockLockState::Locked) {
                haveVoted = false;
                setLockState(ClockLockState::Acquiring);
            }
            leapInserted = true;
            sofOut = 0;
            sofNext = 1;
            beginFrame(edgeSample);
        } else {
            sofOut = sofNext;
            if (sofOut == 0) beginFrame(edgeSample);
            sofNext = (sofNext + 1) % 60;
            // A confident symbol that contradicts the marker skeleton means the
            // second count has slipped (a leap second nobody announced, a lost
            // or duplicated second in the stream). The frame check at s59 would
            // catch it, but up to a minute later, and every second until then
            // would be labelled from a wrong count. Stop certifying time NOW;
            // s59 decides whether the frame was sound after all (and re-locks)
            // or drops the anchor.
            if (lockState == ClockLockState::Locked && conf >= kStructConf &&
                marker != isWwvbMarkerSecond(sofOut)) {
                haveVoted = false;
                setLockState(ClockLockState::Acquiring);
            }
        }
        prevPrevSym = havePrev ? prevSym : ClockSymbol::Unknown;
        prevSym = sym;
        havePrev = true;
    }

    void beginFrame(int64_t s0edge) {
        frStartSample = s0edge;
        frFilledCount = 0;
        frSym.fill(ClockSymbol::Unknown);
        frConf.fill(0.0f);
    }

    void recordFrameSecond(int sof, ClockSymbol sym, float conf) {
        frSym[static_cast<std::size_t>(sof)] = sym;
        frConf[static_cast<std::size_t>(sof)] = conf;
        ++frFilledCount;
        if (sof == 59) finalizeFrame();
    }

    // ---- frame decode + voting -------------------------------------------

    void finalizeFrame() {
        bool ok = (frFilledCount >= 60);
        for (int m : kMarkerSeconds)
            if (frSym[static_cast<std::size_t>(m)] != ClockSymbol::Marker) { ok = false; break; }
        if (!ok) {
            // Corrupted markers: drop the anchor, re-search, never emit a frame.
            anchored = false;
            havePrev = false;
            haveLastFrame = false;
            haveLastFields = false;
            lastLeapWarn = false;
            leapInserted = false;
            haveVoted = false;
            voter.reset();
            setLockState(ClockLockState::Acquiring);
            return;
        }

        ClockFrameInfo fi = decodeFrame();
        if (owner->onFrame) owner->onFrame(fi);

        // A frame that followed a leap second starts 61 s after the previous
        // one; it is still the next broadcast minute and must not reset the
        // voter's window.
        int64_t s0env = (frStartSample - envBaseSample) / decim;
        const long long gapSecs = leapInserted ? 61LL : 60LL;
        leapInserted = false;
        bool consecutive =
            haveLastFrame &&
            std::llabs((s0env - lastFrameS0Env) - gapSecs * kEnvRateHz) <= 2LL * kEdgeTol;
        if (!consecutive) voter.reset();
        haveLastFrame = true;
        lastFrameS0Env = s0env;

        std::array<ClockSymbol, 60> syms = frSym;
        std::array<float, 60> confs = frConf;
        voter.addFrame(syms, confs);

        const bool certified = voter.locked();
        if (certified) {
            votedMinute = voter.votedField(TimeFrameVoter::FieldMinutes);
            votedHour = voter.votedField(TimeFrameVoter::FieldHours);
            votedDoy = voter.votedField(TimeFrameVoter::FieldDoy);
            votedYear = voter.votedField(TimeFrameVoter::FieldYear);
            votedQuality = voter.lockConfidence();
        }
        lastFields = certified ? TimeFields{votedMinute, votedHour, votedDoy, votedYear}
                               : TimeFields{fi.minute, fi.hour, fi.doy, fi.year2};
        haveLastFields = true;

        // The minute that just ended is 23:59 on the last day of a month with
        // the leap warning up (this frame's bit, or the previous frame's in case
        // this one faded): the next second is probably 23:59:60. Every second
        // re-emits time extended from this frame by whole seconds of samples, so
        // the leap second would go out as 00:00:00 and each one after it a
        // second early. Stop certifying before that second is emitted; the
        // triple-marker check re-aligns the minute and the next frame re-locks.
        // If no leap second comes after all, that frame re-locks just the same.
        const bool leapNext = (fi.leapPending || lastLeapWarn) && leapSecondPossible();
        lastLeapWarn = fi.leapPending;

        if (certified && !leapNext) {
            haveVoted = true;
            setLockState(ClockLockState::Locked);
        } else if (certified) {
            haveVoted = false;
            if (lockState == ClockLockState::Locked)
                setLockState(ClockLockState::Acquiring);
        } else {
            // WS-4.5: the voter no longer certifies a timestamp — stop the
            // per-second cached re-emission and demote a stale Locked instead
            // of pinning it (the decoder is the lock authority the engine's
            // state resync trusts).
            haveVoted = false;
            if (lockState == ClockLockState::Locked)
                setLockState(ClockLockState::Acquiring);
        }
    }

    ClockFrameInfo decodeFrame() const {
        auto bit = [&](int s) {
            return frSym[static_cast<std::size_t>(s)] == ClockSymbol::One ? 1 : 0;
        };
        auto field = [&](std::initializer_list<std::pair<int, int>> map) {
            int v = 0;
            for (const auto& p : map) if (bit(p.first)) v += p.second;
            return v;
        };

        ClockFrameInfo fi;
        fi.minute = field({{1, 40}, {2, 20}, {3, 10}, {5, 8}, {6, 4}, {7, 2}, {8, 1}});
        fi.hour   = field({{12, 20}, {13, 10}, {15, 8}, {16, 4}, {17, 2}, {18, 1}});
        fi.doy    = field({{22, 200}, {23, 100}, {25, 80}, {26, 40}, {27, 20},
                           {28, 10}, {30, 8}, {31, 4}, {32, 2}, {33, 1}});
        fi.year2  = field({{45, 80}, {46, 40}, {47, 20}, {48, 10},
                           {50, 8}, {51, 4}, {52, 2}, {53, 1}});

        // DUT1: sign s36=+, s37=-, s38=+ (positive => s36 & s38 set; negative
        // => s37 set). Magnitude weights in tenths of a second: 0.8/0.4/0.2/0.1.
        int sign = bit(37) ? -1 : +1;
        int mag10 = 0;
        if (bit(40)) mag10 += 8;
        if (bit(41)) mag10 += 4;
        if (bit(42)) mag10 += 2;
        if (bit(43)) mag10 += 1;
        fi.dut1Tenths = sign * mag10;

        fi.leapYear    = bit(55) != 0;   // LYI
        fi.leapPending = bit(56) != 0;   // LSW
        // DST pair, mapped to the SAME meaning as WWV's s2/s55 so a consumer
        // never has to know which station it is reading: NIST puts "DST in
        // effect at 00:00Z today" on s58 and "DST in effect at 24:00Z today" on
        // s57. (Read the other way round, "DST begins today" and "DST ends
        // today" swap, which is exactly the day the bits matter.)
        fi.dst1        = bit(58) != 0;   // DST at 00:00Z today
        fi.dst2        = bit(57) != 0;   // DST at 24:00Z today

        double sum = 0.0;
        int cnt = 0;
        for (int s = 0; s < 60; ++s) {
            bool isMarker = false;
            for (int m : kMarkerSeconds) if (m == s) { isMarker = true; break; }
            if (!isMarker) { sum += frConf[static_cast<std::size_t>(s)]; ++cnt; }
        }
        fi.frameConfidence =
            cnt ? static_cast<float>(std::min(1.0, std::max(0.0, sum / cnt))) : 0.0f;
        fi.frameStartSample = frStartSample;
        fi.station = ClockStation::Wwvb;
        return fi;
    }

    // ---- callbacks & state -----------------------------------------------

    void emitSecond(int64_t edgeSample, bool measured, ClockSymbol sym, float conf, int sof,
                    const std::array<float, kEnvRateHz>& w, int best) {
        if (!owner->onSecond) return;
        ClockSecondInfo si;
        si.edgeSample = edgeSample;
        si.edgeMeasured = measured;
        si.symbol = sym;
        si.confidence = conf;
        si.secondOfFrame = sof;
        si.seriesRateHz = kEnvRateHz;
        si.envelope.resize(kEnvRateHz);
        float norm = (pP90 > 1e-6f) ? pP90 : 1.0f;
        for (int k = 0; k < kEnvRateHz; ++k)
            si.envelope[k] = std::min(1.5f, std::max(0.0f, w[k] / norm));
        si.expected.assign(tpl[best].begin(), tpl[best].end());
        owner->onSecond(si);
    }

    void setLockState(ClockLockState s) {
        if (s != lockState) {
            lockState = s;
            if (owner->onStateChanged) owner->onStateChanged(s);
        }
    }

    // ---- members ---------------------------------------------------------

    WwvbDecoder* owner;
    int sr;
    int decim;
    TimeFrameVoter voter;

    ClockLockState lockState = ClockLockState::NoSignal;
    enum class Phase { Searching, Running } phase = Phase::Searching;
    int64_t samplesConsumed = 0;

    // acquisition
    std::vector<double> gFreq, gCoeff, gS1, gS2;
    int64_t acqCount = 0, acqTarget = 0;
    float lastToneSnrDb = 0.0f;   // WS-7 telemetry: last tone-search peak/median (dB)

    // mixer / envelope generation
    double f0 = 1000.0;
    double oscRe = 1.0, oscIm = 0.0, stepRe = 1.0, stepIm = 0.0;
    int oscRenorm = 0;
    Biquad lpI, lpQ;
    double lpfDelay = 0.0;        // DC group delay of lpI/lpQ, input samples
    double magAccum = 0.0;
    int magCount = 0;
    int64_t envBaseSample = 0;

    // envelope ring + adaptive threshold
    std::array<float, kEnvCap> env{};
    int64_t envCount = 0;
    std::vector<float> pctScratch;
    float pP10 = 0.0f, pP90 = 0.0f;

    // second segmentation
    bool phaseKnown = false;
    int64_t curStart = 0, scanPos = 0;
    double curEdge = 0.0;          // current second's edge, fractional input sample
    bool curEdgeMeasured = false;  // false when coasted (no drop found)

    // edge tracker (trackEdge)
    bool trkValid = false;
    double trkEdge = 0.0, trkPeriod = 0.0;
    int trkCount = 0, trkOutliers = 0;

    // matched-filter templates
    std::array<std::array<float, kEnvRateHz>, 3> tpl{};
    std::array<double, 3> tplNorm{};

    // frame sync
    bool anchored = false;
    int sofNext = 0;
    ClockSymbol prevSym = ClockSymbol::Unknown;
    ClockSymbol prevPrevSym = ClockSymbol::Unknown;
    bool havePrev = false;

    // leap-second handling
    bool leapInserted = false;     // current frame began after a 23:59:60 marker
    bool haveLastFields = false;   // lastFields holds the last finalized minute
    TimeFields lastFields;
    bool lastLeapWarn = false;     // previous frame's leap-warning bit

    // frame assembly
    std::array<ClockSymbol, 60> frSym{};
    std::array<float, 60> frConf{};
    int frFilledCount = 0;
    int64_t frStartSample = 0;
    bool haveLastFrame = false;
    int64_t lastFrameS0Env = 0;

    // cached voted result (for onTime while locked)
    bool haveVoted = false;
    int votedMinute = -1, votedHour = -1, votedDoy = -1, votedYear = -1;
    float votedQuality = 0.0f;
};

// ---------------------------------------------------------------------------

WwvbDecoder::WwvbDecoder(int sampleRateHz)
    : m_impl(std::make_unique<Impl>(this, sampleRateHz)) {}

WwvbDecoder::~WwvbDecoder() = default;

void WwvbDecoder::process(const float* mono, std::size_t n) { m_impl->process(mono, n); }
void WwvbDecoder::reset() { m_impl->reset(); }

void WwvbDecoder::setPlausibility(std::function<TimeFields()> referenceNow,
                                  int boundMinutes) {
    m_impl->voter.setPlausibility(std::move(referenceNow), boundMinutes);
}

ClockLockState WwvbDecoder::state() const { return m_impl->lockState; }

ClockStation WwvbDecoder::station() const {
    return (m_impl->lockState == ClockLockState::NoSignal) ? ClockStation::Unknown
                                                           : ClockStation::Wwvb;
}

std::int64_t WwvbDecoder::samplesConsumed() const { return m_impl->samplesConsumed; }

ClockDecoderDiagnostics WwvbDecoder::diagnostics() const {
    const Impl& d = *m_impl;
    ClockDecoderDiagnostics g;
    g.toneSnrDb = d.lastToneSnrDb;
    g.pwmContrast = (d.pP10 > 1e-6f) ? d.pP90 / d.pP10 : 0.0f;
    g.tickBandRatioDb = std::numeric_limits<float>::quiet_NaN();  // WWV-only metric
    g.toneDetected = (d.phase == Impl::Phase::Running);
    g.phaseLocked = d.phaseKnown;
    g.delayEstMs = std::numeric_limits<float>::quiet_NaN();  // WWV-only metric
    g.anchored = d.anchored;
    g.badFrameStreak = 0;  // WWV-only metric (marker-skeleton streak)
    g.framesInWindow = d.voter.frameCount();
    g.windowSize = d.voter.windowSize();
    g.voteQuality = d.voter.lockConfidence();
    g.refusalReason = static_cast<std::uint8_t>(d.voter.lockRefusal());
    return g;
}

} // namespace AetherSDR
