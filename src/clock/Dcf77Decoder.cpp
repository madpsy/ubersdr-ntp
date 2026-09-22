// Dcf77Decoder.cpp — see Dcf77Decoder.h for the format and the design.
//
// Chain, per input sample of complex baseband:
//
//   carrier search (once)  complex DFT bins across +/-20 Hz of the expected
//                          offset; the peak, refined, is where the mixer goes.
//   mixer                  z = x * e^{-j w0 n}, w0 followed slowly after.
//   AM                     z -> 150 Hz biquad on I/Q -> |.| -> 10 ms boxcar ->
//                          100 Hz envelope; second edges from the carrier cut,
//                          exactly as WwvbDecoder finds them.
//   PM                     y = Im(z conj(r)) / |r|^2, r a 1 Hz one-pole of z: the
//                          carrier's own phase as the reference, so y is the
//                          phase deviation in radians-ish (sin phi). Prefix sums
//                          of y and y^2 go in a ring, which is what makes the
//                          512-chip correlation cost one read per chip
//                          TRANSITION rather than one multiply per sample.
//
// Then once per second, 1.24 s after the second began (when both its envelope
// window and its PM burst are complete): measure AM, measure PM, pick the edge,
// classify both, sync, and at second 59 decode the minute.
//
// The reference is causal, so it lags a frequency offset by a constant phase.
// That costs nothing: the chip sequence is exactly balanced (256 of each), so a
// constant in y correlates to zero. The same balance is why the PM does not
// pull the reference -- every second's burst averages to the unmodulated phase
// whichever bit it carries.

#include "Dcf77Decoder.h"

#include "CivilTime.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <utility>
#include <vector>

namespace clockdec {

namespace {

constexpr double kPi = 3.14159265358979323846;

// ---- the broadcast -------------------------------------------------------
constexpr double kCarrierHz  = 77500.0;
constexpr int    kChips      = 512;
constexpr double kChipSec    = 120.0 / kCarrierHz;     // 1.548 ms
constexpr double kPmStartSec = 15500.0 / kCarrierHz;   // 0.200 s exactly

// ---- AM envelope ---------------------------------------------------------
constexpr int    kEnvRateHz   = 100;
constexpr int    kEnvCap      = 1024;   // > 10 s
constexpr int    kPctWin      = 300;    // 3 s of percentiles
constexpr int    kEdgeTol     = 12;     // +/- env samples searched for a cut
constexpr int    kEdgeLook    = 12;     // lookahead an edge candidate needs
constexpr int    kWarmEnv     = 200;
// p90 over p05. The cut is to 15%, a contrast of about 6.7 on a clean signal;
// under 2 there is no cut to be found.
constexpr double kMinContrast = 2.0;
constexpr double kLpfCutHz    = 150.0;

// ---- carrier search ------------------------------------------------------
constexpr double kAcqSeconds  = 2.0;
constexpr double kSearchHz    = 20.0;   // bins searched, for the noise median
// Where the carrier may be taken from. DCF77's carrier is an atomic standard, so
// only the receiver's own clock can move it in the baseband -- 20 ppm is 1.6 Hz
// at 77.5 kHz -- and a peak further out is somebody else's signal. Measured on
// a receiver with VLF interference: a line 10 Hz off, taken as the carrier.
constexpr double kPullHz      = 3.0;
constexpr double kSearchStep  = 0.25;
constexpr double kToneGate    = 12.0;   // peak over median bin power

// ---- carrier reference ---------------------------------------------------
constexpr double kRefHz   = 1.0;   // PM phase reference corner
constexpr double kDcHz    = 0.1;   // what is taken off y before it is summed
// Half-width of the centred mean taken off y before its variance is taken as
// the correlator's noise: 10 ms wide, so a first null at 100 Hz and a 10 Hz
// tone down 36 dB.
constexpr double kHpHalfSec = 0.005;
// Raw power over high-passed power past which a burst is correlated high-passed.
constexpr double kHpTrigger = 2.0;
constexpr double kFreqGain = 0.2;  // per-second fraction of the residual taken

// ---- PM correlator -------------------------------------------------------
// SNR is |C| / sigma_C, linear. Tracking keeps a lock through ten seconds under
// the floor before giving it up to a full search.
// Acquisition sums correlation power over six seconds and wants the best lag at
// an rms of 3.5 sigma a second: on noise alone, 12000 lags of a chi-square with
// six degrees of freedom pass that about once in 10^9 searches, and on a real
// signal it is reached near 21 dB-Hz, where one second is 3.5 sigma on its own.
constexpr int    kPmAcqSeconds  = 6;
constexpr double kPmAcqScore    = 3.5;
constexpr double kPmMinSnr      = 4.0;
constexpr int    kPmMissLimit   = 10;
constexpr double kPmFullConfSnr = 12.0;   // where a PM bit's confidence saturates
// A PM lock that puts the second more than this far from where AM had it
// re-segments: the count of seconds from AM is not to be trusted across it.
constexpr double kResegTolSec   = 0.030;

// ---- symbols & sync ------------------------------------------------------
// AM confidence is a z-score mapped (z - 1)/5 (classifyAm): 0.5 is 3.5 sigma.
constexpr float kStructConf   = 0.50f;   // AM: enough to call a structural fault
constexpr float kPmStructConf = 0.50f;   // PM: the same, on its own scale
constexpr float kSyncConf     = 0.60f;   // AM: its minute mark, and the cut before it
// PM confidence is SNR / 12, and an antipodal bit at SNR s is wrong Q(s) of the
// time: 3 sigma (0.25) is wrong once in 740, so that is enough to count a bit
// as AGREEING with the fixed word; calling one a contradiction -- a structural
// fault -- wants 6 sigma, where a wrong read is a one-in-10^9 event.
constexpr float kPmSyncConf   = 0.25f;
// Below these a bit is not read at all: the minute fails as incomplete rather
// than being handed to the parity checks to pass or fail by chance. PM 0.1 is
// SNR 1.2 (wrong one time in nine); AM 0.05 is z 1.25. Seen live: a five-second
// fade at the end of a minute, read by PM at SNR ~0, passed all three parity
// checks and was caught only by the weekday not matching the date.
constexpr float kPmBitMinConf = 0.10f;
constexpr float kAmBitMinConf = 0.05f;

// Edge trackers: see WwvbDecoder::trackEdge.
constexpr double kTrkAlpha = 1.0 / 8.0;
constexpr double kTrkBeta  = 1.0 / 128.0;
constexpr int    kTrkWarm  = 8;

// A second with neither an AM cut nor a PM peak for this long, running, has
// nothing holding the segmentation to the air any more.
constexpr int kMaxBlindSeconds = 90;

// WWVB's layout, for the synthetic frames the voter is fed (finalizeFrame).
constexpr std::array<int, 7> kVoterMarkers = {0, 9, 19, 29, 39, 49, 59};
const ClockFieldMap kVMin  = {{1, 40}, {2, 20}, {3, 10}, {5, 8}, {6, 4}, {7, 2}, {8, 1}};
const ClockFieldMap kVHour = {{12, 20}, {13, 10}, {15, 8}, {16, 4}, {17, 2}, {18, 1}};
const ClockFieldMap kVDoy  = {{22, 200}, {23, 100}, {25, 80}, {26, 40}, {27, 20},
                              {28, 10}, {30, 8}, {31, 4}, {32, 2}, {33, 1}};
const ClockFieldMap kVYear = {{45, 80}, {46, 40}, {47, 20}, {48, 10},
                              {50, 8}, {51, 4}, {52, 2}, {53, 1}};

// The PM bit every minute carries in these seconds, whatever the time: the sync
// word. -1 elsewhere.
int pmFixedBit(int sof) {
    if (sof == 59) return 0;
    if (sof >= 0 && sof <= 9) return 1;
    if (sof >= 10 && sof <= 14) return 0;
    return -1;
}

// The chip sequence as +1 (a 0 chip, +15.6 deg) / -1 (a 1 chip). The Galois
// LFSR is PTB's, as published; its first chips are 000001000110000100111...
std::array<int, kChips> buildChips() {
    std::array<int, kChips> t{};
    unsigned lfsr = 0;
    for (int i = 0; i < kChips; ++i) {
        const unsigned chip = lfsr & 1u;
        t[static_cast<std::size_t>(i)] = chip ? -1 : 1;
        lfsr >>= 1;
        if (chip ^ (lfsr == 0 ? 1u : 0u)) lfsr ^= 0x110u;
    }
    return t;
}

struct Biquad {
    double b0 = 1.0, b1 = 0.0, b2 = 0.0, a1 = 0.0, a2 = 0.0;
    double z1 = 0.0, z2 = 0.0;
    double process(double x) {
        const double y = b0 * x + z1;
        z1 = b1 * x - a1 * y + z2;
        z2 = b2 * x - a2 * y;
        return y;
    }
    void reset() { z1 = z2 = 0.0; }
    static Biquad lowpass(double fs, double fc) {
        const double w0 = 2.0 * kPi * fc / fs;
        const double c = std::cos(w0), s = std::sin(w0);
        const double alpha = s / (2.0 * 0.70710678118654752440);
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

// Alpha-beta tracker on second edges, in fractional input samples. As
// WwvbDecoder::trackEdge, with the outlier threshold a parameter: a whole
// envelope block for AM, one chip for PM.
struct EdgeTracker {
    bool valid = false;
    double edge = 0.0, period = 0.0;
    int count = 0, outliers = 0;

    void reset() { valid = false; count = 0; outliers = 0; }

    void update(bool measured, double raw, double nominal, double tol) {
        if (!valid) {
            if (!measured) return;
            edge = raw; period = nominal; valid = true; count = 1; outliers = 0;
            return;
        }
        const double pred = edge + period;
        if (!measured) { edge = pred; return; }
        const double r = raw - pred;
        if (std::fabs(r) > tol) {
            if (++outliers >= 3) { edge = raw; period = nominal; count = 1; outliers = 0; }
            else edge = pred;
            return;
        }
        outliers = 0;
        ++count;
        edge = pred + std::max(1.0 / count, kTrkAlpha) * r;
        if (count > kTrkWarm) {
            period = std::clamp(period + kTrkBeta * r, nominal * (1.0 - 2e-4), nominal * (1.0 + 2e-4));
        }
    }
};

// One classified second, kept so a PM sync found at second 14 can fill in the
// fourteen seconds before it.
struct SecRec {
    int64_t edge = 0;
    ClockSymbol am = ClockSymbol::Unknown;
    float amConf = 0.0f;
    int pmSign = 0;          // +1 / -1 raw correlation sign, 0 when no PM this second
    float pmConf = 0.0f;
};

// One minute's time code, decoded from one demodulator's bits.
struct Decoded {
    bool ok = false;
    int minute = 0, hour = 0, day = 0, month = 0, year2 = 0;
    bool cest = false, leapWarn = false;
    TimeFields utc;          // UTC of THIS frame's second 0
    long long utcMs = 0;
};

} // namespace

// ---------------------------------------------------------------------------

struct Dcf77Decoder::Impl {
    Impl(Dcf77Decoder* o, int sampleRateHz, double carrierOffsetHz)
        : owner(o),
          sr(sampleRateHz > 0 ? sampleRateHz : 12000),
          decim(std::max(1, static_cast<int>(std::lround(sr / 100.0)))),
          fNominal(carrierOffsetHz),
          chips(buildChips()),
          voter(buildVoterConfig()) {
        // C(tau) = sum_k t_k (P(tau+(k+1)tc) - P(tau+k tc)) regrouped by the
        // prefix sum each term reads: only a change of chip contributes.
        auto add = [&](int k, int w) { if (w) coeffs.push_back({k, w}); };
        add(0, -chips[0]);
        for (int k = 1; k < kChips; ++k) add(k, chips[k - 1] - chips[k]);
        add(kChips, chips[kChips - 1]);

        std::size_t cap = 1;
        while (cap < static_cast<std::size_t>(3 * sr)) cap <<= 1;
        pre.assign(cap, 0.0);
        pre2.assign(cap, 0.0);
        preH.assign(cap, 0.0);
        preH2.assign(cap, 0.0);
        hpM = std::max<int64_t>(1, std::llround(kHpHalfSec * sr));
        mask = static_cast<int64_t>(cap) - 1;

        acqTarget = static_cast<int64_t>(std::llround(kAcqSeconds * sr));
        for (double f = fNominal - kSearchHz; f <= fNominal + kSearchHz + 1e-9; f += kSearchStep) {
            Bin b;
            b.f = f;
            b.stepRe = std::cos(-2.0 * kPi * f / sr);
            b.stepIm = std::sin(-2.0 * kPi * f / sr);
            bins.push_back(b);
        }
        pctScratch.reserve(kPctWin);
        reset();
    }

    static TimeFrameVoter::Config buildVoterConfig() {
        TimeFrameVoter::Config c;
        c.fields[TimeFrameVoter::FieldMinutes] = kVMin;
        c.fields[TimeFrameVoter::FieldHours] = kVHour;
        c.fields[TimeFrameVoter::FieldDoy] = kVDoy;
        c.fields[TimeFrameVoter::FieldYear] = kVYear;
        c.markerSeconds.assign(kVoterMarkers.begin(), kVoterMarkers.end());
        // The floors WwvDecoder and WwvbDecoder use, for the same reason.
        c.minBitConfidence = 0.05f;
        c.minLockQuality = 0.05f;
        return c;
    }

    // ---- driving ---------------------------------------------------------

    void process(const float* iq, std::size_t n) {
        for (std::size_t i = 0; i < n; ++i) {
            const double xr = iq[2 * i], xi = iq[2 * i + 1];
            const int64_t k = samplesConsumed++;
            if (phase == Phase::Searching) feedAcquisition(xr, xi);
            else feedSteady(xr, xi, k);
        }
    }

    void reset() {
        phase = Phase::Searching;
        acqCount = 0;
        lastToneSnrDb = 0.0f;
        for (Bin& b : bins) { b.accRe = b.accIm = 0.0; b.rotRe = 1.0; b.rotIm = 0.0; }
        f0 = fNominal;
        oscRe = 1.0; oscIm = 0.0; oscRenorm = 0;
        lpI.reset(); lpQ.reset();
        magAccum = 0.0; magCount = 0;
        env.fill(0.0f); envCount = 0; envBaseSample = 0;
        pHi = pLo = 0.0f;
        refRe = refIm = 0.0; yDc = 0.0; freqCount = 0; freqPrevRe = freqPrevIm = 0.0;
        std::fill(pre.begin(), pre.end(), 0.0);
        std::fill(pre2.begin(), pre2.end(), 0.0);
        std::fill(preH.begin(), preH.end(), 0.0);
        std::fill(preH2.begin(), preH2.end(), 0.0);
        steadyStart = 0;
        segValid = false; segEdge = 0.0; scanPos = 0; blindSeconds = 0;
        amTrk.reset(); pmTrk.reset();
        pmLocked = false; pmMiss = 0; lastSearchEnd = 0; acqPint = 0; acqHistN = 0; acqHistNext = 0;
        lastPmSnr = std::numeric_limits<float>::quiet_NaN();
        amMinusPm = 0.0; amMinusPmHave = false;
        lowFrac = 0.15;
        polarity = 1; polarityKnown = false;
        hist.clear();
        timingFromPm = false;
        dropAnchor();
        haveLastFrame = false; lastFrameStart = 0;
        lastFrameFrom = 0;
        prevFrameFrom = 0;
        pmRefusedLocks = 0;
        samplesConsumed = 0;
        setLockState(ClockLockState::NoSignal);
    }

    // ---- carrier search --------------------------------------------------

    void feedAcquisition(double xr, double xi) {
        for (Bin& b : bins) {
            b.accRe += xr * b.rotRe - xi * b.rotIm;
            b.accIm += xr * b.rotIm + xi * b.rotRe;
            const double nr = b.rotRe * b.stepRe - b.rotIm * b.stepIm;
            b.rotIm = b.rotRe * b.stepIm + b.rotIm * b.stepRe;
            b.rotRe = nr;
        }
        if ((acqCount & 1023) == 1023) {
            for (Bin& b : bins) {
                const double m = std::hypot(b.rotRe, b.rotIm);
                if (m > 1e-9) { b.rotRe /= m; b.rotIm /= m; }
            }
        }
        if (++acqCount >= acqTarget) finalizeAcquisition();
    }

    void finalizeAcquisition() {
        std::vector<double> pw(bins.size());
        std::size_t peak = bins.size();
        for (std::size_t i = 0; i < bins.size(); ++i) {
            pw[i] = bins[i].accRe * bins[i].accRe + bins[i].accIm * bins[i].accIm;
            if (std::fabs(bins[i].f - fNominal) <= kPullHz && (peak == bins.size() || pw[i] > pw[peak])) peak = i;
        }
        std::vector<double> med = pw;
        std::nth_element(med.begin(), med.begin() + med.size() / 2, med.end());
        const double median = med[med.size() / 2];
        if (median > 0.0) lastToneSnrDb = static_cast<float>(10.0 * std::log10(pw[peak] / median));

        if (median > 0.0 && pw[peak] > kToneGate * median) {
            // Parabolic peak on the magnitude, to a fraction of the step.
            double f = bins[peak].f;
            if (peak > 0 && peak + 1 < bins.size()) {
                const double a = std::sqrt(pw[peak - 1]), b = std::sqrt(pw[peak]), c = std::sqrt(pw[peak + 1]);
                const double den = a - 2.0 * b + c;
                if (den < 0.0) f += kSearchStep * std::clamp(0.5 * (a - c) / den, -0.5, 0.5);
            }
            f0 = f;
            startSteady();
        } else {
            for (Bin& b : bins) { b.accRe = b.accIm = 0.0; b.rotRe = 1.0; b.rotIm = 0.0; }
            acqCount = 0;
        }
    }

    void setMixer(double f) {
        f0 = std::clamp(f, fNominal - kPullHz, fNominal + kPullHz);
        stepRe = std::cos(-2.0 * kPi * f0 / sr);
        stepIm = std::sin(-2.0 * kPi * f0 / sr);
    }

    void startSteady() {
        phase = Phase::Running;
        envBaseSample = samplesConsumed;
        steadyStart = samplesConsumed;
        pre[static_cast<std::size_t>(steadyStart & mask)] = 0.0;
        pre2[static_cast<std::size_t>(steadyStart & mask)] = 0.0;
        lastSearchEnd = steadyStart;
        setMixer(f0);
        oscRe = 1.0; oscIm = 0.0; oscRenorm = 0;
        lpI = Biquad::lowpass(sr, kLpfCutHz);
        lpQ = Biquad::lowpass(sr, kLpfCutHz);
        // The low-pass's DC group delay, as WwvbDecoder derives it.
        lpfDelay = 1.0 - (lpI.a1 + 2.0 * lpI.a2) / (1.0 + lpI.a1 + lpI.a2);
        aRef = 1.0 - std::exp(-2.0 * kPi * kRefHz / sr);
        aDc = 1.0 - std::exp(-2.0 * kPi * kDcHz / sr);
        setLockState(ClockLockState::Acquiring);
    }

    // ---- per sample ------------------------------------------------------

    void feedSteady(double xr, double xi, int64_t k) {
        const double zr = xr * oscRe - xi * oscIm;
        const double zi = xr * oscIm + xi * oscRe;
        const double nr = oscRe * stepRe - oscIm * stepIm;
        oscIm = oscRe * stepIm + oscIm * stepRe;
        oscRe = nr;
        if (++oscRenorm >= 1024) {
            const double m = std::hypot(oscRe, oscIm);
            if (m > 1e-9) { oscRe /= m; oscIm /= m; }
            oscRenorm = 0;
        }

        // PM: the phase against the carrier's own recent phase.
        refRe += aRef * (zr - refRe);
        refIm += aRef * (zi - refIm);
        const double rp = refRe * refRe + refIm * refIm;
        double y = rp > 1e-30 ? (zi * refRe - zr * refIm) / rp : 0.0;
        yDc += aDc * (y - yDc);
        y -= yDc;
        const std::size_t i0 = static_cast<std::size_t>(k & mask);
        const std::size_t i1 = static_cast<std::size_t>((k + 1) & mask);
        pre[i1] = pre[i0] + y;
        pre2[i1] = pre2[i0] + y * y;

        // The same, high-passed without a phase shift: each sample less the
        // mean of the 2M+1 centred on it, M samples late. A causal filter would
        // move the correlation peak; this cannot. It is always the noise
        // estimate (snrAt), and it is what is correlated when a burst carries
        // strong low-frequency interference (hpNeeded). A steady tone a few
        // hertz off the carrier lands in y as a large slow sinusoid: counted as
        // noise it made every PM bit look like a coin toss, and correlated it
        // flipped bits. But the high-pass costs real signal -- about 3 dB on the
        // KiwiSDR recording -- so a clean burst is correlated on y itself.
        const int64_t c = k - hpM;
        if (c >= steadyStart + hpM) {
            const std::size_t c0 = static_cast<std::size_t>(c & mask);
            const std::size_t c1 = static_cast<std::size_t>((c + 1) & mask);
            if (c == steadyStart + hpM) { preH[c0] = 0.0; preH2[c0] = 0.0; }
            const double raw = pre[c1] - pre[c0];
            const double ma = (pre[i1] - pre[static_cast<std::size_t>((c - hpM) & mask)]) /
                              static_cast<double>(2 * hpM + 1);
            const double yh = raw - ma;
            preH[c1] = preH[c0] + yh;
            preH2[c1] = preH2[c0] + yh * yh;
        }

        // Follow the carrier: a residual offset shows as the reference turning.
        if (++freqCount >= sr) {
            freqCount = 0;
            if (freqPrevRe != 0.0 || freqPrevIm != 0.0) {
                const double cr = refRe * freqPrevRe + refIm * freqPrevIm;
                const double ci = refIm * freqPrevRe - refRe * freqPrevIm;
                const double dphi = std::atan2(ci, cr);
                setMixer(f0 + kFreqGain * dphi / (2.0 * kPi));
            }
            freqPrevRe = refRe; freqPrevIm = refIm;
        }

        // AM envelope.
        const double I = lpI.process(zr), Q = lpQ.process(zi);
        magAccum += std::sqrt(I * I + Q * Q);
        if (++magCount >= decim) {
            pushEnvelope(static_cast<float>(magAccum / magCount));
            magAccum = 0.0;
            magCount = 0;
        }
    }

    void pushEnvelope(float v) {
        env[static_cast<std::size_t>(envCount % kEnvCap)] = v;
        ++envCount;
        updatePercentiles();
        if (!segValid) trySeedAm();
        pmAcquireStep();
        while (segValid && canProcessSecond()) processSecond();
    }

    // ---- prefix sums -----------------------------------------------------

    // First sample index whose prefix sum is still in the ring.
    int64_t prefixLo() const {
        return std::max<int64_t>(steadyStart + hpM, samplesConsumed - hpM - mask + 1);
    }
    // One past the last sample whose high-passed prefix sum exists.
    int64_t prefixHi() const { return samplesConsumed - hpM; }
    // Sum of y (or y^2) over [steadyStart, x), linearly interpolated.
    double prefixAt(const std::vector<double>& p, double x) const {
        const double fl = std::floor(x);
        const int64_t k = static_cast<int64_t>(fl);
        const double a = p[static_cast<std::size_t>(k & mask)];
        const double b = p[static_cast<std::size_t>((k + 1) & mask)];
        return a + (x - fl) * (b - a);
    }
    bool burstInRing(double tau, double tc) const {
        return tau - 2.0 >= static_cast<double>(prefixLo()) &&
               tau + kChips * tc + 2.0 <= static_cast<double>(prefixHi());
    }
    double corrAt(double tau, double tc) const {
        const std::vector<double>& p = useHp ? preH : pre;
        double s = 0.0;
        for (const auto& c : coeffs) s += c.w * prefixAt(p, tau + c.k * tc);
        return s;
    }

    // Whether [a, b) carries strong low-frequency interference: y's power
    // more than double what is left of it high-passed. Noise and the chips
    // alone lose little to the high-pass (a ratio near 1.0-1.2); a tone near
    // the carrier puts most of y's power below it.
    bool hpNeeded(double a, double b) const {
        const double n = b - a;
        if (!(n > 1.0)) return false;
        auto power = [&](const std::vector<double>& p1, const std::vector<double>& p2) {
            const double m = (prefixAt(p1, b) - prefixAt(p1, a)) / n;
            return (prefixAt(p2, b) - prefixAt(p2, a)) / n - m * m;
        };
        return power(pre, pre2) > kHpTrigger * std::max(1e-30, power(preH, preH2));
    }
    // |C| / sigma_C, sigma from the burst's own residual variance.
    double snrAt(double tau, double tc, double C) const {
        const double N = kChips * tc;
        const double s1 = prefixAt(preH, tau + N) - prefixAt(preH, tau);
        const double s2 = prefixAt(preH2, tau + N) - prefixAt(preH2, tau);
        const double mean = s1 / N, sig = C / N;
        // Noise is what is left of the high-passed burst's power once the chip
        // signal is taken out. The high-pass takes a little of the signal too,
        // so on a clean burst that difference can reach zero; floored at 0.1%
        // of the power, past which a PM bit's confidence is saturated anyway.
        const double pow = std::max(1e-30, s2 / N - mean * mean);
        const double var = std::max(1e-3 * pow, pow - sig * sig);
        return std::fabs(C) / std::sqrt(var * N);
    }

    // Early-late on the triangle the chip correlation makes: E and L one half
    // chip either side of the estimate, and (E - L)/(E + L) is the error in
    // units of (tc - d). Twice, from an integer-lag peak.
    double refine(double tau, double tc, double sign) const {
        const double d = 0.5 * tc;
        for (int it = 0; it < 2; ++it) {
            const double E = sign * corrAt(tau - d, tc);
            const double L = sign * corrAt(tau + d, tc);
            if (E + L <= 0.0) break;
            const double eps = std::clamp((E - L) / (E + L) * (tc - d), -d, d);
            tau -= eps;
        }
        return tau;
    }

    double period() const {
        if (pmLocked && pmTrk.valid) return pmTrk.period;
        if (amTrk.valid) return amTrk.period;
        return sr;
    }

    // ---- PM acquisition: every lag of the last second --------------------

    void pmAcquireStep() {
        if (pmLocked) return;
        const double P = period();
        const double tc = kChipSec * P;
        const int64_t total = samplesConsumed;
        if (total - lastSearchEnd < static_cast<int64_t>(P)) return;
        const int64_t hi = static_cast<int64_t>(std::floor(prefixHi() - kChips * tc - tc - 3.0));
        const int64_t lo = hi - static_cast<int64_t>(P) + 1;
        if (lo - static_cast<int64_t>(tc) - 2 < prefixLo()) return;
        lastSearchEnd = total;

        // Integer lags share their fractional chip offsets, so they are worked
        // out once.
        struct Off { int64_t fl; double fr; int w; };
        std::vector<Off> offs;
        offs.reserve(coeffs.size());
        for (const auto& c : coeffs) {
            const double o = c.k * tc;
            const double fl = std::floor(o);
            offs.push_back({static_cast<int64_t>(fl), o - fl, c.w});
        }
        const int pint = static_cast<int>(hi - lo + 1);
        if (pint != acqPint) {
            acqPint = pint;
            acqBase = lo;
            acqHist.assign(kPmAcqSeconds, std::vector<float>(static_cast<std::size_t>(pint), 0.0f));
            acqHistN = 0;
            acqHistNext = 0;
        }
        useHp = hpNeeded(static_cast<double>(lo), static_cast<double>(hi) + kChips * tc);
        const std::vector<double>& pr = useHp ? preH : pre;
        std::vector<double> c(static_cast<std::size_t>(pint));
        for (int64_t tau = lo; tau <= hi; ++tau) {
            double sum = 0.0;
            for (const Off& o : offs) {
                const int64_t k = tau + o.fl;
                const double a = pr[static_cast<std::size_t>(k & mask)];
                const double b = pr[static_cast<std::size_t>((k + 1) & mask)];
                sum += o.w * (a + o.fr * (b - a));
            }
            c[static_cast<std::size_t>(tau - lo)] = sum;
        }
        // sigma^2 of one lag's correlation, from the median over all of them:
        // nearly every lag is noise, and the median of a chi-square with one
        // degree of freedom is 0.455 sigma^2.
        std::vector<double> sq(c.size());
        for (std::size_t i = 0; i < c.size(); ++i) sq[i] = c[i] * c[i];
        std::nth_element(sq.begin(), sq.begin() + sq.size() / 2, sq.end());
        const double var = std::max(1e-30, sq[sq.size() / 2] / 0.455);

        // Filed by lag modulo the second, so the same phase lines up across
        // searches however far apart they ran.
        std::vector<float>& slot = acqHist[static_cast<std::size_t>(acqHistNext)];
        const int64_t off0 = ubersdr_ntp::floorMod(lo - acqBase, pint);
        for (int i = 0; i < pint; ++i)
            slot[static_cast<std::size_t>((off0 + i) % pint)] = static_cast<float>(c[static_cast<std::size_t>(i)] * c[static_cast<std::size_t>(i)] / var);
        acqHistNext = (acqHistNext + 1) % kPmAcqSeconds;
        acqHistN = std::min(acqHistN + 1, kPmAcqSeconds);

        // Non-coherent: the bit flips the sign of every second's peak but not
        // its power, so power adds across seconds where amplitude cannot.
        int bestIdx = 0;
        double bestS = -1.0;
        for (int i = 0; i < pint; ++i) {
            double sum = 0.0;
            for (int h = 0; h < acqHistN; ++h) sum += acqHist[static_cast<std::size_t>(h)][static_cast<std::size_t>(i)];
            if (sum > bestS) { bestS = sum; bestIdx = i; }
        }
        const double score = std::sqrt(std::max(0.0, bestS) / std::max(1, acqHistN));
        lastPmSnr = static_cast<float>(20.0 * std::log10(std::max(score, 1e-3)));
        if (acqHistN < kPmAcqSeconds || score < kPmAcqScore) return;

        // The phase, in this search's own window, refined on this search's data.
        const int64_t tau0 = lo + ubersdr_ntp::floorMod(bestIdx - off0, pint);
        const double c0 = c[static_cast<std::size_t>(tau0 - lo)];
        const double tau = refine(static_cast<double>(tau0), tc, c0 >= 0.0 ? 1.0 : -1.0);
        pmLockAt(tau, P);
    }

    // AM has been reading valid minutes, lately: the last one or the one
    // before it came out of AM's own parity checks.
    bool amDecoding() const {
        return anchored && (lastFrameFrom == 1 || lastFrameFrom == 3 || prevFrameFrom == 1 || prevFrameFrom == 3);
    }

    void pmLockAt(double tau, double P) {
        pmLocked = true;
        pmMiss = 0;
        acqPint = 0;
        pmTrk.reset();
        const double pmEdge = tau - kPmStartSec * P;
        if (!segValid) {
            segValid = true;
            segEdge = pmEdge;
            amTrk.reset();
            hist.clear();
            return;
        }
        // Where PM puts the second the segmentation is about to process.
        const double cand = pmEdge + P * std::round((segEdge - pmEdge) / P);
        if (std::fabs(cand - segEdge) > kResegTolSec * sr && amDecoding()) {
            // AM is decoding valid minutes, which it cannot do at the wrong
            // phase of the second: this PM "lock" is interference that happened
            // to correlate. Refused, and searched for again from nothing. Without
            // this a receiver with QRM had its working AM anchor thrown away
            // every few tens of seconds.
            pmLocked = false;
            ++pmRefusedLocks;
            return;
        }
        if (std::fabs(cand - segEdge) > kResegTolSec * sr) {
            // AM had the second somewhere else. PM wins, and nothing counted
            // from AM's idea of the second survives it.
            if (lockState == ClockLockState::Locked) setLockState(ClockLockState::Acquiring);
            dropAnchor();
            hist.clear();
            amTrk.reset();
        }
        segEdge = cand;
    }

    // Search one chip and a half either side of where the tracker expects the
    // burst. Returns whether the peak was clear enough to time the second by;
    // C and snr are filled either way, at the peak if found and at the
    // prediction if not, so the bit is still read from where the burst is.
    bool pmTrack(double pred, double P, double& tauOut, double& Cout, double& snrOut) {
        const double tc = kChipSec * P;
        const int win = static_cast<int>(std::ceil(1.5 * tc));
        const double base = std::floor(pred);
        if (!burstInRing(base - win - tc, tc) || !burstInRing(base + win + tc, tc)) {
            Cout = 0.0; snrOut = 0.0; tauOut = pred;
            return false;
        }
        useHp = hpNeeded(base - win, base + win + kChips * tc);
        double best = 0.0, bestTau = base;
        for (int d = -win; d <= win; ++d) {
            const double c = corrAt(base + d, tc);
            if (std::fabs(c) > std::fabs(best)) { best = c; bestTau = base + d; }
        }
        const double tau = refine(bestTau, tc, best >= 0.0 ? 1.0 : -1.0);
        const double C = corrAt(tau, tc);
        const double snr = snrAt(tau, tc, C);
        if (snr >= kPmMinSnr && std::fabs(tau - pred) <= win) {
            tauOut = tau; Cout = C; snrOut = snr;
            return true;
        }
        tauOut = pred;
        Cout = corrAt(pred, tc);
        snrOut = snrAt(pred, tc, Cout);
        return false;
    }

    // ---- AM envelope helpers (as WwvbDecoder) ----------------------------

    float envAt(int64_t idx) const { return env[static_cast<std::size_t>(idx % kEnvCap)]; }

    // Envelope position of an input-sample instant, delay of the low-pass included.
    double envPos(double sample) const {
        return (sample - static_cast<double>(envBaseSample) + lpfDelay) / decim;
    }

    void updatePercentiles() {
        const int64_t n = std::min<int64_t>(kPctWin, envCount);
        if (n < 8) { pHi = pLo = 0.0f; return; }
        pctScratch.clear();
        for (int64_t i = envCount - n; i < envCount; ++i) pctScratch.push_back(envAt(i));
        // p05 lands in the cut: 10-20% of every second is cut.
        const std::size_t k05 = static_cast<std::size_t>(0.05 * n);
        const std::size_t k90 = static_cast<std::size_t>(0.90 * n);
        std::nth_element(pctScratch.begin(), pctScratch.begin() + k05, pctScratch.end());
        pLo = pctScratch[k05];
        std::nth_element(pctScratch.begin(), pctScratch.begin() + k90, pctScratch.end());
        pHi = pctScratch[k90];
    }

    bool haveContrast() const { return pHi >= kMinContrast * std::max(pLo, 1e-9f); }

    void edgeThresholds(float& hi, float& mid, float& lo) const {
        hi  = pLo + 0.6f * (pHi - pLo);
        mid = pLo + 0.5f * (pHi - pLo);
        lo  = pLo + 0.4f * (pHi - pLo);
    }

    bool sustainedLow(int64_t i, float thrLo) const {
        int checked = 0, low = 0;
        for (int d = 1; d <= 7; ++d) {
            if (i + d >= envCount) break;
            ++checked;
            if (envAt(i + d) < thrLo) ++low;
        }
        return checked >= 5 && low >= checked - 1;
    }

    bool isFallingEdge(int64_t i, float thrHi, float thrMid, float thrLo) const {
        if (!(envAt(i - 1) >= thrMid && envAt(i) < thrMid)) return false;
        bool high = false;
        for (int d = 1; d <= 3 && !high; ++d) high = envAt(i - d) >= thrHi;
        return high && sustainedLow(i, thrLo);
    }

    // Sub-block position of the cut crossing at env[i], as an input sample.
    // WwvbDecoder::edgeSampleAt explains the area method.
    double edgeSampleAt(int64_t i) const {
        double hi = 0.0, lo = 0.0;
        for (int d = 4; d <= 8; ++d) hi += envAt(i - d);
        for (int d = 4; d <= 8; ++d) lo += envAt(i + d);
        hi /= 5.0;
        lo /= 5.0;
        double pos = static_cast<double>(i);
        const double span = hi - lo;
        if (span > 1e-12) {
            double area = 0.0;
            for (int64_t k = i - 2; k <= i + 3; ++k) area += (envAt(k) - lo) / span;
            pos = std::clamp(static_cast<double>(i - 2) + area, i - 1.5, i + 1.5);
        }
        return static_cast<double>(envBaseSample) + pos * decim - lpfDelay;
    }

    void trySeedAm() {
        if (envCount < kWarmEnv || !haveContrast()) return;
        float thrHi, thrMid, thrLo;
        edgeThresholds(thrHi, thrMid, thrLo);
        const int64_t lo = std::max<int64_t>(std::max<int64_t>(scanPos, 9), envCount - kEnvCap + 9);
        for (int64_t i = lo; i + kEdgeLook < envCount; ++i) {
            scanPos = i;
            if (isFallingEdge(i, thrHi, thrMid, thrLo)) {
                segValid = true;
                segEdge = edgeSampleAt(i);
                hist.clear();
                return;
            }
        }
    }

    bool findFallingEdgeNear(int64_t pred, int tol, double& edgeOut) const {
        if (!haveContrast()) return false;
        float thrHi, thrMid, thrLo;
        edgeThresholds(thrHi, thrMid, thrLo);
        const int64_t lo = std::max<int64_t>(pred - tol, envCount - kEnvCap + 9);
        const int64_t hi = std::min<int64_t>(pred + tol, envCount - 1 - kEdgeLook);
        int64_t bestEdge = pred, bestDist = tol + 1;
        for (int64_t i = lo; i <= hi; ++i) {
            if (i < 9) continue;
            if (isFallingEdge(i, thrHi, thrMid, thrLo)) {
                const int64_t d = std::llabs(i - pred);
                if (d < bestDist) { bestDist = d; bestEdge = i; }
            }
        }
        if (bestDist > tol) return false;
        edgeOut = edgeSampleAt(bestEdge);
        return true;
    }

    // ---- once per second -------------------------------------------------

    bool canProcessSecond() const {
        const int64_t j0 = static_cast<int64_t>(std::floor(envPos(segEdge)));
        if (envCount < j0 + kEnvRateHz + kEdgeTol + kEdgeLook) return false;
        const double P = period();
        const double tc = kChipSec * P;
        const double burstEnd = segEdge + kPmStartSec * P + kChips * tc + 3.0 * tc + 4.0;
        return static_cast<double>(prefixHi()) >= burstEnd;
    }

    // The AM symbol of the second that starts at `edge`: how far the carrier
    // is cut in the first and second tenths, against its level later on.
    //
    // Confidence is a z-score, not a distance: how far the reading sits from
    // the boundary with the runner-up, in units of the envelope's own noise,
    // measured from the spread of the full-carrier stretch of this very second.
    // Mapped (z - 1) / 5, so 0.5 is 3.5 sigma and 1.0 is 6. Without the noise
    // term a reading lost in the noise could still come out "certain".
    //
    // The depth of the cut is learnt, not assumed. In noise the envelope of a
    // carrier cut to 15% averages well above 15% -- the noise does not cancel
    // in a magnitude -- so the cut looks shallower the weaker the signal. It is
    // learnt from every second the frame says MUST be cut (anything but s59 of
    // an anchored minute), and otherwise only from clear cuts.
    void classifyAm(double edge, bool mustCut, ClockSymbol& sym, float& conf,
                    std::array<float, kEnvRateHz>& w) {
        const int64_t j0 = static_cast<int64_t>(std::llround(envPos(edge)));
        for (int k = 0; k < kEnvRateHz; ++k) w[k] = envAt(j0 + k);
        sym = ClockSymbol::Unknown;
        conf = 0.0f;
        if (j0 < envCount - kEnvCap + 1 || !haveContrast()) return;
        auto mean = [&](int a, int b) {
            double sum = 0.0;
            for (int k = a; k <= b; ++k) sum += w[k];
            return sum / (b - a + 1);
        };
        const double H = mean(30, 94);
        if (!(H > 0.0)) return;
        double var = 0.0;
        for (int k = 30; k <= 94; ++k) var += (w[k] - H) * (w[k] - H);
        const double sBlock = std::sqrt(var / 64.0);
        const double d1 = mean(2, 8), d2 = mean(12, 18);
        const double depth = std::max(0.1, 1.0 - lowFrac);
        const double x1 = (1.0 - d1 / H) / depth;
        const double x2 = (1.0 - d2 / H) / depth;
        const double sx = std::max(1e-4, sBlock / std::sqrt(7.0) / (H * depth));

        // Hypotheses in (x1, x2): 0.1 s cut (1,0), 0.2 s cut (1,1), none (0,0).
        static constexpr double kMu[3][2] = {{1, 0}, {1, 1}, {0, 0}};
        double ds[3];
        for (int h = 0; h < 3; ++h)
            ds[h] = (x1 - kMu[h][0]) * (x1 - kMu[h][0]) + (x2 - kMu[h][1]) * (x2 - kMu[h][1]);
        int best = 0;
        for (int h = 1; h < 3; ++h) if (ds[h] < ds[best]) best = h;
        int run = -1;
        for (int h = 0; h < 3; ++h) if (h != best && (run < 0 || ds[h] < ds[run])) run = h;
        const double sep = std::hypot(kMu[best][0] - kMu[run][0], kMu[best][1] - kMu[run][1]);
        const double z = (ds[run] - ds[best]) / (2.0 * sep * sx);
        conf = static_cast<float>(std::clamp((z - 1.0) / 5.0, 0.0, 1.0));
        sym = best == 0 ? ClockSymbol::Zero : best == 1 ? ClockSymbol::One : ClockSymbol::Marker;

        if (mustCut || (best != 2 && conf >= 0.5f))
            lowFrac += (mustCut ? 0.02 : 0.05) * (std::clamp(d1 / H, 0.0, 0.9) - lowFrac);
    }

    void processSecond() {
        const double P = period();
        const double pred = segEdge;

        double amEdge = 0.0;
        const bool amMeas = findFallingEdgeNear(std::llround(envPos(pred)), kEdgeTol, amEdge);

        bool pmMeas = false;
        double pmTau = 0.0, pmC = 0.0, pmSnr = 0.0;
        const bool pmWas = pmLocked;
        if (pmLocked) {
            pmMeas = pmTrack(pred + kPmStartSec * P, P, pmTau, pmC, pmSnr);
            lastPmSnr = static_cast<float>(20.0 * std::log10(std::max(pmSnr, 1e-3)));
            if (pmMeas) pmMiss = 0;
            else if (++pmMiss >= kPmMissLimit) {
                pmLocked = false;
                pmTrk.reset();
                lastSearchEnd = samplesConsumed;
            }
        }
        const double pmEdge = pmTau - kPmStartSec * P;
        if (pmLocked) pmTrk.update(pmMeas, pmEdge, P, kChipSec * P);
        amTrk.update(amMeas, amEdge, amTrk.valid ? amTrk.period : static_cast<double>(sr), decim);
        if (pmMeas && amMeas) {
            const double d = (amEdge - pmEdge) * 1000.0 / sr;
            amMinusPm = amMinusPmHave ? amMinusPm + 0.05 * (d - amMinusPm) : d;
            amMinusPmHave = true;
        }

        timingFromPm = pmLocked && pmTrk.valid;
        const double edge = timingFromPm ? pmTrk.edge : amTrk.valid ? amTrk.edge : pred;
        const bool measured = timingFromPm ? pmMeas : amMeas;
        blindSeconds = (amMeas || pmMeas) ? 0 : blindSeconds + 1;

        SecRec r;
        r.edge = static_cast<int64_t>(std::llround(edge));
        std::array<float, kEnvRateHz> w{};
        const int sofHere = anchored ? sofNext : -1;
        classifyAm(edge, sofHere >= 0 && sofHere < 59, r.am, r.amConf, w);
        if (pmWas && pmSnr > 0.0) {
            r.pmSign = pmC >= 0.0 ? 1 : -1;
            r.pmConf = static_cast<float>(std::clamp(pmSnr / kPmFullConfSnr, 0.0, 1.0));
        }
        hist.push_back(r);
        if (hist.size() > 64) hist.erase(hist.begin());

        handleSecond(r, measured, w);

        segEdge = timingFromPm ? pmTrk.edge + pmTrk.period
                : amTrk.valid  ? amTrk.edge + amTrk.period
                               : edge + sr;

        if (blindSeconds >= kMaxBlindSeconds) {
            // Nothing has been heard at the second for a minute and a half:
            // start again from the air rather than coast on a stale cadence.
            if (lockState == ClockLockState::Locked) setLockState(ClockLockState::Acquiring);
            segValid = false;
            scanPos = 0;
            blindSeconds = 0;
            amTrk.reset();
            dropAnchor();
            hist.clear();
        }
    }

    // ---- sync & frames ---------------------------------------------------

    int pmBitOf(const SecRec& r) const { return r.pmSign * polarity > 0 ? 0 : 1; }

    // The last sixteen seconds read, through PM, as s59 s0..s14 -- this second
    // being s14. Every bit clear, and AM not saying s59 was cut.
    bool pmSyncHere() {
        if (hist.size() < 16) return false;
        const std::size_t base = hist.size() - 16;   // s59
        auto matches = [&](int pol) {
            for (std::size_t j = 0; j < 16; ++j) {
                const SecRec& h = hist[base + j];
                if (h.pmSign == 0 || h.pmConf < kPmSyncConf) return false;
                const int sof = j == 0 ? 59 : static_cast<int>(j) - 1;
                const int bit = h.pmSign * pol > 0 ? 0 : 1;
                if (bit != pmFixedBit(sof)) return false;
            }
            return true;
        };
        const SecRec& s59 = hist[base];
        if (s59.amConf >= kSyncConf && s59.am != ClockSymbol::Marker) return false;
        if (polarityKnown) return matches(polarity);
        for (int pol : {1, -1}) {
            if (matches(pol)) { polarity = pol; polarityKnown = true; return true; }
        }
        return false;
    }

    // This second had no cut, clearly, and the one before did.
    bool amMarkerHere() const {
        if (hist.size() < 2 || !haveContrast()) return false;
        const SecRec& cur = hist.back();
        const SecRec& prev = hist[hist.size() - 2];
        return cur.am == ClockSymbol::Marker && cur.amConf >= kSyncConf &&
               prev.am != ClockSymbol::Marker && prev.amConf >= kSyncConf;
    }

    void dropAnchor() {
        anchored = false;
        sofNext = 0;
        leapMinute = false;
        leapInserted = false;
        lastLeapWarn = false;
        haveVoted = false;
        haveLastFrame = false;
        voter.reset();
        for (auto& f : frame) f = SecRec{};
        frFilled = 0;
    }

    void demote() {
        haveVoted = false;
        if (lockState == ClockLockState::Locked) setLockState(ClockLockState::Acquiring);
    }

    void handleSecond(const SecRec& r, bool measured, const std::array<float, kEnvRateHz>& w) {
        int sof = -1;
        bool record = false;
        if (!anchored) {
            if (pmSyncHere()) {
                anchored = true;
                sof = 14;
                sofNext = 15;
                const std::size_t s0 = hist.size() - 15;
                frStart = hist[s0].edge;
                frFilled = 0;
                for (int s = 0; s < 14; ++s) { frame[static_cast<std::size_t>(s)] = hist[s0 + s]; ++frFilled; }
                record = true;
            } else if (amMarkerHere()) {
                anchored = true;
                sof = 59;          // of a minute not recorded; the next is s0
                sofNext = 0;
            }
        } else {
            sof = sofNext;
            if (sof == 0) { frStart = r.edge; frFilled = 0; }
            if (lockState == ClockLockState::Locked) checkStructure(r, sof);
            if (sof == 60) {
                leapInserted = true;       // the minute just ended ran 61 s
                sofNext = 0;
            } else {
                record = true;
                sofNext = sof + 1;
            }
        }

        emitSecond(r, measured, sof, w);

        if (record) {
            frame[static_cast<std::size_t>(sof)] = r;
            ++frFilled;
            if (sof == 58) leapMinute = tentativeLeapMinute();
            if (sof == 59) {
                finalizeFrame();
                if (anchored) sofNext = leapMinute ? 60 : 0;
            }
        }

        if (lockState == ClockLockState::Locked && haveVoted && owner->onTime) {
            ClockTimeInfo ti;
            ti.minute = votedMinute; ti.hour = votedHour;
            ti.doy = votedDoy; ti.year2 = votedYear;
            ti.quality = votedQuality;
            ti.lastEdgeSample = r.edge;
            ti.lastEdgeSecondOfFrame = sof;
            ti.station = ClockStation::Dcf77;
            owner->onTime(ti);
        }
    }

    // A confident symbol that contradicts the minute's skeleton means the
    // count of seconds has slipped. Stop certifying now; s59 decides.
    void checkStructure(const SecRec& r, int sof) {
        if (haveContrast() && r.amConf >= kStructConf) {
            const bool expectMark = (sof == 59 && !leapMinute) || sof == 60;
            if ((r.am == ClockSymbol::Marker) != expectMark) { demote(); return; }
        }
        const int fixed = pmFixedBit(sof);
        if (polarityKnown && fixed >= 0 && r.pmSign != 0 && r.pmConf >= kPmStructConf && pmBitOf(r) != fixed) demote();
    }

    // ---- the time code ---------------------------------------------------

    enum class Src { Am, Pm };

    bool bitOf(const SecRec& r, Src src, int& bit, float& conf) const {
        if (src == Src::Pm) {
            if (r.pmSign == 0 || !polarityKnown || r.pmConf < kPmBitMinConf) return false;
            bit = pmBitOf(r);
            conf = r.pmConf;
            return true;
        }
        if (r.am != ClockSymbol::Zero && r.am != ClockSymbol::One) return false;
        if (r.amConf < kAmBitMinConf) return false;
        bit = r.am == ClockSymbol::One ? 1 : 0;
        conf = r.amConf;
        return true;
    }

    Decoded decode(Src src) const {
        Decoded d;
        std::array<int, 60> b{};
        for (int s = 15; s <= 58; ++s) {
            int bit = 0; float c = 0.0f;
            if (!bitOf(frame[static_cast<std::size_t>(s)], src, bit, c)) return d;
            b[static_cast<std::size_t>(s)] = bit;
        }
        auto at = [&](int s) { return b[static_cast<std::size_t>(s)]; };
        auto parity = [&](int a, int z) { int p = 0; for (int s = a; s <= z; ++s) p ^= at(s); return p; };
        if (at(20) != 1) return d;                 // start of time, always 1
        if (at(17) == at(18)) return d;            // exactly one of CEST / CET
        if (parity(21, 28) || parity(29, 35) || parity(36, 58)) return d;

        const int minU = at(21) + 2 * at(22) + 4 * at(23) + 8 * at(24);
        const int minT = at(25) + 2 * at(26) + 4 * at(27);
        const int hrU = at(29) + 2 * at(30) + 4 * at(31) + 8 * at(32);
        const int hrT = at(33) + 2 * at(34);
        const int dayU = at(36) + 2 * at(37) + 4 * at(38) + 8 * at(39);
        const int dayT = at(40) + 2 * at(41);
        const int wday = at(42) + 2 * at(43) + 4 * at(44);
        const int monU = at(45) + 2 * at(46) + 4 * at(47) + 8 * at(48);
        const int monT = at(49);
        const int yrU = at(50) + 2 * at(51) + 4 * at(52) + 8 * at(53);
        const int yrT = at(54) + 2 * at(55) + 4 * at(56) + 8 * at(57);
        if (minU > 9 || hrU > 9 || dayU > 9 || monU > 9 || yrU > 9 || yrT > 9) return d;
        d.minute = minT * 10 + minU;
        d.hour = hrT * 10 + hrU;
        d.day = dayT * 10 + dayU;
        d.month = monT * 10 + monU;
        d.year2 = yrT * 10 + yrU;
        if (d.minute > 59 || d.hour > 23 || d.month < 1 || d.month > 12 || d.day < 1 || wday < 1) return d;
        const long long days = ubersdr_ntp::daysFromCivil(2000 + d.year2, static_cast<unsigned>(d.month),
                                                          static_cast<unsigned>(d.day));
        // A day that does not exist, or a weekday that does not match the date,
        // is a misread that happened to pass parity.
        int cy = 0; unsigned cm = 0, cd = 0;
        ubersdr_ntp::civilFromDays(days, cy, cm, cd);
        if (static_cast<int>(cm) != d.month || static_cast<int>(cd) != d.day) return d;
        if (static_cast<int>(ubersdr_ntp::floorMod(days + 3, 7)) + 1 != wday) return d;

        d.cest = at(17) == 1;
        d.leapWarn = at(19) == 1;
        // The code names the minute starting at the NEXT minute mark, in local
        // time. This frame's second 0 is one minute before that, and UTC is an
        // hour (CET) or two (CEST) behind it.
        const long long nextLocalMin = days * 1440LL + d.hour * 60LL + d.minute;
        const long long s0UtcMin = nextLocalMin - (d.cest ? 120 : 60) - 1;
        d.utcMs = s0UtcMin * 60000LL;
        d.utc = ubersdr_ntp::hostNowFields(d.utcMs);
        d.ok = true;
        return d;
    }

    // The minute's time code is complete at s58. Whether it is the minute a
    // leap second closes decides what s59 and s60 should look like.
    bool tentativeLeapMinute() const {
        Decoded d = decode(Src::Pm);
        if (!d.ok) d = decode(Src::Am);
        return d.ok && d.leapWarn && leapSecondPossible(d.utc);
    }

    static bool leapSecondPossible(const TimeFields& f) {
        if (f.minute != 59 || f.hour != 23) return false;
        return ubersdr_ntp::isLastDayOfMonth(ubersdr_ntp::utcMsFromFields(f.year2, f.doy, f.hour, f.minute));
    }

    static bool sameMinute(const Decoded& a, const Decoded& b) {
        return a.utcMs == b.utcMs && a.leapWarn == b.leapWarn;
    }

    // Which way round the PM reads, from the sixteen bits every minute carries
    // regardless of the time. Needed when AM found the minute first: then PM's
    // own sync never ran, and the sign convention is not something to assume --
    // a KiwiSDR recording reads the opposite way to PTB's "+15.6 deg is a 0
    // chip" taken literally, and whether a receiver's I/Q is conjugated is its
    // own business.
    void learnPolarity() {
        if (polarityKnown) return;
        for (int pol : {1, -1}) {
            int match = 0, miss = 0;
            for (int s = 0; s < 60; ++s) {
                const SecRec& r = frame[static_cast<std::size_t>(s)];
                const int fixed = pmFixedBit(s);
                if (fixed < 0 || r.pmSign == 0 || r.pmConf < kPmSyncConf) continue;
                ((r.pmSign * pol > 0 ? 0 : 1) == fixed ? match : miss)++;
            }
            if (match >= 14 && miss == 0) { polarity = pol; polarityKnown = true; return; }
        }
    }

    void finalizeFrame() {
        learnPolarity();

        // ---- is the minute where we think it is? -------------------------
        int pmMatch = 0, pmMismatch = 0, amBad = 0;
        for (int s = 0; s < 60; ++s) {
            const SecRec& r = frame[static_cast<std::size_t>(s)];
            const int fixed = pmFixedBit(s);
            if (polarityKnown && fixed >= 0 && r.pmSign != 0) {
                if (pmBitOf(r) == fixed) { if (r.pmConf >= kPmSyncConf) ++pmMatch; }
                else if (r.pmConf >= kPmStructConf) ++pmMismatch;
            }
            if (r.amConf >= kStructConf && r.am != ClockSymbol::Unknown) {
                const bool expectMark = s == 59 && !leapMinute;
                if ((r.am == ClockSymbol::Marker) != expectMark) ++amBad;
            }
        }
        const SecRec& s59 = frame[59];
        const bool amMarkSeen = s59.amConf >= kStructConf &&
                                (leapMinute ? s59.am == ClockSymbol::Zero : s59.am == ClockSymbol::Marker);
        const bool pmStrong = pmMatch >= 12 && pmMismatch == 0;
        const bool amStrong = amBad == 0 && amMarkSeen;
        const bool framed = frFilled >= 60 &&
                            (pmStrong || amStrong) &&
                            !(amBad > 0 && !pmStrong) &&
                            !(pmMismatch > 0 && !amStrong);
        if (!framed) {
            dropAnchor();
            demote();
            return;
        }

        // ---- the time, twice ---------------------------------------------
        const Decoded pm = decode(Src::Pm);
        const Decoded am = decode(Src::Am);
        prevFrameFrom = lastFrameFrom;
        const Decoded* use = nullptr;
        bool both = false;
        if (pm.ok && am.ok) {
            if (sameMinute(pm, am)) { use = &pm; both = true; lastFrameFrom = 3; }
            else lastFrameFrom = 4;       // two valid decodes that disagree
        } else if (pm.ok) { use = &pm; lastFrameFrom = 2; }
        else if (am.ok) { use = &am; lastFrameFrom = 1; }
        else lastFrameFrom = 0;

        ClockFrameInfo fi;
        fi.frameStartSample = frStart;
        fi.station = ClockStation::Dcf77;
        std::array<ClockSymbol, 60> syms;
        std::array<float, 60> confs;
        syms.fill(ClockSymbol::Unknown);
        confs.fill(0.0f);
        for (int m : kVoterMarkers) syms[static_cast<std::size_t>(m)] = ClockSymbol::Marker;

        if (use) {
            // Confidence of a group of bits: its weakest, from whichever
            // demodulator the minute was taken from -- the stronger of the two
            // when both agreed.
            auto groupConf = [&](int a, int z) {
                float c = 1.0f;
                for (int s = a; s <= z; ++s) {
                    const SecRec& r = frame[static_cast<std::size_t>(s)];
                    int bit = 0; float cp = 0.0f, ca = 0.0f;
                    const bool hp = bitOf(r, Src::Pm, bit, cp);
                    const bool ha = bitOf(r, Src::Am, bit, ca);
                    const float v = both ? std::max(hp ? cp : 0.0f, ha ? ca : 0.0f)
                                  : (use == &pm ? cp : ca);
                    c = std::min(c, v);
                }
                return c;
            };
            const float cMin = groupConf(21, 28);
            const float cHour = groupConf(29, 35);
            const float cDate = groupConf(36, 58);
            const float cZone = groupConf(17, 18);
            // UTC's minute moves with the minute field alone (zones are whole
            // hours); its hour with the zone and a borrow from the minute; the
            // date with everything.
            const float cUtcMin = cMin;
            const float cUtcHour = std::min({cMin, cHour, cZone});
            const float cUtcDate = std::min({cMin, cHour, cZone, cDate});
            auto encode = [&](int v, const ClockFieldMap& map, float c) {
                for (const auto& bw : map) {
                    const bool one = v >= bw.weight;
                    if (one) v -= bw.weight;
                    syms[static_cast<std::size_t>(bw.second)] = one ? ClockSymbol::One : ClockSymbol::Zero;
                    confs[static_cast<std::size_t>(bw.second)] = c;
                }
            };
            encode(use->utc.minute, kVMin, cUtcMin);
            encode(use->utc.hour, kVHour, cUtcHour);
            encode(use->utc.doy, kVDoy, cUtcDate);
            encode(use->utc.year2, kVYear, cUtcDate);

            fi.minute = use->utc.minute;
            fi.hour = use->utc.hour;
            fi.doy = use->utc.doy;
            fi.year2 = use->utc.year2;
            fi.leapPending = use->leapWarn;
            fi.dst1 = fi.dst2 = use->cest;
            fi.frameConfidence = cUtcDate;
        }
        if (owner->onFrame) owner->onFrame(fi);

        const double P = period();
        const double gap = (leapInserted ? 61.0 : 60.0) * P;
        leapInserted = false;
        const bool consecutive = haveLastFrame &&
                                 std::fabs(static_cast<double>(frStart - lastFrameStart) - gap) <= 0.25 * sr;
        if (!consecutive) voter.reset();
        haveLastFrame = true;
        lastFrameStart = frStart;
        voter.addFrame(syms, confs);

        const bool certified = voter.locked();
        if (certified) {
            votedMinute = voter.votedField(TimeFrameVoter::FieldMinutes);
            votedHour = voter.votedField(TimeFrameVoter::FieldHours);
            votedDoy = voter.votedField(TimeFrameVoter::FieldDoy);
            votedYear = voter.votedField(TimeFrameVoter::FieldYear);
            votedQuality = voter.lockConfidence();
        }

        // The leap second, if this minute ends with one, is the second after
        // s59, and every second from it would be dated one early by counting
        // whole seconds from this frame. Stop certifying before it goes out;
        // the next minute re-locks. As WwvbDecoder.
        const bool leapNext = leapMinute || (use && (use->leapWarn || lastLeapWarn) && leapSecondPossible(use->utc));
        lastLeapWarn = use && use->leapWarn;

        if (certified && !leapNext) {
            haveVoted = true;
            setLockState(ClockLockState::Locked);
        } else {
            demote();
        }
    }

    // ---- callbacks & state -----------------------------------------------

    void emitSecond(const SecRec& r, bool measured, int sof, const std::array<float, kEnvRateHz>& w) {
        if (!owner->onSecond) return;
        ClockSecondInfo si;
        si.edgeSample = r.edge;
        si.edgeMeasured = measured;
        si.symbol = r.am;
        si.confidence = r.amConf;
        si.secondOfFrame = sof;
        si.seriesRateHz = kEnvRateHz;
        si.envelope.resize(kEnvRateHz);
        const float norm = pHi > 1e-12f ? pHi : 1.0f;
        for (int k = 0; k < kEnvRateHz; ++k) si.envelope[k] = std::clamp(w[k] / norm, 0.0f, 1.5f);
        const int low = r.am == ClockSymbol::Zero ? 10 : r.am == ClockSymbol::One ? 20 : 0;
        si.expected.assign(kEnvRateHz, 0.0f);
        const float mean = static_cast<float>(kEnvRateHz - low) / kEnvRateHz;
        for (int k = 0; k < kEnvRateHz; ++k) si.expected[k] = (k < low ? 0.0f : 1.0f) - mean;
        owner->onSecond(si);
    }

    void setLockState(ClockLockState s) {
        if (s != lockState) {
            lockState = s;
            if (owner->onStateChanged) owner->onStateChanged(s);
        }
    }

    // ---- members ---------------------------------------------------------

    Dcf77Decoder* owner;
    int sr;
    int decim;
    double fNominal;
    std::array<int, kChips> chips;
    struct Coeff { int k; int w; };
    std::vector<Coeff> coeffs;
    TimeFrameVoter voter;

    ClockLockState lockState = ClockLockState::NoSignal;
    enum class Phase { Searching, Running } phase = Phase::Searching;
    int64_t samplesConsumed = 0;

    // carrier search
    struct Bin { double f = 0, stepRe = 1, stepIm = 0, rotRe = 1, rotIm = 0, accRe = 0, accIm = 0; };
    std::vector<Bin> bins;
    int64_t acqCount = 0, acqTarget = 0;
    float lastToneSnrDb = 0.0f;

    // mixer
    double f0 = 0.0;
    double oscRe = 1.0, oscIm = 0.0, stepRe = 1.0, stepIm = 0.0;
    int oscRenorm = 0;

    // AM envelope
    Biquad lpI, lpQ;
    double lpfDelay = 0.0;
    double magAccum = 0.0;
    int magCount = 0;
    int64_t envBaseSample = 0;
    std::array<float, kEnvCap> env{};
    int64_t envCount = 0;
    std::vector<float> pctScratch;
    float pHi = 0.0f, pLo = 0.0f;
    double lowFrac = 0.15;   // the cut's depth, as a fraction of full carrier

    // PM
    double aRef = 0.0, aDc = 0.0;
    double refRe = 0.0, refIm = 0.0, yDc = 0.0;
    int freqCount = 0;
    double freqPrevRe = 0.0, freqPrevIm = 0.0;
    std::vector<double> pre, pre2;      // prefix sums of y, and of y^2
    bool useHp = false;                 // correlate the high-passed y (hpNeeded)
    std::vector<double> preH, preH2;    // ... of y high-passed, and its square: noise only
    int64_t hpM = 0;                    // half-width of the high-pass's mean
    int64_t mask = 0;
    int64_t steadyStart = 0;
    bool pmLocked = false;
    int pmMiss = 0;
    std::vector<std::vector<float>> acqHist;   // C^2/sigma^2 by lag, last few searches
    int acqPint = 0, acqHistN = 0, acqHistNext = 0;
    int64_t acqBase = 0;
    int64_t lastSearchEnd = 0;
    float lastPmSnr = std::numeric_limits<float>::quiet_NaN();
    int polarity = 1;          // +1: a positive correlation is a 0 bit
    bool polarityKnown = false;

    // segmentation
    bool segValid = false;
    double segEdge = 0.0;
    int64_t scanPos = 0;
    int blindSeconds = 0;
    EdgeTracker amTrk, pmTrk;
    bool timingFromPm = false;
    double amMinusPm = 0.0;
    bool amMinusPmHave = false;

    // seconds & frames
    std::vector<SecRec> hist;
    bool anchored = false;
    int sofNext = 0;
    std::array<SecRec, 60> frame{};
    int frFilled = 0;
    int64_t frStart = 0;
    bool leapMinute = false;      // this minute ends with a leap second
    bool leapInserted = false;    // the minute just finished ran 61 s
    bool haveLastFrame = false;
    int64_t lastFrameStart = 0;
    bool lastLeapWarn = false;
    std::uint8_t lastFrameFrom = 0;
    std::uint8_t prevFrameFrom = 0;
    int pmRefusedLocks = 0;       // PM locks refused for contradicting a decoding AM

    bool haveVoted = false;
    int votedMinute = -1, votedHour = -1, votedDoy = -1, votedYear = -1;
    float votedQuality = 0.0f;
};

// ---------------------------------------------------------------------------

Dcf77Decoder::Dcf77Decoder(int sampleRateHz, double carrierOffsetHz)
    : m_impl(std::make_unique<Impl>(this, sampleRateHz, carrierOffsetHz)) {}

Dcf77Decoder::~Dcf77Decoder() = default;

void Dcf77Decoder::process(const float* iq, std::size_t frames) { m_impl->process(iq, frames); }
void Dcf77Decoder::reset() { m_impl->reset(); }

void Dcf77Decoder::setPlausibility(std::function<TimeFields()> referenceNow, int boundMinutes) {
    m_impl->voter.setPlausibility(std::move(referenceNow), boundMinutes);
}

ClockLockState Dcf77Decoder::state() const { return m_impl->lockState; }

ClockStation Dcf77Decoder::station() const {
    return m_impl->lockState == ClockLockState::NoSignal ? ClockStation::Unknown : ClockStation::Dcf77;
}

std::int64_t Dcf77Decoder::samplesConsumed() const { return m_impl->samplesConsumed; }

ClockDecoderDiagnostics Dcf77Decoder::diagnostics() const {
    const Impl& d = *m_impl;
    ClockDecoderDiagnostics g;
    g.toneSnrDb = d.lastToneSnrDb;
    g.pwmContrast = d.pLo > 1e-12f ? d.pHi / d.pLo : 0.0f;
    g.toneDetected = d.phase == Impl::Phase::Running;
    g.phaseLocked = d.segValid;
    g.delayEstMs = std::numeric_limits<float>::quiet_NaN();
    g.anchored = d.anchored;
    g.framesInWindow = d.voter.frameCount();
    g.windowSize = d.voter.windowSize();
    g.voteQuality = d.voter.lockConfidence();
    g.refusalReason = static_cast<std::uint8_t>(d.voter.lockRefusal());
    g.pmLocked = d.pmLocked;
    g.pmSnrDb = d.lastPmSnr;
    g.timingFromPm = d.timingFromPm;
    g.amMinusPmMs = d.amMinusPmHave ? static_cast<float>(d.amMinusPm)
                                    : std::numeric_limits<float>::quiet_NaN();
    g.carrierOffsetHz = d.phase == Impl::Phase::Running ? static_cast<float>(d.f0)
                                                        : std::numeric_limits<float>::quiet_NaN();
    g.lastFrameFrom = d.lastFrameFrom;
    g.pmRefusedLocks = d.pmRefusedLocks;
    g.pmInterference = d.useHp;
    return g;
}

} // namespace clockdec
