// MsfDecoder.cpp — see MsfDecoder.h for the format and the design.
//
// Chain, per input sample of complex baseband:
//
//   carrier search (once)  complex DFT bins across +/-20 Hz of the expected
//                          offset; the peak, refined, is where the mixer goes.
//   mixer                  z = x * e^{-j w0 n}.
//   envelope               z -> 150 Hz biquad on I/Q -> |.| -> 10 ms boxcar ->
//                          100 Hz envelope; second edges from the carrier going
//                          off, exactly as Dcf77Decoder finds DCF77's AM cut.
//
// Then once per second, 1.24 s after it began (when its envelope is complete):
// find the edge, read the five tenths 0-500 ms, and sync. A minute is decoded
// at the second 00 that ends it, reading its time code back from its end.
//
// The envelope and edge machinery is Dcf77Decoder's, copied rather than shared
// so that neither decoder's tuning can move the other's.

#include "MsfDecoder.h"

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

// ---- envelope ------------------------------------------------------------
constexpr int    kEnvRateHz   = 100;
constexpr int    kEnvCap      = 1024;   // > 10 s
constexpr int    kPctWin      = 300;    // 3 s of percentiles
constexpr int    kEdgeTol     = 12;     // +/- env samples searched for an edge
constexpr int    kEdgeLook    = 12;     // lookahead an edge candidate needs
constexpr int    kWarmEnv     = 200;
// Envelope samples of carrier an edge must follow: NPL's "at least 500 ms",
// less a margin (isFallingEdge).
constexpr int    kCarrierBefore = 45;
// The full-rate edge (refineEdge): the derivative-of-Gaussian's sigma, and how
// far either side of the envelope's edge the steepest point is looked for.
constexpr double kSlopeSigmaSec      = 0.0005;   // the timing kernel
constexpr double kSlopeFineSearchSec = 0.002;
constexpr double kStepSec            = 0.040;    // the step correlator's half-width, first
// Wide: on a weak signal the envelope's edge is often 10 ms out (its crossing
// lands a block or two away in noise), and nothing else falls within 20 ms of a
// second's edge -- A's and B's are 100 ms and more away, and a rise scores
// negative.
constexpr double kSlopeSearchSec     = 0.020;
// Once the tracker has settled, the fine kernel alone, this far either side of
// its prediction; and a measurement further out than kTrkOutlierSec is an
// outlier.
constexpr double kSlopeTrackSec      = 0.0015;
constexpr double kTrkOutlierSec      = 0.002;
// A settled tracker the wide search puts, by the median of its last
// kReseedWindow offsets, more than kReseedSec from the edge is re-seeded.
constexpr double kReseedSec          = 0.001;
constexpr int    kReseedWindow       = 15;
// ...counting only wide searches whose peak stands this far above the noise
// (steepestAt's sig): on a weak signal most of them are noise peaks, and the
// median of those can fall anywhere.
constexpr double kReseedSig          = 4.0;
// Envelope samples of the second read before it is processed: 0-500 ms.
constexpr int    kClassifyEnv        = 50;
// p90 over p05. MSF's carrier goes fully off, so on a clean signal this is
// large; under 2 there is no keying to be found.
constexpr double kMinContrast = 2.0;
constexpr double kLpfCutHz    = 150.0;

// ---- carrier search ------------------------------------------------------
// As Dcf77Decoder: MSF's carrier is held to 2e-12, so only the receiver's own
// clock moves it in the baseband -- 20 ppm is 1.2 Hz at 60 kHz.
constexpr double kAcqSeconds  = 2.0;
constexpr double kSearchHz    = 20.0;
constexpr double kPullHz      = 3.0;
constexpr double kSearchStep  = 0.25;
constexpr double kToneGate    = 12.0;   // peak over median bin power
constexpr double kLiveToneSeconds = 2.0;
constexpr int    kLiveToneSide    = 16;
constexpr double kLiveToneFirstHz = 5.5;

// ---- symbols & sync ------------------------------------------------------
// Confidence is a z-score mapped (z - 1)/5, as Dcf77Decoder's AM: 0.5 is
// 3.5 sigma.
constexpr float kStructConf  = 0.50f;   // enough to call a structural fault
constexpr float kSyncConf    = 0.60f;   // a minute marker to anchor on
// The identifier's eight A bits are judged together: each must be read, and
// their mean at this level.
constexpr float kIdBitMinConf = 0.10f;
constexpr float kIdMeanConf   = 0.40f;
// Below this a bit is not read at all: the minute fails as incomplete rather
// than going to the parity checks to pass or fail by chance.
constexpr float kBitMinConf  = 0.05f;

// Edge tracker (EdgeTracker).
constexpr double kTrkAlpha = 1.0 / 8.0;
constexpr int    kTrkWarm  = 8;
constexpr double kTrkAlphaMin    = 1.0 / 32.0;
constexpr double kTrkNoiseRefSec = 0.0001;
// The carrier's residual frequency: the fraction of each second's measured
// phase error taken (trackCarrier).
constexpr double kFreqGain = 0.5;

constexpr int kMaxBlindSeconds = 90;
constexpr int kMaxUnconfirmedMinutes = 2;

// The longest minute there is, 61 s, plus one second to see its end.
constexpr int kFrameCap = 62;

// WWVB's layout, for the synthetic frames the voter is fed (finalizeFrame).
constexpr std::array<int, 7> kVoterMarkers = {0, 9, 19, 29, 39, 49, 59};
const ClockFieldMap kVMin  = {{1, 40}, {2, 20}, {3, 10}, {5, 8}, {6, 4}, {7, 2}, {8, 1}};
const ClockFieldMap kVHour = {{12, 20}, {13, 10}, {15, 8}, {16, 4}, {17, 2}, {18, 1}};
const ClockFieldMap kVDoy  = {{22, 200}, {23, 100}, {25, 80}, {26, 40}, {27, 20},
                              {28, 10}, {30, 8}, {31, 4}, {32, 2}, {33, 1}};
const ClockFieldMap kVYear = {{45, 80}, {46, 40}, {47, 20}, {48, 10},
                              {50, 8}, {51, 4}, {52, 2}, {53, 1}};

// A 52..59: the minute identifier.
constexpr std::array<int, 8> kIdentifier = {0, 1, 1, 1, 1, 1, 1, 0};

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

// Alpha-beta tracker on second edges, in fractional input samples, as
// Dcf77Decoder's -- except that its gain follows its own measurement noise.
// On a strong signal an edge is good to ~12 us and the usual 1/8 follows the
// air closely; at 30 dB-Hz one is good to half a millisecond, and 1/8 passes
// too much of that on. So alpha falls from kTrkAlpha as the residuals' rms
// rises past kTrkNoiseRefSec, to kTrkAlphaMin, and beta is alpha^2 / 2 (the
// 1/8, 1/128 pair). The receiver's sample clock moves over minutes, so even
// the slowest of these follows it.
struct EdgeTracker {
    bool valid = false;
    double edge = 0.0, period = 0.0;
    int count = 0, outliers = 0;
    double resVar = 0.0;         // residuals' variance, samples^2
    double noiseRef = 1.0;       // kTrkNoiseRefSec in samples

    void reset() { valid = false; count = 0; outliers = 0; resVar = 0.0; }

    double alpha() const {
        const double rms = std::sqrt(resVar);
        if (!(rms > noiseRef)) return kTrkAlpha;
        return std::max(kTrkAlphaMin, kTrkAlpha * noiseRef / rms);
    }

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
            if (++outliers >= 3) { edge = raw; period = nominal; count = 1; outliers = 0; resVar = 0.0; }
            else edge = pred;
            return;
        }
        outliers = 0;
        ++count;
        resVar += (count <= 16 ? 1.0 / count : 1.0 / 16.0) * (r * r - resVar);
        const double a = alpha();
        edge = pred + std::max(1.0 / count, a) * r;
        if (count > kTrkWarm) {
            period = std::clamp(period + 0.5 * a * a * r, nominal * (1.0 - 2e-4), nominal * (1.0 + 2e-4));
        }
    }
};

// One classified second.
struct SecRec {
    int64_t edge = 0;
    double edgeExact = std::numeric_limits<double>::quiet_NaN();
    bool read = false;           // the second was classified at all
    bool marker = false;         // 500 ms off: second 00
    float markConf = 0.0f;       // confidence of marker-or-not
    int a = 0, b = 0;            // bits A and B (1 = carrier off)
    float aConf = 0.0f, bConf = 0.0f;
    bool cut = true;             // the carrier went off at the start at all
};

bool confMarker(const SecRec& r, float c) { return r.read && r.marker && r.markConf >= c; }
bool confNotMarker(const SecRec& r, float c) { return r.read && !r.marker && r.markConf >= c; }

// One minute's time code.
struct Decoded {
    bool ok = false;
    int minute = 0, hour = 0, day = 0, month = 0, year2 = 0, wday = 0;
    bool summer = false, summerSoon = false;
    int dut1Tenths = 0;
    TimeFields utc;          // UTC of THIS frame's second 00
    long long utcMs = 0;
};

} // namespace

// ---------------------------------------------------------------------------

struct MsfDecoder::Impl {
    Impl(MsfDecoder* o, int sampleRateHz, double carrierOffsetHz)
        : owner(o),
          sr(sampleRateHz > 0 ? sampleRateHz : 12000),
          decim(std::max(1, static_cast<int>(std::lround(sr / 100.0)))),
          fNominal(carrierOffsetHz),
          voter(buildVoterConfig()) {
        acqTarget = static_cast<int64_t>(std::llround(kAcqSeconds * sr));
        auto bin = [&](double f) {
            Bin b;
            b.f = f;
            b.stepRe = std::cos(-2.0 * kPi * f / sr);
            b.stepIm = std::sin(-2.0 * kPi * f / sr);
            return b;
        };
        for (double f = fNominal - kSearchHz; f <= fNominal + kSearchHz + 1e-9; f += kSearchStep) bins.push_back(bin(f));
        liveBins.push_back(bin(0.0));
        for (int i = 0; i < kLiveToneSide; ++i) {
            liveBins.push_back(bin(kLiveToneFirstHz + i));
            liveBins.push_back(bin(-(kLiveToneFirstHz + i)));
        }
        liveTarget = static_cast<int64_t>(std::llround(kLiveToneSeconds * sr));
        std::size_t cap = 1;
        while (cap < static_cast<std::size_t>(3 * sr)) cap <<= 1;
        zRe.assign(cap, 0.0f);
        zIm.assign(cap, 0.0f);
        zMask = static_cast<int64_t>(cap) - 1;
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
        c.minBitConfidence = 0.05f;
        c.minLockQuality = 0.05f;
        return c;
    }

    // ---- driving ---------------------------------------------------------

    void process(const float* iq, std::size_t n) {
        for (std::size_t i = 0; i < n; ++i) {
            const double xr = iq[2 * i], xi = iq[2 * i + 1];
            ++samplesConsumed;
            if (phase == Phase::Searching) feedAcquisition(xr, xi);
            else feedSteady(xr, xi);
        }
    }

    void reset() {
        phase = Phase::Searching;
        acqCount = 0;
        lastToneSnrDb = 0.0f;
        for (Bin& b : bins) { b.accRe = b.accIm = 0.0; b.rotRe = 1.0; b.rotIm = 0.0; }
        for (Bin& b : liveBins) { b.accRe = b.accIm = 0.0; b.rotRe = 1.0; b.rotIm = 0.0; }
        liveCount = 0;
        f0 = fNominal;
        oscRe = 1.0; oscIm = 0.0; oscRenorm = 0;
        lpI.reset(); lpQ.reset();
        magAccum = 0.0; magCount = 0;
        env.fill(0.0f); envCount = 0; envBaseSample = 0;
        pHi = pLo = 0.0f;
        lowFrac = 0.05;
        fResidual = 0.0; havePrevTheta = false; prevTheta = prevThetaAt = 0.0;
        segValid = false; segEdge = 0.0; scanPos = 0; blindSeconds = 0;
        trk.reset();
        wideOff.clear();
        hist.clear();
        dropAnchor();
        haveLastFrame = false; lastFrameStart = 0; lastFrameLen = 60;
        lastFrameFrom = 0;
        samplesConsumed = 0;
        setLockState(ClockLockState::NoSignal);
    }

    // ---- carrier search (as Dcf77Decoder) --------------------------------

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
            double f = bins[peak].f;
            if (peak > 0 && peak + 1 < bins.size()) {
                const double a = std::sqrt(pw[peak - 1]), b = std::sqrt(pw[peak]), c = std::sqrt(pw[peak + 1]);
                const double den = a - 2.0 * b + c;
                if (den < 0.0) f += kSearchStep * std::clamp(0.5 * (a - c) / den, -0.5, 0.5);
            }
            f0 = std::clamp(f, fNominal - kPullHz, fNominal + kPullHz);
            startSteady();
        } else {
            for (Bin& b : bins) { b.accRe = b.accIm = 0.0; b.rotRe = 1.0; b.rotIm = 0.0; }
            acqCount = 0;
        }
    }

    void startSteady() {
        phase = Phase::Running;
        envBaseSample = samplesConsumed;
        stepRe = std::cos(-2.0 * kPi * f0 / sr);
        stepIm = std::sin(-2.0 * kPi * f0 / sr);
        oscRe = 1.0; oscIm = 0.0; oscRenorm = 0;
        lpI = Biquad::lowpass(sr, kLpfCutHz);
        lpQ = Biquad::lowpass(sr, kLpfCutHz);
        // The low-pass's DC group delay, as WwvbDecoder derives it.
        lpfDelay = 1.0 - (lpI.a1 + 2.0 * lpI.a2) / (1.0 + lpI.a1 + lpI.a2);
        zStart = samplesConsumed;
        trk.noiseRef = kTrkNoiseRefSec * sr;
        setLockState(ClockLockState::Acquiring);
    }

    void liveToneStep(double zr, double zi) {
        for (Bin& b : liveBins) {
            b.accRe += zr * b.rotRe - zi * b.rotIm;
            b.accIm += zr * b.rotIm + zi * b.rotRe;
            const double nr = b.rotRe * b.stepRe - b.rotIm * b.stepIm;
            b.rotIm = b.rotRe * b.stepIm + b.rotIm * b.stepRe;
            b.rotRe = nr;
        }
        if ((liveCount & 1023) == 1023) {
            for (Bin& b : liveBins) {
                const double m = std::hypot(b.rotRe, b.rotIm);
                if (m > 1e-9) { b.rotRe /= m; b.rotIm /= m; }
            }
        }
        if (++liveCount < liveTarget) return;
        std::array<double, 2 * kLiveToneSide> nb{};
        for (std::size_t i = 1; i < liveBins.size(); ++i)
            nb[i - 1] = liveBins[i].accRe * liveBins[i].accRe + liveBins[i].accIm * liveBins[i].accIm;
        std::nth_element(nb.begin(), nb.begin() + kLiveToneSide, nb.end());
        const double median = nb[kLiveToneSide];
        const double carrier = liveBins[0].accRe * liveBins[0].accRe + liveBins[0].accIm * liveBins[0].accIm;
        if (median > 0.0 && carrier > 0.0) lastToneSnrDb = static_cast<float>(10.0 * std::log10(carrier / median));
        for (Bin& b : liveBins) { b.accRe = b.accIm = 0.0; b.rotRe = 1.0; b.rotIm = 0.0; }
        liveCount = 0;
    }

    // ---- per sample ------------------------------------------------------

    void feedSteady(double xr, double xi) {
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
        liveToneStep(zr, zi);
        {
            const std::size_t at = static_cast<std::size_t>((samplesConsumed - 1) & zMask);
            zRe[at] = static_cast<float>(zr);
            zIm[at] = static_cast<float>(zi);
        }

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
        if (!segValid) trySeed();
        while (segValid && canProcessSecond()) processSecond();
    }

    // ---- envelope helpers (as Dcf77Decoder) ------------------------------

    float envAt(int64_t idx) const { return env[static_cast<std::size_t>(idx % kEnvCap)]; }

    double envPos(double sample) const {
        return (sample - static_cast<double>(envBaseSample) + lpfDelay) / decim;
    }

    void updatePercentiles() {
        const int64_t n = std::min<int64_t>(kPctWin, envCount);
        if (n < 8) { pHi = pLo = 0.0f; return; }
        pctScratch.clear();
        for (int64_t i = envCount - n; i < envCount; ++i) pctScratch.push_back(envAt(i));
        // p05 lands in the carrier-off: 10-50% of every second is off.
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

    // A second's edge, as NPL defines it: the carrier going off after at least
    // 500 ms of carrier. That is what tells it from the other falling edge MSF
    // makes -- at 200 ms, when A is 0 and B is 1, after only 100 ms of carrier.
    // Taken as 450 ms, most of it above the midpoint, so that noise on a weak
    // signal does not refuse a real one. Seeded on the 200 ms edge without this,
    // the tracker coasted 200 ms off for good: the real edge was outside the
    // window it searched.
    bool isFallingEdge(int64_t i, float thrHi, float thrMid, float thrLo) const {
        if (!(envAt(i - 1) >= thrMid && envAt(i) < thrMid)) return false;
        bool high = false;
        for (int d = 1; d <= 3 && !high; ++d) high = envAt(i - d) >= thrHi;
        if (!high || !sustainedLow(i, thrLo)) return false;
        if (i - kCarrierBefore < envCount - kEnvCap) return false;
        int on = 0;
        for (int d = 5; d < kCarrierBefore; ++d) on += envAt(i - d) >= thrMid ? 1 : 0;
        return on >= (kCarrierBefore - 5) * 8 / 10;
    }

    // Sub-block position of the crossing at env[i], as an input sample, by the
    // area method (WwvbDecoder::edgeSampleAt).
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

    void trySeed() {
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
        edgeOut = refineEdge(edgeSampleAt(bestEdge));
        return std::isfinite(edgeOut);
    }

    // ---- the edge at full rate --------------------------------------------
    //
    // The envelope finds the edge to a millisecond or so; this times it to the
    // noise. On the carrier's own amplitude, coherently: z projected on the
    // carrier's phase, taken from the 35 ms of carrier just before the edge.
    // That amplitude is linear in the signal -- no magnitude, so no fold of the
    // noise into a bias -- and unfiltered, so there is no delay to take off.
    //
    // The edge is where that amplitude falls fastest: the peak of its
    // correlation with a derivative-of-Gaussian, refined by a parabola. Not the
    // area under the transition, which is what the envelope uses. MSF's fall on
    // the air is two things: a sharp drop to about 15% over half a millisecond,
    // then a tail ringing down to nothing over the next 8 ms or so (the
    // antenna's stored energy). An area puts the edge at the centroid of both,
    // wherever the window cuts the tail -- measured on M9PSY-1, 1.03 ms after
    // the UTC second with a +/-4 ms window and 0.81 with +/-1 ms. The steepest
    // point is the drop's middle, and the tail barely moves it.
    //
    // Also tried, and worse: the time where a half-sine-smoothed amplitude
    // crosses half the carrier (first order in the noise, so in principle
    // better) -- 16 us scatter on the air at best, and at 35 dB-Hz, where one
    // sample is below the noise, it crosses early and reads 0.2-1.4 ms early.
    //
    // Kernel sigma kSlopeSigmaSec, from the same recording (69 dB-Hz):
    //   sigma 0.15 ms  second-to-second scatter 131 us
    //         0.25              40
    //         0.40              11
    //         0.60              12
    //         1.00              13, and the mean 17 us later (the tail)
    // The steepest fall within `searchSec` of `center`, for a derivative-of-
    // Gaussian of sigma `sgSec`, on the amplitude projected on the carrier's
    // phase just before `center`. NaN when there is no peak inside.
    //
    // `sig`, when asked for, is how far that peak stands above the noise: the
    // peak over the rms of the same correlation run on the quadrature
    // component, which holds the noise and nothing else. An exact noise
    // figure for this very correlation, whatever the noise's spectrum.
    double steepestAt(double center, double sgSec, double searchSec, double* sig = nullptr) const {
        const double nan = std::numeric_limits<double>::quiet_NaN();
        const int64_t newest = samplesConsumed - 1;
        const int64_t oldest = std::max(zStart, samplesConsumed - static_cast<int64_t>(zRe.size()) + 1);
        const double sg = sgSec * sr;
        const int64_t half = static_cast<int64_t>(std::ceil(4.0 * sg));
        const int64_t search = std::max<int64_t>(2, std::llround(searchSec * sr));
        const int64_t c = std::llround(center);
        const int64_t pre0 = c - search - static_cast<int64_t>(std::llround(0.035 * sr));
        const int64_t pre1 = c - search - half;
        if (pre0 < oldest || c + search + half > newest) return nan;

        double sr0 = 0.0, si0 = 0.0;
        for (int64_t n = pre0; n <= pre1; ++n) {
            const std::size_t k = static_cast<std::size_t>(n & zMask);
            sr0 += zRe[k]; si0 += zIm[k];
        }
        const double m = std::hypot(sr0, si0);
        if (!(m > 0.0)) return nan;
        const double cr = sr0 / m, ci = si0 / m;   // e^{j theta}
        auto amp = [&](int64_t n) {
            const std::size_t k = static_cast<std::size_t>(n & zMask);
            return static_cast<double>(zRe[k]) * cr + static_cast<double>(zIm[k]) * ci;   // Re(z e^{-j theta})
        };
        auto quad = [&](int64_t n) {
            const std::size_t k = static_cast<std::size_t>(n & zMask);
            return static_cast<double>(zIm[k]) * cr - static_cast<double>(zRe[k]) * ci;   // Im(z e^{-j theta})
        };
        // -d/dt of a Gaussian: positive before the centre, so a fall scores high.
        std::vector<double> g(static_cast<std::size_t>(2 * half + 1));
        for (int64_t k = -half; k <= half; ++k)
            g[static_cast<std::size_t>(k + half)] = -static_cast<double>(k) * std::exp(-0.5 * k * k / (sg * sg));
        std::vector<double> y(static_cast<std::size_t>(2 * search + 1));
        int64_t bi = 0;
        double q2 = 0.0;
        for (int64_t j = -search; j <= search; ++j) {
            double v = 0.0, q = 0.0;
            for (int64_t k = -half; k <= half; ++k) {
                v += amp(c + j + k) * g[static_cast<std::size_t>(k + half)];
                if (sig) q += quad(c + j + k) * g[static_cast<std::size_t>(k + half)];
            }
            y[static_cast<std::size_t>(j + search)] = v;
            q2 += q * q;
            if (v > y[static_cast<std::size_t>(bi)]) bi = j + search;
        }
        if (sig) *sig = q2 > 0.0 ? y[static_cast<std::size_t>(bi)] / std::sqrt(q2 / static_cast<double>(y.size())) : 0.0;
        if (!(y[static_cast<std::size_t>(bi)] > 0.0) || bi <= 0 || bi >= 2 * search) return nan;
        const double y0 = y[static_cast<std::size_t>(bi - 1)], y1 = y[static_cast<std::size_t>(bi)],
                     y2 = y[static_cast<std::size_t>(bi + 1)];
        const double den = y0 - 2.0 * y1 + y2;
        const double frac = den < 0.0 ? std::clamp(0.5 * (y0 - y2) / den, -0.5, 0.5) : 0.0;
        return static_cast<double>(c + bi - search) + frac;
    }

    // The edge found, before it is timed: a step correlator -- the mean of
    // kStepSec of amplitude before a candidate instant less the mean of
    // kStepSec after -- over `searchSec` either side of `center`. It uses the
    // whole contrast MSF guarantees about an edge (450 ms of carrier before,
    // 100 ms off after), so it detects where a slope kernel cannot: at 27
    // dB-Hz one second's 2 ms derivative-of-Gaussian peak is about 1.5 sigma
    // above the noise, this about 4.5. NaN when there is no peak inside; `sig`
    // as steepestAt's, from the quadrature component.
    double stepAt(double center, double searchSec, double* sig = nullptr) const {
        const double nan = std::numeric_limits<double>::quiet_NaN();
        const int64_t newest = samplesConsumed - 1;
        const int64_t oldest = std::max(zStart, samplesConsumed - static_cast<int64_t>(zRe.size()) + 1);
        const int64_t W = std::max<int64_t>(4, std::llround(kStepSec * sr));
        const int64_t search = std::max<int64_t>(2, std::llround(searchSec * sr));
        const int64_t c = std::llround(center);
        const int64_t lo = c - search - W;          // first sample the sums read
        const int64_t hi = c + search + W;          // one past the last
        const int64_t pre0 = lo - static_cast<int64_t>(std::llround(0.035 * sr));
        if (pre0 < oldest || hi > newest) return nan;

        double sr0 = 0.0, si0 = 0.0;
        for (int64_t n = pre0; n < lo; ++n) {
            const std::size_t k = static_cast<std::size_t>(n & zMask);
            sr0 += zRe[k]; si0 += zIm[k];
        }
        const double m = std::hypot(sr0, si0);
        if (!(m > 0.0)) return nan;
        const double cr = sr0 / m, ci = si0 / m;
        std::vector<double> P(static_cast<std::size_t>(hi - lo + 1), 0.0), Q(P.size(), 0.0);
        for (int64_t n = lo; n < hi; ++n) {
            const std::size_t k = static_cast<std::size_t>(n & zMask);
            const std::size_t i = static_cast<std::size_t>(n - lo);
            P[i + 1] = P[i] + zRe[k] * cr + zIm[k] * ci;
            Q[i + 1] = Q[i] + zIm[k] * cr - zRe[k] * ci;
        }
        // y(j): the step between samples j-1 and j, i.e. at instant j - 1/2.
        auto stepOf = [&](const std::vector<double>& S, int64_t j) {
            const std::size_t i = static_cast<std::size_t>(j - lo);
            return (S[i] - S[i - static_cast<std::size_t>(W)]) - (S[i + static_cast<std::size_t>(W)] - S[i]);
        };
        std::vector<double> y(static_cast<std::size_t>(2 * search + 1));
        int64_t bi = 0;
        double q2 = 0.0;
        for (int64_t j = -search; j <= search; ++j) {
            const double v = stepOf(P, c + j);
            const double q = stepOf(Q, c + j);
            y[static_cast<std::size_t>(j + search)] = v;
            q2 += q * q;
            if (v > y[static_cast<std::size_t>(bi)]) bi = j + search;
        }
        if (sig) *sig = q2 > 0.0 ? y[static_cast<std::size_t>(bi)] / std::sqrt(q2 / static_cast<double>(y.size())) : 0.0;
        if (!(y[static_cast<std::size_t>(bi)] > 0.0) || bi <= 0 || bi >= 2 * search) return nan;
        const double y0 = y[static_cast<std::size_t>(bi - 1)], y1 = y[static_cast<std::size_t>(bi)],
                     y2 = y[static_cast<std::size_t>(bi + 1)];
        const double den = y0 - 2.0 * y1 + y2;
        const double frac = den < 0.0 ? std::clamp(0.5 * (y0 - y2) / den, -0.5, 0.5) : 0.0;
        return static_cast<double>(c + bi - search) - 0.5 + frac;
    }

    // From the envelope's edge: found with the step correlator, which noise
    // cannot fool across a few milliseconds, then timed with the slope kernel.
    // NaN when the full-rate signal has no clear fall there: the second is
    // then unmeasured and the tracker coasts, rather than being handed the
    // envelope's own edge, which is a millisecond cruder and, through its
    // filter, not free of bias.
    double refineEdge(double coarse, double* sig = nullptr) const {
        const double wide = stepAt(coarse, kSlopeSearchSec, sig);
        if (!std::isfinite(wide)) return wide;
        const double fine = steepestAt(wide, kSlopeSigmaSec, kSlopeFineSearchSec);
        return std::isfinite(fine) ? fine : wide;
    }

    // ---- once per second -------------------------------------------------

    // A second is processed once its first half is in: everything it carries
    // is in 0-500 ms, and the carrier level it is read against is the 450 ms
    // before it. About 0.74 s after its edge, where waiting for the whole
    // second took 1.24.
    bool canProcessSecond() const {
        const int64_t j0 = static_cast<int64_t>(std::floor(envPos(segEdge)));
        return envCount >= j0 + kClassifyEnv + kEdgeTol + kEdgeLook;
    }

    // The five tenths 0-500 ms of the second that starts at `edge`, each as
    // how far the carrier is off, 0 (full) to 1 (as off as it goes), against
    // its level in the 440 ms before the edge -- carrier in every second there
    // is, by NPL's definition of the edge.
    //
    // Read coherently: the baseband projected on the carrier's phase, measured
    // over that same 440 ms and carried forward at the residual frequency
    // (trackCarrier). Not the envelope's magnitude, which at low SNR sits at
    // the noise's own level when the carrier is off -- well above zero -- and
    // so shrinks the very contrast a bit is read from. Measured in msftest:
    // on the magnitude nothing locked at 27 dB-Hz.
    //
    // Confidences are z-scores, the noise taken from the quadrature component
    // over the same 440 ms (which holds noise and nothing else), mapped
    // (z - 1)/5 as Dcf77Decoder's AM. The depth of "off" is learnt from every
    // second's first tenth, which is always off.
    void classify(double edge, SecRec& r, std::array<float, kEnvRateHz>& w) {
        const int64_t j0 = static_cast<int64_t>(std::llround(envPos(edge)));
        // The display window: as much of the second as has arrived.
        for (int k = 0; k < kEnvRateHz; ++k) w[k] = envAt(std::min<int64_t>(j0 + k, envCount - 1));
        r.read = false;
        const int64_t c = std::llround(edge);
        const int64_t pre0 = c - static_cast<int64_t>(std::llround(0.450 * sr));
        const int64_t pre1 = c - static_cast<int64_t>(std::llround(0.010 * sr));
        const int64_t end = c + static_cast<int64_t>(std::llround(0.500 * sr));
        const int64_t oldest = std::max(zStart, samplesConsumed - static_cast<int64_t>(zRe.size()) + 1);
        if (pre0 < oldest || end > samplesConsumed - 1) return;

        double sr0 = 0.0, si0 = 0.0;
        for (int64_t n = pre0; n <= pre1; ++n) {
            const std::size_t k = static_cast<std::size_t>(n & zMask);
            sr0 += zRe[k]; si0 += zIm[k];
        }
        const double mag = std::hypot(sr0, si0);
        if (!(mag > 0.0)) return;
        const double theta = std::atan2(si0, sr0);
        const double mid = 0.5 * static_cast<double>(pre0 + pre1);
        trackCarrier(mid, theta);
        const double w0 = 2.0 * kPi * fResidual / sr;
        auto proj = [&](int64_t n, double& re, double& im) {
            const double ph = theta + w0 * (static_cast<double>(n) - mid);
            const double cr = std::cos(ph), ci = std::sin(ph);
            const std::size_t k = static_cast<std::size_t>(n & zMask);
            re = zRe[k] * cr + zIm[k] * ci;
            im = zIm[k] * cr - zRe[k] * ci;
        };
        double H = 0.0, q2 = 0.0;
        for (int64_t n = pre0; n <= pre1; ++n) {
            double re, im; proj(n, re, im);
            H += re; q2 += im * im;
        }
        const double np = static_cast<double>(pre1 - pre0 + 1);
        H /= np;
        const double sigma = std::sqrt(q2 / np);   // per sample, per component
        if (!(H > 0.0) || !haveContrast()) return;
        // Each tenth read over 20-80 ms of it, clear of the transitions.
        const int64_t a0 = static_cast<int64_t>(std::llround(0.020 * sr));
        const int64_t a1 = static_cast<int64_t>(std::llround(0.080 * sr));
        const double per = static_cast<double>(a1 - a0 + 1);
        auto tenth = [&](int t) {
            const int64_t base = c + static_cast<int64_t>(std::llround(0.1 * t * sr));
            double sum = 0.0;
            for (int64_t n = base + a0; n <= base + a1; ++n) { double re, im; proj(n, re, im); sum += re; }
            return sum / per;
        };
        const double depth = std::max(0.1, 1.0 - lowFrac);
        std::array<double, 5> x{};
        double first = 0.0;
        for (int t = 0; t < 5; ++t) {
            const double v = tenth(t);
            if (t == 0) first = v;
            x[static_cast<std::size_t>(t)] = (1.0 - v / H) / depth;
        }
        const double sx = std::max(1e-4, sigma / std::sqrt(per) / (H * depth));
        auto confOf = [](double z) { return static_cast<float>(std::clamp((z - 1.0) / 5.0, 0.0, 1.0)); };

        r.read = true;
        r.cut = x[0] >= 0.5;
        // Marker or not: the carrier off through 300-500 ms, or on.
        const double m = 0.5 * (x[3] + x[4]);
        r.marker = m >= 0.5;
        r.markConf = confOf(std::fabs(m - 0.5) / (sx / std::sqrt(2.0)));
        r.a = x[1] >= 0.5 ? 1 : 0;
        r.b = x[2] >= 0.5 ? 1 : 0;
        r.aConf = confOf(std::fabs(x[1] - 0.5) / sx);
        r.bConf = confOf(std::fabs(x[2] - 0.5) / sx);
        if (!r.cut) {
            // No carrier-off at the start of a second: nothing here is a symbol.
            r.markConf = 0.0f;
            r.aConf = r.bConf = 0.0f;
        }
        if (r.cut) lowFrac += 0.02 * (std::clamp(first / H, 0.0, 0.9) - lowFrac);
    }

    // The carrier's residual frequency in the mixed baseband, from how far its
    // phase turned between one second's pre-edge carrier and the next: used
    // to carry the phase across the half second each second is read over, and
    // fed back to the mixer. Unambiguous to +/-0.5 Hz, and the carrier search
    // leaves it within a tenth of that.
    void trackCarrier(double at, double theta) {
        if (havePrevTheta && at > prevThetaAt) {
            const double dt = (at - prevThetaAt) / sr;
            double d = theta - prevTheta - 2.0 * kPi * fResidual * dt;
            d = std::remainder(d, 2.0 * kPi);
            if (dt > 0.5 && dt < 1.5) {
                fResidual += kFreqGain * d / (2.0 * kPi * dt);
                fResidual = std::clamp(fResidual, -0.5, 0.5);
            }
        }
        havePrevTheta = true;
        prevTheta = theta;
        prevThetaAt = at;
    }

    void processSecond() {
        const double pred = segEdge;
        double measEdge = 0.0;
        bool meas = false;
        const bool warm = trk.valid && trk.count > kTrkWarm;
        if (warm) {
            // Settled: the fine kernel straight at the prediction. The search
            // that has to find the edge first is where weak signals go wrong,
            // and once the tracker knows the edge to a fraction of a
            // millisecond it has nothing left to find.
            measEdge = steepestAt(pred, kSlopeSigmaSec, kSlopeTrackSec);
            meas = std::isfinite(measEdge) && haveContrast();
            // Unless it settled on the wrong thing. A noise peak taken at
            // acquisition on a weak signal holds a tracker a few milliseconds
            // off for good -- the fine search never looks far enough to see the
            // real edge (measured at 27 dB-Hz: 3.9 ms). So the wide search runs
            // too, and when the MEDIAN of its last kReseedWindow offsets from
            // the tracker is past kReseedSec, the tracker is re-seeded there.
            // The median, because one second's wide search is itself noisier
            // than a millisecond at 35 dB-Hz: judged second by second it
            // re-seeded healthy trackers all the time.
            double wideSig = 0.0;
            const double wideEdge = refineEdge(pred, &wideSig);
            if (std::isfinite(wideEdge) && wideSig >= kReseedSig) {
                wideOff.push_back(wideEdge - pred);
                if (wideOff.size() > static_cast<std::size_t>(kReseedWindow)) wideOff.erase(wideOff.begin());
                if (wideOff.size() == static_cast<std::size_t>(kReseedWindow)) {
                    std::vector<double> m = wideOff;
                    std::nth_element(m.begin(), m.begin() + m.size() / 2, m.end());
                    const double med = m[m.size() / 2];
                    if (std::fabs(med) > kReseedSec * sr) {
                        trk.reset();
                        measEdge = pred + med;
                        meas = true;
                        wideOff.clear();
                    }
                }
            }
        }
        if (!meas) meas = findFallingEdgeNear(std::llround(envPos(pred)), kEdgeTol, measEdge);
        // Outliers: a whole envelope block while acquiring, 2 ms once settled.
        trk.update(meas, measEdge, trk.valid ? trk.period : static_cast<double>(sr),
                   warm ? kTrkOutlierSec * sr : decim);
        const double edge = trk.valid ? trk.edge : pred;
        blindSeconds = meas ? 0 : blindSeconds + 1;

        SecRec r;
        r.edge = static_cast<int64_t>(std::llround(edge));
        r.edgeExact = edge;
        std::array<float, kEnvRateHz> w{};
        classify(edge, r, w);
        hist.push_back(r);
        if (hist.size() > 16) hist.erase(hist.begin());

        handleSecond(r, meas, w);

        segEdge = trk.valid ? trk.edge + trk.period : edge + sr;

        if (blindSeconds >= kMaxBlindSeconds) {
            if (lockState == ClockLockState::Locked) setLockState(ClockLockState::Acquiring);
            segValid = false;
            scanPos = 0;
            blindSeconds = 0;
            trk.reset();
            dropAnchor();
            hist.clear();
        }
    }

    // ---- sync & frames ---------------------------------------------------

    // The identifier 01111110 in bit A, ending at frame[end] (or, with
    // fromHist, at the newest second). Every bit read, none of the eight a
    // confident marker, and their mean confidence at kIdMeanConf.
    template <typename At>
    bool identifierAt(At at) const {
        float sum = 0.0f;
        for (int j = 0; j < 8; ++j) {
            const SecRec& r = at(j);
            if (!r.read || !r.cut || r.aConf < kIdBitMinConf) return false;
            if (confMarker(r, kStructConf)) return false;
            if (r.a != kIdentifier[static_cast<std::size_t>(j)]) return false;
            sum += r.aConf;
        }
        return sum >= 8.0f * kIdMeanConf;
    }
    bool identifierInHist() const {
        if (hist.size() < 8) return false;
        const std::size_t base = hist.size() - 8;
        return identifierAt([&](int j) -> const SecRec& { return hist[base + static_cast<std::size_t>(j)]; });
    }
    // The identifier's last bit at frame index `last`.
    bool identifierEndsAt(int last) const {
        if (last < 7 || last >= frFilled) return false;
        return identifierAt([&](int j) -> const SecRec& { return frame[static_cast<std::size_t>(last - 7 + j)]; });
    }

    void dropAnchor() {
        anchored = false;
        sofNext = 0;
        haveVoted = false;
        for (auto& f : frame) f = SecRec{};
        frFilled = 0;
        frStart = 0;
        unconfirmedRun = 0;
    }

    void demote() {
        haveVoted = false;
        if (lockState == ClockLockState::Locked) setLockState(ClockLockState::Acquiring);
    }

    void startFrame(const SecRec& r) {
        for (auto& f : frame) f = SecRec{};
        frame[0] = r;
        frFilled = 1;
        frStart = r.edge;
        sofNext = 1;
    }

    // After a minute is finalized at `r`: r is second 00 of the next, unless
    // the finalize let the anchor go -- then only a clear marker re-anchors.
    void afterFinalize(const SecRec& r, int& sof) {
        if (!anchored && !confMarker(r, kStructConf)) { sof = -1; return; }
        anchored = true;
        sof = 0;
        startFrame(r);
    }

    void handleSecond(const SecRec& r, bool measured, const std::array<float, kEnvRateHz>& w) {
        int sof = -1;
        if (!anchored) {
            if (confMarker(r, kSyncConf)) {
                anchored = true;
                sof = 0;
                startFrame(r);
            } else if (identifierInHist()) {
                // This second is the last of a minute; the next is second 00.
                anchored = true;
                sof = 59;
                sofNext = 0;
                frFilled = 0;
            }
        } else {
            sof = sofNext;
            if (sof == 0) {
                // The identifier just ended a minute (or anchored us): this is
                // second 00 whether or not its marker reads. The identifier is
                // eight bits of evidence where the minute ends; a marker that
                // failed to key, or faded, is one. The next minute's own
                // identifier confirms or contradicts the count.
                startFrame(r);
            } else if (sof >= 59 && confMarker(r, kStructConf)) {
                // The minute that was running has ended: it had `sof` seconds.
                finalizeFrame(sof, true);
                afterFinalize(r, sof);
            } else if (sof < 59 && confMarker(r, kStructConf)) {
                // A clear marker where none belongs: the count has slipped. This
                // is second 00 of a minute; the one running is abandoned.
                demote();
                haveLastFrame = false;
                unconfirmedRun = 0;
                sof = 0;
                startFrame(r);
            } else if (sof >= 60 && identifierEndsAt(sof - 1)) {
                // The marker faded, but the identifier says the minute ended.
                finalizeFrame(sof, false);
                afterFinalize(r, sof);
            } else if (sof >= kFrameCap - 1) {
                dropAnchor();
                demote();
                sof = -1;
            } else {
                frame[static_cast<std::size_t>(sof)] = r;
                frFilled = sof + 1;
                sofNext = sof + 1;
                // The identifier ends the minute: decode it now, at its last
                // second, rather than a second later at the marker. It ends at
                // s59, or at s60 or s58 when a leap second made the minute 61
                // or 59 s long -- which is how that is known. The marker that
                // follows is then second 00 (sofNext 0 checks it).
                if (sof >= 58 && sof <= 60 && identifierEndsAt(sof)) {
                    finalizeFrame(sof + 1, false);
                    if (anchored) sofNext = 0;
                }
            }
        }

        emitSecond(r, measured, sof, w);

        // A second numbered 60 exists only in a leap minute, or when the count
        // has run past a faded marker; neither is labelled.
        if (lockState == ClockLockState::Locked && haveVoted && sof >= 0 && sof < 60 && owner->onTime) {
            ClockTimeInfo ti;
            ti.minute = votedMinute; ti.hour = votedHour;
            ti.doy = votedDoy; ti.year2 = votedYear;
            ti.quality = votedQuality;
            ti.lastEdgeSample = r.edge;
            ti.lastEdgeSampleExact = r.edgeExact;
            ti.lastEdgeSecondOfFrame = sof;
            ti.station = ClockStation::Msf;
            owner->onTime(ti);
        }
    }

    // ---- the time code ---------------------------------------------------

    // Bit A or B of spec second `p` in a minute of `n` seconds. Fields from 17A
    // and 52B on are counted back from the end; the rest from second 00.
    bool bitAt(int n, int p, bool isA, int& bit, float& conf) const {
        const bool fromEnd = isA ? p >= 17 : p >= 52;
        const int idx = fromEnd ? p + (n - 60) : p;
        if (idx < 1 || idx >= frFilled) return false;
        const SecRec& r = frame[static_cast<std::size_t>(idx)];
        if (!r.read || !r.cut || confMarker(r, kStructConf)) return false;
        bit = isA ? r.a : r.b;
        conf = isA ? r.aConf : r.bConf;
        return conf >= kBitMinConf;
    }

    Decoded decode(int n, float& minConf) const {
        Decoded d;
        minConf = 1.0f;
        std::array<int, 60> A{}, B{};
        for (int p = 17; p <= 51; ++p) {
            int bit = 0; float c = 0.0f;
            if (!bitAt(n, p, true, bit, c)) return d;
            A[static_cast<std::size_t>(p)] = bit;
            minConf = std::min(minConf, c);
        }
        for (int p = 53; p <= 58; ++p) {
            int bit = 0; float c = 0.0f;
            if (!bitAt(n, p, false, bit, c)) return d;
            B[static_cast<std::size_t>(p)] = bit;
            minConf = std::min(minConf, c);
        }
        auto a = [&](int p) { return A[static_cast<std::size_t>(p)]; };
        // Odd parity: the group and its parity bit hold an odd number of ones.
        auto oddOk = [&](int from, int to, int pb) {
            int ones = B[static_cast<std::size_t>(pb)];
            for (int p = from; p <= to; ++p) ones += a(p);
            return (ones & 1) == 1;
        };
        if (!oddOk(17, 24, 54) || !oddOk(25, 35, 55) || !oddOk(36, 38, 56) || !oddOk(39, 51, 57)) return d;

        auto msb = [&](int from, std::initializer_list<int> weights) {
            int v = 0, p = from;
            for (int wgt : weights) v += a(p++) * wgt;
            return v;
        };
        // BCD digits, each checked: a nibble past 9 is a misread.
        auto digitsOk = [&](int from, int tensBits) {
            int u = 0;
            for (int i = 0; i < 4; ++i) u = u * 2 + a(from + tensBits + i);
            return u <= 9;
        };
        if (!digitsOk(17, 4) || !digitsOk(25, 1) || !digitsOk(30, 2) || !digitsOk(39, 2) || !digitsOk(45, 3)) return d;
        d.year2 = msb(17, {80, 40, 20, 10, 8, 4, 2, 1});
        d.month = msb(25, {10, 8, 4, 2, 1});
        d.day = msb(30, {20, 10, 8, 4, 2, 1});
        d.wday = msb(36, {4, 2, 1});
        d.hour = msb(39, {20, 10, 8, 4, 2, 1});
        d.minute = msb(45, {40, 20, 10, 8, 4, 2, 1});
        if (d.minute > 59 || d.hour > 23 || d.month < 1 || d.month > 12 || d.day < 1 || d.wday > 6) return d;
        const long long days = ubersdr_ntp::daysFromCivil(2000 + d.year2, static_cast<unsigned>(d.month),
                                                          static_cast<unsigned>(d.day));
        // A day that does not exist, or a weekday that does not match the date,
        // is a misread that happened to pass parity.
        int cy = 0; unsigned cm = 0, cd = 0;
        ubersdr_ntp::civilFromDays(days, cy, cm, cd);
        if (static_cast<int>(cm) != d.month || static_cast<int>(cd) != d.day) return d;
        if (static_cast<int>(ubersdr_ntp::floorMod(days + 4, 7)) != d.wday) return d;   // 0 = Sunday

        d.summer = B[58] == 1;
        d.summerSoon = B[53] == 1;
        // DUT1: B01-B08 for +, B09-B16 for -; not counted from the end.
        int pos = 0, neg = 0;
        for (int p = 1; p <= 16; ++p) {
            int bit = 0; float c = 0.0f;
            if (bitAt(n, p, false, bit, c) && bit) (p <= 8 ? pos : neg)++;
        }
        d.dut1Tenths = pos > 0 && neg == 0 ? pos : neg > 0 && pos == 0 ? -neg : 0;

        // UK clock time of the minute starting at the NEXT marker. This frame's
        // second 00 is one minute before it; UTC is an hour behind in summer.
        const long long nextLocalMin = days * 1440LL + d.hour * 60LL + d.minute;
        const long long s0UtcMin = nextLocalMin - (d.summer ? 60 : 0) - 1;
        d.utcMs = s0UtcMin * 60000LL;
        d.utc = ubersdr_ntp::hostNowFields(d.utcMs);
        d.ok = true;
        return d;
    }

    // The minute in frame[0 .. n-1] has ended. endMarker: its end was a marker
    // that read clearly (rather than the identifier alone).
    void finalizeFrame(int n, bool endMarker) {
        // ---- is the minute where we think it is? -------------------------
        //
        // MSF marks a minute three ways: the marker at its start, the
        // identifier at its end, and the next marker. Confirmed when the
        // identifier is where it belongs, or both markers are; contradicted by
        // a clear marker inside the minute, or the identifier clearly wrong.
        const bool startMarker = confMarker(frame[0], kSyncConf);
        const bool idOk = identifierEndsAt(n - 1);
        int idWrong = 0, strayMarks = 0;
        for (int j = 0; j < 8 && n - 8 + j >= 1; ++j) {
            const SecRec& r = frame[static_cast<std::size_t>(n - 8 + j)];
            if (r.read && r.cut && r.aConf >= kStructConf && r.a != kIdentifier[static_cast<std::size_t>(j)]) ++idWrong;
        }
        for (int s = 1; s < n; ++s) if (confMarker(frame[static_cast<std::size_t>(s)], kStructConf)) ++strayMarks;
        const bool contradicted = strayMarks > 0 || idWrong >= 2;
        const bool confirmed = frFilled >= n && !contradicted && (idOk || (startMarker && endMarker));
        if (contradicted || (!confirmed && ++unconfirmedRun > kMaxUnconfirmedMinutes)) {
            dropAnchor();
            demote();
            haveLastFrame = false;
            return;
        }
        if (confirmed) unconfirmedRun = 0;

        float minConf = 0.0f;
        const Decoded dec = confirmed ? decode(n, minConf) : Decoded{};
        lastFrameFrom = dec.ok ? 1 : 0;

        ClockFrameInfo fi;
        fi.frameStartSample = frStart;
        fi.station = ClockStation::Msf;
        std::array<ClockSymbol, 60> syms;
        std::array<float, 60> confs;
        syms.fill(ClockSymbol::Unknown);
        confs.fill(0.0f);
        for (int m : kVoterMarkers) syms[static_cast<std::size_t>(m)] = ClockSymbol::Marker;
        if (dec.ok) {
            auto encode = [&](int v, const ClockFieldMap& map, float c) {
                for (const auto& bw : map) {
                    const bool one = v >= bw.weight;
                    if (one) v -= bw.weight;
                    syms[static_cast<std::size_t>(bw.second)] = one ? ClockSymbol::One : ClockSymbol::Zero;
                    confs[static_cast<std::size_t>(bw.second)] = c;
                }
            };
            // Every UTC field can move with any of the minute, hour, date and
            // summer-time bits, so each takes the weakest of them.
            encode(dec.utc.minute, kVMin, minConf);
            encode(dec.utc.hour, kVHour, minConf);
            encode(dec.utc.doy, kVDoy, minConf);
            encode(dec.utc.year2, kVYear, minConf);
            fi.minute = dec.utc.minute;
            fi.hour = dec.utc.hour;
            fi.doy = dec.utc.doy;
            fi.year2 = dec.utc.year2;
            fi.dut1Tenths = dec.dut1Tenths;
            fi.dst1 = fi.dst2 = dec.summer;
            fi.frameConfidence = minConf;
        }
        if (owner->onFrame) owner->onFrame(fi);

        const double P = trk.valid ? trk.period : static_cast<double>(sr);
        const bool consecutive = haveLastFrame &&
            std::fabs(static_cast<double>(frStart - lastFrameStart) - lastFrameLen * P) <= 0.25 * sr;
        if (!consecutive) voter.reset();
        haveLastFrame = true;
        lastFrameStart = frStart;
        lastFrameLen = n;
        voter.addFrame(syms, confs);

        const bool certified = voter.locked();
        if (certified) {
            votedMinute = voter.votedField(TimeFrameVoter::FieldMinutes);
            votedHour = voter.votedField(TimeFrameVoter::FieldHours);
            votedDoy = voter.votedField(TimeFrameVoter::FieldDoy);
            votedYear = voter.votedField(TimeFrameVoter::FieldYear);
            votedQuality = voter.lockConfidence();
        }
        // A minute that was not 60 seconds long had a leap second in it, which
        // MSF does not announce. Counting whole seconds from its start would
        // date the next minute one out; the next minute locks afresh.
        if (certified && confirmed && n == 60) {
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
        si.edgeSampleExact = r.edgeExact;
        si.edgeMeasured = measured;
        si.symbol = !r.read ? ClockSymbol::Unknown : r.marker ? ClockSymbol::Marker
                  : r.a ? ClockSymbol::One : ClockSymbol::Zero;
        si.confidence = r.marker ? r.markConf : std::min(r.markConf, r.aConf);
        si.secondOfFrame = sof;
        si.seriesRateHz = kEnvRateHz;
        si.envelope.resize(kEnvRateHz);
        const float norm = pHi > 1e-12f ? pHi : 1.0f;
        for (int k = 0; k < kEnvRateHz; ++k) si.envelope[k] = std::clamp(w[k] / norm, 0.0f, 1.5f);
        // What the second should look like: off in the first tenth, in A's and
        // B's tenths when they are 1, and to 500 ms for the marker.
        std::array<float, kEnvRateHz> e{};
        for (int k = 0; k < kEnvRateHz; ++k) {
            bool off = k < 10;
            if (r.marker) off = k < 50;
            else if (k >= 10 && k < 20) off = r.a == 1;
            else if (k >= 20 && k < 30) off = r.b == 1;
            e[k] = off ? 0.0f : 1.0f;
        }
        float mean = 0.0f;
        for (float v : e) mean += v;
        mean /= kEnvRateHz;
        si.expected.assign(kEnvRateHz, 0.0f);
        for (int k = 0; k < kEnvRateHz; ++k) si.expected[k] = e[k] - mean;
        owner->onSecond(si);
    }

    void setLockState(ClockLockState s) {
        if (s != lockState) {
            lockState = s;
            if (owner->onStateChanged) owner->onStateChanged(s);
        }
    }

    // ---- members ---------------------------------------------------------

    MsfDecoder* owner;
    int sr;
    int decim;
    double fNominal;
    TimeFrameVoter voter;

    ClockLockState lockState = ClockLockState::NoSignal;
    enum class Phase { Searching, Running } phase = Phase::Searching;
    int64_t samplesConsumed = 0;

    struct Bin { double f = 0, stepRe = 1, stepIm = 0, rotRe = 1, rotIm = 0, accRe = 0, accIm = 0; };
    std::vector<Bin> bins;
    int64_t acqCount = 0, acqTarget = 0;
    float lastToneSnrDb = 0.0f;
    std::vector<Bin> liveBins;
    int64_t liveCount = 0, liveTarget = 0;

    double f0 = 0.0;
    double oscRe = 1.0, oscIm = 0.0, stepRe = 1.0, stepIm = 0.0;
    int oscRenorm = 0;

    Biquad lpI, lpQ;
    double lpfDelay = 0.0;
    // The mixed baseband at full rate, for refineEdge: a ring by sample index.
    std::vector<float> zRe, zIm;
    int64_t zMask = 0;
    int64_t zStart = 0;       // first sample index written since the carrier was found
    double magAccum = 0.0;
    int magCount = 0;
    int64_t envBaseSample = 0;
    std::array<float, kEnvCap> env{};
    int64_t envCount = 0;
    std::vector<float> pctScratch;
    float pHi = 0.0f, pLo = 0.0f;
    double lowFrac = 0.05;   // "off" as a fraction of full carrier, learnt
    // The carrier's phase between seconds (trackCarrier).
    double fResidual = 0.0;
    bool havePrevTheta = false;
    double prevTheta = 0.0, prevThetaAt = 0.0;

    bool segValid = false;
    double segEdge = 0.0;
    int64_t scanPos = 0;
    int blindSeconds = 0;
    EdgeTracker trk;
    std::vector<double> wideOff;   // the wide search's last offsets from the tracker

    std::vector<SecRec> hist;
    bool anchored = false;
    int sofNext = 0;
    std::array<SecRec, kFrameCap> frame{};
    int frFilled = 0;
    int64_t frStart = 0;
    bool haveLastFrame = false;
    int64_t lastFrameStart = 0;
    int lastFrameLen = 60;
    std::uint8_t lastFrameFrom = 0;
    int unconfirmedRun = 0;

    bool haveVoted = false;
    int votedMinute = -1, votedHour = -1, votedDoy = -1, votedYear = -1;
    float votedQuality = 0.0f;
};

// ---------------------------------------------------------------------------

MsfDecoder::MsfDecoder(int sampleRateHz, double carrierOffsetHz)
    : m_impl(std::make_unique<Impl>(this, sampleRateHz, carrierOffsetHz)) {}

MsfDecoder::~MsfDecoder() = default;

void MsfDecoder::process(const float* iq, std::size_t frames) { m_impl->process(iq, frames); }
void MsfDecoder::reset() { m_impl->reset(); }

void MsfDecoder::setPlausibility(std::function<TimeFields()> referenceNow, int boundMinutes) {
    m_impl->voter.setPlausibility(std::move(referenceNow), boundMinutes);
}

ClockLockState MsfDecoder::state() const { return m_impl->lockState; }

ClockStation MsfDecoder::station() const {
    return m_impl->lockState == ClockLockState::NoSignal ? ClockStation::Unknown : ClockStation::Msf;
}

std::int64_t MsfDecoder::samplesConsumed() const { return m_impl->samplesConsumed; }

ClockDecoderDiagnostics MsfDecoder::diagnostics() const {
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
    g.carrierOffsetHz = d.phase == Impl::Phase::Running ? static_cast<float>(d.f0)
                                                        : std::numeric_limits<float>::quiet_NaN();
    g.lastFrameFrom = d.lastFrameFrom;
    return g;
}

} // namespace clockdec
