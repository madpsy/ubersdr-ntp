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
// Chain, per input sample of complex baseband:
//
//   carrier search   DFT bins across +/-kPullHz of the expected offset, on a
//                    600 Hz decimation, repeated every 2 s until the carrier
//                    stands out; the peak is where the mixer goes, and it is
//                    followed after (kFreqGain).
//   reference        c = a 15 Hz low-pass of the mixed baseband z, and z itself
//                    delayed by that low-pass's group delay, so c is the
//                    carrier's phase AT the sample it is compared with: no lag
//                    to follow a Doppler shift with, and nothing that depends
//                    on the offset.
//   AM, coherently   m = Re(z conj(c)) / <|c|^2>. The modulation itself, on
//                    the carrier's own phase, weighted by the carrier's
//                    strength (a faded second counts for less, as it should),
//                    with no envelope detector to lose it in noise. The
//                    quadrature Im(z conj(c)) carries no AM at all, and is how
//                    the noise under m is measured.
//
// Then, from m:
//
//   seconds tick     the 5 ms burst of 1000 Hz (WWV) or 1200 Hz (WWVH) at the
//                    start of every second, correlated against its own
//                    waveform (TickTimer) and averaged coherently from second
//                    to second. THIS is the second edge served: NIST puts the
//                    second at the start of the tick, and the correlation's
//                    peak is exactly there, whatever filters the receiver used.
//   tick rails       bandpass 1000 / 1200 Hz, envelope, 200 Hz series, folded
//                    mod 1 s: the coarse tick phase that cuts the seconds, and
//                    the station tag.
//   BCD              coherent 100 Hz demod (quadrature mixer, 25 Hz LPF, the
//                    subcarrier's own phase once known) -> 200 Hz series a[]
//      -> per-second matched-filter classify (zero-mean 170/470/770 ms
//         templates at +30 ms; confidence = best correlation minus runner-up)
//      -> marker frame sync (P markers at 9/19/29/39/49/59), mod-10 degeneracy
//         resolved by the s0 minute-mark subcarrier hole + minute-increment
//         scoring -> NIST BCD field map -> TimeFrameVoter.
//
// The BCD pulse's edge is still measured, but only as a check on the tick
// (bcdMinusTickMs): after a 25 Hz low-pass its edges are tens of milliseconds
// long, and where it puts the second depends on the pulse's shape on the air.

#include <algorithm>
#include <array>
#include <cmath>
#include <complex>
#include <cstdint>
#include <deque>
#include <limits>
#include <vector>
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

// Each second's window is cut this many series samples (50 ms) before the
// tick, so the pulse's matched-filter shift sits in the middle of the range
// searched (0..kMaxShift) with room both ways for a receiver clock to walk it.
// Cut at the tick itself the shift settled 2.5 samples above the lower rail,
// and a clock 60 ppm slow put it on the rail within minutes, over and over.
constexpr int kWindowLead = 10;
// The matched-filter shift a clean, drift-free stream settles at, in whole
// series samples: the lead, plus the 25 Hz low-pass and the decimation (about
// 20 ms). The tracked delay estimate starts here.
constexpr int kNominalDelaySamples = kWindowLead + 4;
// The same, exactly, as tools/decodertest measures it: what the BCD edge takes
// off the smoothed shift to put a synthetic pulse on its true second. Only the
// BCD check uses it (bcdMinusTickMs); nothing served depends on it.
constexpr double kBcdEdgeDelaySamples = 3.52;

// Matched-filter shift search ceiling (series samples, 200 ms). Wide (WS-4.5)
// so slow sample-clock drift is ABSORBED by the tracked delay estimate instead
// of accumulating into systematic misreads; drift beyond the rails triggers a
// soft resynchronization.
constexpr int kMaxShift = 40;

// Leaky tick-fold decay per hit (each phase bin is hit once per second, so
// this is τ ≈ 100 s). The fold must FORGET a stale phase: after a sample
// discontinuity the re-seed has to find the CURRENT tick phase, not history.
constexpr double kFoldDecay = 0.99;
// The fold's other lock (foldZ): its peak this many spreads above the median
// bin, after at least this long. The largest of 200 bins of noise is about 3.
constexpr double kFoldZ = 10.0;
constexpr int kFoldZMinSecs = 10;

// Station tag (see onSeriesSample): one band's folded tick excess must be this
// many times the other's for a verdict. A single station's own tick leaks into
// the other band at roughly a third of its excess (3.2x clean, never below
// 1.77x through a lossy codec with a programme tone in offline measurement), so 1.5x
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

// ---- Carrier. --------------------------------------------------------------
// How far from where the dial puts it the carrier is looked for. WWV's carrier
// is an atomic standard, so only the receiver's clock moves it: 2 ppm at 25 MHz
// is 50 Hz. Further out than this the search would start to see the 100 Hz
// subcarrier's sidebands, which are the next strongest lines.
constexpr double kPullHz      = 50.0;
constexpr double kSearchStep  = 0.25;    // Hz between bins
constexpr int    kSearchRate  = 600;     // Hz, the decimated stream searched
constexpr double kSearchSec   = 2.0;
constexpr double kToneGate    = 12.0;    // carrier bin over the median bin, power
// A carrier found but no tick in this long: it was not the carrier. Search again.
constexpr double kResearchSec = 60.0;
// The reference low-pass: two Butterworth sections, fourth order. 15 Hz follows
// any fade or Doppler HF produces and is 66 dB down at the subcarrier's 100 Hz.
constexpr double kRefHz       = 15.0;
constexpr double kFreqGain    = 0.2;     // per-second fraction of the residual taken
constexpr double kLevelSec    = 10.0;    // carrier power, averaged, that m is scaled by
constexpr double kNoiseSec    = 10.0;    // the quadrature rail's variance, averaged
constexpr double kDcSec       = 1.0;     // what is taken off m before the BCD demod
// The subcarrier's long-run phase against the 100 Hz mixer, averaged this
// long: what steadies each second's own (processSecond).
constexpr double kBcdPhaseSec = 20.0;

// ---- Tick timer (see TickTimer). -------------------------------------------
constexpr double kTickSec       = 0.005;   // WWV: 5 cycles of 1000 Hz; WWVH: 6 of 1200
constexpr double kTickTrackSec  = 0.015;   // half-width of the lags searched when locked
constexpr double kTickAcqSec    = 0.030;   // ... and while acquiring
constexpr double kTickAlpha     = 1.0 / 16.0;
// Averaged-correlation SNR (power, linear) to lock, to hold, and to move the
// prediction at all. 14 dB at lock puts the edge within a few samples; 10 dB
// holds it through a fade without letting noise walk it.
// Until locked, the prediction moves only on a lock-grade apex: a noise peak
// that wins one second would carry the average's edge of the window with it.
constexpr double kTickLockSnr   = 25.0;
constexpr double kTickHoldSnr   = 10.0;
constexpr int    kTickMissLimit = 10;
// Seconds averaged before a lock: the largest of 700 lags of a thin average
// is noise far more often than the largest of a thick one.
constexpr int    kTickMinSecs   = 8;
// The step between seconds, once locked: the slope of the timed edges over the
// last two minutes, from ten seconds of them on, and nominal again when the
// lock goes -- a period learned from noise must not outlive it. A loop on each
// second's move was tried and wandered by tens of ppm: those moves are a
// sixteen-second average's, not a second's, and summing them walks.
constexpr std::size_t kTickRateSecs = 120;
constexpr std::size_t kTickRateMin  = 10;
// Locked, the tick moves by the receiver's clock error and the ionosphere's
// drift, both steady from one second to the next; a move this far from the
// recent ones is noise. (Not from zero: until the slope is known, a receiver
// tens of ppm off moves the average several samples a second, every second.)
constexpr double kTickMaxStep   = 3.0;
constexpr double kTickStepAlpha = 0.5;
constexpr double kTickMaxPpm    = 200.0;
// A second whose correlation 10-20 ms after the edge -- inside NIST's protected
// zone, where nothing is sent -- is this much of the edge's own is not a tick:
// the 800 ms minute or hour tone.
constexpr double kToneRatio     = 0.3;
// Nothing else is sent at the tick's frequency within 10 ms before it or 25 ms
// after (NIST's protected zone), so a real tick's average stands alone: its
// apex this far above anything more than 1.5 tick lengths away. A profile that
// is broad or has rivals -- the other station's minute tone, a carrier that is
// not WWV's -- is not a tick.
constexpr double kTickIsolation = 4.0;
constexpr double kBcdCheckAlpha = 1.0 / 32.0;

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

// A low-pass biquad's group delay at DC, in samples: one for the symmetric
// numerator, less the denominator's.
double dcDelaySamples(const Biquad& b) {
    return 1.0 - (b.a1 + 2.0 * b.a2) / (1.0 + b.a1 + b.a2);
}

using cd = std::complex<double>;

// One station's seconds tick, timed.
//
// The tick is a 5 ms burst, and in m (coherent AM) it is exactly the waveform
// NIST sends: a sinusoid at 1000 or 1200 Hz with a rectangular envelope,
// starting on the second. Correlated against e^{-jwt} over those 5 ms, the
// magnitude is a triangle 10 ms wide whose apex is the start of the burst --
// symmetric, so any linear-phase filtering of the whole signal leaves it where
// it is, and the phase along it turns at w, which is the whole of the
// waveform's fine structure.
//
// One second's correlation is noisy. The profile around the predicted edge is
// averaged across seconds, coherently: the tick has the same phase against the
// carrier every second, so a weak tick adds up as signal and the noise as its
// root. That needs every second's profile on the SAME footing, to a fraction of
// a sample, which is what the prediction x is for: the template is shifted by
// x's fraction of a sample (its first and last taps weighted by it, its phase
// turned by it), so each profile is taken exactly relative to x. Then the
// average's apex is how far x is from the tick, x moves there, and the average
// is moved with it (shift). The step from second to second is followed as well
// (period), so a receiver whose sample clock is tens of ppm off stays aligned.
//
// The apex is found by fitting a line to each side of the triangle, on the
// average projected onto its own phase, and intersecting them: every lag of
// the 10 ms carries evidence, and a real value has no Rayleigh floor.
struct TickTimer {
    double hz = 1000.0;
    double w = 0.0;            // rad per sample
    int L = 60;                // template length, samples
    int W = 180;               // lags searched each side when locked
    int Wacq = 360;            // ... and the profile's half-width
    std::vector<cd> tpl;       // e^{-j w t}, t = 0..L
    double wo = 0.0;           // the other station's tick, rad per sample
    std::vector<cd> tplO;      // ... and its template
    std::vector<cd> avg;       // 2*Wacq+1 lags, lag 0 at x
    std::vector<cd> cur;       // this second's profile
    // The same lags against the OTHER station's tick. At full overlap the two
    // are orthogonal (200 Hz over 5 ms is one whole cycle), but part-way on
    // they are not: a strong WWVH tick puts a bump in WWV's profile 2.5 ms off
    // its own edge, a third of its height. Where the other template explains
    // a lag better than this one, it is the other station there, not this one.
    std::vector<cd> avgO, curO;
    double varAvg = 0.0;       // noise variance of one lag of avg (complex)
    int n = 0;                 // seconds in the average
    bool active = false;
    double x = 0.0;            // this second's predicted edge, m index
    double period = 12000.0;   // samples a second
    double nominal = 12000.0;
    bool locked = false;
    int miss = 0;
    double stepRef = 0.0;      // recent accepted moves, averaged
    std::int64_t tickCount = 0;                          // seconds timed, for the slope
    std::deque<std::pair<double, double>> hist;          // (second, edge) while locked
    double snr = 0.0;          // averaged, power, at the apex
    double edge = 0.0;         // the timed edge after this second's update

    void init(double f, double other, int fs) {
        hz = f;
        w = 2.0 * 3.14159265358979323846 * f / fs;
        wo = 2.0 * 3.14159265358979323846 * other / fs;
        L = static_cast<int>(std::lround(kTickSec * fs));
        W = static_cast<int>(std::lround(kTickTrackSec * fs));
        Wacq = static_cast<int>(std::lround(kTickAcqSec * fs));
        tpl.resize(static_cast<std::size_t>(L + 1));
        tplO.resize(static_cast<std::size_t>(L + 1));
        for (int t = 0; t <= L; ++t) {
            tpl[static_cast<std::size_t>(t)] = std::polar(1.0, -w * t);
            tplO[static_cast<std::size_t>(t)] = std::polar(1.0, -wo * t);
        }
        avg.assign(static_cast<std::size_t>(2 * Wacq + 1), cd(0.0, 0.0));
        cur.assign(avg.size(), cd(0.0, 0.0));
        avgO.assign(avg.size(), cd(0.0, 0.0));
        curO.assign(avg.size(), cd(0.0, 0.0));
        nominal = period = fs;
        reset();
    }
    void reset() {
        std::fill(avg.begin(), avg.end(), cd(0.0, 0.0));
        std::fill(avgO.begin(), avgO.end(), cd(0.0, 0.0));
        varAvg = 0.0; n = 0; active = false; locked = false; miss = 0; snr = 0.0; stepRef = 0.0;
        hist.clear();
        period = nominal;
    }
    cd& at(std::vector<cd>& v, int lag) { return v[static_cast<std::size_t>(lag + Wacq)]; }

    // Move the average by e samples: avg(l) <- avg(l + e). Taken off its
    // carrier ramp first, what is left is the triangle times a constant phase,
    // which linear interpolation moves exactly along each side.
    void shift(double e) {
        if (e == 0.0) return;
        shiftOne(avg, w, e);
        shiftOne(avgO, wo, e);
    }
    void shiftOne(std::vector<cd>& v, double wr, double e) {
        std::vector<cd> out(v.size(), cd(0.0, 0.0));
        for (int l = -Wacq; l <= Wacq; ++l) {
            const double s = l + e;
            const int i0 = static_cast<int>(std::floor(s));
            const double f = s - i0;
            if (i0 < -Wacq || i0 + 1 > Wacq) continue;
            const cd b0 = at(v, i0) * std::polar(1.0, -wr * i0);
            const cd b1 = at(v, i0 + 1) * std::polar(1.0, -wr * (i0 + 1));
            out[static_cast<std::size_t>(l + Wacq)] = ((1.0 - f) * b0 + f * b1) * std::polar(1.0, wr * s);
        }
        v.swap(out);
    }
};

} // namespace

// ---------------------------------------------------------------------------

struct WwvDecoder::Impl {
    Impl(int sampleRateHz, double carrierOffsetHz);

    // Public-surface state.
    int fs;
    int decim;                       // input samples per 200 Hz series sample
    ClockLockState state = ClockLockState::NoSignal;
    ClockStation station = ClockStation::Unknown;
    std::int64_t samplesConsumed = 0;

    // Callbacks (owned by the outer WwvDecoder; copied pointers here).
    WwvDecoder* owner = nullptr;

    // --- Carrier search (once found, repeated only if no tick follows). -----
    double fNominal = 0.0;           // where the dial puts the carrier, Hz
    bool carrierFound = false;
    std::int64_t carrierFoundAt = 0; // samplesConsumed when it was
    int searchDecim = 20;
    double sAccR = 0.0, sAccI = 0.0; int sAccN = 0;
    double sOscR = 1.0, sOscI = 0.0, sStepR = 1.0, sStepI = 0.0; int sRenorm = 0;
    std::vector<cd> searchBuf;       // kSearchSec at kSearchRate
    float carrierSnrDb = std::numeric_limits<float>::quiet_NaN();

    // --- Mixer, reference, coherent AM. -------------------------------------
    double f0 = 0.0;                 // carrier, Hz in the baseband
    double oscR = 1.0, oscI = 0.0, stepR = 1.0, stepI = 0.0; int oscRenormZ = 0;
    std::array<Biquad, 2> refI, refQ;
    int refDelay = 0;                // z is delayed this many samples to meet c
    std::vector<cd> zLine; std::int64_t zMask = 0;
    double level = 0.0;              // <|c|^2>
    double qVar = 0.0;               // variance of the quadrature rail
    double aLevel = 0.0, aNoise = 0.0, aDc = 0.0;
    double mDc = 0.0;
    int freqCount = 0;
    cd freqPrev{0.0, 0.0};

    // m, kept for the tick timers: index = the input sample it stands for.
    std::vector<float> mRing, qRing; std::int64_t mMask = 0;
    std::int64_t mHead = -1;         // newest m index written

    // --- BCD and tick-rail front end (runs on m). ---------------------------
    std::array<Biquad, 2> lpI;       // 25 Hz LPF, in-phase rail
    std::array<Biquad, 2> lpQ;       // 25 Hz LPF, quadrature rail
    std::array<Biquad, 2> bpTickV;   // WWV  tick band (1000 Hz)
    std::array<Biquad, 2> bpTickH;   // WWVH tick band (1200 Hz)

    // 100 Hz quadrature oscillator (running rotation — no per-sample trig).
    double oscC = 1.0, oscS = 0.0, rotC = 1.0, rotS = 0.0;
    int oscRenorm = 0;
    // The subcarrier's phase against it (see kBcdPhaseSec).
    cd bcdPhase{0.0, 0.0};
    double aBcdPhase = 0.0;

    // Decimation accumulators.
    double accI = 0.0, accQ = 0.0, accTickV = 0.0, accTickH = 0.0;
    int decCount = 0;
    std::int64_t n200 = 0;           // count of 200 Hz series samples emitted

    // --- Tick timers, one per station. ---------------------------------------
    TickTimer timerV, timerH;
    bool tickTiming = false;         // the last second's edge came from a tick
    double bcdMinusTick = std::numeric_limits<double>::quiet_NaN();   // samples
    int bcdCheckN = 0;

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
    std::array<cd, kSecLen> curSec{};
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
    double lastEdgeSampleExact = std::numeric_limits<double>::quiet_NaN();
    int lastEdgeSecondOfFrame = -1;

    // Cross-frame voter.
    TimeFrameVoter voter;

    // --- Methods. ----------------------------------------------------------
    void designFilters();
    void buildTemplates();
    void setState(ClockLockState s);
    void setMixer(double f);
    void searchStep(double xr, double xi);
    void finishSearch();
    void processSample(double xr, double xi);
    void processM(std::int64_t k, double m);
    void onSeriesSample(cd b, double tickV, double tickH);
    TickTimer& timingTimer();
    void timeTick(TickTimer& t, double windowStart, int secOfFrame);
    void processSecond(std::int64_t startJ, const std::array<cd, kSecLen>& b);
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

WwvDecoder::Impl::Impl(int sampleRateHz, double carrierOffsetHz)
    : fs(sampleRateHz > 0 ? sampleRateHz : 12000),
      decim(std::max(1, (sampleRateHz > 0 ? sampleRateHz : 12000) / kSeriesRate)),
      fNominal(carrierOffsetHz),
      voter(makeVoterConfig()) {
    designFilters();
    buildTemplates();
}

void WwvDecoder::Impl::designFilters() {
    double f = static_cast<double>(fs);
    // Coherent-demod baseband LPF (25 Hz) — matches the prototype's 25 Hz mask.
    for (auto& b : lpI) b = designLowpass(25.0, 0.70710678, f);
    for (auto& b : lpQ) b = designLowpass(25.0, 0.70710678, f);
    // Tick bands, 167 Hz wide as the 2000/2200 Hz pair of the USB decoder was,
    // so 1000 Hz (WWV) and 1200 Hz (WWVH) separate as those did.
    for (auto& b : bpTickV) b = designBandpass(1000.0, 6.0, f);
    for (auto& b : bpTickH) b = designBandpass(1200.0, 7.2, f);

    double dth = 2.0 * kPi * 100.0 / f;   // 100 Hz demod rotation per sample
    rotC = std::cos(dth);
    rotS = std::sin(dth);

    // The carrier reference: fourth-order Butterworth, and the delay that
    // brings z level with it.
    const double qs[2] = {0.54119610014619698, 1.3065629648763766};
    double delay = 0.0;
    for (int s = 0; s < 2; ++s) {
        refI[static_cast<std::size_t>(s)] = designLowpass(kRefHz, qs[s], f);
        refQ[static_cast<std::size_t>(s)] = refI[static_cast<std::size_t>(s)];
        delay += dcDelaySamples(refI[static_cast<std::size_t>(s)]);
    }
    refDelay = static_cast<int>(std::lround(delay));
    std::int64_t zn = 1;
    while (zn < refDelay + 2) zn <<= 1;
    zLine.assign(static_cast<std::size_t>(zn), cd(0.0, 0.0));
    zMask = zn - 1;

    // m is kept 2.5 s back: a second is timed once its whole window is in,
    // and the window can sit up to a quarter-second off the tick.
    std::int64_t mn = 1;
    while (mn < static_cast<std::int64_t>(2.5 * f)) mn <<= 1;
    mRing.assign(static_cast<std::size_t>(mn), 0.0f);
    qRing.assign(static_cast<std::size_t>(mn), 0.0f);
    mMask = mn - 1;

    aLevel = 1.0 / (kLevelSec * f);
    aNoise = 1.0 / (kNoiseSec * f);
    aDc = 1.0 / (kDcSec * f);
    aBcdPhase = 1.0 / (kBcdPhaseSec * kSeriesRate);

    searchDecim = std::max(1, fs / kSearchRate);
    const double sdth = -2.0 * kPi * fNominal / f;
    sStepR = std::cos(sdth); sStepI = std::sin(sdth);
    searchBuf.clear();
    searchBuf.reserve(static_cast<std::size_t>(kSearchSec * kSearchRate));
    setMixer(fNominal);

    timerV.init(1000.0, 1200.0, fs);
    timerH.init(1200.0, 1000.0, fs);
}

void WwvDecoder::Impl::setMixer(double f) {
    f0 = std::clamp(f, fNominal - kPullHz, fNominal + kPullHz);
    stepR = std::cos(-2.0 * kPi * f0 / fs);
    stepI = std::sin(-2.0 * kPi * f0 / fs);
}

// The search: the baseband mixed by the nominal offset, block-averaged down to
// 600 Hz (the carrier +/- 50 Hz is all that is wanted of it), kSearchSec of
// that, then DFT bins across +/-kPullHz.
void WwvDecoder::Impl::searchStep(double xr, double xi) {
    const double zr = xr * sOscR - xi * sOscI;
    const double zi = xr * sOscI + xi * sOscR;
    const double nr = sOscR * sStepR - sOscI * sStepI;
    sOscI = sOscR * sStepI + sOscI * sStepR;
    sOscR = nr;
    if (++sRenorm >= 1024) {
        sRenorm = 0;
        const double m = std::hypot(sOscR, sOscI);
        if (m > 0.0) { sOscR /= m; sOscI /= m; }
    }
    sAccR += zr; sAccI += zi;
    if (++sAccN < searchDecim) return;
    searchBuf.emplace_back(sAccR / sAccN, sAccI / sAccN);
    sAccR = sAccI = 0.0; sAccN = 0;
    if (searchBuf.size() >= static_cast<std::size_t>(kSearchSec * kSearchRate)) finishSearch();
}

void WwvDecoder::Impl::finishSearch() {
    const double rate = static_cast<double>(fs) / searchDecim;
    const int nb = static_cast<int>(std::lround(2.0 * kPullHz / kSearchStep)) + 1;
    std::vector<double> pw(static_cast<std::size_t>(nb));
    int peak = 0;
    for (int b = 0; b < nb; ++b) {
        const double f = -kPullHz + b * kSearchStep;
        const cd step = std::polar(1.0, -2.0 * kPi * f / rate);
        cd rot(1.0, 0.0), acc(0.0, 0.0);
        for (const cd& v : searchBuf) { acc += v * rot; rot *= step; }
        pw[static_cast<std::size_t>(b)] = std::norm(acc);
        if (pw[static_cast<std::size_t>(b)] > pw[static_cast<std::size_t>(peak)]) peak = b;
    }
    std::vector<double> med = pw;
    std::nth_element(med.begin(), med.begin() + nb / 2, med.end());
    const double median = med[static_cast<std::size_t>(nb / 2)];
    searchBuf.clear();
    if (median > 0.0) carrierSnrDb = static_cast<float>(10.0 * std::log10(pw[static_cast<std::size_t>(peak)] / median));
    if (!(median > 0.0) || pw[static_cast<std::size_t>(peak)] < kToneGate * median) return;
    double f = -kPullHz + peak * kSearchStep;
    if (peak > 0 && peak + 1 < nb) {
        const double a = std::sqrt(pw[static_cast<std::size_t>(peak - 1)]);
        const double b = std::sqrt(pw[static_cast<std::size_t>(peak)]);
        const double c = std::sqrt(pw[static_cast<std::size_t>(peak + 1)]);
        const double den = a - 2.0 * b + c;
        if (den < 0.0) f += kSearchStep * std::clamp(0.5 * (a - c) / den, -0.5, 0.5);
    }
    setMixer(fNominal + f);
    carrierFound = true;
    carrierFoundAt = samplesConsumed;
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

void WwvDecoder::Impl::processSample(double xr, double xi) {
    const std::int64_t n = samplesConsumed++;

    if (!carrierFound) searchStep(xr, xi);
    else if (!timerV.locked && !timerH.locked && !tickLocked &&
             n - carrierFoundAt > static_cast<std::int64_t>(kResearchSec * fs)) {
        carrierFound = false;   // a carrier with no tick after a minute: look again
    }

    // 1) Mixer: the carrier to 0 Hz.
    const double zr = xr * oscR - xi * oscI;
    const double zi = xr * oscI + xi * oscR;
    {
        const double nr = oscR * stepR - oscI * stepI;
        oscI = oscR * stepI + oscI * stepR;
        oscR = nr;
        if (++oscRenormZ >= 1024) {
            oscRenormZ = 0;
            const double m = std::hypot(oscR, oscI);
            if (m > 0.0) { oscR /= m; oscI /= m; }
        }
    }

    // 2) The carrier's phase and strength, and z delayed to meet it.
    double cr = zr, ci = zi;
    for (auto& b : refI) cr = b.process(cr);
    for (auto& b : refQ) ci = b.process(ci);
    zLine[static_cast<std::size_t>(n & zMask)] = cd(zr, zi);

    // 3) Follow the carrier: a residual offset shows as the reference turning.
    if (++freqCount >= fs) {
        freqCount = 0;
        const cd c(cr, ci);
        if (std::norm(freqPrev) > 0.0 && std::norm(c) > 0.0) {
            const double dphi = std::arg(c * std::conj(freqPrev));
            setMixer(f0 + kFreqGain * dphi / (2.0 * kPi));
        }
        freqPrev = c;
    }

    if (n < refDelay) return;
    const cd zd = zLine[static_cast<std::size_t>((n - refDelay) & zMask)];

    // 4) Coherent AM, scaled by the carrier's average power.
    const double p = cr * cr + ci * ci;
    const std::int64_t k = n - refDelay;           // the sample m stands for
    level += std::max(aLevel, 1.0 / static_cast<double>(k + 1)) * (p - level);
    const double inv = level > 1e-30 ? 1.0 / level : 0.0;
    const double m = (zd.real() * cr + zd.imag() * ci) * inv;
    const double q = (zd.imag() * cr - zd.real() * ci) * inv;
    qVar += std::max(aNoise, 1.0 / static_cast<double>(k + 1)) * (q * q - qVar);
    mRing[static_cast<std::size_t>(k & mMask)] = static_cast<float>(m);
    qRing[static_cast<std::size_t>(k & mMask)] = static_cast<float>(q);
    mHead = k;

    processM(k, m);
}

void WwvDecoder::Impl::processM(std::int64_t k, double m) {
    // 1) Coherent 100 Hz demod: m less its mean (the carrier), mixed down by
    //    the running quadrature oscillator, both rails at 25 Hz.
    mDc += std::max(aDc, 1.0 / static_cast<double>(k + 1)) * (m - mDc);
    const double mb = m - mDc;
    double i = mb * oscC;
    double q = mb * oscS;
    for (auto& b : lpI) i = b.process(i);
    for (auto& b : lpQ) q = b.process(q);

    double nc = oscC * rotC - oscS * rotS;
    double ns = oscS * rotC + oscC * rotS;
    oscC = nc; oscS = ns;
    if (++oscRenorm >= 1024) {
        oscRenorm = 0;
        double r = std::sqrt(oscC * oscC + oscS * oscS);
        if (r > 0.0) { oscC /= r; oscS /= r; }
    }

    // 2) Tick rails: bandpass around each station's tick, rectify.
    double tv = m, th = m;
    for (auto& b : bpTickV) tv = b.process(tv);
    for (auto& b : bpTickH) th = b.process(th);

    // 3) Decimate by block-average to the 200 Hz series.
    accI += i; accQ += q; accTickV += std::fabs(tv); accTickH += std::fabs(th);
    if (++decCount >= decim) {
        const double invd = 1.0 / decim;
        const cd b(accI * invd, accQ * invd);
        // The subcarrier's long-run phase against the mixer (see processSecond).
        bcdPhase += aBcdPhase * (b - bcdPhase);
        onSeriesSample(b, accTickV * invd, accTickH * invd);
        accI = accQ = accTickV = accTickH = 0.0;
        decCount = 0;
    }
}

// The timer the second is served from: the tagged station's, WWV until tagged,
// which is also what the engine's delay model assumes.
TickTimer& WwvDecoder::Impl::timingTimer() {
    return station == ClockStation::Wwvh ? timerH : timerV;
}

void WwvDecoder::Impl::timeTick(TickTimer& t, double windowStart, int secOfFrame) {
    // Predict. A timer that has lost the windows -- a soft reacquisition cut
    // them somewhere new -- starts over from the window.
    if (!t.active || std::fabs(t.x + t.period - windowStart) > 0.25 * fs) {
        t.reset();
        t.active = true;
        t.x = windowStart;
    } else {
        t.x += t.period;
    }
    ++t.tickCount;   // the slope's time axis: one a second, timed or not

    const int L = t.L, Wa = t.Wacq;
    const std::int64_t R = std::llround(t.x);
    const double g = t.x - static_cast<double>(R);   // -0.5 .. 0.5
    if (R - Wa < mHead - mMask || R + Wa + L + 1 > mHead) return;   // not in the ring

    // This second's profile, lag l meaning an edge at x + l. Each sample
    // stands for the half-sample either side of it, so the 5 ms from x + l
    // takes its first tap at 1/2 - g and one more, at L, at 1/2 + g; and the
    // phase is turned by g. The profile is then taken from x exactly, to a
    // fraction of a sample.
    const cd turn = std::polar(1.0, t.w * g);
    const cd turnO = std::polar(1.0, t.wo * g);
    const double edgeW = 0.5 + g;
    auto mAt = [&](std::int64_t i) { return static_cast<double>(mRing[static_cast<std::size_t>(i & mMask)]); };
    for (int l = -Wa; l <= Wa; ++l) {
        const std::int64_t s = R + l;
        cd acc(0.0, 0.0), accO(0.0, 0.0);
        for (int u = 0; u < L; ++u) {
            const double v = mAt(s + u);
            acc += t.tpl[static_cast<std::size_t>(u)] * v;
            accO += t.tplO[static_cast<std::size_t>(u)] * v;
        }
        const double v0 = mAt(s), vL = mAt(s + L);
        acc += edgeW * (t.tpl[static_cast<std::size_t>(L)] * vL - v0);
        accO += edgeW * (t.tplO[static_cast<std::size_t>(L)] * vL - v0);
        t.at(t.cur, l) = acc * turn;
        t.at(t.curO, l) = accO * turnO;
    }

    // Not a tick: seconds 29 and 59 have none, second 0 is the 800 ms minute
    // (or hour) tone, known by frame once anchored and by the tone carrying on
    // into the protected zone before that.
    const double lagNoise = L * qVar;   // one lag's noise, complex variance
    bool exclude = anchored && (secOfFrame == 0 || secOfFrame == 29 || secOfFrame == 59);
    if (!exclude) {
        // Judged at this second's own peak (the tone's onset is a peak too, and
        // its plateau carries on past it), or at the prediction once the
        // average says where the tick is.
        // Own less other, as the apex is found: WWVH's tick lands 15-20 ms after
        // WWV's on a North American path -- just where the tone is looked for
        // -- and leaks into the 1000 Hz template there; heard about as strong
        // as WWV (K3FEF, 10 MHz), it had nearly every second thrown out.
        auto own = [&](int l) {
            return std::max(0.0, std::norm(t.at(t.cur, l)) - std::norm(t.at(t.curO, l)));
        };
        int l0 = 0;
        if (!t.locked) {
            double best = -1.0;
            for (int l = -Wa; l <= Wa - 3 * L; ++l) {
                const double v = own(l);
                if (v > best) { best = v; l0 = l; }
            }
        }
        const double at0 = own(l0);
        const double after = 0.5 * (own(std::min(Wa, l0 + 2 * L)) + own(std::min(Wa, l0 + 3 * L)));
        if (at0 > 4.0 * lagNoise && after > kToneRatio * at0) exclude = true;
    }
    if (!exclude) {
        ++t.n;
        const double a = std::max(1.0 / t.n, kTickAlpha);
        for (std::size_t i = 0; i < t.avg.size(); ++i) {
            t.avg[i] += a * (t.cur[i] - t.avg[i]);
            t.avgO[i] += a * (t.curO[i] - t.avgO[i]);
        }
        t.varAvg = (1.0 - a) * (1.0 - a) * t.varAvg + a * a * lagNoise;
    }
    if (t.n == 0 || !(t.varAvg > 0.0)) { t.edge = t.x; return; }

    // The apex: the strongest lag within reach, the average projected onto
    // its phase there, and a line through each side.
    // Strongest where this station's tick explains the lag better than the
    // other's does: a leaked bump of the other tick scores below zero.
    const int reach = (t.locked ? t.W : Wa) - L;
    int lp = 0;
    double best = -std::numeric_limits<double>::infinity();
    for (int l = -reach; l <= reach; ++l) {
        const double v = std::norm(t.at(t.avg, l)) - std::norm(t.at(t.avgO, l));
        if (v > best) { best = v; lp = l; }
    }
    double theta = std::arg(t.at(t.avg, lp)) - t.w * lp;
    auto y = [&](int l) { return std::real(t.at(t.avg, l) * std::polar(1.0, -(t.w * l + theta))); };
    auto fit = [&](int from, int to, double& a0, double& b0) {
        double sx = 0, sy = 0, sxx = 0, sxy = 0; int cnt = 0;
        for (int l = from; l <= to; ++l) {
            const double v = y(l);
            sx += l; sy += v; sxx += double(l) * l; sxy += l * v; ++cnt;
        }
        const double den = cnt * sxx - sx * sx;
        if (cnt < 2 || den == 0.0) return false;
        b0 = (cnt * sxy - sx * sy) / den;
        a0 = (sy - b0 * sx) / cnt;
        return true;
    };
    // The strongest lag is only somewhere on the triangle's top, which on a
    // weak signal is flat against the noise; the lines are what place it. So
    // fit around the strongest lag, and if the lines meet elsewhere on the
    // top, fit again around where they met.
    const int span = static_cast<int>(0.8 * L);
    double apex = lp;
    for (int pass = 0; pass < 2; ++pass) {
        double aL = 0, bL = 0, aR = 0, bR = 0;
        if (!(fit(lp - span, lp - 2, aL, bL) && fit(lp + 2, lp + span, aR, bR) && bL > 0.0 && bR < 0.0)) break;
        const double c = (aR - aL) / (bL - bR);
        if (std::fabs(c - lp) <= 2.0) { apex = c; break; }
        if (pass == 1 || std::fabs(c - lp) > span / 3.0) break;
        lp = static_cast<int>(std::lround(c));
        if (std::abs(lp) > reach) break;
        theta = std::arg(t.at(t.avg, lp)) - t.w * lp;
        apex = lp;
    }
    const double peak = y(lp);
    // The other template, at the apex, is noise alone for a genuine tick of
    // this station's; anything it holds there counts against it.
    t.snr = std::max(0.0, peak * peak - std::norm(t.at(t.avgO, lp))) / (0.5 * t.varAvg);
    // Rivals judged as the apex was: the other station's tick, heard 14 ms
    // after WWV's on a European path, leaks a bump into this profile that is
    // no rival -- the other template explains it.
    double rival = 0.0;
    for (int l = -Wa; l <= Wa; ++l)
        if (std::abs(l - lp) > 3 * L / 2)
            rival = std::max(rival, std::norm(t.at(t.avg, l)) - std::norm(t.at(t.avgO, l)));
    if (std::norm(t.at(t.avg, lp)) < kTickIsolation * rival) t.snr = 0.0;

    if (!t.locked) {
        if (t.snr >= kTickLockSnr && t.n >= kTickMinSecs) {
            t.locked = true; t.miss = 0; t.hist.clear(); t.stepRef = 0.0;
        }
    } else if (t.snr < kTickHoldSnr) {
        // Weak: hold where the tick was, and let go only if it stays weak.
        apex = 0.0;
        if (++t.miss >= kTickMissLimit) { t.locked = false; t.period = t.nominal; t.hist.clear(); }
    } else if (std::fabs(apex - t.stepRef) > kTickMaxStep) {
        // A jump no receiver clock makes in a second: follow it no further than
        // one that it could. Holding still instead let a receiver whose clock
        // runs tens of ppm off -- K3FEF, +26 ppm, before the slope was fitted --
        // fall further behind every second it held, until every second was a
        // jump and the lock cycled every eleven seconds.
        apex = t.stepRef + std::clamp(apex - t.stepRef, -kTickMaxStep, kTickMaxStep);
        if (++t.miss >= kTickMissLimit) { t.locked = false; t.period = t.nominal; t.hist.clear(); }
    } else {
        t.miss = 0;
        t.stepRef += kTickStepAlpha * (apex - t.stepRef);
    }

    // Move the prediction to the apex and the average with it; then the step
    // between seconds from the slope of where the tick has been.
    if (t.locked || (t.snr >= kTickLockSnr && t.n >= kTickMinSecs)) {
        t.shift(apex);
        t.x += apex;
    }
    if (t.locked && t.miss == 0) {
        t.hist.emplace_back(static_cast<double>(t.tickCount), t.x);
        while (t.hist.size() > kTickRateSecs) t.hist.pop_front();
        if (t.hist.size() >= kTickRateMin) {
            // Relative to the first point, so the sums stay small.
            const double s0 = t.hist.front().first, x0 = t.hist.front().second;
            double sx = 0, sy = 0, sxx = 0, sxy = 0;
            for (const auto& h : t.hist) {
                const double u = h.first - s0, v = h.second - x0;
                sx += u; sy += v; sxx += u * u; sxy += u * v;
            }
            const double nn = static_cast<double>(t.hist.size());
            const double den = nn * sxx - sx * sx;
            if (den > 0.0) {
                const double lim = t.nominal * kTickMaxPpm * 1e-6;
                t.period = std::clamp((nn * sxy - sx * sy) / den, t.nominal - lim, t.nominal + lim);
            }
        }
    }
    t.edge = t.x;
}

void WwvDecoder::Impl::onSeriesSample(cd b, double tickV, double tickH) {
    const double a = std::abs(b);
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
    // 1.36x; through a 16 kbps lossy codec plus that tone the 2200 Hz band came out
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
    // How far a fold's peak stands above its other bins, in units of their
    // own spread (the median absolute deviation). On a weak signal the ratio
    // above cannot pass however long the fold runs -- the noise under every
    // bin is most of the peak's height too -- but the spread of that noise
    // shrinks as the fold integrates, and the tick's excess does not.
    auto foldZ = [](const std::array<double, kSecLen>& fold, int arg) {
        std::array<double, kSecLen> tmp = fold;
        std::nth_element(tmp.begin(), tmp.begin() + kSecLen / 2, tmp.end());
        const double median = tmp[kSecLen / 2];
        for (double& v : tmp) v = std::fabs(v - median);
        std::nth_element(tmp.begin(), tmp.begin() + kSecLen / 2, tmp.end());
        const double mad = 1.4826 * tmp[kSecLen / 2];
        return mad > 0.0 ? (fold[static_cast<std::size_t>(arg)] - median) / mad : 0.0;
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
        if (std::max(rV, rH) > 2.5 ||
            (j >= kFoldZMinSecs * kSecLen && std::max(foldZ(foldV, argV), foldZ(foldH, argH)) > kFoldZ)) {
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
    // fold at tick lock, and never revisited: offline, one lossy-coded WWV run
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
    bool boundary = (((j - tickPhase + kWindowLead) % kSecLen) == 0) && (j + kWindowLead >= tickPhase);
    if (boundary) {
        if (secStarted && curFill == kSecLen) processSecond(secStartJ, curSec);
        curFill = 0;
        secStarted = true;
        secStartJ = j;
    }
    if (secStarted && curFill < kSecLen) curSec[curFill++] = b;
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
                                     const std::array<cd, kSecLen>& b) {
    // The pulse, read as the projection of the 100 Hz demod onto the
    // subcarrier's phase: a real value, noise and all, where the magnitude
    // reads the noise as a floor under every sample and loses a weak pulse in
    // it. NIST starts every pulse on the subcarrier's positive-going zero
    // crossing, so its phase against the mixer is one value -- but on a
    // skywave path the modes' 100 Hz envelopes add at their own delays (a
    // millisecond is 36 degrees of it), and live, from one second to the next,
    // it swings by 60 degrees and more (K3FEF and M9PSY, 5 MHz, 2026-09-29).
    // Projected on the long-run phase such a second read as no pulse at all.
    // So the phase is this second's own -- the whole second summed, which is
    // well clear of the noise even where its samples are not -- steadied by the
    // long-run one, which it outweighs when the second is strong and differs.
    cd sum(0.0, 0.0);
    for (const cd& v : b) sum += v;
    const cd ref = sum / static_cast<double>(kSecLen) + bcdPhase;
    const cd rot = std::abs(ref) > 0.0 ? std::conj(ref) / std::abs(ref) : cd(1.0, 0.0);
    std::array<float, kSecLen> w{};
    for (int n = 0; n < kSecLen; ++n) w[static_cast<std::size_t>(n)] = static_cast<float>(std::real(b[static_cast<std::size_t>(n)] * rot));

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
    const double bcdEdge = (static_cast<double>(startJ) + reportDelay - kBcdEdgeDelaySamples) * decim;

    // The tick, both stations', and the second served from the tagged one's.
    // Where it is not locked the BCD edge still frames the second -- the count
    // of whole seconds needs an edge every second -- but it is not served.
    const double windowStart = static_cast<double>(startJ + kWindowLead) * decim;
    timeTick(timerV, windowStart, secOfFrame);
    timeTick(timerH, windowStart, secOfFrame);
    const TickTimer& tt = timingTimer();
    tickTiming = tt.locked;
    const bool tickSecond = !(anchored && (secOfFrame == 0 || secOfFrame == 29 || secOfFrame == 59));
    const double edgeExact = tickTiming ? tt.edge : bcdEdge;
    const std::int64_t edgeSample = static_cast<std::int64_t>(std::llround(edgeExact));
    if (tickTiming && timed && tickSecond) {
        const double d = bcdEdge - tt.edge;
        if (std::fabs(d) < 0.1 * fs) {
            const double a = std::max(1.0 / ++bcdCheckN, kBcdCheckAlpha);
            bcdMinusTick = bcdCheckN == 1 ? d : bcdMinusTick + a * (d - bcdMinusTick);
        }
    }

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
    lastEdgeSampleExact = edgeExact;
    lastEdgeSecondOfFrame = secOfFrame;

    // Emit the classified second (drives the alignment display).
    if (owner && owner->onSecond) {
        ClockSecondInfo info;
        info.edgeSample = edgeSample;
        info.edgeSampleExact = edgeExact;
        // Measured: by the tick when it is timing (not at seconds 0, 29, 59,
        // which have none -- the tracker's prediction stands there), by the
        // pulse otherwise. Servable only from the tick.
        info.edgeMeasured = tickTiming ? tickSecond : timed;
        info.edgeServable = tickTiming;
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
                t.lastEdgeSampleExact = lastEdgeSampleExact;
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
    for (auto& b : lpI) b.reset();
    for (auto& b : lpQ) b.reset();
    for (auto& b : bpTickV) b.reset();
    for (auto& b : bpTickH) b.reset();
    for (auto& b : refI) b.reset();
    for (auto& b : refQ) b.reset();
    oscC = 1.0; oscS = 0.0; oscRenorm = 0;
    accI = accQ = accTickV = accTickH = 0.0; decCount = 0; n200 = 0;
    carrierFound = false; carrierFoundAt = 0; carrierSnrDb = std::numeric_limits<float>::quiet_NaN();
    sAccR = sAccI = 0.0; sAccN = 0; sOscR = 1.0; sOscI = 0.0; sRenorm = 0; searchBuf.clear();
    oscR = 1.0; oscI = 0.0; oscRenormZ = 0; setMixer(fNominal);
    std::fill(zLine.begin(), zLine.end(), cd(0.0, 0.0));
    std::fill(mRing.begin(), mRing.end(), 0.0f);
    std::fill(qRing.begin(), qRing.end(), 0.0f);
    mHead = -1; level = 0.0; qVar = 0.0; mDc = 0.0; freqCount = 0; freqPrev = cd(0.0, 0.0);
    bcdPhase = cd(0.0, 0.0);
    timerV.reset(); timerH.reset(); tickTiming = false;
    bcdMinusTick = std::numeric_limits<double>::quiet_NaN(); bcdCheckN = 0;
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
    lastEdgeSample = 0; lastEdgeSampleExact = std::numeric_limits<double>::quiet_NaN();
    lastEdgeSecondOfFrame = -1;
    if (!stationPinned) station = ClockStation::Unknown;
    samplesConsumed = 0;
    voter.reset();
    setState(ClockLockState::NoSignal);
}

// ---------------------------------------------------------------------------
// Public surface.

WwvDecoder::WwvDecoder(int sampleRateHz, double carrierOffsetHz)
    : m_impl(std::make_unique<Impl>(sampleRateHz, carrierOffsetHz)) {
    m_impl->owner = this;
}

WwvDecoder::~WwvDecoder() = default;

void WwvDecoder::process(const float* iq, std::size_t frames) {
    if (!iq) return;
    Impl* d = m_impl.get();
    for (std::size_t k = 0; k < frames; ++k) d->processSample(iq[2 * k], iq[2 * k + 1]);
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

    const TickTimer& tt = d.station == ClockStation::Wwvh ? d.timerH : d.timerV;
    g.tickTiming = d.tickTiming;
    g.tickSnrDb = tt.n > 0 && tt.snr > 0.0 ? static_cast<float>(10.0 * std::log10(tt.snr))
                                            : std::numeric_limits<float>::quiet_NaN();
    g.bcdMinusTickMs = std::isfinite(d.bcdMinusTick)
        ? static_cast<float>(d.bcdMinusTick * 1000.0 / d.fs) : std::numeric_limits<float>::quiet_NaN();
    if (d.carrierFound) g.carrierOffsetHz = static_cast<float>(d.f0 - d.fNominal);

    g.framesInWindow = d.voter.frameCount();
    g.windowSize = d.voter.windowSize();
    g.voteQuality = d.voter.lockConfidence();
    g.refusalReason = static_cast<std::uint8_t>(d.voter.lockRefusal());
    return g;
}

} // namespace clockdec
