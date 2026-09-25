// AllouisDecoder.cpp — see AllouisDecoder.h for the format and the design.
//
// Chain, per input sample of complex baseband:
//
//   carrier search (once)  as Dcf77Decoder: DFT bins across +/-20 Hz, the peak
//                          refined; the mixer goes there and follows after.
//   phase                  y = Im(z conj(r)) / |r|^2, r a 1 Hz one-pole of z:
//                          the carrier's own recent phase as the reference, so
//                          y is sin(phi) of the phase modulation (Dcf77Decoder's
//                          PM, which the ramps here average out of r as the
//                          chips do there). Kept in a ring at full rate.
//
// Acquisition folds y, decimated to 1 kHz, over kAcqSeconds seconds and
// correlates the fold with the data excursion every second but one starts
// with: that finds the second, and which way round the phase reads.
//
// Then once per second, 0.9 s after its edge (everything it carries is in
// 0-900 ms, and its noise is read from the quiet 900-1000 ms of the second
// before): read the data excursions, name the second from its position code,
// time it by correlating the whole second's expected phase, sync, and at
// second 59 decode the minute.

#include "AllouisDecoder.h"

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
constexpr double kSubSec   = 0.025;   // one ramp
constexpr int    kCodeSubs = 32;      // 200-1000 ms
constexpr double kCodeStart = 0.200;
constexpr double kDataEnd  = 0.900;   // nothing is modulated after this
// The unmodulated stretch every second ends with, less 5 ms each side.
constexpr double kQuietFrom = 0.905;
constexpr double kQuietTo   = 0.995;
// Where the second is: 50.48 ms after the excursion starts -- close to its
// midpoint, the zero crossing between +1 and -1 rad, but not on it. Nothing
// published says where. Measured on M9PSY-1, where MSF, Allouis and DCF77 come
// off one RX888 and one set of radiod capture stamps, so every receiver term
// cancels and the paths are modelled as propagation alone:
//   against DCF77's phase modulation, recorded at the same moment (paths 1057
//   and 1061 km): the excursion started 50.47 ms before DCF77's second at
//   02:02 UTC 2026-09-25 (49.96 at 01:20, the one outlier), and with 50.00
//   taken, Allouis read +0.43 ms against DCF77 live over 14 minutes;
//   against a GPS-disciplined stratum 1 on the same host, live, capture-timed:
//   +0.480 ms over 15 settled minutes (halves +0.454, +0.503), DCF77 reading
//   -0.004 against the same reference over the same minutes.
// Night, all of it; to be confirmed by day. Everything this decoder works in
// (templates, the quiet window, the processing) is counted from the
// excursion's start; only what it reports is moved.
constexpr double kSecondAfterStartSec = 0.05048;

// The position codes, per henningM1r/gr_ALS162_Receiver (GPLv3),
// python/ALS162_codes.py: the phase's change over each 25 ms of 200-1000 ms,
// in radians. Indexed here by the second each is SENT in -- the model's key
// for second s is s + 1 -- and confirmed so on the air: in a recording from
// M9PSY-1 the code matched this way in 295 of 302 seconds.
constexpr std::array<std::array<int, kCodeSubs>, 60> kCodes = {{
    {{ 0,+1,-1, 0, 0, 0,+1,-1, 0, 0, 0, 0, 0, 0, 0,-1, 0,+2, 0,-2, 0,+2, 0,-2, 0,+1, 0, 0, 0, 0, 0, 0}},   // s00 (key "01")
    {{+1, 0,-2, 0,+2,-1,-1,+1,+1,-1,-1,+1, 0,+1, 0,-2,+1, 0, 0,+1,-2, 0,+1, 0,+1,-1,-1,+1, 0, 0, 0, 0}},   // s01 (key "02")
    {{+1,-1, 0, 0,-1,+2, 0,-1,-1, 0,+2,-1,-1,+2, 0,-1,-1, 0,+2,-1,-1,+2, 0,-1,-1, 0,+1, 0, 0, 0, 0, 0}},   // s02 (key "03")
    {{+1,-1,-1,+1,+1, 0,-1,-1,+1, 0,-1,+1, 0, 0, 0,+1, 0,-2,+1,+1,-1,-1, 0,+1,+1,-1,-1,+1, 0, 0, 0, 0}},   // s03 (key "04")
    {{+1,-1, 0,+1,-2, 0,+1, 0,+1, 0,-1,-1, 0,+2, 0,-1,-1, 0,+1,+1,-1,-1,+2,-1,-1,+1, 0, 0, 0, 0, 0, 0}},   // s04 (key "05")
    {{+1,-1,-1,+1,+1, 0,-1,-1, 0,+1, 0,+1, 0,-2, 0,+2, 0,-2,+1,+1,-1,-1, 0,+2,-1,-1,+1, 0, 0, 0, 0, 0}},   // s05 (key "06")
    {{ 0, 0, 0, 0,+1, 0,-1, 0,-1, 0,+1,+1, 0,-1,-1,+1, 0, 0, 0,-1,+1,+1, 0,-1,-1, 0,+1, 0, 0, 0, 0, 0}},   // s06 (key "07")
    {{ 0, 0,+1, 0,-1, 0, 0, 0, 0,-1, 0,+1, 0, 0, 0, 0, 0, 0, 0, 0, 0,+1,-1,-1,+2,-1,-1,+1, 0, 0, 0, 0}},   // s07 (key "08")
    {{+1,-1, 0,+1,-2,+1, 0,-1,+1,+1,-1,-1,+1,+1, 0,-1,-1, 0,+1,+1, 0,-1,-1, 0,+1, 0, 0, 0, 0, 0, 0, 0}},   // s08 (key "09")
    {{ 0,+1, 0,-1,-1, 0,+1,+1, 0,-1,-1, 0,+2,-1,-1,+1,+1, 0,-1,-1,+1,+1,-2, 0,+1, 0, 0, 0, 0, 0, 0, 0}},   // s09 (key "10")
    {{ 0,+1, 0,-1,-1,+1,+1,-2,+1, 0,-1,+1, 0,+1,-1, 0,+1,-1, 0,-1, 0,+2,-1, 0, 0,-1,+1, 0, 0, 0, 0, 0}},   // s10 (key "11")
    {{ 0,+1, 0,-1, 0, 0,-1, 0,+2,-1,-1,+1,+1, 0,-1, 0,-1, 0,+1, 0, 0, 0, 0, 0,+1,-1,-1,+1, 0, 0, 0, 0}},   // s11 (key "12")
    {{+1,-1,-1,+2, 0,-1,-1, 0,+1,+1,-1, 0,+1,-2, 0,+1,+1,-1,-1,+1, 0, 0,+1, 0,-1,-1, 0,+1, 0, 0, 0, 0}},   // s12 (key "13")
    {{+1, 0,-2,+1,+1,-2,+1, 0,-1,+2, 0,-2, 0,+1,+1,-1, 0, 0,-1,+1,+1,-1,-1,+2, 0,-2, 0,+1, 0, 0, 0, 0}},   // s13 (key "14")
    {{ 0,+1,-1,-1,+1,+1, 0,-1, 0,-1, 0,+2, 0,-1, 0,-1, 0,+2,-1, 0,+1,-2, 0,+2,-1,-1,+1, 0, 0, 0, 0, 0}},   // s14 (key "15")
    {{ 0, 0,+1, 0,-2,+1,+1,-1, 0, 0,-1,+1, 0,-1,+1, 0, 0, 0, 0,+1, 0,-2,+1,+1,-1,-1, 0,+1, 0, 0, 0, 0}},   // s15 (key "16")
    {{+1, 0,-2, 0,+2,-1, 0,+1,-1, 0, 0,-1,+1,+1,-1, 0, 0,-1,+1,+1,-2,+1,+1,-2, 0,+1, 0, 0, 0, 0, 0, 0}},   // s16 (key "17")
    {{+1,-1, 0,+1,-2, 0,+2,-1, 0,+1,-1, 0, 0,-1, 0,+2, 0,-1, 0,-1,+1, 0,-1,+1, 0, 0, 0, 0, 0, 0, 0, 0}},   // s17 (key "18")
    {{+1, 0,-1, 0,-1, 0,+1,+1,-1, 0,+1,-2, 0,+2,-1,-1,+1, 0, 0,+1,-1,-1,+1, 0, 0, 0, 0, 0, 0, 0, 0, 0}},   // s18 (key "19")
    {{+1,-1,-1,+1, 0,+1, 0,-2, 0,+1, 0,+1,-1, 0,+1,-2, 0,+2,-1, 0,+1,-2, 0,+2,-1,-1,+1, 0, 0, 0, 0, 0}},   // s19 (key "20")
    {{ 0, 0,+1,-1,-1,+2,-1, 0,+1,-1, 0,-1, 0,+1,+1,-1, 0,+1,-2,+1,+1,-1, 0,-1, 0,+1, 0, 0, 0, 0, 0, 0}},   // s20 (key "21")
    {{+1,-1,-1,+2,-1, 0, 0,-1,+1, 0,+1, 0,-1,-1, 0,+2, 0,-2, 0,+2, 0,-1, 0,-1,+1, 0,-1,+1, 0, 0, 0, 0}},   // s21 (key "22")
    {{ 0, 0, 0, 0,+1, 0,-1, 0, 0, 0, 0, 0,-1,+1,+1,-2,+1, 0, 0,+1,-1,-1, 0,+2, 0,-2, 0,+1, 0, 0, 0, 0}},   // s22 (key "23")
    {{ 0, 0, 0, 0,+1,-1, 0, 0, 0,+1,-2,+1, 0,-1,+2, 0,-1,-1,+1,+1,-2,+1,+1,-1, 0,-1, 0,+1, 0, 0, 0, 0}},   // s23 (key "24")
    {{+1, 0,-2, 0,+1, 0, 0, 0,+1, 0,-2, 0,+2, 0,-1, 0, 0,-1,+1, 0, 0,+1,-2,+1,+1,-2, 0,+1, 0, 0, 0, 0}},   // s24 (key "25")
    {{+1,-1, 0,+1,-2, 0,+2,-1, 0, 0, 0,+1,-2, 0,+2,-1,-1,+1, 0,+1,-1, 0,+1,-1,-1, 0,+1, 0, 0, 0, 0, 0}},   // s25 (key "26")
    {{+1, 0,-2, 0,+1,+1,-1,-1,+1, 0,+1, 0,-2, 0,+2, 0,-1,-1,+1, 0,-1,+1,+1, 0,-2, 0,+1, 0, 0, 0, 0, 0}},   // s26 (key "27")
    {{+1, 0,-1,-1,+1,+1,-2,+1, 0, 0,+1,-2, 0,+1, 0,+1,-1,-1,+2, 0,-1, 0,-1,+1,+1,-2, 0,+1, 0, 0, 0, 0}},   // s27 (key "28")
    {{ 0,+1,-1,-1,+2, 0,-2, 0,+2, 0,-1, 0, 0, 0, 0,-1, 0,+2,-1,-1,+1, 0,+1,-1,-1,+1, 0, 0, 0, 0, 0, 0}},   // s28 (key "29")
    {{+1, 0,-2, 0,+2,-1,-1,+1, 0,+1, 0,-2, 0,+2, 0,-2,+1, 0, 0,+1,-2, 0,+1,+1, 0,-2, 0,+1, 0, 0, 0, 0}},   // s29 (key "30")
    {{ 0,+1,-1, 0,+1,-2, 0,+1,+1, 0,-2,+1,+1,-1, 0,-1, 0,+2, 0,-1,-1, 0,+2,-1,-1,+1, 0, 0, 0, 0, 0, 0}},   // s30 (key "31")
    {{ 0, 0,+1, 0,-2,+1,+1,-1,-1, 0,+1,+1, 0,-2, 0,+1, 0, 0, 0,+1, 0,-2,+1, 0, 0, 0,-1,+1, 0, 0, 0, 0}},   // s31 (key "32")
    {{+1, 0,-1, 0,-1,+1, 0,-1,+1, 0, 0,+1,-1, 0,+1,-1, 0, 0, 0,-1, 0,+1,+1, 0,-2, 0,+1, 0, 0, 0, 0, 0}},   // s32 (key "33")
    {{ 0, 0, 0, 0,+1, 0,-2, 0,+2,-1,-1,+2, 0,-1, 0, 0, 0,-1,+1,+1,-1, 0, 0, 0,-1, 0,+1, 0, 0, 0, 0, 0}},   // s33 (key "34")
    {{ 0, 0, 0,+1, 0,-1, 0, 0,-1, 0,+1, 0,+1, 0,-2,+1, 0, 0,+1,-1, 0,-1, 0,+1,+1,-1,-1,+1, 0, 0, 0, 0}},   // s34 (key "35")
    {{+1,-1, 0,+1,-1, 0, 0, 0,-1, 0,+2,-1, 0, 0, 0, 0,-1,+1, 0, 0, 0,+1,-1, 0, 0,-1,+1, 0, 0, 0, 0, 0}},   // s35 (key "36")
    {{+1,-1,-1,+1, 0,+1,-1,-1,+1,+1,-1,-1,+1, 0,+1, 0,-2,+1, 0, 0,+1,-1,-1, 0,+1, 0, 0, 0, 0, 0, 0, 0}},   // s36 (key "37")
    {{ 0,+1, 0,-2,+1,+1,-1, 0, 0, 0,-1, 0,+2,-1,-1,+1,+1, 0,-2, 0,+1,+1,-1,-1,+1, 0, 0, 0, 0, 0, 0, 0}},   // s37 (key "38")
    {{ 0,+1, 0,-1,-1,+1,+1,-2, 0,+2,-1,-1,+1,+1,-1, 0,+1,-1, 0,-1, 0,+2,-1,-1,+1, 0, 0, 0, 0, 0, 0, 0}},   // s38 (key "39")
    {{ 0,+1, 0,-1, 0, 0,-1, 0,+2,-1,-1,+1,+1,-1,-1,+1,+1, 0,-1, 0, 0, 0, 0, 0,-1, 0,+1, 0, 0, 0, 0, 0}},   // s39 (key "40")
    {{ 0,+1, 0,-1,-1,+1,+1,-2,+1,+1,-1, 0,-1, 0,+2,-1,-1,+1, 0,+1, 0,-2,+1, 0,-1,+1, 0, 0, 0, 0, 0, 0}},   // s40 (key "41")
    {{+1, 0,-1,-1,+1, 0,-1,+2, 0,-1,-1,+1,+1,-2, 0,+2,-1,-1,+2, 0,-2, 0,+2, 0,-1,-1, 0,+1, 0, 0, 0, 0}},   // s41 (key "42")
    {{+1, 0,-1, 0,-1, 0,+2, 0,-2, 0,+2,-1, 0,+1,-1, 0, 0, 0, 0,-1,+1, 0,-1,+1,+1,-1,-1,+1, 0, 0, 0, 0}},   // s42 (key "43")
    {{ 0,+1,-1,-1,+2, 0,-2, 0,+2,-1, 0, 0, 0, 0, 0,+1,-2,+1, 0,-1,+1, 0, 0, 0,+1,-1,-1,+1, 0, 0, 0, 0}},   // s43 (key "44")
    {{ 0,+1, 0,-1, 0, 0,-1,+1,+1,-2,+1, 0,-1,+2, 0,-2, 0,+1, 0, 0, 0, 0, 0,+1,-1,-1,+1, 0, 0, 0, 0, 0}},   // s44 (key "45")
    {{+1,-1,-1,+1,+1,-1,-1,+2, 0,-2, 0,+2,-1,-1,+1,+1, 0,-2,+1,+1,-2, 0,+2, 0,-2, 0,+1, 0, 0, 0, 0, 0}},   // s45 (key "46")
    {{ 0, 0,+1, 0,-2, 0,+1, 0,+1, 0,-2,+1, 0,-1,+2,-1, 0,+1,-1,-1,+1,+1,-2,+1,+1,-2, 0,+1, 0, 0, 0, 0}},   // s46 (key "47")
    {{+1, 0,-1,-1,+1, 0, 0,+1,-2,+1, 0,-1,+2, 0,-2,+1, 0,-1,+2, 0,-2,+1, 0, 0,+1,-2, 0,+1, 0, 0, 0, 0}},   // s47 (key "48")
    {{+1,-1,-1,+2,-1,-1,+2, 0,-2, 0,+2, 0,-1, 0, 0, 0,-1,+1,+1,-2,+1, 0,-1,+1,+1,-1,-1,+1, 0, 0, 0, 0}},   // s48 (key "49")
    {{ 0, 0,+1, 0,-2, 0,+1, 0,+1,-1, 0, 0, 0, 0, 0,+1,-1, 0, 0,-1,+1,+1,-1, 0,-1, 0,+1, 0, 0, 0, 0, 0}},   // s49 (key "50")
    {{+1,-1,-1,+1, 0, 0, 0,+1,-1, 0,+1,-1,-1, 0,+2, 0,-2,+1, 0, 0, 0,-1,+1, 0, 0, 0, 0, 0, 0, 0, 0, 0}},   // s50 (key "51")
    {{+1, 0,-1,-1,+1,+1,-1, 0, 0, 0, 0,-1,+1, 0,-1,+2,-1,-1,+2, 0,-1,-1,+1,+1,-1,-1, 0,+1, 0, 0, 0, 0}},   // s51 (key "52")
    {{ 0,+1, 0,-1,-1, 0,+2, 0,-2, 0,+1,+1,-1,-1,+1,+1, 0,-1, 0,-1,+1, 0,-1,+1,+1,-1,-1,+1, 0, 0, 0, 0}},   // s52 (key "53")
    {{+1, 0,-1,-1, 0,+1,+1, 0,-2,+1, 0, 0, 0, 0, 0, 0, 0,-1,+2,-1, 0, 0, 0, 0,-1,+1, 0, 0, 0, 0, 0, 0}},   // s53 (key "54")
    {{ 0, 0,+1,-1, 0,+1,-2, 0,+2,-1,-1,+2,-1, 0, 0, 0, 0,-1,+2, 0,-1, 0,-1,+1, 0,-1,+1, 0, 0, 0, 0, 0}},   // s54 (key "55")
    {{ 0, 0, 0, 0, 0, 0, 0,+1,-1,-1,+2, 0,-2, 0,+1, 0, 0,+1,-1, 0, 0,-1,+2,-1, 0, 0,-1,+1, 0, 0, 0, 0}},   // s55 (key "56")
    {{ 0, 0,+1, 0,-2,+1, 0, 0,+1,-1,-1,+1, 0, 0,+1,-2,+1,+1,-1,-1, 0,+1, 0,+1, 0,-2, 0,+1, 0, 0, 0, 0}},   // s56 (key "57")
    {{+1, 0,-1, 0,-1,+1, 0,-1,+2,-1,-1,+2,-1,-1,+2,-1, 0,+1,-1,-1, 0,+1, 0, 0,+1,-1,-1,+1, 0, 0, 0, 0}},   // s57 (key "58")
    {{+1, 0,-2, 0,+2, 0,-2, 0,+2, 0,-2, 0,+2, 0,-2, 0,+2, 0,-2, 0,+1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}},   // s58 (key "59")
    {{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}},   // s59 (key "00")
}};

// ---- carrier search (as Dcf77Decoder) ------------------------------------
constexpr double kAcqCarrierSec = 2.0;
constexpr double kSearchHz    = 20.0;
constexpr double kPullHz      = 3.0;   // 20 ppm is 3.2 Hz at 162 kHz
constexpr double kSearchStep  = 0.25;
constexpr double kToneGate    = 12.0;
constexpr double kLiveToneSeconds = 2.0;
constexpr int    kLiveToneSide    = 16;
constexpr double kLiveToneFirstHz = 5.5;

// ---- carrier reference ---------------------------------------------------
constexpr double kRefHz    = 1.0;
constexpr double kFreqGain = 0.2;
// The carrier counts as gone below this fraction of its usual level, and the
// mixer is not followed then (feedSteady).
constexpr double kCarrierGone = 1.0 / 3.0;

// ---- acquisition ---------------------------------------------------------
constexpr int    kFoldRateHz  = 1000;
constexpr int    kAcqSeconds  = 6;     // seconds folded before the excursion is looked for
// The fold's peak over the best peak elsewhere, and folds running that must
// agree on where it is (foldStep).
constexpr double kAcqRatio    = 1.25;
constexpr int    kAcqAgree    = 2;
// Seconds running, carrier present, in which no second has named itself by
// its position code: the phase is wrong, and the second is looked for again.
// At the lock limit (29 dB-Hz) a code still scores about 19 sigma.
constexpr int    kMaxUnnamedSeconds = 30;

// ---- timing --------------------------------------------------------------
// Lags searched either side of the tracker's prediction, while it settles and
// once it has.
constexpr double kSearchSec     = 0.004;
constexpr double kSearchWarmSec = 0.0015;
// A second's correlation must stand this far above its own noise to time it.
constexpr double kTimeMinSig    = 5.0;

// Edge tracker: as MsfDecoder's (its gain follows its own residuals).
constexpr double kTrkAlpha       = 1.0 / 8.0;
constexpr double kTrkAlphaMin    = 1.0 / 32.0;
constexpr double kTrkNoiseRefSec = 0.0001;
constexpr int    kTrkWarm        = 8;
constexpr double kTrkOutlierSec  = 0.002;
constexpr double kTrkAcqOutlierSec = 0.010;

// Seconds with nothing timed before the second is looked for again.
constexpr int kMaxBlindSeconds = 60;

// ---- symbols & sync ------------------------------------------------------
// Confidences are z-scores mapped (z - 1)/5, as the other decoders'.
constexpr float kStructConf  = 0.50f;
constexpr float kBitMinConf  = 0.05f;
// A second is named by its position code when the best code scores this many
// sigma, and beats the next best by kKeyMarginSig.
constexpr double kKeyMinSig    = 6.0;
constexpr double kKeyMarginSig = 4.0;
constexpr int kMaxUnconfirmedMinutes = 2;
// Unread bits a minute may have and still be decoded, filled in by its checks.
constexpr int kMaxErasures = 2;
// The excursion's amplitude, learnt (processSecond): the fraction taken each second.
constexpr double kAmpGain = 0.05;
// A minute is confirmed when this many of its 59 coded seconds named
// themselves as expected and none clearly named another.
constexpr int kConfirmMatches = 30;

// WWVB's layout, for the synthetic frames the voter is fed (finalizeFrame).
constexpr std::array<int, 7> kVoterMarkers = {0, 9, 19, 29, 39, 49, 59};
const ClockFieldMap kVMin  = {{1, 40}, {2, 20}, {3, 10}, {5, 8}, {6, 4}, {7, 2}, {8, 1}};
const ClockFieldMap kVHour = {{12, 20}, {13, 10}, {15, 8}, {16, 4}, {17, 2}, {18, 1}};
const ClockFieldMap kVDoy  = {{22, 200}, {23, 100}, {25, 80}, {26, 40}, {27, 20},
                              {28, 10}, {30, 8}, {31, 4}, {32, 2}, {33, 1}};
const ClockFieldMap kVYear = {{45, 80}, {46, 40}, {47, 20}, {48, 10},
                              {50, 8}, {51, 4}, {52, 2}, {53, 1}};

// The data excursion's phase at t seconds into it (0 outside 0-100 ms).
double excursion(double t) {
    if (t < 0.0 || t >= 4 * kSubSec) return 0.0;
    const double u = t / kSubSec;             // 0..4
    if (u < 1.0) return u;                     // 0 -> +1
    if (u < 3.0) return 2.0 - u;               // +1 -> -1
    return u - 4.0;                            // -1 -> 0
}

// Second s's position code's phase at t seconds into the second.
double codePhase(int s, double t) {
    if (t < kCodeStart || t >= kCodeStart + kCodeSubs * kSubSec) return 0.0;
    const double u = (t - kCodeStart) / kSubSec;
    const int k = static_cast<int>(u);
    double ph = 0.0;
    const auto& c = kCodes[static_cast<std::size_t>(s)];
    for (int i = 0; i < k; ++i) ph += c[static_cast<std::size_t>(i)];
    return ph + c[static_cast<std::size_t>(k)] * (u - k);
}

struct EdgeTracker {
    bool valid = false;
    double edge = 0.0, period = 0.0;
    int count = 0, outliers = 0;
    double resVar = 0.0;
    double noiseRef = 1.0;

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
        if (count > kTrkWarm)
            period = std::clamp(period + 0.5 * a * a * r, nominal * (1.0 - 2e-4), nominal * (1.0 + 2e-4));
    }
};

// One second, read.
struct SecRec {
    int64_t edge = 0;
    double edgeExact = std::numeric_limits<double>::quiet_NaN();
    bool read = false;
    bool marker = false;        // no data excursion: second 59
    float markConf = 0.0f;
    int bit = 0;
    float bitConf = 0.0f;
    int key = -1;               // the second its position code names, -1 if none clearly
    double keySig = 0.0;        // that code's score, sigma
    double expSig = 0.0;        // the expected second's code's score, sigma (anchored)
    double timeSig = 0.0;       // the timing correlation's SNR
};

struct Decoded {
    bool ok = false;
    int minute = 0, hour = 0, day = 0, month = 0, year2 = 0;
    bool cest = false, leapWarn = false;
    TimeFields utc;
    long long utcMs = 0;
};

} // namespace

// ---------------------------------------------------------------------------

struct AllouisDecoder::Impl {
    Impl(AllouisDecoder* o, int sampleRateHz, double carrierOffsetHz)
        : owner(o),
          sr(sampleRateHz > 0 ? sampleRateHz : 12000),
          fNominal(carrierOffsetHz),
          voter(buildVoterConfig()) {
        acqTarget = static_cast<int64_t>(std::llround(kAcqCarrierSec * sr));
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
        while (cap < static_cast<std::size_t>(4 * sr)) cap <<= 1;
        yRing.assign(cap, 0.0f);
        zRe.assign(cap, 0.0f);
        zIm.assign(cap, 0.0f);
        yMask = static_cast<int64_t>(cap) - 1;

        // Templates at the input rate: sin of the phase, as y is. Sample n of
        // a template is the instant n / sr into its span.
        nData = static_cast<int>(std::llround(4 * kSubSec * sr));
        tri.resize(static_cast<std::size_t>(nData));
        for (int n = 0; n < nData; ++n) tri[static_cast<std::size_t>(n)] = static_cast<float>(std::sin(excursion(static_cast<double>(n) / sr)));
        triE = 0.0;
        for (float v : tri) triE += static_cast<double>(v) * v;
        codeOff = static_cast<int>(std::llround(kCodeStart * sr));
        nCode = static_cast<int>(std::llround(kCodeSubs * kSubSec * sr));
        for (int s = 0; s < 60; ++s) {
            auto& c = codeT[static_cast<std::size_t>(s)];
            c.resize(static_cast<std::size_t>(nCode));
            double e = 0.0;
            for (int n = 0; n < nCode; ++n) {
                const double v = std::sin(codePhase(s, kCodeStart + static_cast<double>(n) / sr));
                c[static_cast<std::size_t>(n)] = static_cast<float>(v);
                e += v * v;
            }
            codeE[static_cast<std::size_t>(s)] = e;
        }

        // The fold, at 1 kHz: one excursion's template there, bin centres.
        foldDecim = std::max(1, static_cast<int>(std::lround(static_cast<double>(sr) / kFoldRateHz)));
        foldLen = static_cast<int>(std::lround(static_cast<double>(sr) / foldDecim));
        const int nTriD = static_cast<int>(std::llround(4 * kSubSec * sr / foldDecim));
        triD.resize(static_cast<std::size_t>(nTriD));
        for (int i = 0; i < nTriD; ++i)
            triD[static_cast<std::size_t>(i)] = std::sin(excursion((i + 0.5) * foldDecim / static_cast<double>(sr)));
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
        refRe = refIm = 0.0; freqCount = 0; freqPrevRe = freqPrevIm = 0.0; carrierLevel = 0.0;
        std::fill(yRing.begin(), yRing.end(), 0.0f);
        yStart = 0;
        restartAcquisition();
        polarity = 1;
        amp = 1.0; ampHave = false;
        lastTimeSig = 0.0;
        hist.clear();
        dropAnchor();
        haveLastFrame = false; lastFrameStart = 0;
        lastFrameFrom = 0;
        samplesConsumed = 0;
        setLockState(ClockLockState::NoSignal);
    }

    void restartAcquisition() {
        segValid = false;
        segEdge = 0.0;
        trk.reset();
        blindSeconds = 0;
        foldAcc = 0.0; foldCount = 0; foldPos = 0;
        foldSlots.assign(static_cast<std::size_t>(kAcqSeconds), std::vector<double>(static_cast<std::size_t>(foldLen), 0.0));
        foldCur.assign(static_cast<std::size_t>(foldLen), 0.0);
        foldSlotsN = 0; foldSlotNext = 0;
        foldBlockStart = -1;
        acqAgree = 0; acqLast = -1; acqSign = 1;
        foldFresh = 0;
        unnamedRun = 0;
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
        if (++acqCount >= acqTarget) finalizeCarrierSearch();
    }

    void finalizeCarrierSearch() {
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
            setMixer(f);
            phase = Phase::Running;
            yStart = samplesConsumed;
            aRef = 1.0 - std::exp(-2.0 * kPi * kRefHz / sr);
            trk.noiseRef = kTrkNoiseRefSec * sr;
            setLockState(ClockLockState::Acquiring);
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
        const int64_t k = samplesConsumed - 1;
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

        refRe += aRef * (zr - refRe);
        refIm += aRef * (zi - refIm);
        const double rp = refRe * refRe + refIm * refIm;
        const double y = rp > 1e-30 ? polarity * (zi * refRe - zr * refIm) / rp : 0.0;
        yRing[static_cast<std::size_t>(k & yMask)] = static_cast<float>(y);
        zRe[static_cast<std::size_t>(k & yMask)] = static_cast<float>(zr);
        zIm[static_cast<std::size_t>(k & yMask)] = static_cast<float>(zi);

        // Follow the carrier: a residual offset shows as the reference turning.
        // Not while it is gone -- Allouis is off air every Tuesday morning,
        // and a reference that is only noise turns at random: followed, it
        // walked the mixer to the edge of its range, where the carrier, when
        // it came back, was too far off for the reference to hold and was
        // never found again. So the carrier's level is watched, and the
        // mixer held while it is under a third of what it has been.
        if (++freqCount >= sr) {
            freqCount = 0;
            const double level = std::hypot(refRe, refIm);
            const bool present = !(carrierLevel > 0.0) || level >= kCarrierGone * carrierLevel;
            carrierPresent = present;
            if (present) carrierLevel = carrierLevel > 0.0 ? carrierLevel + 0.1 * (level - carrierLevel) : level;
            if (present && (freqPrevRe != 0.0 || freqPrevIm != 0.0)) {
                const double cr = refRe * freqPrevRe + refIm * freqPrevIm;
                const double ci = refIm * freqPrevRe - refRe * freqPrevIm;
                setMixer(f0 + kFreqGain * std::atan2(ci, cr) / (2.0 * kPi));
            }
            freqPrevRe = refRe; freqPrevIm = refIm;
        }

        if (!segValid) foldStep(k, y);
        while (segValid && canProcessSecond()) processSecond();
    }

    float yAt(int64_t n) const { return yRing[static_cast<std::size_t>(n & yMask)]; }
    int64_t oldest() const { return std::max(yStart, samplesConsumed - static_cast<int64_t>(yRing.size()) + 1); }

    // ---- acquisition: the fold -------------------------------------------

    void foldStep(int64_t k, double y) {
        if (foldBlockStart < 0) { foldBlockStart = k; foldPos = 0; foldAcc = 0.0; foldCount = 0; }
        foldAcc += y;
        if (++foldCount < foldDecim) return;
        foldCur[static_cast<std::size_t>(foldPos)] = foldAcc / foldCount;
        foldAcc = 0.0; foldCount = 0;
        if (++foldPos < foldLen) return;
        // A second of 1 kHz phase, filed; look once enough are in.
        foldSlots[static_cast<std::size_t>(foldSlotNext)] = foldCur;
        foldSlotNext = (foldSlotNext + 1) % kAcqSeconds;
        foldSlotsN = std::min(foldSlotsN + 1, kAcqSeconds);
        const int64_t blockStart = foldBlockStart;
        foldBlockStart = k + 1;
        foldPos = 0;
        // Only a fold of kAcqSeconds seconds none of which an earlier fold
        // used: see the agreement test below.
        if (++foldFresh < kAcqSeconds || foldSlotsN < kAcqSeconds) return;
        foldFresh = 0;
        // Nor while the carrier is away: a fold of noise has a peak somewhere.
        if (!carrierPresent) { acqAgree = 0; return; }

        std::vector<double> S(static_cast<std::size_t>(foldLen), 0.0);
        for (const auto& sl : foldSlots)
            for (int i = 0; i < foldLen; ++i) S[static_cast<std::size_t>(i)] += sl[static_cast<std::size_t>(i)];
        std::vector<double> c(static_cast<std::size_t>(foldLen));
        for (int l = 0; l < foldLen; ++l) {
            double v = 0.0;
            for (std::size_t i = 0; i < triD.size(); ++i) v += S[static_cast<std::size_t>((l + static_cast<int>(i)) % foldLen)] * triD[i];
            c[static_cast<std::size_t>(l)] = v;
        }
        int best = 0;
        for (int l = 1; l < foldLen; ++l) if (std::fabs(c[static_cast<std::size_t>(l)]) > std::fabs(c[static_cast<std::size_t>(best)])) best = l;
        // Is it the excursion? Not a threshold on the peak: the position codes
        // differ every second and leave structure at every lag of the fold,
        // so even a clean signal's peak is only three or four robust sigma up
        // (measured, synthetic and on the air). But the excursion is at the
        // same lag in every fold and that structure is not: the peak must
        // clear the best of the rest -- more than 60 ms from it -- by
        // kAcqRatio, and do so at the same lag, to 2 ms, kAcqAgree folds
        // running, each of seconds the others did not use. Sliding folds that
        // shared five of their six seconds agreed with each other on noise
        // alone -- they were nearly the same fold -- and acquired during a
        // carrier outage, at a lag the carrier did not have when it returned.
        double rest = 0.0;
        for (int l = 0; l < foldLen; ++l) {
            int dd = std::abs(l - best);
            dd = std::min(dd, foldLen - dd);
            if (dd > kFoldRateHz * 6 / 100) rest = std::max(rest, std::fabs(c[static_cast<std::size_t>(l)]));
        }
        const double ratio = rest > 0.0 ? std::fabs(c[static_cast<std::size_t>(best)]) / rest : 0.0;
        lastTimeSig = ratio;
        const int sign = c[static_cast<std::size_t>(best)] >= 0.0 ? 1 : -1;
        if (ratio < kAcqRatio) { acqAgree = 0; return; }
        int dd = std::abs(best - acqLast);
        dd = std::min(dd, foldLen - dd);
        if (acqAgree > 0 && dd <= 2 * kFoldRateHz / 1000 && sign == acqSign) ++acqAgree;
        else acqAgree = 1;
        acqLast = best;
        acqSign = sign;
        if (acqAgree < kAcqAgree) return;
        acqAgree = 0;

        // Found. Which way round the phase reads, and the excursion's start
        // in the block just folded; the per-second correlation times it.
        if (c[static_cast<std::size_t>(best)] < 0.0) {
            polarity = -polarity;
            // y already in the ring was stored the other way round.
            for (int64_t n = oldest(); n <= k; ++n) yRing[static_cast<std::size_t>(n & yMask)] = -yAt(n);
        }
        segValid = true;
        segEdge = static_cast<double>(blockStart) + static_cast<double>(best) * foldDecim - 0.5;
        // The oldest whole second still in the ring, so that nothing is lost.
        while (segEdge - 0.1 * sr - static_cast<double>(sr) > static_cast<double>(oldest())) segEdge -= sr;
        while (segEdge - 0.1 * sr < static_cast<double>(oldest())) segEdge += sr;
        trk.reset();
        blindSeconds = 0;
    }

    // ---- once per second -------------------------------------------------

    // A second is processed once its own quiet 900-1000 ms is in: its phase is
    // measured against the carrier there (buildSecondPhase).
    bool canProcessSecond() const {
        return static_cast<double>(samplesConsumed) >= segEdge + static_cast<double>(codeOff + nCode) + kSearchSec * sr + 3.0;
    }

    // The phase of the second at `c`, as y = sin(phi) (Im of z against the
    // carrier, over its amplitude), into yLoc. The carrier's phase comes from
    // the unmodulated 900-1000 ms either side -- the second before's and this
    // one's -- joined by a straight line. Not from a running reference: a
    // one-pole of the carrier follows the modulation a little, which is a
    // high-pass on the phase, and its phase lead put every edge 1.45 ms early
    // (measured in allouistest). Here nothing the modulation does reaches the
    // reference at all. Returns false when the stretch is not in the ring;
    // sigmaY is y's noise, per sample, from the two quiet windows.
    bool buildSecondPhase(int64_t c, int64_t L, double& sigmaY) {
        const int64_t a0 = c - static_cast<int64_t>(std::llround((1.0 - kQuietFrom) * sr));
        const int64_t a1 = c - static_cast<int64_t>(std::llround((1.0 - kQuietTo) * sr));
        const int64_t b0 = c + static_cast<int64_t>(std::llround(kQuietFrom * sr));
        const int64_t b1 = c + static_cast<int64_t>(std::llround(kQuietTo * sr));
        // Everything a correlation at up to L lags either way can read: the
        // position code runs to the second's end.
        const int64_t lo = std::min(a0, c - L), hi = std::max(b1, c + codeOff + nCode + L + 1);
        if (lo < oldest() || hi > samplesConsumed - 1) return false;
        auto sumZ = [&](int64_t from, int64_t to, double& re, double& im) {
            re = im = 0.0;
            for (int64_t n = from; n <= to; ++n) {
                const std::size_t k = static_cast<std::size_t>(n & yMask);
                re += zRe[k]; im += zIm[k];
            }
        };
        double ar, ai, br, bi;
        sumZ(a0, a1, ar, ai);
        sumZ(b0, b1, br, bi);
        const double na = static_cast<double>(a1 - a0 + 1), nb = static_cast<double>(b1 - b0 + 1);
        const double A = 0.5 * (std::hypot(ar, ai) / na + std::hypot(br, bi) / nb);
        if (!(A > 0.0)) return false;
        const double tA = std::atan2(ai, ar);
        const double d = std::remainder(std::atan2(bi, br) - tA, 2.0 * kPi);
        const double ca = 0.5 * static_cast<double>(a0 + a1), cb = 0.5 * static_cast<double>(b0 + b1);
        const double slope = d / (cb - ca);
        yLocBase = lo;
        yLoc.resize(static_cast<std::size_t>(hi - lo + 1));
        for (int64_t n = lo; n <= hi; ++n) {
            const double th = tA + slope * (static_cast<double>(n) - ca);
            const std::size_t k = static_cast<std::size_t>(n & yMask);
            const double y = (zIm[k] * std::cos(th) - zRe[k] * std::sin(th)) / A;
            yLoc[static_cast<std::size_t>(n - lo)] = static_cast<float>(polarity * y);
        }
        double m2 = 0.0;
        for (int64_t n = a0; n <= a1; ++n) { const double v = yLoc[static_cast<std::size_t>(n - lo)]; m2 += v * v; }
        for (int64_t n = b0; n <= b1; ++n) { const double v = yLoc[static_cast<std::size_t>(n - lo)]; m2 += v * v; }
        sigmaY = std::sqrt(std::max(1e-30, m2 / (na + nb)));
        return true;
    }

    // <y, T> with T's sample 0 at instant `at` (integer), on this second's y.
    double dot(int64_t at, const std::vector<float>& T, double /*dc*/) const {
        double s = 0.0;
        const float* y = yLoc.data() + (at - yLocBase);
        for (std::size_t i = 0; i < T.size(); ++i) s += static_cast<double>(y[i]) * T[i];
        return s;
    }

    // The second at `pred` with symbol and code as given, correlated at lags
    // either side; the peak refined by a parabola. Returns the SNR at the peak.
    double timeSecond(double pred, int sof, int bit, double dc, double sigmaY, double searchSec, double& edgeOut) const {
        const int64_t c = std::llround(pred);
        const int64_t L = std::max<int64_t>(2, std::llround(searchSec * sr));
        const bool hasTri = sof != 59;
        const bool hasCode = sof >= 0 && sof != 59;
        double E = 0.0;
        if (hasTri) E += triE * (bit == 1 ? 2.0 : 1.0);
        if (hasCode) E += codeE[static_cast<std::size_t>(sof)];
        if (!(E > 0.0) || !(sigmaY > 0.0)) return 0.0;
        std::vector<double> y(static_cast<std::size_t>(2 * L + 1));
        std::size_t bi = 0;
        for (int64_t j = -L; j <= L; ++j) {
            double v = 0.0;
            if (hasTri) {
                v += dot(c + j, tri, dc);
                if (bit == 1) v += dot(c + j + nData, tri, dc);
            }
            if (hasCode) v += dot(c + j + codeOff, codeT[static_cast<std::size_t>(sof)], dc);
            y[static_cast<std::size_t>(j + L)] = v;
            if (v > y[bi]) bi = static_cast<std::size_t>(j + L);
        }
        double frac = 0.0;
        if (bi > 0 && bi + 1 < y.size()) {
            const double den = y[bi - 1] - 2.0 * y[bi] + y[bi + 1];
            if (den < 0.0) frac = std::clamp(0.5 * (y[bi - 1] - y[bi + 1]) / den, -0.5, 0.5);
        } else {
            return 0.0;   // on the window's edge: not a peak
        }
        edgeOut = static_cast<double>(c - L + static_cast<int64_t>(bi)) + frac;
        return y[bi] / (sigmaY * std::sqrt(E));
    }

    void processSecond() {
        const double pred = segEdge;
        const int64_t c = std::llround(pred);
        SecRec r;

        const double dc = 0.0;
        double sigmaY = 0.0;
        if (!buildSecondPhase(c, std::llround(kSearchSec * sr) + 1, sigmaY)) { segEdge += sr; return; }

        // The data excursions, at the prediction.
        const double sTri = sigmaY * std::sqrt(triE);
        const double a1 = dot(c, tri, dc) / triE;
        const double a2 = dot(c + nData, tri, dc) / triE;
        const double sA = sTri / triE;
        auto confOf = [](double z) { return static_cast<float>(std::clamp((z - 1.0) / 5.0, 0.0, 1.0)); };
        const double A = ampHave ? amp : std::max(a1, 0.0);
        r.read = A > 0.0;
        r.marker = a1 < 0.5 * A;
        r.markConf = confOf(std::fabs(a1 - 0.5 * A) / sA);
        r.bit = a2 > 0.5 * A ? 1 : 0;
        r.bitConf = r.marker ? 0.0f : confOf(std::fabs(a2 - 0.5 * A) / sA);


        // Which second this is, by its position code.
        std::array<double, 60> score{};
        int best = -1, second = -1;
        for (int s = 0; s < 59; ++s) {
            const double sc = dot(c + codeOff, codeT[static_cast<std::size_t>(s)], dc) /
                              (sigmaY * std::sqrt(codeE[static_cast<std::size_t>(s)]));
            score[static_cast<std::size_t>(s)] = sc;
            if (best < 0 || sc > score[static_cast<std::size_t>(best)]) { second = best; best = s; }
            else if (second < 0 || sc > score[static_cast<std::size_t>(second)]) second = s;
        }
        if (best >= 0 && second >= 0 && score[static_cast<std::size_t>(best)] >= kKeyMinSig &&
            score[static_cast<std::size_t>(best)] - score[static_cast<std::size_t>(second)] >= kKeyMarginSig) {
            r.key = best;
            r.keySig = score[static_cast<std::size_t>(best)];
        }
        const int expect = anchored ? sofNext : -1;
        if (expect >= 0 && expect < 59) r.expSig = score[static_cast<std::size_t>(expect)];
        // The excursion's amplitude, learnt from every second known to carry
        // one -- counted into the minute and not second 59, or named by its
        // own position code -- whatever this second's reading. Learnt only
        // from seconds already READ as carrying one, it came out 15% high at
        // 30 dB-Hz: the low readings were the ones left out. With the
        // threshold at half of it, readings at the low edge of the noise then
        // fell below and were taken for second 59.
        if ((expect >= 0 && expect < 59) || r.key >= 0) {
            amp = ampHave ? amp + kAmpGain * (a1 - amp) : a1;
            ampHave = true;
        }

        // Timed by all of it: the excursion(s) and the code of the second it
        // is taken to be. A second 59 has nothing to time it by.
        const int sofT = expect >= 0 ? expect : r.key >= 0 ? r.key : (r.marker && r.markConf >= kStructConf ? 59 : -1);
        const bool warm = trk.valid && trk.count > kTrkWarm;
        double edgeMeas = pred;
        double tsig = 0.0;
        if (sofT != 59 && (!r.marker || r.markConf < kStructConf))
            tsig = timeSecond(pred, sofT, r.bitConf >= kStructConf ? r.bit : 0, dc, sigmaY,
                              warm ? kSearchWarmSec : kSearchSec, edgeMeas);
        const bool meas = tsig >= kTimeMinSig;
        r.timeSig = tsig;
        if (meas) lastTimeSig = tsig;
        trk.update(meas, edgeMeas, trk.valid ? trk.period : static_cast<double>(sr),
                   (warm ? kTrkOutlierSec : kTrkAcqOutlierSec) * sr);
        const double edge = trk.valid ? trk.edge : pred;
        blindSeconds = meas ? 0 : blindSeconds + 1;
        timing = trk.valid && meas;

        // What is reported is the second, not the excursion's start.
        r.edgeExact = edge + kSecondAfterStartSec * sr;
        r.edge = static_cast<int64_t>(std::llround(r.edgeExact));
        hist.push_back(r);
        if (hist.size() > 8) hist.erase(hist.begin());

        handleSecond(r, meas, c, dc);

        segEdge = trk.valid ? trk.edge + trk.period : pred + sr;

        // With the carrier there, a second at the right phase names itself;
        // thirty that do not, running, and the phase is not right.
        if (r.key >= 0 || !carrierPresent || (anchored && sofNext == 0)) unnamedRun = 0;
        else ++unnamedRun;

        if (blindSeconds >= kMaxBlindSeconds || unnamedRun >= kMaxUnnamedSeconds) {
            if (lockState == ClockLockState::Locked) setLockState(ClockLockState::Acquiring);
            dropAnchor();
            hist.clear();
            restartAcquisition();
        }
    }

    // ---- sync & frames ---------------------------------------------------

    void dropAnchor() {
        anchored = false;
        sofNext = 0;
        haveVoted = false;
        for (auto& f : frame) f = SecRec{};
        frFilled = 0;
        frStart = 0;
        frMatches = frMismatches = 0;
        slipRun = 0;
        unconfirmedRun = 0;
        lastLeapWarn = false;
    }

    void demote() {
        haveVoted = false;
        if (lockState == ClockLockState::Locked) setLockState(ClockLockState::Acquiring);
    }

    // Anchor so that the second just read is `sof`.
    void anchorAt(int sof, const SecRec& r) {
        anchored = true;
        frMatches = frMismatches = 0;
        slipRun = 0;
        for (auto& f : frame) f = SecRec{};
        frFilled = 0;
        if (sof == 0) frStart = r.edge;
        sofNext = sof;
    }

    void handleSecond(const SecRec& r, bool measured, int64_t c, double dc) {
        int sof = -1;
        if (!anchored) {
            // Two seconds in a row that name themselves, one after the other.
            if (hist.size() >= 2) {
                const SecRec& p = hist[hist.size() - 2];
                if (r.key >= 0 && p.key >= 0 && r.key == p.key + 1) anchorAt(r.key, r);
                else if (r.key == 0 && p.marker && p.markConf >= kStructConf) anchorAt(0, r);
            }
        }
        if (anchored) {
            sof = sofNext;
            if (sof == 0) { frStart = r.edge; frFilled = 0; frMatches = frMismatches = 0; }
            // Does the second name itself as expected?
            if (sof < 59) {
                if (r.key >= 0 && r.key != sof) {
                    ++frMismatches;
                    if (++slipRun >= 2) {
                        // Two seconds running that clearly name another: the
                        // count has slipped. Follow them.
                        demote();
                        haveLastFrame = false;
                        anchorAt(r.key, r);
                        sof = r.key;
                        if (sof == 0) frStart = r.edge;
                    }
                } else {
                    slipRun = 0;
                    if (r.expSig >= 3.0) ++frMatches;
                }
            } else if (!r.marker && r.markConf >= kStructConf) {
                ++frMismatches;   // an excursion in second 59
            }
            if (sof >= 0 && sof < 60) {
                frame[static_cast<std::size_t>(sof)] = r;
                frFilled = std::max(frFilled, sof + 1);
            }
            sofNext = (sof + 1) % 60;
        }

        emitSecond(r, measured, sof, c, dc);

        if (anchored && sof == 59) finalizeFrame();

        if (lockState == ClockLockState::Locked && haveVoted && sof >= 0 && owner->onTime) {
            ClockTimeInfo ti;
            ti.minute = votedMinute; ti.hour = votedHour;
            ti.doy = votedDoy; ti.year2 = votedYear;
            ti.quality = votedQuality;
            ti.lastEdgeSample = r.edge;
            ti.lastEdgeSampleExact = r.edgeExact;
            ti.lastEdgeSecondOfFrame = sof;
            ti.station = ClockStation::Allouis;
            owner->onTime(ti);
        }
    }

    // The minute's bits, read; up to kMaxErasures that could not be read are
    // filled in by the checks. ALS162 carries a good deal of redundancy -- three
    // parity bits, the count of 1s in 21-58 (bits 3-6), a weekday that must fit
    // the date, bits 0 and 20 fixed, and exactly one of 17 and 18 -- so every
    // value of the unread bits is tried and the minute taken only when exactly
    // one passes every check. Two that pass is ambiguous, and refused. Without
    // this, at 30 dB-Hz a minute's 59 bits held one too weak to read more often
    // than not, and the whole minute was lost for it.
    Decoded decode(float& minConf) const {
        Decoded d;
        minConf = 1.0f;
        std::array<int, 60> b{};
        std::vector<int> unread;
        for (int s = 0; s <= 58; ++s) {
            const SecRec& r = frame[static_cast<std::size_t>(s)];
            if (!r.read) return d;
            if (r.marker || r.bitConf < kBitMinConf) {
                unread.push_back(s);
                if (static_cast<int>(unread.size()) > kMaxErasures) return d;
                continue;
            }
            b[static_cast<std::size_t>(s)] = r.bit;
            if (s >= 17) minConf = std::min(minConf, r.bitConf);
        }
        int passes = 0;
        for (int m = 0; m < (1 << unread.size()); ++m) {
            for (std::size_t i = 0; i < unread.size(); ++i) b[static_cast<std::size_t>(unread[i])] = (m >> i) & 1;
            const Decoded t = decodeBits(b);
            if (t.ok) { d = t; if (++passes > 1) return Decoded{}; }
        }
        // An unread bit filled in by the checks is worth what the checks are,
        // not what its reading was: the minute stands on its weakest READ bit,
        // held a little lower for each one filled.
        if (d.ok && !unread.empty()) minConf *= unread.size() == 1 ? 0.7f : 0.5f;
        return d;
    }

    Decoded decodeBits(const std::array<int, 60>& b) const {
        Decoded d;
        auto at = [&](int s) { return b[static_cast<std::size_t>(s)]; };
        auto parity = [&](int a, int z) { int p = 0; for (int s = a; s <= z; ++s) p ^= at(s); return p; };
        if (at(0) != 0 || at(20) != 1) return d;
        if (at(17) == at(18)) return d;
        if (parity(21, 28) || parity(29, 35) || parity(36, 58)) return d;
        int ones = 0;
        for (int s = 21; s <= 58; ++s) ones += at(s);
        if (ones != 2 * at(3) + 4 * at(4) + 8 * at(5) + 16 * at(6)) return d;

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
        int cy = 0; unsigned cm = 0, cd = 0;
        ubersdr_ntp::civilFromDays(days, cy, cm, cd);
        if (static_cast<int>(cm) != d.month || static_cast<int>(cd) != d.day) return d;
        if (static_cast<int>(ubersdr_ntp::floorMod(days + 3, 7)) + 1 != wday) return d;   // 1 = Monday

        d.cest = at(17) == 1;
        d.leapWarn = at(1) == 1 || at(2) == 1;
        // Legal time of the minute starting at the NEXT minute mark: this
        // frame's second 0 is one minute before it, UTC one or two hours back.
        const long long nextLocalMin = days * 1440LL + d.hour * 60LL + d.minute;
        const long long s0UtcMin = nextLocalMin - (d.cest ? 120 : 60) - 1;
        d.utcMs = s0UtcMin * 60000LL;
        d.utc = ubersdr_ntp::hostNowFields(d.utcMs);
        d.ok = true;
        return d;
    }

    static bool leapSecondPossible(const TimeFields& f) {
        if (f.minute != 59 || f.hour != 23) return false;
        return ubersdr_ntp::isLastDayOfMonth(ubersdr_ntp::utcMsFromFields(f.year2, f.doy, f.hour, f.minute));
    }

    void finalizeFrame() {
        const bool contradicted = frMismatches > 1;
        const bool confirmed = frFilled >= 60 && frMatches >= kConfirmMatches && !contradicted;
        if (contradicted || (!confirmed && ++unconfirmedRun > kMaxUnconfirmedMinutes)) {
            demote();
            haveLastFrame = false;
            frMatches = frMismatches = 0;
            return;
        }
        if (confirmed) unconfirmedRun = 0;

        float minConf = 0.0f;
        const Decoded dec = confirmed ? decode(minConf) : Decoded{};
        lastFrameFrom = dec.ok ? 2 : 0;

        ClockFrameInfo fi;
        fi.frameStartSample = frStart;
        fi.station = ClockStation::Allouis;
        std::array<ClockSymbol, 60> syms;
        std::array<float, 60> confs;
        syms.fill(ClockSymbol::Unknown);
        confs.fill(0.0f);
        for (int m : kVoterMarkers) syms[static_cast<std::size_t>(m)] = ClockSymbol::Marker;
        if (dec.ok) {
            auto encode = [&](int v, const ClockFieldMap& map, float cf) {
                for (const auto& bw : map) {
                    const bool one = v >= bw.weight;
                    if (one) v -= bw.weight;
                    syms[static_cast<std::size_t>(bw.second)] = one ? ClockSymbol::One : ClockSymbol::Zero;
                    confs[static_cast<std::size_t>(bw.second)] = cf;
                }
            };
            encode(dec.utc.minute, kVMin, minConf);
            encode(dec.utc.hour, kVHour, minConf);
            encode(dec.utc.doy, kVDoy, minConf);
            encode(dec.utc.year2, kVYear, minConf);
            fi.minute = dec.utc.minute;
            fi.hour = dec.utc.hour;
            fi.doy = dec.utc.doy;
            fi.year2 = dec.utc.year2;
            fi.leapPending = dec.leapWarn;
            fi.dst1 = fi.dst2 = dec.cest;
            fi.frameConfidence = minConf;
        }
        if (owner->onFrame) owner->onFrame(fi);

        const double P = trk.valid ? trk.period : static_cast<double>(sr);
        const bool consecutive = haveLastFrame &&
            std::fabs(static_cast<double>(frStart - lastFrameStart) - 60.0 * P) <= 0.25 * sr;
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
        // A leap second after this minute: stop certifying before it, as
        // Dcf77Decoder does; the position codes find the count again after.
        const bool leapNext = dec.ok && (dec.leapWarn || lastLeapWarn) && leapSecondPossible(dec.utc);
        lastLeapWarn = dec.ok && dec.leapWarn;
        if (certified && confirmed && !leapNext) {
            haveVoted = true;
            setLockState(ClockLockState::Locked);
        } else {
            demote();
        }
        frMatches = frMismatches = 0;
    }

    // ---- callbacks & state -----------------------------------------------

    // The display: the second's phase at 100 Hz, and what it should be.
    void emitSecond(const SecRec& r, bool measured, int sof, int64_t c, double dc) {
        if (!owner->onSecond) return;
        ClockSecondInfo si;
        si.edgeSample = r.edge;
        si.edgeSampleExact = r.edgeExact;
        si.edgeMeasured = measured;
        si.symbol = !r.read ? ClockSymbol::Unknown : r.marker ? ClockSymbol::Marker
                  : r.bit ? ClockSymbol::One : ClockSymbol::Zero;
        si.confidence = r.marker ? r.markConf : r.bitConf;
        si.secondOfFrame = sof;
        si.seriesRateHz = 100;
        si.envelope.assign(100, 0.0f);
        si.expected.assign(100, 0.0f);
        const int per = std::max(1, sr / 100);
        for (int k = 0; k < 100; ++k) {
            double v = 0.0;
            int n = 0;
            for (int i = 0; i < per; ++i) {
                const int64_t at = c + static_cast<int64_t>(k) * per + i;
                if (at < samplesConsumed) { v += yAt(at) - dc; ++n; }
            }
            si.envelope[static_cast<std::size_t>(k)] = n ? static_cast<float>(v / n) : 0.0f;
            const double t = (k + 0.5) / 100.0;
            double ph = 0.0;
            if (!r.marker) ph += excursion(t) + (r.bit ? excursion(t - 4 * kSubSec) : 0.0);
            if (sof >= 0 && sof < 59) ph += codePhase(sof, t);
            si.expected[static_cast<std::size_t>(k)] = static_cast<float>(std::sin(ph));
        }
        owner->onSecond(si);
    }

    void setLockState(ClockLockState s) {
        if (s != lockState) {
            lockState = s;
            if (owner->onStateChanged) owner->onStateChanged(s);
        }
    }

    // ---- members ---------------------------------------------------------

    AllouisDecoder* owner;
    int sr;
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

    double aRef = 0.0;
    double refRe = 0.0, refIm = 0.0;
    int freqCount = 0;
    double freqPrevRe = 0.0, freqPrevIm = 0.0;
    double carrierLevel = 0.0;     // |reference|, while the carrier is there
    int polarity = 1;              // y's sign: +1 when the excursion reads +1 rad first

    std::vector<float> yRing;      // y against the one-pole reference: acquisition only
    std::vector<float> zRe, zIm;   // the mixed baseband, for the per-second phase
    int64_t yMask = 0;
    // One second's phase, against the quiet-window reference (buildSecondPhase).
    std::vector<float> yLoc;
    int64_t yLocBase = 0;
    int64_t yStart = 0;

    // templates
    int nData = 0;                 // one excursion, in samples
    std::vector<float> tri;
    double triE = 0.0;
    int codeOff = 0, nCode = 0;
    std::array<std::vector<float>, 60> codeT;
    std::array<double, 60> codeE{};

    // acquisition fold
    int foldDecim = 12, foldLen = 1000;
    std::vector<double> triD;
    std::vector<std::vector<double>> foldSlots;
    std::vector<double> foldCur;
    int foldSlotsN = 0, foldSlotNext = 0, foldPos = 0, foldCount = 0;
    double foldAcc = 0.0;
    int64_t foldBlockStart = -1;
    int acqAgree = 0, acqLast = -1, acqSign = 1;
    int foldFresh = 0;
    bool carrierPresent = true;
    int unnamedRun = 0;

    // timing
    bool segValid = false;
    double segEdge = 0.0;
    EdgeTracker trk;
    int blindSeconds = 0;
    bool timing = false;
    double lastTimeSig = 0.0;
    double amp = 1.0;              // the excursion's amplitude in y, learnt
    bool ampHave = false;

    // seconds & frames
    std::vector<SecRec> hist;
    bool anchored = false;
    int sofNext = 0;
    std::array<SecRec, 60> frame{};
    int frFilled = 0;
    int64_t frStart = 0;
    int frMatches = 0, frMismatches = 0;
    int slipRun = 0;
    bool haveLastFrame = false;
    int64_t lastFrameStart = 0;
    bool lastLeapWarn = false;
    std::uint8_t lastFrameFrom = 0;
    int unconfirmedRun = 0;

    bool haveVoted = false;
    int votedMinute = -1, votedHour = -1, votedDoy = -1, votedYear = -1;
    float votedQuality = 0.0f;
};

// ---------------------------------------------------------------------------

AllouisDecoder::AllouisDecoder(int sampleRateHz, double carrierOffsetHz)
    : m_impl(std::make_unique<Impl>(this, sampleRateHz, carrierOffsetHz)) {}

AllouisDecoder::~AllouisDecoder() = default;

void AllouisDecoder::process(const float* iq, std::size_t frames) { m_impl->process(iq, frames); }
void AllouisDecoder::reset() { m_impl->reset(); }

void AllouisDecoder::setPlausibility(std::function<TimeFields()> referenceNow, int boundMinutes) {
    m_impl->voter.setPlausibility(std::move(referenceNow), boundMinutes);
}

ClockLockState AllouisDecoder::state() const { return m_impl->lockState; }

ClockStation AllouisDecoder::station() const {
    return m_impl->lockState == ClockLockState::NoSignal ? ClockStation::Unknown : ClockStation::Allouis;
}

std::int64_t AllouisDecoder::samplesConsumed() const { return m_impl->samplesConsumed; }

ClockDecoderDiagnostics AllouisDecoder::diagnostics() const {
    const Impl& d = *m_impl;
    ClockDecoderDiagnostics g;
    g.toneSnrDb = d.lastToneSnrDb;
    g.toneDetected = d.phase == Impl::Phase::Running;
    g.phaseLocked = d.segValid;
    g.delayEstMs = std::numeric_limits<float>::quiet_NaN();
    g.anchored = d.anchored;
    g.framesInWindow = d.voter.frameCount();
    g.windowSize = d.voter.windowSize();
    g.voteQuality = d.voter.lockConfidence();
    g.refusalReason = static_cast<std::uint8_t>(d.voter.lockRefusal());
    g.pmLocked = d.segValid && d.trk.valid;
    g.pmSnrDb = d.lastTimeSig > 0.0 ? static_cast<float>(20.0 * std::log10(d.lastTimeSig))
                                    : std::numeric_limits<float>::quiet_NaN();
    g.timingFromPm = d.timing;
    g.carrierOffsetHz = d.phase == Impl::Phase::Running ? static_cast<float>(d.f0)
                                                        : std::numeric_limits<float>::quiet_NaN();
    g.lastFrameFrom = d.lastFrameFrom;
    return g;
}

} // namespace clockdec
