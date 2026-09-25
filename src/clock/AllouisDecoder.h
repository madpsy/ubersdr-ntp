#pragma once

// ALS162 (Allouis, formerly TDF) 162 kHz decoder: phase modulation only.
//
// Format facts per ITU-R TF.2487 section 9.1 (the French standard itself, NF
// C90-002, is not public), and in detail from henningM1r/gr_ALS162_Receiver
// (GPLv3), a GNU Radio model of the transmitter whose tables are used here --
// both checked against a recording from M9PSY-1 (tools/testdata):
//
//   Data    every second but 59 starts with a phase excursion of +1 rad and
//           -1 rad in 100 ms: ramps of 25 ms, 0 -> +1 -> 0 -> -1 -> 0 rad. A
//           binary 1 sends it twice (0-200 ms); a 0 once, then 100 ms of
//           nothing. Second 59 has none: the minute marker.
//   Code    from 200 to 900 ms, a pseudo-random sequence of ramps of 0, +/-1
//           and +/-2 rad per 25 ms, different for every second and returning
//           to 0; second 59's is empty, so that whole second is unmodulated.
//           900-1000 ms is always unmodulated. The model keys its table one
//           ahead: key "NN" is sent in second NN-1 (confirmed on the air).
//   Time    BCD LSB first, DCF77's layout from second 20: minute 21-27, hour
//           29-34, day 36-41, weekday 42-44 (1 = Monday), month 45-49, year
//           50-57, EVEN parity 28, 35, 58. Before that: 0 always 0, 1/2 leap
//           second warning (+/-), 3-6 the count of 1s in 21-58 (weights 2, 4,
//           8, 16), 13/14 public holiday tomorrow/today, 16 clock change, 17/18
//           CEST/CET, 20 always 1. French legal time, naming the minute that
//           BEGINS at the next minute mark (confirmed on the air).
//   Off     every Tuesday 08:00-12:00 French legal time, for maintenance.
//
// Input contract: complex baseband, interleaved I/Q float32, from an UberSDR
// "iq" session (12 kHz) tuned on the carrier.
//
// Timing is a correlation of the whole second's known phase -- the data
// excursion(s) and that second's position code -- against the received phase,
// so the second is timed by 700 ms and more of known modulation, not by an
// amplitude edge. Everything above is counted from the excursion's start,
// which is 50 ms BEFORE the second: the excursion's midpoint, its zero crossing
// from +1 to -1 rad, is on the second. Not published anywhere; measured against
// DCF77 (AllouisDecoder.cpp, kSecondAfterStartSec). The edge reported is the
// second itself.
//
// Decoded minutes are converted to UTC before they reach the shared voter, as
// synthetic frames in WWVB's field layout, as Dcf77Decoder does.
//
// Pure DSP -- no I/O. Streaming: process() accumulates internally.

#include "TimeFrameVoter.h"

#include <cstddef>
#include <functional>
#include <memory>

namespace clockdec {

class AllouisDecoder {
public:
    explicit AllouisDecoder(int sampleRateHz = 12000, double carrierOffsetHz = 0.0);
    ~AllouisDecoder();

    AllouisDecoder(const AllouisDecoder&) = delete;
    AllouisDecoder& operator=(const AllouisDecoder&) = delete;

    // Feed `frames` complex samples, interleaved I,Q (2*frames floats). Fires
    // callbacks inline, on the calling thread.
    void process(const float* iq, std::size_t frames);

    void reset();

    // As WwvbDecoder::setPlausibility.
    void setPlausibility(std::function<TimeFields()> referenceNow, int boundMinutes);

    ClockLockState state() const;
    ClockStation station() const;      // Allouis once the carrier is found
    std::int64_t samplesConsumed() const;

    // pmLocked: the second is being timed by the full-second correlation;
    // pmSnrDb: that correlation's last SNR.
    ClockDecoderDiagnostics diagnostics() const;

    std::function<void(const ClockSecondInfo&)> onSecond;
    std::function<void(const ClockFrameInfo&)> onFrame;
    std::function<void(const ClockTimeInfo&)> onTime;  // voted updates while locked
    std::function<void(ClockLockState)> onStateChanged;

private:
    struct Impl;
    std::unique_ptr<Impl> m_impl;
};

} // namespace clockdec
