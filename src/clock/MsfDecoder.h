#pragma once

// MSF 60 kHz decoder (NPL, Anthorn).
//
// Format facts per NPL, "MSF 60 kHz time and date code" (2019), repeated in
// ITU-R TF.2487 section 3:
//
//   Keying  on-off, 100% ("carrier off" = 1). Every second starts with the
//           carrier off: 100 ms at least, then bit A in 100-200 ms and bit B
//           in 200-300 ms, then carrier on to the end of the second. Second 00
//           is instead 500 ms off: the minute marker. The falling edge is on
//           time to better than 1 ms.
//   Code    bit A, BCD, MOST significant bit first: year 17-24, month 25-29,
//           day of month 30-35, day of week 36-38 (0 = Sunday), hour 39-44,
//           minute 45-51. A 52-59 is always 01111110, the minute identifier,
//           which occurs nowhere else in bit A. Bit B: DUT1 in 01-16, 53 summer
//           time imminent, 54-57 ODD parity over A 17-24, 25-35, 36-38, 39-51,
//           58 summer time in effect.
//   Time    UK civil time (UTC, or UTC+1 when B58 is set), naming the minute
//           that BEGINS at the next minute marker.
//   Leap    a minute may run 61 (or 59) seconds, and then every field from 17A
//           and 52B on moves one second later (earlier), so the code stays
//           aligned with the END of the minute. MSF sends no warning.
//
// So the time code is read backwards from the end of the minute, which is
// found by the next minute marker (or, if that fades, by the identifier). A
// minute is decoded when it has ended -- at the next second 00 -- not at its
// second 59 as DCF77's is: only then is its length known.
//
// Input contract: complex baseband, interleaved I/Q float32, from an UberSDR
// "iq" session (12 kHz) tuned on the carrier, with the carrier near
// carrierOffsetHz (0 when the dial is 60 kHz itself).
//
// Timing is the carrier's falling edge at the start of every second, found on
// the envelope exactly as Dcf77Decoder finds DCF77's AM cut.
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

class MsfDecoder {
public:
    explicit MsfDecoder(int sampleRateHz = 12000, double carrierOffsetHz = 0.0);
    ~MsfDecoder();

    MsfDecoder(const MsfDecoder&) = delete;
    MsfDecoder& operator=(const MsfDecoder&) = delete;

    // Feed `frames` complex samples, interleaved I,Q (2*frames floats). Fires
    // callbacks inline, on the calling thread.
    void process(const float* iq, std::size_t frames);

    void reset();

    // As WwvbDecoder::setPlausibility.
    void setPlausibility(std::function<TimeFields()> referenceNow, int boundMinutes);

    ClockLockState state() const;
    ClockStation station() const;      // Msf once the carrier is found
    std::int64_t samplesConsumed() const;

    ClockDecoderDiagnostics diagnostics() const;

    // Callbacks. A frame's ClockFrameInfo arrives at the second 00 that ends
    // it; its frameStartSample is its own second 00, 60 (or 61, or 59) seconds
    // earlier.
    std::function<void(const ClockSecondInfo&)> onSecond;
    std::function<void(const ClockFrameInfo&)> onFrame;
    std::function<void(const ClockTimeInfo&)> onTime;  // voted updates while locked
    std::function<void(ClockLockState)> onStateChanged;

private:
    struct Impl;
    std::unique_ptr<Impl> m_impl;
};

} // namespace clockdec
