#pragma once

// DCF77 77.5 kHz decoder: the amplitude AND the phase modulation, together.
//
// Format facts per PTB (the DCF77 time-code and phase-modulation pages) and
// Hetzel, "Time dissemination via the LF transmitter DCF77 using a
// pseudo-random phase-shift keying of the carrier", EFTF 1988:
//
//   AM   carrier cut to 15% for 0.1 s (binary 0) or 0.2 s (binary 1) at the
//        start of every second; second 59 is not cut, which marks the minute.
//        A leap second sends a 0 in second 59 and leaves second 60 uncut.
//   PM   from 200 ms to 992.77 ms of every second, 512 chips of 120 carrier
//        cycles (1.548 ms) each, from a 9-bit LFSR, +15.6 deg for a 0 chip and
//        -15.6 deg for a 1. The chips are XORed with the second's time-code bit,
//        which is the AM bit except in seconds 59 (0), 0-9 (1) and 10-14 (0) --
//        a fixed sixteen-bit word that finds the minute without the AM.
//   Code BCD, LSB-first, with even parity per field. CET/CEST, and it names the
//        minute that BEGINS at the next minute mark, not the one being sent.
//
// Input contract: complex baseband, interleaved I/Q float32, from an UberSDR
// "iq" session (12 kHz) with the carrier near carrierOffsetHz -- 0 when the
// dial is 77.5 kHz itself. The carrier is found to a fraction of a hertz at
// acquisition and followed after, so an offset of a few hertz costs nothing.
//
// WHY BOTH. They fail differently. PM is a 793 ms spread-spectrum correlation
// and measures the second to tens of microseconds, holding through noise that
// buries the AM; AM needs no correlator to find and is how the minute has
// always been marked. So both run every second, and:
//
//   timing  PM's edge whenever the correlator is tracking, AM's otherwise.
//   bits    each minute is decoded twice, once from PM bits and once from AM
//           bits, each checked by its own parity. Both valid and agreeing, or
//           only one valid: that minute. Both valid and DIFFERENT is a
//           contradiction and nothing is certified from it.
//   frame   PM's sync word and AM's minute mark each check the other's idea of
//           where the minute is.
//
// Decoded minutes are converted to UTC before they reach the shared voter, as
// synthetic frames in WWVB's field layout -- the voter is built around
// day-of-year, and DCF77 sends a local calendar date. See finalizeFrame.
//
// Pure DSP — no I/O. Streaming: process() accumulates internally.

#include "TimeFrameVoter.h"

#include <cstddef>
#include <functional>
#include <memory>

namespace clockdec {

class Dcf77Decoder {
public:
    explicit Dcf77Decoder(int sampleRateHz = 12000, double carrierOffsetHz = 0.0);
    ~Dcf77Decoder();

    Dcf77Decoder(const Dcf77Decoder&) = delete;
    Dcf77Decoder& operator=(const Dcf77Decoder&) = delete;

    // Feed `frames` complex samples, interleaved I,Q (2*frames floats). Fires
    // callbacks inline, on the calling thread.
    void process(const float* iq, std::size_t frames);

    void reset();

    // As WwvbDecoder::setPlausibility: a voted timestamp farther than
    // boundMinutes from the reference refuses to lock.
    void setPlausibility(std::function<TimeFields()> referenceNow, int boundMinutes);

    ClockLockState state() const;
    ClockStation station() const;      // Dcf77 once the carrier is found
    std::int64_t samplesConsumed() const;

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
