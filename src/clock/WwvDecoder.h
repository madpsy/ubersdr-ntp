#pragma once

// WWV/WWVH 100 Hz-subcarrier BCD time-code decoder — streaming port of the
// gate-passed reference chain (research/wwv_decode_proto.py).
// Format facts per the NIST WWV/WWVH time-code table (NIST SP 432).
//
// Input contract: complex baseband, interleaved I/Q float32, from an UberSDR
// "iq" session (12 kHz) with the carrier near carrierOffsetHz -- 0 when the
// dial is on the carrier. The carrier is found to a fraction of a hertz and
// followed after, so a receiver a few ppm off costs nothing.
//
// Timing is the seconds tick: WWV's 5 ms burst of 1000 Hz, WWVH's of 1200 Hz,
// which starts ON the second. It is taken from coherent AM (the carrier's own
// phase as the reference) by a matched filter averaged from second to second,
// so it is exact to the synthetic test's resolution and has no filter delay to
// calibrate. The 100 Hz BCD subcarrier gives the time code; its own edge is
// kept only as a check on the tick (bcdMinusTickMs).
//
// Chain: carrier search -> mixer -> 15 Hz carrier reference (delay-matched) ->
// coherent AM m -> {tick timers at 1000/1200 Hz} + {tick-band fold: coarse
// tick phase + station tag} + {coherent 100 Hz demod (25 Hz LPF) -> 200 Hz
// series -> per-second matched-filter classify (170/470/770 ms templates at
// +30 ms) -> marker frame sync (P markers at seconds 9/19/29/39/49/59;
// marker-only anchoring is DEGENERATE mod 10 s -- disambiguated via the s0
// minute-mark subcarrier hole and minute-increment scoring) -> NIST BCD field
// map -> TimeFrameVoter}.
//
// Pure DSP — no Qt (EB1/EB2). Streaming: process() accumulates internally,
// no whole-file transforms.

#include "TimeFrameVoter.h"

#include <cstddef>
#include <functional>
#include <memory>

namespace clockdec {

class WwvDecoder {
public:
    explicit WwvDecoder(int sampleRateHz = 12000, double carrierOffsetHz = 0.0);
    ~WwvDecoder();

    WwvDecoder(const WwvDecoder&) = delete;
    WwvDecoder& operator=(const WwvDecoder&) = delete;

    // Feed `frames` complex samples, interleaved I,Q (2*frames floats). Fires
    // callbacks inline (same thread) as seconds/frames/time updates become
    // available.
    void process(const float* iq, std::size_t frames);

    void reset();

    // Arm the shared voter's absolute-plausibility gate (WS-4.5): a voted
    // timestamp farther than boundMinutes from the reference clock refuses to
    // lock. The engine plumbs the host clock here; default is disarmed so pure
    // decoder use (tests, corpus runners) is reference-free.
    void setPlausibility(std::function<TimeFields()> referenceNow,
                         int boundMinutes);

    // Start from a station tag already established -- the one this source's
    // previous decoder held -- rather than Unknown. The tag is still judged
    // every second and switched or released on the usual evidence; this only
    // spares a restarted decoder the half-minute it takes to re-derive what was
    // already known. Ignored once pinned, and for anything but Wwv or Wwvh.
    void presetStation(ClockStation s);
    // Fix the tag for good, for a carrier only one station transmits on (WWV
    // alone uses 20 and 25 MHz). Never judged afterwards; survives reset().
    void pinStation(ClockStation s);

    ClockLockState state() const;
    ClockStation station() const;      // Wwv or Wwvh once tick-tagged
    std::int64_t samplesConsumed() const;

    // WS-7 acquisition telemetry: read-only snapshot assembled ON CALL from
    // state the decoder already keeps (tick fold, delay estimate, anchor,
    // voter) — zero cost on the sample path, no feedback into decoding.
    ClockDecoderDiagnostics diagnostics() const;

    // Callbacks (any may be left unset).
    std::function<void(const ClockSecondInfo&)> onSecond;
    std::function<void(const ClockFrameInfo&)> onFrame;
    std::function<void(const ClockTimeInfo&)> onTime;  // voted updates while locked
    std::function<void(ClockLockState)> onStateChanged;

private:
    struct Impl;
    std::unique_ptr<Impl> m_impl;
};

} // namespace clockdec
