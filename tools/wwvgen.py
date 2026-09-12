#!/usr/bin/env python3
"""
Synthetic WWV/WWVH time-code signal generator — raw PCM on stdout.

Produces what a receiver tuned USB at (carrier - 1 kHz) would hear, per
NIST SP 432:

  * the RF carrier as a 1000 Hz audio tone;
  * the 100 Hz BCD subcarrier amplitude-modulated onto it, present from
    +30 ms into each second for 170 ms (binary 0), 470 ms (binary 1) or
    770 ms (marker), and absent entirely at second 0 (the minute hole the
    decoder's frame anchoring depends on to break the mod-10 marker
    degeneracy);
  * the seconds tick as a 5 ms burst at 2000 Hz — the audio image of WWV's
    1000 Hz tick. Use --wwvh for 2200 Hz (WWVH's 1200 Hz tick), which is
    what the decoder tags the station from.

This exists because there is no other way to test the decoder without a
receiver and a fair HF path. It is not a propagation model: --snr adds white
noise, and that is the whole channel.

  ./tools/wwvgen.py --minutes 6 | ./ubersdr-clock_amd64 --sample-rate 12000
"""

import argparse
import datetime as dt
import math
import random
import struct
import sys

MARKER_SECONDS = {9, 19, 29, 39, 49, 59}

# {second: weight}, LSB-first within each field — NIST SP 432, and the same
# tables WwvDecoder.cpp carries as kMin/kHr/kDoy/kYr.
FIELD_MINUTES = {10: 1, 11: 2, 12: 4, 13: 8, 15: 10, 16: 20, 17: 40}
FIELD_HOURS = {20: 1, 21: 2, 22: 4, 23: 8, 25: 10, 26: 20}
FIELD_DOY = {30: 1, 31: 2, 32: 4, 33: 8, 35: 10, 36: 20, 37: 40, 38: 80,
             40: 100, 41: 200}
FIELD_YEAR = {4: 1, 5: 2, 6: 4, 7: 8, 51: 10, 52: 20, 53: 40, 54: 80}

# WWVB (NIST SP 250-67) is a different broadcast in every respect the decoder
# cares about: the code is PWM on carrier amplitude rather than a subcarrier,
# the BCD weights run MSB-first, and second 0 is a marker rather than a hole —
# it is the double marker s59 -> s0 that gives the minute boundary, so there is
# no anchoring search and no hole to look for.
WWVB_MARKER_SECONDS = {0, 9, 19, 29, 39, 49, 59}
WWVB_FIELD_MINUTES = {1: 40, 2: 20, 3: 10, 5: 8, 6: 4, 7: 2, 8: 1}
WWVB_FIELD_HOURS = {12: 20, 13: 10, 15: 8, 16: 4, 17: 2, 18: 1}
WWVB_FIELD_DOY = {22: 200, 23: 100, 25: 80, 26: 40, 27: 20, 28: 10,
                  30: 8, 31: 4, 32: 2, 33: 1}
WWVB_FIELD_YEAR = {45: 80, 46: 40, 47: 20, 48: 10, 50: 8, 51: 4, 52: 2, 53: 1}

# Low-power (reduced-carrier) durations: binary 0, binary 1, marker.
WWVB_LOW = {0: 0.200, 1: 0.500, 2: 0.800}
WWVB_DROP_DB = 17.0      # NIST: carrier power reduced 17 dB during the low

# Symbol pulse lengths in seconds: binary 0, binary 1, marker.
PULSE_ZERO = 0.170
PULSE_ONE = 0.470
PULSE_MARKER = 0.770
PULSE_START = 0.030      # every pulse rises 30 ms into the second
TICK_LEN = 0.005


def encode_field(value, weights):
    """Which seconds carry a One for this field value."""
    ones = set()
    remaining = value
    for second, weight in sorted(weights.items(), key=lambda kv: -kv[1]):
        if remaining >= weight:
            remaining -= weight
            ones.add(second)
    if remaining:
        raise ValueError(f"value {value} not representable in {weights}")
    return ones


def frame_symbols(when):
    """
    One minute of symbols: 0 = binary zero, 1 = binary one, 2 = marker,
    None = no pulse at all (second 0's subcarrier hole).
    """
    ones = set()
    ones |= encode_field(when.minute, FIELD_MINUTES)
    ones |= encode_field(when.hour, FIELD_HOURS)
    ones |= encode_field(when.timetuple().tm_yday, FIELD_DOY)
    ones |= encode_field(when.year % 100, FIELD_YEAR)

    symbols = []
    for second in range(60):
        if second == 0:
            symbols.append(None)
        elif second in MARKER_SECONDS:
            symbols.append(2)
        elif second in ones:
            symbols.append(1)
        else:
            symbols.append(0)
    return symbols


def wwvb_frame_symbols(when):
    """One minute of WWVB symbols: 0, 1 or 2 (marker). No hole — s0 is a marker."""
    ones = set()
    ones |= encode_field(when.minute, WWVB_FIELD_MINUTES)
    ones |= encode_field(when.hour, WWVB_FIELD_HOURS)
    ones |= encode_field(when.timetuple().tm_yday, WWVB_FIELD_DOY)
    ones |= encode_field(when.year % 100, WWVB_FIELD_YEAR)

    return [2 if s in WWVB_MARKER_SECONDS else (1 if s in ones else 0)
            for s in range(60)]


def wwvb_render(symbols, rate):
    """One minute of WWVB as a 1000 Hz tone with the PWM on its amplitude."""
    low_amp = 10.0 ** (-WWVB_DROP_DB / 20.0)
    out = []
    for symbol in symbols:
        low_end = WWVB_LOW[symbol]
        for n in range(rate):
            t = n / rate
            amp = low_amp if t < low_end else 1.0
            out.append(amp * math.sin(2.0 * math.pi * 1000.0 * t))
    return out


def render(symbols, rate, tick_hz, subcarrier_depth, tick_amp):
    """One second of float samples."""
    out = []
    length = {0: PULSE_ZERO, 1: PULSE_ONE, 2: PULSE_MARKER}
    for symbol in symbols:
        pulse_end = 0.0 if symbol is None else PULSE_START + length[symbol]
        for n in range(rate):
            t = n / rate
            # Carrier, with the 100 Hz subcarrier riding on its amplitude
            # while the pulse is up.
            envelope = 1.0
            if PULSE_START <= t < pulse_end:
                envelope += subcarrier_depth * math.sin(2.0 * math.pi * 100.0 * t)
            sample = envelope * math.sin(2.0 * math.pi * 1000.0 * t)
            # Seconds tick, as its audio image.
            if t < TICK_LEN:
                sample += tick_amp * math.sin(2.0 * math.pi * tick_hz * t)
            out.append(sample)
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--rate", type=int, default=12000, help="sample rate (default: 12000)")
    ap.add_argument("--minutes", type=int, default=6,
                    help="minutes to generate; the decoder needs 2 to anchor "
                         "and 2 more to vote (default: 6)")
    ap.add_argument("--start", default=None,
                    help="UTC start as YYYY-MM-DDTHH:MM (default: the next whole "
                         "minute, so the plausibility gate is satisfied)")
    ap.add_argument("--wwvh", action="store_true",
                    help="tick at 2200 Hz (WWVH) instead of 2000 Hz (WWV)")
    ap.add_argument("--wwvb", action="store_true",
                    help="generate WWVB (PWM on carrier amplitude) instead")
    ap.add_argument("--snr", type=float, default=None,
                    help="add white noise at this SNR in dB (default: none)")
    ap.add_argument("--seed", type=int, default=1, help="noise seed (default: 1)")
    args = ap.parse_args()

    if args.start:
        start = dt.datetime.strptime(args.start, "%Y-%m-%dT%H:%M").replace(tzinfo=dt.timezone.utc)
    else:
        now = dt.datetime.now(dt.timezone.utc).replace(second=0, microsecond=0)
        start = now + dt.timedelta(minutes=1)

    tick_hz = 2200.0 if args.wwvh else 2000.0
    rng = random.Random(args.seed)

    which = "WWVB" if args.wwvb else ("WWVH" if args.wwvh else "WWV")
    print(f"wwvgen: {args.minutes} min of {which} from {start:%Y-%m-%dT%H:%MZ} "
          f"(doy {start.timetuple().tm_yday}), {args.rate} Hz"
          + ("" if args.wwvb else f", tick {tick_hz:.0f} Hz")
          + (f", SNR {args.snr} dB" if args.snr else ""),
          file=sys.stderr)

    # Peak of the clean signal, for the noise level and for scaling to int16.
    peak = 1.0 if args.wwvb else (1.0 + 0.5 + 0.8)
    noise_sigma = 0.0
    if args.snr is not None:
        noise_sigma = (1.0 / math.sqrt(2.0)) * (10.0 ** (-args.snr / 20.0))
        peak += 4.0 * noise_sigma

    scale = 28000.0 / peak
    out = sys.stdout.buffer

    for m in range(args.minutes):
        when = start + dt.timedelta(minutes=m)
        if args.wwvb:
            samples = wwvb_render(wwvb_frame_symbols(when), args.rate)
        else:
            samples = render(frame_symbols(when), args.rate, tick_hz, 0.5, 0.8)
        if noise_sigma:
            samples = [s + rng.gauss(0.0, noise_sigma) for s in samples]
        out.write(struct.pack(f"<{len(samples)}h",
                              *(max(-32768, min(32767, int(s * scale))) for s in samples)))
        out.flush()


if __name__ == "__main__":
    main()
