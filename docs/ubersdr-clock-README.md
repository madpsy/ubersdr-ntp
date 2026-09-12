# ubersdr-clock

Standalone WWV / WWVH / WWVB time-code decoder. Reads raw PCM audio from stdin, writes decoded time as JSON to stdout. No dependencies beyond a C++20 compiler — no Qt, no FFTW, no system libraries beyond libstdc++.

Built for UberSDR as an external decoder binary, in the same shape as `cw-decoder`, `ubersdr-drm` and `freedv-ka9q`: the Go audio extension spawns it, pipes the session's demodulated audio into stdin, and reads events from stdout. It runs fine on its own from a WAV file too.

The DSP is AetherSDR's AetherClock chain, taken unmodified — see [Provenance](#provenance).

## Dependencies

```bash
# Debian / Ubuntu / Raspberry Pi OS
sudo apt-get install -y cmake ninja-build g++
```

## Build

For release binaries — both architectures, built the way they will run:

```bash
./build.sh
```

This builds amd64 and arm64 inside `ubuntu:24.04` (the same image UberSDR's container uses for its runtime stage), then feeds each binary a synthetic WWV signal and checks it reaches a lock with the right date before saying it worked. arm64 is built by running an arm64 container under binfmt/qemu rather than cross-compiling, so the toolchain is the target toolchain and CMake sees the target arch.

Building on the host instead works right up until the host is newer than the container. libstdc++ is this binary's only dependency, and one built against a newer one dies at startup on a `GLIBCXX_` version error that names everything except the actual problem.

`--arch amd64` for one of them, `--no-check` to skip the decode test, `--help` for the rest. arm64 needs the qemu binfmt handler:

```bash
docker run --privileged --rm tonistiigi/binfmt --install all
```

For a quick edit-compile loop on this host only:

```bash
make            # or: ./build.sh --native
```

Either way the output is `ubersdr-clock_<arch>` (e.g. `ubersdr-clock_amd64`, `ubersdr-clock_arm64`). The suffix is Go's `GOARCH` spelling, so it matches Docker's `${TARGETARCH}` and the wrapper's `runtime.GOARCH` lookup without translation. CMake derives it from `CMAKE_SYSTEM_PROCESSOR`, not `uname -m` — those agree for a native build, but under a cross-toolchain `uname` reports the host and would name an arm64 binary `amd64`.

To install to `/opt/ubersdr-clock/`:

```bash
sudo make install
```

## Releasing

The UberSDR container downloads the binaries at image build time rather than compiling them, from the moving `latest` tag:

```
https://github.com/madpsy/ubersdr-clock/releases/download/latest/ubersdr-clock_${TARGETARCH}
```

The asset names are constants and the tag never moves, so publishing **replaces** what every container build pulls:

```bash
./build.sh --publish          # builds both, checks both, then asks before uploading
./build.sh --publish --yes    # answer that question in advance, for an unattended run
```

It asks by default and only skips the question for `--yes`. That is a flag rather than an environment variable deliberately — an exported variable is inherited by everything a shell starts, so a `yes` meant for one release would sit there quietly authorising the next.

Two combinations are refused before anything is built:

- **`--publish --no-check`** — that would upload a decoder nothing has watched decode anything. On a receiver a broken build shows up as a clock stuck in `acquiring`, which reads as bad propagation rather than a bad binary, so the check is the only thing catching it.
- **`--publish --native`** — a host-built binary against a host libstdc++, uploaded to run inside `ubuntu:24.04`.

Only the architectures a given run built are replaced. `./build.sh --arch amd64 --publish` leaves whatever arm64 asset is already on the release exactly as it was, at whatever age it was, and the release page will not say so.

`UBERSDR_CLOCK_REPO` and `UBERSDR_CLOCK_TAG` override the target for a fork or a test release.

Then in `ka9q_ubersdr/docker/Dockerfile`, alongside the other decoder binaries:

```dockerfile
    && mkdir -p /opt/ubersdr-clock \
    && wget https://github.com/madpsy/ubersdr-clock/releases/download/latest/ubersdr-clock_${TARGETARCH} \
         -O /opt/ubersdr-clock/ubersdr-clock_${TARGETARCH} \
    && chmod +x /opt/ubersdr-clock/ubersdr-clock_${TARGETARCH} \
```

The arch suffix is kept on the installed name rather than stripped, so the Go wrapper resolves it with `runtime.GOARCH` and several architectures can share the directory — `ubersdr-drm`'s arrangement, and the reason its `resolveBinaryPath()` also falls back to the unsuffixed name for installs predating the change.

## Tuning

This matters more than anything else here. The decoder is written against a specific audio spectrum, and the filter centres are not negotiable.

| Station | Tune | Why |
|---|---|---|
| **WWV / WWVH** | **USB at (carrier − 1 kHz)** — 4.999, 9.999, 14.999, 19.999 MHz | Puts the RF carrier at 1000 Hz audio, the 100 Hz BCD subcarrier sidebands at 900/1100 Hz, and the seconds tick at its 2000 Hz (WWV) / 2200 Hz (WWVH) image |
| **WWVB** | **USB at 0.059 MHz** | Puts the 60 kHz carrier at ~1000 Hz audio, where the PWM rides on its amplitude |

**The passband must reach 2.2 kHz.** The WWV/WWVH second edge is recovered entirely from the tick image, and the station tag (WWV vs WWVH) is decided by which of the 2000/2200 Hz bands folds to an impulse. A 2.4 kHz SSB filter clips one or both, and the decoder never gets a second edge to classify against — it will sit in `acquiring` forever with `tone_detected: false`. WWVB only needs its ~1 kHz tone.

## Input

**Mono signed 16-bit little-endian raw PCM on stdin** at the rate given by `--sample-rate` (default: 12000 Hz).

No WAV header — raw samples only. 12 kHz is UberSDR's rate for `usb`/`lsb`/`cwu`/`cwl`; AM/FM sessions are 24 kHz.

```bash
# From a WAV file
sox wwv.wav -t raw -r 12000 -c 1 -e signed -b 16 - | ./ubersdr-clock_amd64

# From an RTL-SDR, 10 MHz WWV
rtl_fm -f 9.999M -M usb -s 12000 - | ./ubersdr-clock_amd64

# Synthetic signal, for a smoke test with no receiver
./tools/wwvgen.py --minutes 6 | ./ubersdr-clock_amd64
```

The sample rate must be a multiple of the decoder's internal series rate — 200 Hz for WWV/WWVH, 100 Hz for WWVB — or decimation drifts. 12000 and 24000 both divide cleanly; anything that does not is refused at startup rather than decoded wrongly.

## Output

One JSON object per line on stdout, flushed per line. Five event types.

### `state` — lock state changed

```json
{"type":"state","state":"locked","station":"wwv"}
```

| Field | Values |
|---|---|
| `state` | `nosignal`, `acquiring`, `locked` |
| `station` | `unknown`, `wwv`, `wwvh`, `wwvb` |

`station` stays `unknown` until the tick fold separates the two bands confidently; it is never guessed.

### `time` — a voted timestamp (this is the one you want)

Emitted while locked, once per decoded frame for WWV/WWVH and once per second for WWVB.

```json
{"type":"time","utc":"2026-09-07T07:01:59Z","utc_ms":1788764519000,
 "minute":1,"hour":7,"doy":250,"year2":26,"quality":100,"offset_ms":-341.2,
 "last_edge_sample":3587760,"frame_start_sample":2880000,
 "host_anchor_ms":1788764175267,"station":"wwv"}
```

| Field | Description |
|---|---|
| `utc` / `utc_ms` | The decoded broadcast time, composed from the voted frame's second 0 plus the elapsed samples to the last second edge |
| `quality` | Voter lock confidence, 0–100. The **minimum** winning margin across the voted bits, not the mean — a timestamp is only as trustworthy as its least-certain bit |
| `offset_ms` | Decoded time minus host clock at the same instant. Positive = the host clock is slow. **See the warning below** |
| `last_edge_sample` | Input sample index of the second edge the timestamp is anchored to |
| `frame_start_sample` | Input sample index of second 0 of the voted frame |
| `host_anchor_ms` | Wall clock when the first sample was read |

**`offset_ms` is only meaningful on a real-time stream.** A pipe carries samples, not timestamps, so the host anchor is the wall clock at the first sample advanced by the sample count. Replay a file and the number is nonsense — the file arrives at disk speed. Even on a live stream it inherits every buffer between the receiver and this process.

The sample indices are there so the caller can do better. UberSDR's audio extensions receive a per-packet arrival timestamp on every `AudioSample`, so its wrapper tracks how many samples it has written, maps `last_edge_sample` onto that, and replaces `offset_ms` — re-anchoring continuously instead of once, which removes the pipe latency and the drift. It marks the result `"offset_source": "packet"`.

That timestamp is **not** a hardware or GPS one, despite what the field is called in UberSDR's source: it is `time.Now()` when the RTP packet arrives from radiod. So a fixed bias remains — radiod's own buffering plus the multicast hop, some tens of milliseconds — and it is a bias rather than a drift. Good enough to say a clock is seconds out; not good enough to discipline anything with.

### `frame` — one raw frame decode, before voting

```json
{"type":"frame","minute":1,"hour":7,"doy":250,"year2":26,"dut1_tenths":-3,
 "dst1":false,"dst2":false,"leap_pending":false,"leap_year":false,
 "confidence":0.55,"frame_start_sample":2880000,"station":"wwv"}
```

Unvoted, so it can be wrong where `time` is right — that is the point of the voter. Useful for showing the decode working before lock, and it carries the fields voting does not touch: `dut1_tenths` (signed tenths of a second of UT1−UTC), the two DST bits, and the leap-second warning. `leap_year` is WWVB-only; WWV/WWVH carry no such bit.

### `second` — one classified second

On by default (`--no-seconds` to suppress). ~1 line/s, and the sign of life during the minutes before a lock.

```json
{"type":"second","edge_sample":59820,"symbol":1,"confidence":0.4888,
 "second_of_frame":17,"series_rate":200,"window_shift":-4,"station":"wwv"}
```

| Field | Description |
|---|---|
| `symbol` | 0 = binary zero, 1 = binary one, 2 = marker, −1 = unknown |
| `confidence` | Matched-filter margin: best correlation minus runner-up, ≥ 0 |
| `second_of_frame` | 0–59 once frame-synced, −1 before |
| `window_shift` | Where the received pulse sits versus the template's nominal position, in series samples. ~0 on a drift-free stream; nonzero while the decoder absorbs sample-clock drift |

With `--envelope` each event also carries `envelope` (the received 1 s amplitude series, normalised) and `expected` (the matched template of the decoded symbol), both at `series_rate` samples per second. Overlaying the two is the alignment display; shift `expected` by `window_shift` or the two disagree about where the second is.

### `diag` — acquisition telemetry

Every `--diag-seconds` seconds (default 10, `0` to disable). This is the pre-lock funnel: it tells you *which stage* is failing.

```json
{"type":"diag","state":"acquiring","station":"wwv","tone_snr_db":18.04,
 "pwm_contrast":0.000,"tone_detected":true,"phase_locked":true,
 "delay_est_ms":14.42,"anchored":false,"bad_frame_streak":0,
 "frames_in_window":0,"window_size":8,"vote_quality":0.000,"refusal":"none",
 "samples_consumed":360448}
```

| Stage | Field | Reading it |
|---|---|---|
| 1 — carrier | `tone_snr_db`, `tone_detected` | WWV/WWVH: folded tick-band peak-to-mean in dB. WWVB: tone-search peak/median, with `pwm_contrast` as the p90/p10 envelope contrast. `false` here usually means the passband is clipping the tick |
| 2 — timing | `phase_locked`, `delay_est_ms` | Second edge found. `delay_est_ms` is `null` until the tracked matched-filter delay settles, then tracks the chain delay plus any accumulated sample-clock drift |
| 3 — frame | `anchored`, `bad_frame_streak` | Frame sync found. A rising streak means the marker skeleton is breaking and a resync is coming |
| 5 — vote | `frames_in_window`, `vote_quality`, `refusal` | `refusal` names the gate saying no: `none`, `quality_floor`, `plausibility`, `staleness`, `contested` |

`refusal: none` with a low `frames_in_window` just means the window is still filling.

### `error`

```json
{"type":"error","message":"stdin read error"}
```

Argument errors go to stderr with exit code 2 instead — nothing is decoding yet, so there is no stream to report them on.

## Options

| Option | Description |
|---|---|
| `--sample-rate HZ` | Input PCM rate (default: 12000) |
| `--station NAME` | `wwv`, `wwvh` or `wwvb` (default: `wwv`) |
| `--no-seconds` | Suppress the per-second events |
| `--envelope` | Include the 1 s alignment arrays in second events |
| `--diag-seconds N` | Diagnostics interval, 0 to disable (default: 10) |
| `--plausibility-minutes N` | Refuse a lock more than N minutes from the host clock, 0 to disarm (default: 1440) |
| `--version`, `--help` | |

### What a caller has to pass, and what is automatic

Two of these are not optional for an embedding caller:

- **`--sample-rate` always.** The default of 12000 matches UberSDR's `usb`/`lsb`/`cwu`/`cwl` sessions, but an AM or FM session is 24000. Passing the wrong one does not fail — it decodes at the wrong speed and never locks.
- **`--station wwvb` when tuned to WWVB.** WWV/WWVH and WWVB are genuinely different decoders: a 100 Hz BCD subcarrier against PWM on the carrier's own amplitude, LSB-first weights against MSB-first, a marker-hole anchoring search against a double marker. Nothing can be shared, and neither will decode the other's signal. A caller that knows the dial frequency can decide this without asking anyone — below 1 MHz is WWVB, everything else is WWV/WWVH.

Everything else is automatic. In particular **`wwv` and `wwvh` select the same decoder**: it identifies the station itself from which tick band folds to an impulse, and reports it in `station` on every event. Frame sync, symbol timing, sample-clock drift tracking and resynchronisation after a discontinuity need no help either.

The remaining options only shape the output — `--no-seconds`, `--envelope`, `--diag-seconds` — or arm a safety gate, below.

### On `--plausibility-minutes`

This is a safety gate, and turning it off is riskier than it looks. A deep fade zero-biases the *same* bits in *every* frame of the voter's window, so the misread is unanimous — full agreement, maximum margin — and no disagreement-based quality metric can catch it. The upstream project has a documented case of a receiver decoding **2006-01-01 at quality 100** this way. An independent reference clock is the only evidence that can veto a self-consistent misread.

The default bound of one day is deliberately generous: a host clock that is hours wrong is exactly what this tool measures, and only a decode *decades* out can be assumed garbage. Disarm it (`0`) for offline corpus work where there is no meaningful reference, not on a live receiver.

## Testing without a receiver

`tools/wwvgen.py` synthesises the audio a correctly-tuned receiver would hear — the 1000 Hz carrier, the 100 Hz BCD subcarrier with the NIST pulse lengths, the second 0 subcarrier hole, and the tick at its audio image. `--wwvb` generates the PWM-on-amplitude form instead. It needs only the Python standard library.

```bash
# Clean WWV, six minutes: enough to anchor (2) and vote (2 more)
./tools/wwvgen.py --minutes 6 | ./ubersdr-clock_amd64 --no-seconds

# WWVH — same signal, tick at 2200 Hz; the decoder should tag it itself
./tools/wwvgen.py --minutes 6 --wwvh | ./ubersdr-clock_amd64 --no-seconds

# WWVB
./tools/wwvgen.py --minutes 6 --wwvb | ./ubersdr-clock_amd64 --station wwvb --no-seconds

# Under noise
./tools/wwvgen.py --minutes 6 --snr 3 | ./ubersdr-clock_amd64 --no-seconds
```

It defaults to starting at the next whole minute so the plausibility gate is satisfied; pass `--start YYYY-MM-DDTHH:MM` for a fixed time (and then `--plausibility-minutes 0`, or the gate will correctly refuse it).

This is a signal generator, not a channel model. It proves the decode chain end to end and catches regressions; it says nothing about how the decoder behaves on a real fading HF path.

### What the synthetic signal does not validate

**Absolute edge accuracy.** `WwvDecoder.cpp` carries `kNominalDelaySamples = 7` (35 ms) as the biquad chain's group delay, calibrated upstream against real WWV audio through a real receive chain. Against `wwvgen.py` the tracked estimate settles at ~14.8 ms instead, leaving a constant `window_shift` of −4 series samples (−20 ms). That is the generator's idealised instant-rise pulse differing from what upstream calibrated against, not a decode error — the decoder reports the discrepancy honestly in `window_shift`, which is exactly what that field is for.

It does confirm the part that matters for porting: the figure is **identical at 12 kHz and 24 kHz**, so nothing in the chain depends on the 24 kHz rate AetherSDR fed it.

But it means the ~20 ms sits in `offset_ms` as an unvalidated systematic term until someone runs this against a real off-air recording. Treat sub-100 ms offsets as unproven until then.

## Provenance

`src/WwvDecoder.*`, `src/WwvbDecoder.*` and `src/TimeFrameVoter.*` are copied **unmodified** from [AetherSDR](https://github.com/aethersdr/AetherSDR) (`src/core/`, commit `b9f44f3`), where they are the DSP half of its AetherClock feature. They are already Qt-free and dependency-free by design, so they compile here as they are.

They are kept byte-identical on purpose: it makes upstream fixes a straight `cp`, and the voter in particular carries calibration constants tuned against live WWV corpora that should not drift without upstream's evidence behind the change. Anything UberSDR-specific belongs in `src/main.cpp`.

`src/main.cpp` is written for this repo. It replaces AetherSDR's `AetherClockEngine` (which is Qt, and also owns FlexRadio DAX channel lifecycle) with a stdio front end, keeping only what the engine did on the receive path: hold the sample↔host anchor, arm the voter's plausibility gate against the host clock, and compose the UTC timestamp from the voted frame's second 0 plus elapsed samples.

Format facts throughout are per NIST SP 432 (WWV/WWVH) and NIST SP 250-67 (WWVB).

## Licence

GPL-3.0-or-later, inherited from AetherSDR. See [LICENSE](LICENSE).
