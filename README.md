# ubersdr-ntp

An NTP server disciplined by WWV, WWVH or WWVB, heard over one or more
[UberSDR](https://ubersdr.org) receivers.

It connects to a receiver's audio WebSocket, tunes it to a time-signal
frequency, decodes the broadcast time code in process, and serves the result as
NTP on port 123 — plus a read-only JSON API, a one-event-per-second stream, and
a status page for watching it work.

Several receivers and several frequencies, because HF propagation makes any
single one unreliable: 10 MHz from Fort Collins is excellent in the afternoon
and gone at 3am, 5 MHz is the other way round, and a receiver on another
continent hears a different subset again. Redundancy here is about the band, not
about hardware.

## What this is, and what it is not

It is a **stratum-1 radio clock**: its reference is not another NTP server. That
is a statement about topology, not accuracy.

**Measured**, against K3FEF in Milford PA hearing Fort Collins on 10 and 15 MHz,
with the receiver 2516 km away and 97 ms of network round trip between it and
the client:

| | |
|---|---|
| Raw offset, 10 MHz | −67.6 ms |
| Raw offset, 15 MHz | −67.3 ms |
| **Agreement between two independent frequencies** | **0.3 ms** |
| Modelled delay (8.8 propagation + 48.3 network + 8.0 codec) | 65.1 ms |
| Served offset after correction | −0.8 ms |
| **Disagreement with a reference NTP server** | **2.5 ms** |
| Root dispersion it claimed while doing so | 24.2 ms |

Do not read 2.5 ms as the general figure. The two terms that dominate the
remaining error are *path-specific*: how symmetric the network route is, and how
much the receiver buffers between `radiod` and the WebSocket. Both happened to
be small here. On a path where the route is asymmetric, or the receiver buffers
more, the error is tens of milliseconds until `extra_delay_ms` is set — and
that is what the root dispersion is claiming, rather than the 2.5 ms it
actually achieved.

So: **tens of milliseconds by default, single-digit milliseconds when the delay
model fits the path.** A client weighing this against a GPS source will
correctly prefer the GPS, which is the intended outcome.

The 0.3 ms agreement between two frequencies is the more transferable number.
Those are two independent decoders on two independent propagation paths sharing
only a receiver and a network link, so it bounds everything except what they
share — which is exactly the part the delay model is for.

It is **not** a substitute for a local GPS reference, and it should not be the
only source on a machine that has one. What it is good for is a machine with no
reference of its own, a sanity check against a clock you do not trust, or the
pleasure of running a clock off a shortwave broadcast.

Sub-100 ms claims are unproven in one specific respect, and it is worth knowing
which: the decoder carries a chain-delay constant calibrated upstream against
real WWV audio through a real receive chain, and it has never been validated
against an off-air recording at the two rates used here. See *Provenance*.

## Build

```bash
sudo apt-get install -y cmake ninja-build g++ pkg-config \
    libopus-dev libcurl4-openssl-dev libssl-dev python3
make            # or: ./build.sh --native
```

For release binaries — both architectures, built the way they will run, each one
smoke-tested:

```bash
./build.sh
```

To publish them as a GitHub release as well, add `--release <tag>` (needs the
`gh` CLI, a clean tree, and HEAD pushed; the tag is created at HEAD if absent):

```bash
./build.sh --release v0.1.0
```

That builds amd64 and arm64 inside `ubuntu:24.04` (the image UberSDR's own
container runtime uses), then runs `tools/selftest.py` against each binary,
which drives its real NTP socket and its real HTTP service. arm64 is built by
running an arm64 container under binfmt/qemu rather than cross-compiling, so the
toolchain is the target toolchain; that needs the handler:

```bash
docker run --privileged --rm tonistiigi/binfmt --install all
```

**That registration is lost on every reboot.** `binfmt_misc` is a kernel mount
written at runtime, so a machine that built arm64 happily last week fails on it
today with no other change — and the symptom, `exec format error` from inside
the container, names neither binfmt nor the reboot. `build.sh` checks for the
handler before building anything and says so.

The output is `ubersdr-ntp_<arch>` using Go's `GOARCH` spellings, matching how
`ubersdr-clock` and the other UberSDR binaries are installed so several
architectures can share one directory.

To run on a target host, install the runtime libraries — `libstdc++` is linked
statically, the rest are not:

```bash
sudo apt-get install -y libopus0 libcurl4 libssl3
```

## Run

```bash
# Quickest possible thing, no config file:
./ubersdr-ntp_amd64 --source https://sdr.example.org@10 --port 12300

# What you actually want:
cp config.example.json config.json      # then edit it
./ubersdr-ntp_amd64 --config config.json
```

Port 123 is privileged. Grant just that capability rather than running as root:

```bash
sudo setcap 'cap_net_bind_service=+ep' ./ubersdr-ntp_amd64
```

Then point a client at it:

```bash
ntpdate -q -p 3 -u 127.0.0.1        # or, on a non-default port:
chronyc -h 127.0.0.1 -p 12300 tracking
```

### As a service

```ini
[Unit]
Description=NTP server disciplined by WWV over UberSDR
After=network-online.target
Wants=network-online.target

[Service]
ExecStart=/usr/local/bin/ubersdr-ntp --config /etc/ubersdr-ntp.json
ExecReload=/bin/kill -HUP $MAINPID
AmbientCapabilities=CAP_NET_BIND_SERVICE
CapabilityBoundingSet=CAP_NET_BIND_SERVICE
NoNewPrivileges=yes
DynamicUser=yes
Restart=always
RestartSec=10

[Install]
WantedBy=multi-user.target
```

`SIGHUP` reopens the log file, for logrotate. `SIGUSR1` writes a status block
immediately instead of waiting for the next interval.

## Tuning is automatic, and not negotiable

Give it the **transmitter's carrier** — `carrier_hz: 10000000` for the 10 MHz
outlet — and it tunes USB 1 kHz below, with the passband open to 3 kHz. Do not
subtract the kilohertz yourself.

| Station | Carrier | Dial it tunes | Why |
|---|---|---|---|
| WWV / WWVH | 2.5, 5, 10, 15 MHz | carrier − 1 kHz | Puts the RF carrier at 1000 Hz audio, the 100 Hz BCD subcarrier at 900/1100 Hz, and the seconds tick at its 2000 Hz (WWV) / 2200 Hz (WWVH) image |
| WWV only | 20, 25 MHz | carrier − 1 kHz | 25 MHz is an experimental broadcast: real, but intermittent and lower power |
| WWVB | 60 kHz | 59 kHz | Puts the 60 kHz carrier at ~1000 Hz audio, where the PWM rides on its amplitude |

**The passband must reach 2.2 kHz**, which is why it asks for 0–3 kHz and why
you should not narrow it. The WWV/WWVH second edge is recovered *entirely* from
the tick's audio image, and the station tag is decided by which of the 2000/2200
Hz bands folds to an impulse. A 2.4 kHz SSB filter clips one or both and the
decoder sits in `acquiring` for ever with `tone_detected: false`.

WWVB is chosen automatically for any dial below 1 MHz: it is a genuinely
different decoder — pulse-width modulation on the carrier's own amplitude
against a 100 Hz BCD subcarrier — not a setting.

## Audio format

**Opus is the default** and it works: measured against synthetic WWV through an
encode/decode round trip at exactly the server's settings (12 kHz, 24 kbps,
`APPLICATION_VOIP`, complexity 5), the decoder still reaches quality-100 lock at
10, 6 and 3 dB SNR, with the second edge landing a consistent 5–10 ms late. That
is a *bias*, not jitter — it did not move between clean and noisy signals — and
the delay model accounts for it.

`"format": "pcm-v4"` is the predictive lossless codec instead: about four times
the bandwidth, and no codec delay to account for. Worth it on a receiver on your
own network, rarely worth it over the internet. (The query parameter on the wire
is still spelt `pcm-zstd` for compatibility with older servers; version 4
carries no zstd at all.)

## The delay model

This is where the accuracy actually lives, so it is worth understanding.

The decoder reports what UTC the *transmitter* was sending. NTP needs to know
what UTC it is *here, now*. Between them sits a one-way delay that nothing in the
audio stream measures:

| Term | Typical | How it is obtained |
|---|---|---|
| Propagation | 9–45 ms | Computed, from the receiver's published coordinates to whichever transmitter the decoder says it is hearing |
| Network | 5–100 ms | Measured, as half the TLS round trip to the server |
| Codec | 8 ms (Opus), 0 (PCM v4) | A measured constant |
| `extra_delay_ms` | **unknown** | Yours to set — chiefly the receiver's internal buffering |

The first three are handled. The fourth is the one that matters and the one
nothing here can see: the buffering between `radiod` and the WebSocket is real,
is tens of milliseconds, and is invisible from this end because a constant delay
is indistinguishable from a clock that is simply wrong — which is the very thing
being measured.

Leave `extra_delay_ms` at zero until you have something to calibrate against.
Then set it per source to whatever that source's offset reads when it disagrees
with a reference you trust. The status page and `/api/status` break the total
down term by term so you can see what you are adjusting.

Getting it wrong does not break anything; it biases the served time by exactly
the amount you got it wrong by. Leaving it at zero biases the served time by
whatever the receiver's buffering is.

### How the offset is measured at all

The audio arrives over TCP, so each packet's arrival carries a transport delay
that is bounded below and unbounded above — a packet can be arbitrarily late but
never early. Least squares through raw arrival times therefore fits the middle
of the delay distribution and moves with network load.

So `SampleClock` buckets the arrivals, keeps the *minimum* residual in each
bucket, and fits a line through that lower envelope — the same estimator NTP's
clock filter and PTP both use. The line's intercept is the sample-to-host anchor
and its slope is the receiver's sample-clock error against this host's, which it
reports in ppm. In practice the fit residual runs well under a millisecond.

A `time` event — a voted, plausibility-checked timestamp — arrives once a minute
on WWV, which is a thin diet for a filter. But it anchors the UTC of one sample
index, and every second edge after it is exactly one second later. So the anchor
is extended forwards and each second edge yields an independent measurement of
the same offset: sixty a minute instead of one, from the same voted timestamp.
Those are reduced by median and MAD, which an occasional edge landing on a fade
cannot drag.

## Several sources

Each source runs its own session, its own decoder and its own offset estimate.
Combining them is the same problem NTP solves, so it is solved the same way:

1. **Candidates** — locked, fresh, and with enough measurements to have been
   filtered.
2. **Intersection** — each candidate asserts an interval it believes contains
   the true offset; the largest agreeing set wins. A source that is
   *confidently wrong* is discarded rather than averaged in. This matters here
   more than usual: a deep fade biases the same bits in every frame of the
   decoder's voter window, so the misread is unanimous with maximum margin and
   no metric inside one decoder can catch it. An independent receiver can.
3. **Combine** — survivors weighted by 1/dist², plus their spread, so two
   sources 40 ms apart cannot produce an answer claiming 10 ms.

With one source there is nothing to intersect and the answer is that source's
offset and its dispersion — the correct and slightly humbling result.

Set `ntp.min_sources: 2` if you want the agreement test to be load-bearing.

### What two sources do and do not independently verify

Worth being precise about, because it is easy to buy less independence than it
looks like.

Two frequencies on **one receiver** are two independent decoders on two
independent propagation paths. That catches a misread — the thing the
intersection test is for — and it is what produced the 0.3 ms agreement above.
What it does *not* catch is anything the two share: the receiver's own
buffering, its sample clock, and the network path to it. Those are exactly the
terms the delay model is guessing at, so two such sources can agree closely
while both being biased by the same amount.

Two **different receivers** decorrelate those too, and are the only thing that
does. The cost is that their delay models are separate, so each needs its own
`extra_delay_ms`.

So: several frequencies on one receiver is good protection against a bad
decode, and no protection against a bad delay model. Several receivers is both.

### A practical limit

Public receivers usually cap concurrent sessions per IP — K3FEF allows two — so
several frequencies on one receiver often is not possible. Multi-source
redundancy in practice means several *receivers*, which is better anyway: it
decorrelates the propagation path as well as the frequency. A bypass password,
where you have one, also removes the session time limit; without one a receiver
may disconnect you, and re-acquiring a lock costs about four minutes of clean
signal.

## Reconnection

Forever, with exponential backoff from 2 s to **30 s**, jittered by ±25%.

Forever because no failure here is worth giving up on: a receiver that is full
will have room later, a band that is dead will open, a tunnel that is down will
come back. A time server that stopped trying at 3am and was still stopped at
noon is worse than useless, because it looks like it is working.

Capped at 30 s because `/connection` is rate limited at ten requests a minute
per IP, and a daemon that hammers it turns a transient refusal into a persistent
one. Jittered because every source fails at the same instant when a receiver
goes down, and without jitter they then retry in lockstep for ever — arriving as
exactly the burst the rate limit exists to stop.

Each attempt uses a **fresh session UUID**. The server binds a UUID to an IP and
remembers that it kicked one, so carrying a rejected id into the next attempt
would inherit the rejection.

The backoff resets only once a connection has been streaming for 30 s — proved
itself, rather than merely opened. A receiver that accepts the WebSocket and
drops it a second later, which is what hitting a session limit looks like from
this end, would otherwise reset the backoff every time and be retried every two
seconds indefinitely.

The log says the first three failures and then one line every five minutes
carrying the count, so a source unreachable overnight does not leave seven
hundred identical warnings behind.

A connection is also torn down and remade if audio stops for 10 seconds. Audio
arrives 50 times a second, so ten seconds of silence is unambiguous — and a
WebSocket over a path that has silently gone away stays "open" until the OS
gives up on the TCP connection, which can take minutes.

## Identifying itself

Every HTTP and WebSocket request carries:

```
User-Agent: ubersdr-ntp/1.0.0 (+https://github.com/madpsy/ubersdr-ntp)
```

That is the only thing telling a receiver operator what has taken one of their
slots, so it names the program, the version and where to look it up — and it
lets a receiver that filters by User-Agent allow or refuse this specifically
rather than guessing. Overridable with `user_agent`, but identifying the program
is the point.

## When nothing is locked

It keeps answering, coasting on the last good offset with root dispersion
growing at `coast_drift_ppm` (15 ppm, NTP's own assumed wander for an
undisciplined clock) until `coast_seconds`. Past that it answers stratum 16 with
LI=3 — unsynchronised — which tells a client to look elsewhere immediately
rather than making it wait for a timeout. That is what a real refclock does.

## Watching it

With `log.file` set, the log carries one line per state change plus a detailed
per-source block every `status_interval_seconds` — which is how you see what
each source is doing under systemd with no terminal:

```
source         link       stn    state     stage      tickdB  vote    qual      offset      disp       n
--------------------------------------------------------------------------------------------------------
wwv10          streaming  wwv    locked    locked       18.3   4/8    100%    +12.4 ms    21.0ms      97
wwv15          streaming  unknown acquiring no tick       0.4   0/8       -           -         -       0
```

The `stage` column is the useful one. A time-signal decoder that is not working
looks exactly like one working on a dead band — it says nothing either way — so
the first failing stage of the acquisition funnel is named:

| Stage | Meaning |
|---|---|
| `no audio` | The link is not delivering samples |
| `no tick` | No seconds tick in the audio. **Usually a passband too narrow to pass the 2 kHz image** |
| `no edge` | The tick is there but its phase has not settled. Fading; give it a minute |
| `no frame` | Seconds are being classified, the minute has not been located. Needs ~2 clean minutes |
| `voting` | Collecting. Two consecutive good minutes are the minimum |
| `quality_floor` | The window agrees but not confidently enough to certify a time |
| `plausibility` | A time was decoded more than a day from this host's clock. Refusing it is correct |
| `staleness` | The signal was there and has gone |
| `contested` | The frames in the window disagree with each other |
| `locked` | Working |

## HTTP API

Read-only without qualification: no route changes anything, no route takes a
body, and anything but `GET`/`HEAD` is answered 405. Binds `127.0.0.1:1234` by
default — it serves no passwords, but it does advertise which receivers this
daemon uses.

| Route | |
|---|---|
| `/` | The status page. Driven by the event stream, so it ticks with the broadcast |
| `/api/events` | **SSE.** One `tick` event per *corrected* second — the instant the broadcast's own second rolls over, not this host's — plus a `status` event every 5 s with the full document |
| `/api/time` | The time, for clients that do not speak NTP |
| `/api/status` | Everything this daemon knows, pretty-printed |
| `/api/sources` | Just the per-source array |
| `/api/health` | 200 when synchronised, 503 when not, tiny either way |

### `/api/time`

NTP over UDP remains the primary and the better interface — it is what
disciplines a system clock, and TCP plus TLS make HTTP asymmetric in a way that
costs accuracy. But the answer carries the same receive/transmit pair, so a
client can do the same round-trip correction:

```bash
curl -s "http://127.0.0.1:1234/api/time?t=$(date +%s.%N)" | jq
```

```json
{
  "synchronised": true, "stratum": 1, "refid": "WWV",
  "unix": 1789220625.417, "unix_ms": 1789220625417,
  "utc": "2026-09-12T14:23:45Z",
  "offset_ms": -34.2, "dispersion_ms": 22.5,
  "reference_age_seconds": 12.4, "sources_used": 2, "leap_pending": false,
  "roundtrip": {
    "originate": 1789220625.331, "receive": 1789220625.402,
    "transmit": 1789220625.417, "server_raw_receive": 1789220625.436
  }
}
```

`offset = ((receive − originate) + (transmit − destination)) / 2` and
`delay = (destination − originate) − (transmit − receive)`, exactly as in
RFC 5905. A client that just wants the time reads `unix` and ignores the rest.

### `/api/events`

```bash
curl -N http://127.0.0.1:1234/api/events
```

The `tick` fires on the corrected second boundary, so a display driven from it
ticks with WWV rather than with the machine it is running on — and keeps doing so
across a change in the offset, because the boundary moves with the correction.

## What is in here from elsewhere

`src/clock/WwvDecoder.*`, `src/clock/WwvbDecoder.*` and
`src/clock/TimeFrameVoter.*` are copied **verbatim** from
[ubersdr-clock](https://github.com/madpsy/ubersdr-clock), which took them
verbatim from [AetherSDR](https://github.com/aethersdr/AetherSDR), where they are
the DSP half of its AetherClock feature. They are kept byte-identical on purpose:
it makes an upstream fix a straight `cp`, and the voter in particular carries
calibration constants tuned against live WWV corpora that should not drift
without upstream's evidence behind the change.

`third_party/pcm_v4.hpp` is shared verbatim with `ka9q_ubersdr/clients` — keep it
in step with the copies there. It has no Opus reader, because the C++ clients
that share it are IQ clients and never negotiate Opus; `src/OpusV4Header.h` is
that half, written here rather than patched in.

`third_party/IXWebSocket` and `third_party/json.hpp` are vendored as-is.

Format facts throughout are per NIST SP 432 (WWV/WWVH) and NIST SP 250-67
(WWVB).

## Licence

GPL-3.0-or-later, inherited from AetherSDR by way of ubersdr-clock. See
[LICENSE](LICENSE).
