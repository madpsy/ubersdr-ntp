# ubersdr-ntp

An NTP server disciplined by WWV, WWVH, WWVB or DCF77, heard over one or more
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

It can also take time from ordinary **upstream NTP servers**, as a second and
deliberately different class of source — see [Two classes of
source](#two-classes-of-source). The band dies every night and takes every
receiver hearing the same transmitter with it; the network dies for its own
unrelated reasons. Neither failure covers the other.

## What this is, and what it is not

It is a **stratum-1 radio clock**: its reference is not another NTP server. That
is a statement about topology, not accuracy.

(That holds while the time is coming from the radio, which is the default and
the point. If it has failed over to an upstream NTP server it is stratum *N+1*
for that server's *N*, and it says so in every packet — see [Two classes of
source](#two-classes-of-source).)

### How accurate is it, actually

It will never be exact. What follows is what it measured, under conditions that
are stated so the number can be read for what it is.

**Measured 2026-09-13, 23:42–00:49 UTC**, 122 samples at 30 s intervals. Two
public receivers in the north-eastern US, both hearing Fort Collins on 10 MHz,
about 2500 km from the transmitter and about 95 ms of network round trip from
the client. The reference is the client's own host, disciplined by `ntpd` and
sitting 0.15 ms from its server.

| | |
|---|---|
| **Served offset against the reference** | **+3.4 ms mean** |
| Stability of that, over the hour | 1.7 ms sd, +1.5 to +8.6 ms |
| Agreement between the two receivers | 2.3 ms mean, ±11 ms worst |
| Root dispersion it claimed while doing so | 17–27 ms |

So: **a few milliseconds, biased a few milliseconds fast, on a path the delay
model fits.** The true error stayed well inside the dispersion advertised, which
is the property that matters — a client is told to trust it less than it
deserves, never more.

Three things that number is not:

- **It is not exact, and part of it was a systematic since corrected.** Every
  source shares the chain constant, so no arrangement of receivers can measure
  it and only an absolute reference can. The NTP class later measured it about
  1 ms too large, and it was lowered; see [the delay model](#the-delay-model).
  The figures above predate that.
- **It is one evening, one band, one client location.** HF propagation is not
  the same at 03:00 as at 23:00, and the same two receivers wandered ±11 ms
  against *each other* over this hour — which bounds how well the per-source
  delay models fit, and is larger than the bias being chased.
- **The reference is `ntpd`, not GPS.** It is good to well under a millisecond
  here, but it is not an independent primary standard.

The terms that dominate the remaining error are *path-specific*: how symmetric
the network route is, and how much the receiver buffers between `radiod` and the
WebSocket. Both were small here. On a path where the route is asymmetric, or the
receiver buffers more, the error is tens of milliseconds until `extra_delay_ms`
is set — and that is what the root dispersion is claiming.

**Tens of milliseconds by default, single-digit milliseconds when the delay
model fits the path.** A client weighing this against a GPS source will
correctly prefer the GPS, which is the intended outcome.

An earlier session on one receiver, hearing 10 and 15 MHz at once, agreed with
itself across the two frequencies to **0.3 ms** — the more transferable number
of the two. Those are independent decoders on independent propagation paths
sharing only a receiver and a network link, so that figure bounds everything
except what they share, which is exactly the part the delay model is for. The
3.4 ms above is what happens when the shared part is included.

It is **not** a substitute for a local GPS reference, and it should not be the
only source on a machine that has one. What it is good for is a machine with no
reference of its own, a sanity check against a clock you do not trust, or the
pleasure of running a clock off a shortwave broadcast.

The residual uncertainty is concentrated in the constants every source shares,
and they are the ones nothing in this arrangement can measure: the chain-delay
constant, the codec delay, and the decoder's edge bias all cancel between
receivers. The chain constant has since been calibrated against the NTP class
(see the delay model), but never against an off-air recording at the two rates
used here.

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

### Tests

`build.sh` runs `tools/selftest.py` against each release binary — it drives the
real NTP socket and the real HTTP service, including synchronising from an
upstream NTP server (a fake one on the loopback, so it needs no network) and
publishing to MQTT through a fake of UberSDR's addon ingest port that applies
the receiver's own validation to every topic and entity. The rest are offline,
need no receiver and no network, and are built alongside the daemon:

```bash
./build-native/ubersdr-ntp-decodertest   # the DSP: synthesised WWV/WWVH/WWVB
./build-native/ubersdr-ntp-dcf77test     # DCF77 AM + PM, synthesised and a real recording
./build-native/ubersdr-ntp-clocktest     # offset/rate estimator, sample clock, Selector
./build-native/ubersdr-ntp-ntptest       # the NTP client, against a fake server
./build-native/ubersdr-ntp-configtest    # the configuration reader
./build-native/ubersdr-ntp-eventtest     # the event log's transitions, and the metric history's bounds
python3 tools/selftest.py ./build-native/ubersdr-ntp
```

`clocktest` covers the two-class logic: failover and failback, the
[anti-oscillation hysteresis](#failing-over-and-not-oscillating), the stratum
rule, and that the agreement test never lets one class convict the other.
`ntptest` points a real peer at a fake server whose clock is deliberately wrong
by a known amount, and checks that the offset is recovered, that queued replies
are filtered out, and that a forged reply, an unsynchronised server, an
excessive stratum or root distance, and a synchronisation loop are each refused
with a reason a person can read.

## Run

```bash
# Quickest possible thing, no config file:
./ubersdr-ntp_amd64 --source https://sdr.example.org@10 --port 12300

# What you actually want:
cp config.addon.json config.json        # then edit it
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

### As an UberSDR addon

On a machine running an [UberSDR](https://ubersdr.org) receiver, it installs as
an addon container beside it, like the receiver's other addons:

```bash
curl -fsSL https://raw.githubusercontent.com/madpsy/ubersdr-ntp/main/install.sh | bash
```

That sets up `~/ubersdr/ntp/` with the compose file, the start/stop/restart/update
scripts and `config/config.json`. Out of the box it listens to the local
receiver on WWV's 5, 10 and 15 MHz, with `time.cloudflare.com` as its network
reference. On a first install it reads the receiver's own location from
`/api/description`, and if that is within 2000 km of Mainflingen and the
receiver tunes down to 77.5 kHz it listens to DCF77 alone instead, with the
WWV sources left in the file but disabled. `config.addon.json` documents every setting, including why adding
more upstreams is worth it. The addon container is on the receiver's
own Docker network, which UberSDR's default `timeout_bypass_ips` exempts from
session limits, so it needs no password. Edit `config/config.json` and
`./restart.sh` to change any of it; `install.sh` never overwrites that file.

Then add it under **UberSDR Admin → Addon Proxies** (or from `addons.json`):

| Field | Value |
|---|---|
| Name | `ntp` |
| Host | `ntp` |
| Port | `6099` |
| Enabled | `true` |
| Strip prefix | `true` |
| Rate limit | `100` |

The status page is then at `http://your-ubersdr-host/addon/ntp/`. The page
addresses the API relatively, so it works the same there as at the root of its
own port. It publishes to MQTT and Home Assistant through the receiver with no
further setup ([MQTT and Home Assistant](#mqtt-and-home-assistant)).

**NTP itself is not published yet.** The container listens on 123/udp inside
the Docker network only; the compose file has no `ports:` section at all.

The images are multi-architecture, amd64 and arm64, built from source on
`ubuntu:24.04`:

```bash
./docker.sh build     # linux/amd64, loaded locally
./docker.sh arm64     # linux/arm64, loaded locally
./docker.sh push      # both, as one manifest, to madpsy/ubersdr-ntp:latest
```

### As a service

A hardened unit is in [`systemd/ubersdr-ntp.service`](systemd/ubersdr-ntp.service).
It runs as a dynamic user with only `CAP_NET_BIND_SERVICE`, and needs
**systemd 247 or later** (Ubuntu 22.04, Debian 11 and newer):

```bash
sudo install -m 0755 ubersdr-ntp_amd64 /usr/local/bin/ubersdr-ntp
sudo install -m 0600 -o root -g root config.json /etc/ubersdr-ntp.json
sudo install -m 0644 systemd/ubersdr-ntp.service /etc/systemd/system/
sudo install -m 0644 logrotate.ubersdr-ntp /etc/logrotate.d/ubersdr-ntp   # if log.file is set
sudo systemctl daemon-reload
sudo systemctl enable --now ubersdr-ntp
```

The essential lines:

```ini
[Service]
LoadCredential=config:/etc/ubersdr-ntp.json
ExecStart=/usr/local/bin/ubersdr-ntp --config ${CREDENTIALS_DIRECTORY}/config
ExecReload=/bin/kill -HUP $MAINPID
AmbientCapabilities=CAP_NET_BIND_SERVICE
CapabilityBoundingSet=CAP_NET_BIND_SERVICE
LogsDirectory=ubersdr-ntp
DynamicUser=yes
NoNewPrivileges=yes
ProtectSystem=strict
Restart=always
RestartSec=10
```

Two of those are there because of `DynamicUser=`, whose UID is not known until
the service starts:

- **The configuration holds receiver passwords**, so it stays `0600 root`.
  The service could not read it directly without making it world-readable, so
  `LoadCredential=` has systemd read it at start and hand the service a private
  copy under `$CREDENTIALS_DIRECTORY`. After editing `/etc/ubersdr-ntp.json`,
  restart the service. Reloading does not refresh the copy.
- **The log file must live in `/var/log/ubersdr-ntp/`**, which `LogsDirectory=`
  creates and gives to the service's UID on every start. Set
  `"log": { "file": "/var/log/ubersdr-ntp/ubersdr-ntp.log" }`. Anywhere else is
  read-only under `ProtectSystem=strict`, and the daemon exits if it cannot open
  its log file.

On a systemd older than 247, use a fixed user instead:
`sudo useradd --system --no-create-home ubersdr-ntp`. In the unit, replace
`DynamicUser=yes` and `LoadCredential=` with `User=ubersdr-ntp` and
`Group=ubersdr-ntp`, and point `--config` at `/etc/ubersdr-ntp.json`. Make that
file `root:ubersdr-ntp 0640`, and give the user `/var/log/ubersdr-ntp` if you
set `log.file`.

`SIGHUP` reopens the log file, for logrotate. `SIGUSR1` writes a status block
immediately instead of waiting for the next interval.

## Tuning is automatic, and not negotiable

Give it the **transmitter's carrier** — `carrier_hz: 10000000` for the 10 MHz
outlet — and it tunes USB 1 kHz below, with the passband open to 3 kHz. Do not
subtract the kilohertz yourself. DCF77 is the exception: it is tuned on the
carrier, as IQ.

| Station | Carrier | Dial it tunes | Why |
|---|---|---|---|
| WWV / WWVH | 2.5, 5, 10, 15 MHz | carrier − 1 kHz | Puts the RF carrier at 1000 Hz audio, the 100 Hz BCD subcarrier at 900/1100 Hz, and the seconds tick at its 2000 Hz (WWV) / 2200 Hz (WWVH) image |
| WWV only | 20, 25 MHz | carrier − 1 kHz | 25 MHz is an experimental broadcast: real, but intermittent and lower power |
| WWVB | 60 kHz | 59 kHz | Puts the 60 kHz carrier at ~1000 Hz audio, where the PWM rides on its amplitude |
| DCF77 | 77.5 kHz | 77.5 kHz, **IQ ±6 kHz** | The carrier at 0 Hz in complex baseband, so its phase is there to be read; always lossless |

**The passband must reach 2.2 kHz**, which is why it asks for 0–3 kHz and why
you should not narrow it. The WWV/WWVH second edge is recovered *entirely* from
the tick's audio image, and the station tag is decided by which of the 2000/2200
Hz bands folds to an impulse. A 2.4 kHz SSB filter clips one or both and the
decoder sits in `acquiring` for ever with `tone_detected: false`.

The tag matters beyond the display: it chooses which transmitter the
propagation delay is modelled from, and WWV and WWVH are 14 ms apart on a
European path. So it is made to be steady:

- On 20 and 25 MHz it is fixed to WWV. Nothing else transmits there.
- On a shared carrier, one tick band has to lead the other by 1.5× (+1.8 dB)
  for 10 s in a row before a tag is adopted. Once adopted, a lead of 1.2×
  (+0.8 dB) is enough to keep it. It switches after 30 s of the other station
  clearly leading, and is dropped only after 2 minutes with no support.
- A reconnect, or a restart the consensus orders, starts the new decoder from
  the tag the last one held, if that tag was backed by a heard tick within
  15 minutes. It is judged again as soon as the new decoder has heard enough.
- While there is no tag, the status page shows which way the tick leans
  (`WWV?` / `WWVH?`, with the ratio on hover) rather than a blank. Every
  change is logged with the ratio behind it.

WWVB is chosen automatically for any other dial below 1 MHz: it is a genuinely
different decoder — pulse-width modulation on the carrier's own amplitude
against a 100 Hz BCD subcarrier — not a setting.

### DCF77: amplitude and phase, both at once

`carrier_hz: 77500` is DCF77, from Mainflingen, and nothing else about it needs
setting. It is taken as UberSDR's `iq` mode — 12 kHz of complex baseband, which
every receiver offers publicly and always sends lossless (`format` is forced to
`pcm-v4`; the URL says `min_margin=0`) — because half of DCF77 is in the
carrier's **phase**, and a demodulated audio channel keeps none.

The station sends its time code twice over. The **AM** is the classic one: the
carrier cut to 15% for 0.1 s (a 0) or 0.2 s (a 1) at the start of every second,
second 59 left alone to mark the minute. The **PM** is a 512-chip pseudo-random
phase code at ±15.6°, from 200 ms to 993 ms of every second, inverted for a 1 —
spread spectrum, and correlated here to find the second to tens of
microseconds and to hold on through noise that buries the AM. Both run every
second, and each checks the other:

- **Timing** comes from PM whenever its correlator is tracking, from AM
  otherwise. The log says which each time it changes, and the status page
  shows it with the two edges' difference — AM's edge minus PM's is the one
  check AM timing has, and reads a fraction of a millisecond on a healthy path.
- **The minute** is decoded twice, from PM's bits and from AM's, each through
  its own parity checks. Both valid and the same: taken. One valid: taken from
  that one — which is AM carrying on when PM fades, or PM when the AM cut is
  lost in the noise. Both valid and *different*: nothing is certified from it,
  and the page says `refused: AM and PM read different times`.
- **Where the minute starts** is found by either: AM's uncut second 59, or the
  sixteen seconds (59, 0–14) PM always sends as a fixed pattern. Each then
  vetoes the other's contradicting reads.

The code is in CET/CEST and names the minute *about to begin*; it is converted
to UTC before it is voted on, so the CET/CEST changeover is a non-event, and a
leap second (a 0 in second 59, an uncut second 60) is handled as WWVB's is.

Which way round the phase reads is learnt from those fixed sixteen seconds, not
assumed: a KiwiSDR recording reads the opposite way to PTB's description taken
literally, and whether a receiver's I/Q is conjugated is its own business.

Interference is expected at LF, and three things keep it from deciding
anything. The carrier is only looked for within ±3 Hz of 77.5 kHz — it is an
atomic standard, so only the receiver's own clock can move it, and 20 ppm is
1.6 Hz — which stops a strong line nearby being taken for it. A burst whose
phase carries more low-frequency power than the chips can account for is
correlated through a zero-phase high-pass, so a steady tone near the carrier
neither flips PM bits nor passes as noise; the status page says when this is
happening. And a PM lock that puts the second somewhere AM — while decoding
valid minutes — does not, is refused rather than allowed to throw away AM's
count of seconds, and the refusals are counted on the page.

`tools/dcf77test.cpp` synthesises all of that — noise down to 24 dB-Hz, a
carrier off DC, a tone twice the carrier's strength 10 Hz from it, five-second
fades at the end of a minute, AM with no PM, PM with no AM, inverted I/Q, a leap
second, the changeover, and AM and PM that disagree — and then decodes
`tools/testdata/dcf77_live.wav`, five minutes of real DCF77 from a KiwiSDR
35 km from the transmitter. On that recording AM and PM agree on every minute,
PM's edges sit on a straight line to better than the 0.08 ms sample grid can
show, AM's edges fall 0.2 ms after PM's, and the time read is 09:20 UTC on
Sunday 2026-05-24, which is what the recording's own README shows. Live, on an
UberSDR 290 km from the transmitter, both read every minute alike -- a
five-second fade apart, which both refused -- and AM's edges sat 0.1 to 0.3 ms
after PM's.

One thing is not known yet: the UberSDR chain constant below was measured on USB
sessions, and IQ is served from a different radiod preset. The class delta on a
DCF77 source is what will say whether it holds, and it is applied unchanged
until that has been measured properly.

## Audio format

**PCM v4 is the default**: the predictive lossless codec, asked for at full
quality (`min_margin=0`, the same as DCF77's IQ). No codec delay to account for,
so one fewer constant in the delay model, at about four times the bandwidth of
Opus. (The query parameter on the wire is still spelt `pcm-zstd` for
compatibility with older servers; version 4 carries no zstd at all.)

`"format": "opus"` works too, for a slow or metered link: measured against
synthetic WWV through an encode/decode round trip at exactly the server's
settings (12 kHz, 24 kbps, `APPLICATION_VOIP`, complexity 5), the decoder still
reaches quality-100 lock at 10, 6 and 3 dB SNR, with the second edge landing a
consistent 5–10 ms late. That is a *bias*, not jitter — it did not move between
clean and noisy signals — and the delay model accounts for it (8 ms).

## The delay model

This is where the accuracy actually lives, so it is worth understanding.

The decoder reports what UTC the *transmitter* was sending. NTP needs to know
what UTC it is *here, now*. Between them sits a one-way delay that nothing in the
audio stream measures:

| Term | Typical | How it is obtained |
|---|---|---|
| Propagation | 9–45 ms | Computed, from the receiver's published coordinates to whichever transmitter the decoder says it is hearing |
| Network | 5–100 ms | Measured, as half the round trip of a JSON ping that the receiver itself answers, down the audio connection |
| Codec | 8 ms (Opus), 0 (PCM v4) | A measured constant |
| UberSDR chain | 14.1 ms | A constant: RF reaching the SDR to audio leaving the WebSocket. Calibrated against the NTP class — see below |
| Decoder bias | −13.6 ms (WWV/WWVH), 0 (WWVB, DCF77) | A measured constant, from `tools/decodertest.cpp` and `tools/dcf77test.cpp` |
| `extra_delay_ms` | 0 | Yours, for anything genuinely local |

The network term is measured over the **WebSocket**, not with an HTTP request,
because a TCP handshake ends at whatever accepted the SYN — and most UberSDR
instances are reached through a tunnel that terminates TCP near the client, not
at the receiver. One measured here answered its handshake in 12 ms for an origin
94 ms away, which cost 40 ms of delay budget and put its offset 39 ms from a
receiver on the same band it should have agreed with to a millisecond.

The ping is UberSDR's own JSON `{"type":"ping"}`, timed to its `pong`, and not
a WebSocket protocol ping. A proxy that terminates the WebSocket answers protocol
pings itself, and `tunnel.ubersdr.org` does: through it, one receiver's protocol
ping came back in 13.4 ms while its JSON ping took 33.7 — and the half of that
difference missing from the delay model read as a source 10 ms late. The JSON
ping is an ordinary message that every proxy passes on and only the receiver can
answer. On a receiver served directly the two agree (30.5 and 30.6 ms), so
nothing calibrated against the old measurement moves.

Every source is measured this way, including the ones reached directly, and that
is deliberate. The handshake is the cleaner ruler where it is valid — a SYN is
answered by the kernel, a ping waits for the server's event loop, which costs
between 0.6 and 6.5 ms on the instances measured here — so it is tempting to
prefer it when the two agree. Doing that would measure different sources with
different rulers, which turns a *shared* error into a *per-source* one. The
shared kind is removable: it lands in the same pile as the chain constant and
one calibration takes it out. The per-source kind is not, because the sources
are the only check on each other. So the handshake is still measured and shown
as a raw figure, but nothing classifies an instance by how the two compare, and
the handshake does not get a vote.

The HTTP figure is used only when a server never answers a ping at all.

### Calibrating the chain constant against NTP

The terms every source shares cannot be measured by comparing sources: two
receivers hearing the same transmitter cancel the chain delay, the codec delay
and the decoder bias exactly, so they agree just as well whatever those are set
to. Only a reference outside the radio can see them, and one evening against a
host clock could not separate a fixed over-count from that night's ionosphere
(served time ran +3.4 ms, then +5.4 ms, while the receivers wandered ±11 ms
against each other).

The NTP class is that outside reference, measured continuously. With two
receivers primary and `time.cloudflare.com` in standby, the page's *Difference*
— radio minus NTP — held between **+0.8 and +1.5 ms for many hours**: the radio
put UTC about 1 ms late. A per-path error does not hold that steady across hours
and two receivers; a shared constant does. So the chain constant was lowered
from 13.6 ms to **12.6 ms** (2026-09-17). After the change the Difference should
sit near zero; if it settles somewhere else, that is the next correction.

That move was later withdrawn, and the constant is **14.1 ms**. The 12.6 had been
fitted while the propagation model was a strict lower bound, so the reference
carried a one-signed deficit of its own; 11.5 settled hours of class delta on a
receiver 2516 km away put it back at 14.1 (2026-09-20), and 64 further minutes,
with the crystal's rate shared across classes and a second upstream checking the
first, centred on zero (mean +0.11 ms, sd 0.49). Anything under about twenty
minutes of the smoothed delta describes the last disturbance rather than the
constant: two of the three moves of it were read off windows that short.

It was calibrated on WWV sources over Opus, so it carries whatever error sits in
the terms those sources subtract and a DCF77 source does not -- the WWV decoder
bias and the Opus delay -- and any asymmetry in the transatlantic paths it was
measured over. Two DCF77 receivers in Belgium and France both read ahead of a
GPS-disciplined server by a shared ~2.3 ms (2026-09-22). Which of those it is
has not been separated yet, so nothing has been moved for it.

The part nothing in the stream can see is the delay inside the receiver —
`radiod`'s demodulator and filters, its block framing, the server's handling —
because a constant delay is indistinguishable from a clock that is simply wrong.
But it is a property of the software, the same on every UberSDR instance, so it
is one built-in constant rather than something to calibrate per receiver. It was
measured live against one receiver hearing WWV on 10 and 15 MHz, from a host
disciplined by ntpd: served time averaged +0.1 ms and stayed within ±2.6 ms over
14 minutes of lock. That is one receiver, so treat the constant as good to a few
milliseconds.

The decoder bias is separate because it differs by decoder: the WWV/WWVH decoder
reports second edges 13.6 ms early, the WWVB decoder is exact. On WWV the two
cancel almost exactly, which is why the earlier measurement below came out
near zero with neither term modelled; on WWVB only the chain term applies.

Leave `extra_delay_ms` at zero unless one source disagrees with a reference you
trust for a reason of its own. Getting it wrong biases the served time by
exactly the amount you got it wrong by. The status page and `/api/status` break
the total down term by term.

### How the offset is measured at all

The audio arrives over TCP, so each packet's arrival carries a transport delay
that is bounded below and unbounded above — a packet can be arbitrarily late but
never early. Least squares through raw arrival times therefore fits the middle
of the delay distribution and moves with network load.

So `SampleClock` buckets the arrivals, keeps the *minimum* residual in each
bucket, and fits a line through that lower envelope — the same estimator NTP's
clock filter and PTP both use. The line's intercept is the sample-to-clock anchor
and its slope is the receiver's sample-clock error against the daemon's clock
(below), which it reports in ppm. In practice the fit residual runs well under a
millisecond.

A `time` event — a voted, plausibility-checked timestamp — arrives once a minute
on WWV, which is a thin diet for a filter. But it anchors the UTC of one sample
index, and every second edge after it is exactly one second later. So the anchor
is extended forwards and each second edge yields an independent measurement of
the same offset: sixty a minute instead of one, from the same voted timestamp.

### Time does not jump

A time code is a few dozen bits, and a bit can be misread. A fade that biases
the weight-4 bit of the minutes, or of the hours, produces a frame that passes
every check the decoder can make and votes clean, yet is four minutes, or four
hours, out. Both have happened live: on 2026-09-17 one receiver decoded a time
240 s early and, that evening, the other 14 400 s early. With two receivers the
consensus cannot tell which of two is wrong, so the served time followed each
one. The decoder's plausibility bound (a day either side of the host clock) is
far too wide to catch it, and the host clock is set from this daemon anyway.

So each receiver checks every decoded time against **its own history**, and no
other source is consulted. Its filtered offset is taken against the daemon clock,
the raw oscillator (below), and that offset only moves at a crystal's rate. So,
carried forward along its measured rate, it predicts where the next decode
must land to a fraction of a second.

- A decode more than 0.5 s from that prediction is **refused**. Allowance is
  added for 50 ppm of drift since the last good offset. Every real effect is
  milliseconds; the smallest misread is a whole second. The anchor it would
  have replaced keeps extending, so the receiver goes on measuring the time it
  already had.
- The **first** time after the daemon starts has no history. It is used only once
  three readings of it over two minutes agree, so a misread at the moment of
  locking cannot become the history everything later is judged against.
- A history that is itself wrong must not stick for ever. This can happen when a
  bad first time slipped through, or when the machine was suspended with the
  daemon clock stopped. A new time that holds in every reading for **10
  minutes**, with none agreeing with the old history in between, replaces it.
  A misread comes and goes; a real step stays.
- An announced leap second (exactly one second, in the first hours of a month,
  after a warning) goes straight through.

The history survives a reconnect, a decoder restart and a re-acquisition. Those
are exactly when a misread lock is most likely, and the history describes UTC,
not the stream.

The selector respects the same rule. When the sources split into sets that do
not overlap and none is a majority (two receivers disagreeing), it keeps the set
closest to the time it is already serving. At startup, with nothing served yet,
it keeps the better-measured set. It used to keep whichever set had the lower
offset, and on 2026-09-17 that was the one four hours out.

A refusal says why on the status page and in the source's
details. It makes one `time_refused` event per run of refusals, and a point on
the *Refused times* chart, so a receiver that does this every night stands out.

### Its own clock

The radio decides what time it is, but only at each broadcast second, and late.
Between those instants something has to count: to put an arrival time on each
audio packet, and to read the moment an NTP request lands. That clock only has
to count — over a second even a cheap crystal is off by microseconds — but it
must not be a clock something else is steering.

The host's clock is exactly that. Whatever disciplines the host slews it, by
hundreds of ppm for a minute at a time while it corrects an offset, and a host
whose NTP client takes this daemon as its source slews it by what this daemon
serves, which closes a loop through the measurement. Measured on such a host,
the arrival fit saw the rate swing from +177 to −500 ppm within a minute,
refused its slope, and put about 40 ms of slope uncertainty on every edge: root
dispersion near 100 ms, and a served offset wandering by 25 ms.

So the daemon keeps its own clock: `CLOCK_MONOTONIC_RAW`, the machine's
oscillator with nothing applied to it, offset once at startup so it reads close
to Unix time. Packet arrivals, second edges, NTP receive and transmit stamps and
the HTTP API are all read on it, and the served time is that clock plus what the
radio measures it to be wrong by. The host clock is read only to report the
correction the host would need — `offset_ms`, "correction to this host" on the
page — and nothing is formed from it. It makes no difference to the served time
what, if anything, disciplines the host, including this daemon.

A raw crystal is steadily wrong by tens of ppm (the host this was written on
measured −12 ppm against public NTP), so its offset from UTC is a line rather
than a constant, and each source estimates both (`OffsetEstimator`):

- **the rate** from half an hour of second edges by least squares with gross
  outliers trimmed. What limits it is not the edges' scatter but the path
  delay's wander — milliseconds over minutes, which over a short span passes for
  rate (live, ten minutes of one receiver read anything from +23 to −21 ppm) —
  so none is used before ten minutes, and its uncertainty is judged from
  one-minute means, which carry that wander, as if the whole window might be one
  excursion of it;
- **the level** from the last two minutes, each edge carried forward along that
  rate, then the median and MAD — which an occasional edge on a fade cannot
  drag, and which lets a genuine step work its way out in two minutes.

Until ten minutes of history pin the rate, none is assumed and up to 50 ppm of
doubt is carried in the dispersion instead; several sources' rates are combined,
and no more tightly than they agree. The status block, the page
and `/api/status` show the rate each source measures and how well.

## Several sources

Each source runs its own connection and its own offset estimate — a receiver
its decoder, an upstream peer its clock filter — and each reports the same
quantity: UTC minus this daemon's own free-running oscillator. Combining them
is the same problem NTP solves, so it is solved the same way:

1. **Candidates** — locked, fresh, and with enough measurements to have been
   filtered.
2. **Intersection** — each candidate asserts an interval it believes contains
   the true offset; the largest agreeing set wins. A source that is
   *confidently wrong* is discarded rather than averaged in. This matters here
   more than usual: a deep fade biases the same bits in every frame of the
   decoder's voter window, so the misread is unanimous with maximum margin and
   no metric inside one decoder can catch it. An independent receiver can.
3. **Combine** — survivors weighted by 1/uncertainty², plus their spread, so
   two sources 40 ms apart cannot produce an answer claiming 10 ms.

The uncertainty that decides the weighting is each source's *own* — its jitter,
its sample-clock residual, and how well that clock's slope is known — and not
the interval it asserts in step 2. The asserted interval includes the delay
model's uncertainty, which is ~16 ms for any source on a typical path because
they all run the same model; weighting by a figure they share compresses the
ratio between a clean source and a struggling one until it stops meaning
anything. A receiver whose stream keeps forcing its sample clock to be rebuilt
loses its share of the vote on its own, without anyone having to set a weight by
hand, and regains it when the stream settles.

With one source there is nothing to intersect and the answer is that source's
offset and its dispersion — the correct and slightly humbling result.

### The consensus veto

Every source hears the same transmitter, so once each delay model has done its
job they must all report the same offset — not nearly, exactly, because the
event they are timing is one event. Each source's smoothed distance from the
median of all of them is therefore model error by construction, and it is
reported per source as `agreement`.

A source more than 30 ms from that median is refused outright. Nothing the
sources do not share reaches that far: the propagation difference between two
receivers on one continent is under 20 ms, and the largest genuine
disagreement available — one hearing WWVH in Hawaii while the others hear WWV
in Colorado — is about 19 ms and is modelled from the decoder's own station
tag. Out there, a source has decoded something else. Live, a receiver whose
edge tracker had locked about 100 ms late was refused 3325 times over forty
minutes while the other two were never refused once.

It is deliberately a veto and not a correction. Learning a per-source offset
that made a disagreeing receiver agree would have absorbed that 100 ms into its
delay model and reported a healthy source, turning the one fault the
arrangement can detect into one it cannot. The agreement figure says which
source to distrust; it is not licence to bend the model until nobody disagrees.

A refused source is not left refused. The veto protects the served time, but it
does not fix the source, and nothing inside one decoder can: an edge tracker
locked onto the wrong part of the pulse still puts every minute marker in the
right second, so the frame decodes and the lock holds for as long as the
connection does. So a source refused for five minutes without being accepted in
between is made to start over. Time it cannot be judged at all does not count as
acceptance: live, a third receiver that kept losing lock left the disagreeing
source with a single peer every few minutes, and a clock that restarted on that
never ran out. Made to start over, it — it drops its connection and acquires from nothing, and its
agreement history is discarded so the new lock is judged on its own. Sent back
again without having been accepted in between, it waits twice as long, up to an
hour; accepted once, the wait returns to five minutes. A receiver that is
genuinely broken costs one reconnection an hour, not one every few minutes, and
nobody has to restart anything. The status page shows the pending wait in the
source's reason, and `link.reacquisitions` counts how often it has happened.

Two sources cannot use any of this: their residuals come out equal and opposite
whichever of them is wrong, which is not a defect to be worked around but what
two measurements of one event can tell you. Three is where it starts to work.

Set `clock.min_radio_sources: 2` if you want the agreement test to be
load-bearing: the radio class then counts as healthy only while at least two
receivers are usable (locked, fresh and not refused), and with fewer it neither
serves nor holds off a failover.

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

Public receivers usually cap concurrent sessions per IP — two is typical — so
several frequencies on one receiver often is not possible. Multi-source
redundancy in practice means several *receivers*, which is better anyway: it
decorrelates the propagation path as well as the frequency. A bypass password,
where you have one, also removes the session time limit; without one a receiver
may disconnect you, and re-acquiring a lock costs about four minutes of clean
signal.

## Two classes of source

There are two kinds of reference: the **radio** sources above, and **upstream
NTP servers**. They are not interchangeable, and the difference is not that one
is more accurate — a public NTP server is very likely more accurate than this
daemon's own radio estimate. It is **what they fail to**. The band dies every
night and takes every receiver hearing the same transmitter with it. The network
dies for its own unrelated reasons. Neither covers the other, and a clock meant
to run for months unattended wants both.

So one class is the **primary** and the other the **secondary**:

```jsonc
"clock": {
  "primary":   "radio",     // or "ntp"
  "secondary": "standby",   // "always" | "standby" | "cold"
  "failover_after_seconds": 60,
  "failback_after_seconds": 300,
  "min_radio_sources": 1,  // receivers that must be usable for radio to be healthy
  "min_ntp_sources": 1     // ...and upstreams, for NTP
},
"ntp_sources": [
  "time.cloudflare.com",
  "0.pool.ntp.org",
  { "server": "192.168.1.1", "poll_seconds": 32, "weight": 2.0 }
]
```

As many upstreams as you like, exactly as with the radio sources, and for the
same reason: each is an independent candidate, so the agreement test that
catches one receiver misdecoding also catches one NTP server that has gone
wrong. A bare string is a hostname; `HOST:PORT` and `[v6addr]:PORT` also work.

A hostname is looked up again whenever its DNS TTL runs out (at least every 30
seconds' worth of TTL, and at most an hour apart). The address in use is kept
while the name still includes it, so a pool that answers in a different order
each time does not bounce the peer between servers. If a lookup fails, the
working address is kept and the lookup is retried every minute. An address
literal is never looked up.

A peer that is polled every 64 seconds produces one sample per poll, so a
lost packet or a slow reply costs more than it does for a receiver. A lost or
slow reply is retried two seconds later, up to twice. A reply is only dropped
as a queueing spike when its round trip is well outside what the path
normally does, and the third slow reply in a row is kept, because by then the
path itself has got slower. The Selector also allows an NTP peer's newest
sample to be four polls old rather than the radio sources' three minutes.

### The three secondary modes

They differ in two independent things — whether the secondary is **connected**,
and whether it **contributes**:

| | connected | contributes | failover costs |
|---|---|---|---|
| `always` | yes | yes | nothing; it is already in the answer |
| `standby` *(default)* | yes | only on failover | nothing; it is already warm |
| `cold` | only on failover | only on failover | minutes, while it acquires |

`standby` is the default because it is almost free and it buys two things
`cold` does not: the standby is **known to be working before it is needed**, and
the difference between the two classes is measured continuously — which is worth
more than the failover, and is the next section.

`cold` earns its place on a public receiver, which caps concurrent sessions per
address (two is common). A standby nobody is using should not hold one of them.

### The difference between the classes is the interesting number

Every radio source shares the same chain delay, the same codec delay and the
same decoder edge bias. Those terms **cancel exactly** in any comparison between
receivers, so no number of receivers can measure them — which is why [the chain
constant](#the-chain-constant-is-about-3-ms-too-large) has only ever been an
estimate, and why the 3.4 ms bias at the top of this README could be stated but
not attributed.

An upstream NTP server does not share them. So while both classes are measured
at once — `always`, and `standby`, which is what standby is *for* — the
difference between their consensuses is a **direct reading of the radio delay
model's absolute error**. It is on the status page as a headline figure, in
`/api/status` as `clock.class_delta`, and in the log block:

```
clock: primary is radio (2 candidate(s)), ntp is standby (2 candidate(s))
       serving from: primary
       radio minus ntp: +3.2 ms averaged (+2.9 ms now), over 41m, 2 vs 2 source(s)
         — the radio delay model's absolute error, which no number of
           receivers can measure. Reported, never applied.
```

**Reported, never applied.** Correcting the radio to agree with the network
would make the two agree by construction and destroy the only independent check
in the arrangement — and it would be steering a stratum-1 radio clock to match a
stratum-2 network one, which is the wrong way round on a machine whose whole
purpose is not depending on that network.

### Failing over, and not oscillating

The two hold-downs are deliberately different numbers. Failing over is cheap and
failing over late is expensive, so a minute is enough. Failing *back* costs a
step in the served time for no gain if the primary is about to drop out again —
and a decoder coming out of a fade does not return cleanly, it locks and loses
the lock repeatedly for several minutes. So the primary has to stay healthy for
five minutes before it is believed again.

That asymmetry is what keeps a flapping receiver from turning into a sawtooth on
every client. Measured in `tools/clocktest.cpp`, against a radio source flapping
for an hour with the two classes 6 ms apart:

| flap | switches in an hour | unsynchronised | worst step seen |
|---|---|---|---|
| 30 s up / 30 s down | **0** — it coasts through | none | 0.000 ms |
| 200 s up / 200 s down (9 cycles) | **1** | none | 6.0 ms |

Zero for the fast flap because 30 s never clears the 60 s hold-down; one for the
slow flap because the 200 s healthy half never reaches the 300 s failback
interval, so it fails over once and stays put rather than switching every cycle.
Hysteresis that never releases would be a one-way door, so that is checked too:
a primary healthy for longer than the failback interval does take back.

In `cold` mode the standby is brought up **the moment the primary drops**, not
after the hold-down — acquiring takes minutes, so waiting would spend the
hold-down doing nothing — and is stood down again only once the primary has
proved itself for the full failback interval.

### Stratum, honestly

A radio source is a *reference*, not a server, so it counts as stratum 0 and
serving from one is stratum 1. An upstream at stratum 2 makes this stratum 3.
The rule is NTP's own — one more than the lowest stratum among the selected
sources — so a mixed set is still stratum 1, because a radio reference really is
in it.

When the answer does come from an upstream, the reference identifier becomes
that server's address (as RFC 5905 requires above stratum 1) and **root delay
stops being zero**: there is an NTP path above this server now, and its length
is reported. Serving stratum 1 off a pool server would be a lie of exactly the
kind the rest of this program takes trouble to avoid.

### What the client does

It is an ordinary NTPv4 client — mode 3 out, mode 4 back, offset and delay from
the four timestamps — with the usual protections and two details worth naming.

The transmit timestamp it sends is **64 random bits, not the clock**, and a
reply is refused unless its originate field returns them verbatim. Forging a
reply then requires being on the path; a real clock is guessable to within the
poll interval, and random bits are not. `chrony` does the same.

And the timestamps are taken on the **daemon clock** (the free-running
oscillator this program measures everything on), not the host clock, so the
offset a peer produces is "UTC minus the daemon clock" — bit for bit the same
quantity a WWV decoder produces. That is what lets one Selector intersect and
average both kinds without a conversion in between.

A reply is refused, with a reason that reaches the status page, if it does not
echo the nonce, if the server says it is unsynchronised, if its stratum or its
own root distance is past the configured limit, or if **its reference is one of
this host's own addresses** — which means it is synchronised to this daemon and
taking time from it would close a loop with no radio anywhere in the circle.
Kiss-o'-death `DENY` and `RSTR` stop the polling for good; `RATE` slows it down
to what the server asked for, and eases back after eight clean polls so one kiss
on the opening burst does not leave a fallback permanently too slow to be one.

Eight samples sit in a clock filter and the smallest round trip among them sets
the reference, because delay above the minimum is queueing and queueing is what
path asymmetry is made of. Samples far behind that minimum are **dropped**
rather than averaged in.

### The agreement test does not cross the classes

The [consensus veto](#the-consensus-veto) refuses a source more than 30 ms from
its fellows, and that is only sound because every radio source is hearing one
transmitter and one second edge. A radio source and an NTP server have no such
relationship — the gap between them is the delay model's error, a real quantity
that can honestly exceed 30 ms on a path the model does not fit. So residuals
are computed **within a class**, and the two classes never convict each other.
Their disagreement is reported instead, above.

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
User-Agent: ubersdr-ntp/0.1.9 (+https://github.com/madpsy/ubersdr-ntp)
```

with the version of the release it was built from. That version is not written
anywhere in the source: the build takes it from the release tag, or from
`git describe` for any other build, so `--version`, the status page and this
header always agree with the release that shipped them.

That is the only thing telling a receiver operator what has taken one of their
slots, so it names the program, the version and where to look it up — and it
lets a receiver that filters by User-Agent allow or refuse this specifically
rather than guessing. It is fixed in the code and not configurable, for the same
reason: an operator can only trust what it says if nobody can change it.

## When nothing is locked

If a secondary class is configured, it takes over — after
`failover_after_seconds`, which pre-empts the coast below, because a reachable
NTP server is a better answer than an hour of extrapolating a crystal. What
follows is what happens when there is nothing to fail over to, or the secondary
is down as well.

It keeps answering, coasting along the last good offset at the rate it was
last measured moving — the daemon clock is a crystal, and a crystal keeps its
rate when the radio goes quiet — with root dispersion growing at
`coast_drift_ppm` (15 ppm, NTP's own assumed wander for an undisciplined clock)
plus the doubt in that rate, until `coast_seconds`. Past that it answers stratum 0 with
LI=3 — unsynchronised — which tells a client to look elsewhere immediately
rather than making it wait for a timeout. That is what a real refclock does.

## Watching it

With `log.file` set, the log carries one line per state change plus a detailed
per-source block every `status_interval_seconds` — which is how you see what
each source is doing under systemd with no terminal:

```
clock: primary is radio (2 candidate(s)), ntp is standby (2 candidate(s))
       serving from: primary
       radio minus ntp: +0.4 ms averaged (-0.0 ms now), over 12m, 2 vs 2 source(s)
         — the radio delay model's absolute error, which no number of
           receivers can measure. Reported, never applied.

radio source   link       stn    state     stage      tickdB  vote    qual      offset      disp       n
--------------------------------------------------------------------------------------------------------
wwv10          streaming  wwv    locked    locked       18.3   4/8    100%    +12.4 ms    21.0ms      97
wwv15          streaming  unknown acquiring no tick       0.4   0/8       -           -         -       0

ntp source     state       st refid           reach    poll    delay      offset      disp       n
--------------------------------------------------------------------------------------------------
time.cloudflare.com locked  3 10.29.8.4         377     64s    11.9ms     +0.1 ms    18.9ms       9
192.168.9.1    locked       2 192.168.9.99      377     64s     0.2ms     -2.0 ms    26.6ms       9
```

The two classes get their own tables, because the columns that matter differ
completely: a tick SNR and a vote count say nothing about an upstream server,
and a stratum and a reach register say nothing about a receiver. `reach` is
NTP's own eight-bit shift register, one bit per poll with the newest at the top
and printed in octal as every other NTP tool prints it — `377` is eight polls
answered out of eight.

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
body, and anything but `GET`/`HEAD` is answered 405. Binds `0.0.0.0:1234` by
default — it serves no passwords, but it does advertise which receivers this
daemon uses, so set `http.listen` to `127.0.0.1` to keep it on this machine.
`::` binds IPv6 and IPv4 both.

| Route | |
|---|---|
| `/` | The status page. It times its own requests to `/api/time` and corrects its clock for the browser-to-server delay, so it ticks with the broadcast; local time alongside |
| `/api/events` | **SSE.** One `tick` event per *corrected* second — the instant the broadcast's own second rolls over, not this host's — plus a `status` event every 5 s with the full document |
| `/api/time` | The time, for clients that do not speak NTP |
| `/api/status` | Everything this daemon knows, pretty-printed |
| `/api/sources` | Just the per-source array |
| `/api/health` | 200 when synchronised, 503 when not, tiny either way |
| `/api/eventlog` | The last 100 events worth knowing about, newest first, and the catalogue of event types |
| `/api/metrics` | Recent history for the page's charts: `?range=hour` (1-minute averages) or `?range=day` (30-minute), and which class served when |

### History

The page draws a chart under the figures worth watching over time: the clock
offset and root dispersion, each class's median offset and the difference
between them, and per source its offset, its agreement with the rest of its
class, and a figure of its own kind (a receiver's tick SNR, an upstream's round
trip). For receivers there is also the number of decoded times
[refused as jumps](#time-does-not-jump) in each bucket, with how far out they
were in the tooltip. A bar across the top of the primary-and-secondary card shows which class
served the time, so a failover and the failback are a coloured stretch rather
than two log lines. One switch covers every chart: the last hour at 1-minute
averages, or the last 24 hours at 30-minute averages.

Kept in memory and bounded: 60 plus 48 buckets per series, series only for the
served figures and the configured sources, and only the *changes* of serving
class, capped at 2000. A restart starts it again.

`/api/metrics` serves one range at a time; `group=served,class,radio,ntp` narrows
it. Each point is `[bucket start, mean, min, max, samples]`, oldest first, the
last still filling; `serving` is `[start, state]` pairs, each lasting until the
next, the first saying what held at the start of the window.

### Events

The status page says what *is*; the event log says what *happened*: a receiver
locking or dropping out, a class of source becoming healthy or not, a failover
and the failback, coasting, a refusal by the consensus, an upstream sending
kiss-o'-death. The daemon keeps the newest hundred, in memory only, so a restart
starts the list again with a `daemon_started` entry. The page shows them ten at
a time, filterable by type, source and severity.

`/api/eventlog` returns them with the type catalogue alongside, so a client can
label and filter without knowing the vocabulary in advance:

```jsonc
{
  "capacity": 100, "latest_id": 7,
  "types": [ { "type": "failover", "label": "Failed over", "category": "class",
               "severity": "warning", "description": "…" }, … ],
  "events": [ { "id": 7, "utc": "2026-09-17T00:34:48Z", "unix": 1789605288.1,
                "uptime_seconds": 10.3, "type": "failover", "label": "Failed over",
                "category": "class",
                "severity": "warning", "source": null, "kind": "ntp",
                "message": "serving from the ntp sources: …" }, … ]
}
```

Optional filters, combined as AND: `type=a,b`, `category=a,b`,
`source=name`, `kind=radio|ntp`, `severity=` (that or worse: `info`, `notice`,
`warning`, `error`), `since_id=n`, `limit=n`. Each one-second `tick` on
`/api/events` carries `events_latest_id`, so a follower asks again only when it
moves.

Every source in `/api/status` carries a `kind` of `radio` or `ntp`, a
`primary_class` flag, and `ready` / `not_ready_reason` — the one question both
kinds answer, and the one the Selector asks. An upstream peer adds an `ntp`
object with its stratum, refid, reach register, round trip, filter jitter, root
distance against the configured limit, the counts of polls sent, answered,
refused and dropped as delay spikes, and `dns_ttl_seconds` /
`next_resolve_seconds` for a server given by name (-1 for an address).

`served.stratum` and `served.root_delay_ms` say where the time actually came
from, and a `clock` object carries the arrangement and the headline figure:

```jsonc
"clock": {
  "primary": "radio", "secondary": "ntp", "secondary_mode": "standby",
  "serving": "primary",              // primary | secondary | both | coasting | none
  "primary_candidates": 2, "secondary_candidates": 2,
  "failover_in_seconds": null,       // non-null while a swap is pending
  "failback_in_seconds": null,
  "class_delta": {                   // the radio delay model's absolute error
    "valid": true, "delta_ms": 0.42, "instant_ms": 0.07,
    "settled_for_seconds": 1412, "primary_sources": 2, "secondary_sources": 2
  },
  "primary_median_ms": -0.31,        // each class's median offset; null with
  "secondary_median_ms": -0.73       // nothing ready in it
}
```

Alongside it, `events` carries the newest event's id and how many of each
[type](#events) there have been since startup — not bounded by the event log's
hundred, so a failover last week still counts — and `http.stream_clients` how
many pages and other followers hold `/api/events` open. Each source carries its
`agreement` with the others of its kind: the residual the consensus judges it
on, or `null` when it is not in the comparison.

The same summary rides on every one-second `tick` event, so a display follows a
failover as it happens rather than at the next 5 s `status`.

The status page treats the two countdowns as *anchors* rather than redrawing
them per tick. A tick is emitted on the **corrected** second boundary, and that
boundary moves whenever the served offset does — which is precisely what is
happening while a class is failing over — so two ticks can land inside one wall
second and the next can be skipped. Redrawn per tick the figure reads 27, 27,
25, 24: right every time it is sent, and visibly erratic, which is no use on a
number someone is watching to know how long is left. The page interpolates from
its own monotonic clock at a steady 1 Hz instead and re-anchors on every tick,
so the sequence is smooth and still honest — a countdown that jumps back up is
then a real reset, the primary having recovered and lost it again, rather than
an artefact of when a packet arrived.

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
  "offset_ms": -34.2, "clock_offset_ms": 118.6, "clock_rate_ppm": 11.9,
  "dispersion_ms": 22.5,
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

`offset_ms` is the correction *this host* would need to reach the served time;
the served time itself does not depend on the host's clock (see
[its own clock](#its-own-clock)). `clock_offset_ms` and `clock_rate_ppm` are the
served time against the daemon's own clock and how fast that moves, which is
what a client timing repeated requests needs to tell a change in the server's
estimate from steady movement — the status page uses them that way.
`server_raw_receive` is the host's clock at the receive.

### `/api/events`

```bash
curl -N http://127.0.0.1:1234/api/events
```

The `tick` fires on the corrected second boundary, so a display driven from it
ticks with WWV rather than with the machine it is running on — and keeps doing so
across a change in the offset, because the boundary moves with the correction.

## MQTT and Home Assistant

Running beside an UberSDR receiver that has MQTT enabled, this publishes through
the receiver's own MQTT connection, and appears in Home Assistant as a device of
its own, nested under the receiver. **There is nothing to configure**: it uses
UberSDR's addon ingest port, as the receiver's other addons do, which knows who
is calling from the TCP connection itself — there is no broker address, no
credential and no topic to set.

Where there is no such port to reach — MQTT off on the receiver, or no receiver
beside it — or where the port does not recognise this machine as an installed
addon, it says so once in the log, stays dormant, and asks again every 30 s, so
it starts publishing by itself when the receiver comes up. Nothing about it can
stop time being served: it runs on a thread of its own, and no failure there is
fatal.

### Topics

Under the receiver's topic prefix, `ubersdr/metrics` by default:

| Topic | Retained | Contents |
|---|---|---|
| `…/addons/ntp/summary` | yes | The served time, the two classes, the NTP server's counters, a line per source and the newest event. Every 30 s, and within seconds of any event |
| `…/addons/ntp/source/<name>` | yes | Everything `/api/status` says about one source. Every 30 s |
| `…/addons/ntp/events` | no | Every [event](#events), once and in order, in the `/api/eventlog` shape |
| `…/addons/ntp/status` | yes | `online` / `offline`, maintained by UberSDR |

`ntp` is whatever name the receiver's `addons.yaml` gives this addon.
`<name>` is the source's name folded to what a topic allows — lowercase letters,
digits, `-` and `_` — so `Local 10 MHz` is `source/local-10-mhz`; the
summary's line for each source names its topic.

Between them the summary and the source topics carry every field of
`/api/status` — the self-test checks that, key by key. The summary is
`/api/status` without its `sources` array, plus:

```json
{
  "time_utc": "2026-09-19T00:55:57Z",
  "started_utc": "2026-09-18T21:12:04Z",
  "ntp": { "requests": 48213, "requests_per_minute": 31.5, "…": "…" },
  "sources": {
    "local-10": { "kind": "radio", "state": "live", "in_use": true, "ready": true,
                  "stage": "locked", "link": "streaming", "offset_vs_served_ms": 0.42,
                  "not_used_reason": null, "topic": "source/local-10" },
    "cloudflare": { "kind": "ntp", "state": "standby", "…": "…" }
  },
  "counts": { "radio": 2, "radio_ready": 2, "ntp": 1, "ntp_ready": 1 },
  "last_event": { "type": "source_ready", "message": "…", "…": "…" },
  "mqtt": { "events_pending": 0, "events_lost": 0 }
}
```

`state` is the status page diagram's: `live` feeds the served time, `standby`
is measured but held out of it, `down` is neither.

No event is dropped to save the rate limit: one that does not fit waits its
turn, one the receiver could not pass on — its broker down — is sent again, and
those recorded while the receiver was unreachable are sent when it comes back,
each stamped with when it happened. `mqtt.events_pending` is how many are
waiting. The only way to lose one is for the event log to overwrite it first,
more than a hundred behind, and `mqtt.events_lost` counts those.

### Staying inside the receiver's limit

The receiver allows an addon 120 publishes a minute by default and refuses the
rest. This reads the actual figure from the port and paces itself to use about
half of it: the summary twice a minute, plus at most one every 5 s while events
are arriving; events from a bucket of their own a quarter of the limit deep,
which they wait for rather than being dropped; and the source topics every
30 s, stretched as sources are added so that all of them together fit in what
is left. A publish refused for the rate (429), or because the receiver's broker
is down (503), holds everything for 15 s.

### Home Assistant

When the receiver has Home Assistant discovery on, these are declared at
connection, all reading the one retained summary, so they have values the moment
Home Assistant subscribes:

| Entity | Type | Notes |
|---|---|---|
| Synchronised | binary sensor | Attributes: why not, and the reference id |
| Stratum | sensor | 16 while unsynchronised |
| Serving | sensor | `primary`, `secondary`, `both`, `coasting` or `none`; the arrangement as attributes |
| Failed Over | binary sensor | Problem: the secondary class is serving |
| Host Clock Offset | sensor | ms; the correction this host would need |
| Root Dispersion | sensor | ms; how far out the served time could be |
| Sources In Use | sensor | Each source's line from the summary as attributes |
| Receivers Locked | sensor | Radio sources usable now |
| Upstream Servers Usable | sensor | NTP sources usable now |
| Class Delta | sensor | ms; [the difference between the classes](#the-difference-between-the-classes-is-the-interesting-number), while both are measured |
| NTP Requests | sensor | Running total, `total_increasing` |
| NTP Request Rate | sensor | Requests a minute |
| Failovers | sensor | Since startup, `total_increasing` |
| Decoded Times Refused | sensor | Misread time codes caught since startup, `total_increasing` |
| Leap Second Pending | binary sensor | |
| Last Event | sensor | The message; the whole event as attributes |
| Reference Age, Clock Drift, Reference, Started | sensors | Diagnostic |

The version is on the device card rather than an entity of its own.

A figure with nothing to show yet — the offset before the first measurement,
the class delta while only one class is measured — is left `unknown` rather than
shown as a zero that was never measured. Per-source entities are left out
deliberately: their number would grow with the configuration and the receiver
caps an addon at 20. Each source's state is an attribute of Sources In Use, and
its full record is on its own topic.

Entities go unavailable when either the receiver or this stops publishing, so a
dead daemon shows as unavailable rather than leaving a stale "synchronised"
looking current.

### Overriding the endpoint

Only needed where the receiver's container is not called `ubersdr`, or its
operator has moved the port from 6926:

```jsonc
"mqtt": { "ingest_url": "http://ubersdr:6926" }   // or "enabled": false
```

The environment variable `UBERSDR_INGEST_URL` overrides it, as it does for the
receiver's other addons. The ingest API itself is `addon_mqtt.md` in the
ka9q_ubersdr repository.

## What is in here from elsewhere

`src/clock/WwvDecoder.*`, `src/clock/WwvbDecoder.*` and
`src/clock/TimeFrameVoter.*` come from
[ubersdr-clock](https://github.com/madpsy/ubersdr-clock), where they are the DSP
half of its clock feature. They have since been changed here, so an upstream fix
needs a diff rather than a `cp`:

- sub-sample second edges: WWVB from the carrier-drop crossing, WWV from a
  smoothed matched-filter shift;
- the WWV/WWVH tag, decided from each tick band's energy above its own
  background (the old peak-to-mean test could call a lone WWV "WWVH", which
  costs about 19 ms of propagation on an eastern-US path);
- leap-second (61-second) minutes, and the WWVB DST bits.

`src/clock/Dcf77Decoder.*` was written here, from PTB's description of the
time code and phase modulation and Hetzel's EFTF 1988 paper, and feeds the same
voter. `tools/testdata/dcf77_live.wav` is the recording from
[KiwiSDR_DCF77_Decoder](https://github.com/karastoyanov/KiwiSDR_DCF77_Decoder)
(GPL-3.0), made through a public KiwiSDR near Mainflingen.

The voter's calibration constants and `kNominalDelaySamples` are still
upstream's. `tools/decodertest.cpp` (`ubersdr-ntp-decodertest`) generates all
three stations and checks every timestamp and edge against the truth.

`third_party/pcm_v4.hpp` is shared verbatim with `ka9q_ubersdr/clients` — keep it
in step with the copies there. It has no Opus reader, because the C++ clients
that share it are IQ clients and never negotiate Opus; `src/OpusV4Header.h` is
that half, written here rather than patched in.

`third_party/IXWebSocket` and `third_party/json.hpp` are vendored as-is.

Format facts throughout are per NIST SP 432 (WWV/WWVH), NIST SP 250-67
(WWVB), and PTB's DCF77 time-code and phase-modulation pages (DCF77).

## Licence

GPL-3.0-or-later, inherited with the decoders by way of ubersdr-clock. See
[LICENSE](LICENSE).
