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

- **It is not exact, and about 3 ms of it is a known systematic.** Every source
  shares the chain constant, so no arrangement of receivers can measure it and
  only an absolute reference can; see [the delay model](#the-delay-model). It is
  written down rather than tuned away.
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
receivers. The chain constant is known to be about 3 ms large and has never been
validated against an off-air recording at the two rates used here.

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
| Network | 5–100 ms | Measured, as half the round trip of a WebSocket ping down the audio connection itself |
| Codec | 8 ms (Opus), 0 (PCM v4) | A measured constant |
| UberSDR chain | 13.6 ms | A constant: RF reaching the SDR to audio leaving the WebSocket. **Known to be about 3 ms too large** — see below |
| Decoder bias | −13.6 ms (WWV/WWVH), 0 (WWVB) | A measured constant, from `tools/decodertest.cpp` |
| `extra_delay_ms` | 0 | Yours, for anything genuinely local |

The network term is measured over the **WebSocket**, not with an HTTP request,
because a TCP handshake ends at whatever accepted the SYN — and most UberSDR
instances are reached through a tunnel that terminates TCP near the client, not
at the receiver. One measured here answered its handshake in 12 ms for an origin
94 ms away, which cost 40 ms of delay budget and put its offset 39 ms from a
receiver on the same band it should have agreed with to a millisecond.

A ping down the audio connection cannot be answered by the tunnel. That is
verified rather than assumed: an application-level ping through the same
connection, which only the receiver can reply to, agreed with the protocol ping
to 0.1 ms.

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

### The chain constant is about 3 ms too large

Measured over an evening against two receivers and a host disciplined to
+0.15 ms, served time ran **+3.4 ms** (sd 1.7, n=122); an earlier session gave
+5.4 ms. Something common to every source over-counts by a few milliseconds,
and the chain constant is the loosest term in the budget, so it is the
suspect.

It has not been changed, for a reason worth stating. The terms every source
shares cannot be measured by comparing sources: two receivers hearing the same
transmitter cancel the chain delay, the codec delay and the decoder bias
exactly, so they agree just as well whatever those are set to. Only an absolute
reference can see the common mode, and over the same evening the two receivers
wandered ±11 ms against *each other* — so a single evening cannot separate a
fixed over-count from the ionosphere, and re-deriving the constant from one
session would bake that night's propagation into a figure that claims to
describe a buffer. Pinning it wants a run spanning day and night.

Until then the bias is inside the dispersion the server advertises, which is
the honest place for it: the answer is a few milliseconds high and says it
could be sixteen out.

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

Public receivers usually cap concurrent sessions per IP — two is typical — so
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

It keeps answering, coasting on the last good offset with root dispersion
growing at `coast_drift_ppm` (15 ppm, NTP's own assumed wander for an
undisciplined clock) until `coast_seconds`. Past that it answers stratum 0 with
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

The voter's calibration constants and `kNominalDelaySamples` are still
upstream's. `tools/decodertest.cpp` (`ubersdr-ntp-decodertest`) generates all
three stations and checks every timestamp and edge against the truth.

`third_party/pcm_v4.hpp` is shared verbatim with `ka9q_ubersdr/clients` — keep it
in step with the copies there. It has no Opus reader, because the C++ clients
that share it are IQ clients and never negotiate Opus; `src/OpusV4Header.h` is
that half, written here rather than patched in.

`third_party/IXWebSocket` and `third_party/json.hpp` are vendored as-is.

Format facts throughout are per NIST SP 432 (WWV/WWVH) and NIST SP 250-67
(WWVB).

## Licence

GPL-3.0-or-later, inherited with the decoders by way of ubersdr-clock. See
[LICENSE](LICENSE).
