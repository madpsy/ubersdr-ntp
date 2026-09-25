# Adding MSF and Allouis (TDF)

A plan for two more LF time stations: MSF (60 kHz, Anthorn, UK) and Allouis
(162 kHz, TDF/ALS162, France).

**Status (2026-09-25):** both decoders are written (`src/clock/MsfDecoder.*`,
`src/clock/AllouisDecoder.*`), tested (`tools/msftest.cpp`,
`tools/allouistest.cpp`: synthetic scenarios plus the M9PSY-1 recordings,
checked against their GPS capture stamps) and wired into the daemon:

- `carrier_hz: 162000` is Allouis and `carrier_hz: 60000` is MSF or WWVB,
  both in IQ tuned on the carrier. Which 60 kHz station is decided by the
  receiver's coordinates -- the nearer of Anthorn and Fort Collins
  (`Source::resolveLf60`); no coordinates, WWVB. WWVB from IQ is fed to its
  existing decoder through a 900 Hz low-pass and a 1 kHz shift, the low-pass's
  delay taken off in the delay model. `dial_hz: 59000` keeps the old WWVB
  over USB. (Chosen over deciding from the signal, and over a dial convention.)
- `install.sh` adds DCF77, MSF and Allouis each when the receiver is within
  2000 km of that transmitter and tunes down to its carrier;
  `config.addon.json` carries both as examples.
- Allouis gets DCF77's 1 ms dispersion floor under the same conditions
  (capture-timed, timed by its phase, within 2000 km of Allouis). MSF keeps
  10 ms until its edge bias is settled (see *Recordings*).

Measured limits (synthetic): MSF locks down to about 27 dB-Hz, with edges
p99 0.15 ms at 40 dB-Hz; Allouis down to 29 dB-Hz, p99 0.12-0.18 ms at 40.
On M9PSY-1 (~60-69 dB-Hz): MSF 24-43 us scatter, Allouis 4-6 us second to
second.

## Why

Each extra station on the same receiver shares everything with DCF77 except
the path and the decoder: the RX888, radiod's capture timing, the host and this
daemon. Those shared terms cancel in any comparison between stations, so the
Agreement chart (which today says "needs two receivers locked") would show
path and decoder differences only.

From M9PSY-1 (56.04°N, 3.35°W):

| Station | Carrier | Distance | Path |
|---|---|---|---|
| MSF, Anthorn | 60 kHz | 125 km | pure groundwave, ~0.42 ms, effectively no skywave |
| TDF, Allouis | 162 kHz | 1057 km | groundwave, the same length as DCF77 on another bearing |
| DCF77, Mainflingen | 77.5 kHz | 1061 km | groundwave, with night-time skywave |

- **MSF is the reference.** At 125 km there is almost no path to wander, so
  DCF77 − MSF shows DCF77's path directly, including the night-time jitter
  spikes the spike hold was written for.
- **Allouis against DCF77** compares two paths of the same length on different
  bearings. That shows whether night wander scales with distance, as the delay
  model assumes, or is specific to the path.
- More than one radio source also gives the selector a real cross-check, so a
  single disturbed path can't carry the served time on its own.

## Order

1. **MSF.** Mostly an adaptation of `WwvbDecoder`. Useful on its own.
2. **Allouis.** Needs a new phase demodulator. The format is now known well
   enough to build it (see *Station facts*); what a recording must still
   settle before it serves time is the timing point, and the position-code
   table needs corroborating.

## Station facts

Each fact is marked **confirmed** (checked against the reference, which is
cited) or **open** (the reference doesn't settle it).

### MSF

Reference: [NPL, MSF 60 kHz time and date code (2019)](https://www.npl.co.uk/getattachment/2afd4c3b-fc8d-486c-b2b9-fc9a5a66394c/msf-time-date-code-2019.pdf),
two pages, complete. ITU-R TF.2487 §3 repeats it. Every fact below is confirmed.

- **Keying.** Simple on-off carrier modulation, with bit polarity 0 = carrier
  on and 1 = carrier off. The NPL sheet doesn't say how deep "off" is, but
  ITU-R TF.2487 §3.1 and §3.3.2 call it "100% amplitude modulation" and "100%
  on/off keying": the carrier goes fully off.
- **Second marker.** Every UTC second is marked by an "off" preceded by at
  least 500 ms of carrier. Seconds 01–59 begin with at least 100 ms off and end
  with at least 700 ms of carrier. Bit A is in 100–200 ms and bit B in
  200–300 ms.
- **Minute marker.** Second 00 begins with 500 ms off.
- **Edge accuracy.** The second marker "is transmitted with an accuracy better
  than 1 ms" (ITU §3.1 words it as "an uncertainty of 1 ms"), and the rise and fall times "are determined by the combination of
  antenna and transmitter" (no figure is given). **This bounds MSF's timing:**
  whatever the decoder does, the transmitted edge is only promised to 1 ms, so
  MSF's dispersion floor can't honestly go below about 1 ms plus the decoder's
  own doubt. It's still a clean reference for path *stability*, since a
  constant transmit offset cancels out of MSF's own wander.
- **Time and date code, bit A, BCD, most significant bit first:** year (00–99)
  17A–24A; month 25A–29A; day of month 30A–35A; day of week 36A–38A
  (0 = Sunday … 6 = Saturday); hour 39A–44A; minute 45A–51A.
- **Minute identifier.** 52A = 0, 53A–58A = 1, 59A = 0. The sequence `01111110`
  never appears elsewhere in bit A, so it uniquely identifies the next
  second-00 marker.
- **DUT1, bit B.** 01B–08B set in sequence for +100 to +800 ms; 09B–16B for
  −100 to −800 ms; none set for 0.
- **Parity, odd, bit B.** 54B over 17A–24A (year); 55B over 25A–35A (month and
  day of month); 56B over 36A–38A (day of week); 57B over 39A–51A (hour and
  minute).
- **Summer time.** 58B = 1 while UK summer time is in effect. 53B = 1 for the
  61 minutes before a change, the last being the minute-59 when 58B changes.
  58B may one day change without a change in UK clock time (a permanent offset).
- **Civil time.** The code is UK clock time: UTC in winter and UTC+1h in summer.
  It relates to the minute *following* the one in which it is transmitted.
- **Leap seconds.** A minute can have 61 seconds (the extra one numbered 60) or
  59. Every bit number marked `*` in the spec (17A–59A, and 52B–59B including
  the parity and summer-time bits; also 16A and 51B, the last bits of the
  reserved ranges) then shifts by one, so the code stays
  aligned with the end of the minute, not its start. **The decoder must find
  the fields relative to the minute identifier at the end, not by counting from
  second 00.** DUT1 (01B–16B) doesn't shift.
- **Reserved bits.** 01A–16A, 17B–51B, 52B and 59B are currently 0 (carrier on)
  but "may be used in the future". Don't treat a 1 there as an error.
- **Transmitter.** Anthorn, 54° 55′ N, 3° 15′ W (54.9167°, −3.2500°), 15 kW
  EMRP, omnidirectional. Carrier within 2 × 10⁻¹². The spec gives it only to
  the arcminute, about 1.8 km, which is about 6 µs of path: negligible, but say
  so where the constant is defined.

### Allouis (ALS162, formerly TDF)

Reference: [ITU-R Report TF.2487 (2021)](https://www.itu.int/dms_pub/itu-r/opb/rep/R-REP-TF.2487-2021-PDF-E.pdf),
§9.1. This is a **summary**, not the specification. It names the specification
as French standard **NF C90-002 (1988)**, *Broadcasting and telecommunication:
Data broadcasting system compatible with AM sound broadcasting*, available only
in French.

**Confirmed by the ITU report:**

- **Site.** Near Vierzon, 47° 10′ 05″ N, 02° 12′ 02″ E (47.1681°, 2.2006°).
  800 kW at the antenna input. Mostly groundwave across France.
- **Carrier.** 162 kHz, relative uncertainty 2 × 10⁻¹².
- **Modulation.** Phase modulation (G2B): "+1 and −1 rad in 0.1 s every second
  except the 59th second of each minute. This modulation is doubled to indicate
  binary 1."
- **Minute marker.** No modulation in second 59.
- **Content.** Minute, hour, day of month, day of week, month and year in
  seconds 21–58, in French legal time. Second 17 = 1 means summer time (UTC+2);
  second 18 = 1 means winter time (UTC+1). Second 14 = 1 means today is a public
  holiday; second 13 = 1 means tomorrow is.
- **Scheduled outage.** It transmits 24 hours a day *except every Tuesday from
  08:00 to 12:00 French legal time*. The source will go quiet for four hours a
  week. That's not a fault, so it shouldn't raise alarms, and the selector has
  to cope with it.
- **Bandwidth.** The report's noise calculations (§9, Annex 2) use 250 Hz as
  the "Allouis signal bandwidth". That's a starting point for the channel
  filter and for the SNR's noise bandwidth.
- The site "currently broadcasts the time signal … 24 hours a day", so the
  carrier question in the earlier draft is settled. (The report says the site
  was built for a radio programme but doesn't say outright that the programme
  has ended; that part comes from elsewhere.)

**From [henningM1r/gr_ALS162_Receiver](https://github.com/henningM1r/gr_ALS162_Receiver)**
(GNU Radio ALS162 transmitter model, channel and receiver, GPLv3, last commit
2023-08-14 `34c8e6c`). This is a hobbyist implementation, not the standard.
It's consistent with everything in the ITU report and internally consistent:
its example frame decodes to 17:43, Monday 1 May 2023, with every parity and
the weight check passing. Its README says the receiver is "probably not
accurate in terms of milliseconds", so it settles the *format* but not the
*timing point*. It's GPLv3 like ubersdr-ntp, so its tables (the position codes)
can be reused with attribution. Its code is Python on GNU Radio and won't be
reused; what we take from it is the format.

- **Excursion shape (confirmed by the model).** The phase moves in straight
  ramps over 25 ms sub-slots, as an integrated derivative: +1 rad/25 ms, then
  −1, −1, +1. So the phase goes 0 → +1 rad at 25 ms → 0 at 50 ms → −1 rad at
  75 ms → 0 at 100 ms. That's a triangle wave: up 25 ms, down 50 ms, back up
  25 ms. (Transmitter flowgraph: derivative vectors, a repeat to sample rate,
  an integrating IIR filter, then scaling by `40/samp_rate`, so one unit held
  for 25 ms is exactly 1 rad.)
- **Binary 1 = the excursion twice**, back to back in 0–200 ms (derivative
  `+1,-1,-1,+1,+1,-1,-1,+1`). Binary 0 is one excursion in 0–100 ms followed by
  100 ms of nothing.
- **Minute marker.** Second 59 has no excursion at all (the model's symbol "2").
  It also carries the all-zero position code (below), so the *whole* of second
  59 is unmodulated, which matches the ITU's "no modulation in the 59th second".
- **Position codes (new, not in the ITU report).** In the 800 ms after the data
  slots (200–1000 ms, 32 sub-slots of 25 ms) the phase carries a
  pseudo-random ramp sequence that is different for each second of the minute.
  Its derivative values are 0, ±1 and ±2 per sub-slot. The model has the full
  table, `ALS162_codes.py`, `position_code_dict["00".."59"]`. Checked against
  the table:
  - **The key is not the second it's sent in.** The transmitter flowgraph
    sends the keys in the order "01", "02" … "59", "00", starting at second 0,
    so **key "NN" goes out in second NN−1** and the all-zero "00" goes out in
    second 59. The model's decoder agrees: it counts a second's position as its
    key, and closes the minute when it gets symbol "2" with count 60. Whether
    the keys are the standard's numbering or just the model's, the timing
    doesn't care; only the mapping does, so store the table by the second it's
    sent in.
  - Every sequence brings the phase back to 0, and the phase stays within
    ±1 rad throughout, like the data excursion.
  - Each sequence ends by sub-slot 27, so **900–1000 ms is always quiet**. The
    shortest ends at sub-slot 20 (725 ms).
  - Any two sequences differ in at least 8 of the 32 sub-slots, so a
    correlation names the second with a good margin.

  This is ALS162's counterpart of DCF77's PM: a correlation over up to 700 ms
  that can time the second far more precisely than the 100 ms data excursion,
  and that names the second, which helps minute sync. **To corroborate**
  against the standard or a recording before relying on it.
- **How the model receives it (and why it isn't millisecond-accurate).** IQ
  from an SDR, shifted to baseband and resampled to 24 kHz, AGC, GNU Radio's
  carrier-tracking PLL, then the phase (`complex_to_arg`) with a hand-tuned
  drift correction. It then thresholds the phase *slope* into +2/+1/0/−1/−2,
  counts it per 25 ms sub-slot (±4 ms tolerance) and matches the 40 sub-slot
  symbols against the tables. No correlation or interpolation, so the second
  edge is only good to about a sub-slot. That confirms IQ (phase is needed)
  and sets what to do differently: correlate against the full expected phase.
- **Bit layout (confirmed by the model's decoder and its example frame).**
  Numbered by second; BCD least significant bit first, as DCF77:

  | Second | Content |
  |---|---|
  | 0 | start bit, always 0 |
  | 1 / 2 | positive / negative leap second warning |
  | 3–6 | the number of 1s in seconds 21–58, as weights 2, 4, 8, 16 (it's always even); a check on the whole time code |
  | 7–12, 15, 19 | unused, 0 |
  | 13 / 14 | tomorrow / today is a public holiday |
  | 16 | clock change announced |
  | 17 / 18 | CEST (UTC+2) / CET (UTC+1); exactly one is set |
  | 20 | start of time information, always 1 |
  | 21–27 | minute, units 21–24 (1, 2, 4, 8), tens 25–27 (10, 20, 40) |
  | 28 | even parity over 21–27 |
  | 29–34 | hour, units 29–32, tens 33–34 (10, 20) |
  | 35 | even parity over 29–34 |
  | 36–41 | day of month, units 36–39, tens 40–41 |
  | 42–44 | day of week, 1 = Monday … 7 = Sunday |
  | 45–49 | month, units 45–48, tens 49 |
  | 50–57 | year (00–99), units 50–53, tens 54–57 |
  | 58 | even parity over 36–57 |
  | 59 | no modulation: minute marker |

  From 20 on, this is DCF77's layout, so the field map and parity code can be
  shared. Bits 0–19 differ.

**Still open:**

- **The timing point.** Which instant is on time: the start of the first ramp
  at 0 ms is the natural assumption (every second's excursion begins there),
  but nothing authoritative says so, and the transmitted accuracy isn't stated
  (compare MSF's "better than 1 ms"). This is the one that can hide a bias, so
  measure it: capture-timed on M9PSY-1 against DCF77 PM and the GPS-fed
  stratum 1.
- **The position-code table**, from the standard or a recording.
- The standard itself, **NF C90-002 (1988)** (AFNOR, French only), would settle
  both.

**Ways to close them:**

1. Get NF C90-002 from AFNOR. It's a paid, French-language standard.
2. Other public descriptions (ANFR's ALS162 pages, other open-source decoders),
   checked against each other and against the model above.
3. **Measure it.** A recording from M9PSY-1 shows the actual excursion, the
   position codes and where they sit relative to our capture-timed clock. That
   settles the timing point, which nothing else can, and checks everything
   else. It's the most reliable of the three.

## Design

### MSF decoder (`src/clock/MsfDecoder.{h,cpp}`)

Built on `WwvbDecoder`'s chain, taken from IQ (see *Capture mode*): the
carrier sits near 0 Hz rather than at an audio tone, so the tone search and
mix become a small offset correction. Then LPF, envelope (the IQ magnitude), second edge from the amplitude drop, then classification per second,
minute sync, then `TimeFrameVoter`.

The differences:

- **Keying.** MSF switches fully off where WWVB drops 17 dB, so the edge is
  sharper and the classification threshold can sit lower. It is still an
  amplitude edge, subject to the channel filter's group delay and the
  envelope's rise time.
- **Two bits a second.** Classify the envelope in 100–200 ms and 200–300 ms
  separately, rather than WWVB's one pulse-width decision.
- **Minute sync.** The 500 ms marker plus the `01111110` pattern in A 52–59,
  which together are much more robust than WWVB's double marker. Find the
  fields by counting back from the identifier, not forward from second 00, so
  a leap-second minute (61 or 59 seconds) decodes correctly.
- **Field map and parity.** Implement MSF's layout and its four odd-parity
  groups. A frame failing parity goes to the voter as low confidence, as other
  decoders do.
- **Summer time.** The code is UK civil time: subtract one hour when B 58 is
  set, before the voter sees it. Test across the changeover.

### Telling MSF from WWVB

MSF and WWVB are both on 60 kHz, and `broadcastFor()` in `Config.h` decides a
decoder from the tuning, never from a setting ("they are different signals, not
options"). To keep that rule, the 60 kHz path should tell the two apart from the
signal, as `WwvDecoder` tells WWV from WWVH:

- MSF keys fully off and WWVB drops 17 dB, so the envelope's depth differs.
- MSF's minute structure can't occur in WWVB: the `01111110` identifier in
  A 52–59 followed by the 500 ms marker, against WWVB's 800 ms markers at
  0, 9, 19, 29, 39, 49 and 59. **Pulse width alone doesn't separate them.**
  MSF's 500 ms minute marker looks like a WWVB binary 1 (500 ms reduced), and
  its 100–300 ms offs overlap WWVB's 200 ms zero. So decide from the frame
  structure, not from one second.
- On IQ, phase: WWVB has carried a BPSK code (180° flips) for about a
  decade (ITU-R TF.2487 §4.3; the report gives both 2013 and 2016), and MSF has no phase modulation at all.

So run both classifiers at 60 kHz until one of them has a confident minute,
then keep the winner and set `station` to `"msf"` or `"wwvb"`, just as WWV sets
`"wwv"` or `"wwvh"`. The receiver's location (Europe or North America) can be a
prior, but not the decision.

### Allouis decoder (`src/clock/AllouisDecoder.{h,cpp}`)

New work. The format is settled enough to build against (see *Station
facts*); the timing point comes from measurement. **IQ only**: it's a phase
signal, and a USB audio channel keeps no phase.

- **Phase demodulation.** Carrier recovery with a PLL, locked tightly enough to
  follow a slow carrier offset but not the ±1 rad excursions. Then the phase
  series.
- **Data bits.** A matched filter for the 100 ms triangle (0 → +1 → 0 → −1 → 0
  rad in 25 ms ramps): one in 0–100 ms is 0, a second one in 100–200 ms makes
  it 1, and none at all is second 59.
- **Second edge.** Correlate the whole second against its expected phase: the
  data triangle(s) plus that second's 800 ms position code. Like DCF77's PM
  correlator, a long known sequence times the second far more tightly than the
  100 ms triangle alone, and a phase edge beats any amplitude edge. That's the
  reason to add this station.
- **Minute sync.** Second 59 is entirely unmodulated (no excursion, all-zero
  position code), and every other second's position code names it. Mind the
  model's off-by-one: key "NN" is sent in second NN−1.
- **Field map.** From second 20 on it's DCF77's layout, so share it with
  `Dcf77Decoder`. Add seconds 1–6 (leap warnings and the weight check on
  21–58), 13–14, 16 and 17–18.
- **Tuesday outage.** No signal every Tuesday 08:00–12:00 French legal time.
  The source should report it as scheduled, not as a fault, and coast or hand
  over as for any other gap.
- Reuse what fits from `Dcf77Decoder`'s PM path: carrier tracking, how lock is
  decided, and the SNR measurement. Its 512-chip correlator doesn't apply.

### Capture mode: IQ for both

Both new stations are taken as IQ with the carrier at 0 Hz, as DCF77 is
(`mode=iq`, ±6 kHz).

- **Allouis: IQ is required.** It's a pure phase signal, and a USB audio
  channel keeps no phase (the reason `Config.h` gives for DCF77).
- **MSF: IQ, by choice.** The edge is timed on the IQ magnitude, without the
  audio chain's filter and AGC in the way. The phase gives a third test
  between MSF and WWVB (WWVB's BPSK; see above). And all three LF stations then
  share one capture path, which is what the Agreement comparison wants.
- **What that costs at 60 kHz.** `Source.cpp` decides IQ from the broadcast
  (`m_iq(m_broadcast == Broadcast::Dcf77)`). A 60 kHz source doesn't know
  whether it's MSF or WWVB until it has decoded, so it has to open in IQ
  either way. That moves **WWVB to IQ too**: `WwvbDecoder` needs an IQ input
  (magnitude in place of its audio envelope), and its edge bias has to be
  measured again.
- **Reduced-depth IQ (`min_margin`).** MSF's carrier goes fully off, deeper
  than anything DCF77's AM does. Check that the margin doesn't clip the
  off-period into the floor in a way that blunts or shifts the falling edge;
  if it does, MSF runs at `min_margin=0`.

### Integration points

| File | Change |
|---|---|
| `src/Config.h` | Carrier constants: `kAllouisCarrierHz = 162000`. MSF shares WWVB's range, so the 60 kHz path decides between them (see above). Extend `enum class Broadcast` and `broadcastFor()`. |
| `src/Propagation.{h,cpp}` | `msfSite()` and `allouisSite()`. Both are LF paths, so they use `lfDelaySeconds()`. |
| `src/Source.cpp` | Build the new decoders in `ensureDecoder`. Route `station` through the tag switch. Edge bias constants per decoder, 0 until measured. `updateDelayModel` picks the site by station. |
| `src/Source.cpp` dispersion | MSF: keep the 10 ms floor until its amplitude edge's bias is measured; see *Dispersion*. Allouis: a floor like DCF77's `kDelayUncertaintyFloorLfPmSec` once phase timing is validated, under the same conditions (capture-timed, phase-locked, inside groundwave range). |
| `src/clock/TimeFrameVoter` | Add the station to `ClockStation`. |
| `src/web/index.html`, `Status.cpp`, `HttpApi.cpp` | Station names and per-station decoder diagnostics, as DCF77 has (`dcf77` block with PM SNR etc.). |
| `install.sh`, `config.addon.json` | Offer MSF within range of Anthorn and Allouis within range of Allouis, like the DCF77 range check. Both IQ, tuned on the carrier (60 kHz, 162 kHz), like DCF77; see *Capture mode*. |
| `README.md` | A station section each, and the path comparison once measured. |
| `tools/` | `msftest` and `allouistest`, like `dcf77test`. |

## Timing and dispersion

- **Edge bias.** Each decoder needs a measured edge bias, like
  `kWwvDecoderEdgeBiasSec`: first on synthetic signals with the true edge off
  the sample grid (the lesson from `c4d25ad`), then live.
- **Live validation.** The class delta against the GPS-fed stratum 1, with each
  station capture-timed and serving on its own, over at least twenty settled
  minutes by day and by night, as the README asks for DCF77.
- **MSF floor.** It stays at 10 ms at first. Once MSF's live bias is known and
  its night behaviour at 125 km is seen (there should be none), it can have a
  path-specific floor like DCF77's, sized from what was measured and not
  assumed from the path being short. It can't go below about 1 ms, because NPL
  only promises the transmitted second marker to "better than 1 ms". An
  amplitude edge adds rise-time doubt on top, so expect a few milliseconds.
- **Allouis floor.** 1 ms, under the DCF77 conditions, once its phase timing
  agrees with the reference.

## Testing

- **Synthetic** (`tools/msftest`, `tools/allouistest`): generated signals with
  known edges off the sample grid, noise at several SNRs, fading, a leap-second
  minute, the summer-time changeover, and a missing second. For MSF, also a
  WWVB signal, to check the discrimination.
- **Recordings.** A few minutes of each station from M9PSY-1, in the same format
  the decoder is fed (IQ), by day and by night. These are what show
  the real edge shape, fading and interference, and they should go in the test
  corpus.

  **Night set, taken 2026-09-25 01:20:27 UTC** with `tools/iqrecord.cpp`
  (`ubersdr-ntp-iqrecord`), in `tools/testdata/`: `msf_`, `als162_` and
  `dcf77_m9psy1_20260925T0120Z.wav`, 305 s each, 12 kHz lossless IQ on the
  carrier (`min_margin=0`), recorded at the same time so all three share one
  capture clock. Each has a `.times.csv` beside it: radiod's GPS capture
  stamp for every 20 ms packet, and the WAV frame it lands on. No packets lost
  and no decode errors. **The day set is still to take.**

  First look (quick offline analysis, not a decoder):
  - Carriers sit at 0.000 Hz: MSF 94 dB, Allouis 89 dB, DCF77 69 dB over the
    median bin.
  - MSF's 50% falling edge is at +0.85 ms from the UTC second, spread
    0.04 ms (path 0.42 ms). DCF77's AM edge is at +3.7 ms.
  - **Allouis position codes confirmed on air:** in 295 of 302 seconds the
    best-matching code was key "second+1", within 2 of 32 sub-slots; the 7
    misses were noisy seconds, and none matched another key cleanly.
  - **Allouis names the following minute**, like DCF77 and MSF: the frame
    sent during 01:21 UTC reads 03:22 CEST (01:22 UTC), Friday. The weight
    check (bits 3–6) caught the one frame with a bit error.
  - **Timing point, measured: the excursion starts 50 ms before the second**,
    so its midpoint (the zero crossing from +1 to -1 rad) is on the second.
    Against DCF77's PM edge from the same simultaneous recording (every
    receiver term cancels; paths 1057 and 1061 km): -49.96 ms (01:20 UTC set)
    and -50.47 ms (02:02 UTC set), night. `AllouisDecoder` reports the
    midpoint (`kSecondAfterStartSec`). Still to confirm by day.

  **Second set, 2026-09-25 02:02 UTC, 605 s each**: M9PSY-1 (MSF, ALS162,
  DCF77) and Heppen (MSF, ALS162; its stamps are ~150-200 ms off and jitter by
  ms, so decode only). Both decoders decode every minute from both receivers,
  0 wrong labels. On M9PSY-1 against the stamps, DCF77's decoder alongside:

  | | 01:20 set | 02:02 set |
  |---|---|---|
  | DCF77 (PM) | +3.53 ms | +3.63 ms |
  | MSF | +0.678 ms, sd 24 us | +0.791 ms, sd 43 us |
  | ALS162 (midpoint) | +3.57 ms, 5.9 us second to second | +3.17 ms, 3.8 us |
  | MSF - DCF77 | -2.854 ms | -2.843 ms |
  | ALS162 - DCF77 | +0.04 ms | -0.47 ms |

  MSF - DCF77 holds to 11 us across both; the paths predict -3.12 ms, so MSF's
  edge (the steepest point of its fall) reads about 0.27 ms late. A candidate
  edge bias, from 15 minutes at night: not a constant until it has 20+ settled
  minutes by day too and the NTP class agrees.
- **Live.** All three LF stations on M9PSY-1 at once, then watch the Agreement
  chart and each station's class delta. With the shared crosshair (`4b4f975`),
  jitter, SNR and offset can be read across stations at any moment.

## References to obtain

- **NPL, "MSF 60 kHz time and date code"** (the official specification, 2019
  edition):
  <https://www.npl.co.uk/getattachment/2afd4c3b-fc8d-486c-b2b9-fc9a5a66394c/msf-time-date-code-2019.pdf>.
  **Obtained and read: complete.** It settles every MSF question (see
  *Station facts*). The one thing it doesn't specify is how deep "off" is;
  ITU-R TF.2487 §3 says 100%.
- **ITU-R Report TF.2487 (2021)**, which covers ALS162:
  <https://www.itu.int/dms_pub/itu-r/opb/rep/R-REP-TF.2487-2021-PDF-E.pdf>.
  **Obtained and read (§3 MSF, §4 WWVB, §9 ALS162): a summary only.** §9.3–9.4
  are about consumer receivers' sensitivity and add nothing on the format. It gives the site, the carrier, the
  modulation in outline, the content and the Tuesday outage. For the
  excursion's shape and timing point and the exact bit layout, it points to
  **NF C90-002 (1988)** (AFNOR, French only), which is still needed, or to
  measurement (see *Station facts*).
- **[henningM1r/gr_ALS162_Receiver](https://github.com/henningM1r/gr_ALS162_Receiver)**
  (GPLv3, `34c8e6c`). **Read: settles the format.** The excursion's shape,
  binary 1, the full bit layout (checked by decoding its example frame) and the
  per-second position codes. Doesn't settle the timing point. Checked in a
  local clone (`~/repos/gr_ALS162_Receiver`): the table's properties and the
  second each key is sent in are under *Station facts*.

## Open questions

- Can UberSDR open IQ channels on 60 and 162 kHz on the RX888? DCF77 already
  uses IQ at 77.5 kHz, so probably yes; confirm before building.
- **Which minute does an Allouis frame name?** DCF77 and MSF both send the
  *following* minute. Neither the ITU report nor the model says which one
  ALS162 sends. Settle it from a recording, against DCF77.
- **French legal time.** The frame is CET/CEST (bits 17/18). Convert to UTC
  before the voter, as for MSF's summer time, and test across the changeover
  (bit 16 announces it).
- **Leap seconds on Allouis.** Bits 1/2 warn of one, but nothing says what the
  leap-second minute looks like (61 seconds? where does the extra second
  go, and which position code does it carry?). Handle it as "don't trust
  this minute" until a recording or the standard says.
- **The position-code numbering.** The model sends key "NN" in second NN−1.
  If NF C90-002 numbers them differently, only the mapping changes; a
  recording settles it either way.
- Does `TimeFrameVoter` take MSF's civil-time frames as they are, or does
  summer time need handling before the voter?
