# Reference material

Not built, not installed. Kept because both files answer questions that come up
when changing `src/Source.cpp` or the decoders.

## `ubersdr-clock-README.md`

The upstream documentation for the decoders in `../src/clock/`: the JSON event
format, the acquisition funnel's fields, what `--plausibility-minutes` guards
against and why disarming it is riskier than it looks, and the tuning table.
`src/clock/` is a byte-identical copy of upstream's DSP, so this describes the
code that is actually running here.

## `ubersdr-clock-stdio-frontend.cpp.txt`

Upstream's own front end for those decoders — the stdio program that reads raw
PCM and writes JSON. This project replaces it with `src/Source.cpp`, so it is
here as the reference for the two things that front end does which are easy to
get subtly wrong:

- how a `time` event's fields are composed into an instant (against the voted
  frame's own `frameStartSample`, **not** `lastEdgeSecondOfFrame`, which can
  point into a later frame);
- how the plausibility gate's reference clock is plumbed in.

`Source::onClockTime` follows it deliberately. If the decoders are ever updated
from upstream, check this file for changes to either.
