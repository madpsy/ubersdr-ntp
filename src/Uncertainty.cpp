#include "Uncertainty.h"

#include <algorithm>

namespace ubersdr_ntp {

// Floor on how well the delay model can be trusted, whatever it computed. The
// receiver's own buffering between radiod and the WebSocket is inside this and
// nothing here can see it.
//
// Set from the worst errors measured against an absolute reference, with
// margin, rather than guessed. Two north-eastern US receivers, ~65 ms of
// modelled delay each: against a PPS-disciplined stratum 1 on 2026-09-17 the
// worst single receiver was 9.0 ms out and the served time 5.3 ms; against
// ntpd on 2026-09-13 the served time was at worst +8.6 ms, about 7.6 ms after
// the chain constant was corrected. With the 1-3 ms each source measures of
// itself on top, a 10 ms floor still covers every one of those by about 1.4x.
// It was 15 ms, which covered them about 2x and left the served dispersion
// near 21 ms for an error that has not been seen past 9.
const double kDelayUncertaintyFloorSec = 0.010;

// And a proportional part, because a long path is a worse-known path: more
// hops, more spread between them, more of the virtual height assumption. At
// 15% it only overtakes the floor past about 67 ms of delay, i.e. on paths
// longer than the ones the floor was measured on.
constexpr double kDelayUncertaintyFraction = 0.15;

// The floor above is the doubt in an HF skywave path timed by packet arrival,
// with UberSDR's buffering inside it. None of that is in an LF source that is
// capture-timed on a groundwave path with a sharp second edge: radiod stamps
// the samples at the RX888 (so no chain, network or buffering is in the path),
// the groundwave delay is geometry at a known velocity (the night-time skywave
// off the D/E layer is at most ~90 us later; Propagation.h), and the edge is
// the decoder's to within its tests' resolution. What is left is that skywave
// bias, the RX888's own latency (50-300 us, not yet taken off in radiod) and
// the decoder's residual. 1 ms covers the unmeasured terms together about
// twice over. Measured on M9PSY-1 against a GPS-fed stratum 1 on the same
// host, one RX888 and one set of capture stamps:
//
//   DCF77 by PM, 1061 km   within 0.03 ms smoothed over 30 settled minutes
//                          (2026-09-24), jitter spikes to 0.2 ms
//   Allouis by its phase   +0.48 ms before the second-offset fix, then 0.02 ms
//   correlation, 1057 km   from DCF77 on the same stamps (02:02 recording,
//                          2026-09-25)
//   MSF by its carrier     -0.19 ms of steepest-fall edge bias, now in the
//   going off, 125 km      model; night halves -0.12 and -0.25 ms (one-hop
//                          skywave). By day (afternoon of 2026-09-25) within
//                          40 us of Allouis, and the served time from the two
//                          inside the GPS peer's own +-0.25 ms minute-to-minute
//                          scatter; jitter 5-30 us
//
// MSF qualifies on its AM because its second is the carrier switched fully
// off, a sharp edge. DCF77's AM does not: a cut to 15% through a high-Q
// antenna, read late by a millisecond or more, and DCF77 is timed by PM alone
// (Dcf77Decoder, edgeServable). WWVB's is a 17 dB reduction, not measured
// against a reference here, and keeps the 10 ms.
//
// Only inside the groundwave service area; past it the modes interfere.
const double kDelayUncertaintyFloorLfSec = 0.001;
const double kLfGroundwaveServiceM = 2000e3;

// WWV and WWVH, capture-timed, from a receiver whose decoder has named the
// station. The 10 ms floor was measured on arrival-timed WWV, where the chain
// and the receiver's buffering were inside the error; capture timing takes both
// out. What is left is the skywave geometry, bounded from the model itself by
// skywaveModeSpreadSeconds() (Propagation.h), and the decoder's edge on live
// ticks, whose bias was measured on synthetic ones: this allowance is for
// that, not yet measured against a reference. Until the decoder names the
// station the transmitter is a guess worth ~14 ms, and the 10 ms stands.
constexpr double kWwvDecoderAllowanceSec = 0.001;

// The smallest uncertainty a source may claim when it is weighed against the
// others, where it counts as weight / uncertainty^2.
//
// It keeps a source that reports a suspiciously perfect zero -- a synthetic
// stream, or a window too short to have scattered yet -- from taking an
// unbounded share of the weight. It must also stay below what honest sources
// actually measure, or it erases real differences between them: it was 1 ms,
// "about the decoder's own edge resolution", from when every source was WWV.
// The LF decoders resolve far finer -- MSF and Allouis measure 5-30 us of
// jitter, DCF77 by PM 40-100 us (M9PSY-1, 2026-09-25) -- and at 1 ms all of
// them counted the same as a DCF77 timed by AM at 1.5 ms, which took 13-33% of
// the weight and pulled the served time 0.25 ms off the two sources that
// agreed with each other to 40 us. Replayed at 0.1 ms, that source's share is
// under 1%, the served time sits on MSF and Allouis, and 0.01 ms changes
// nothing further: so 0.1 ms, which keeps the most protection that costs no
// accuracy. Sources below it weigh the same, which at that level is fair.
const double kWeightDispersionFloorSec = 0.0001;

namespace {

GeoPoint lfSite(const UncertaintyInputs& in, bool& known) {
    known = true;
    if (in.broadcast == Broadcast::Dcf77) return dcf77Site();
    if (in.broadcast == Broadcast::Allouis) return allouisSite();
    if (in.broadcast == Broadcast::Lf60 && in.station == "msf") return msfSite();
    known = false;
    return {};
}

} // namespace

double delayUncertaintySec(const UncertaintyInputs& in) {
    const bool captured = in.autoDelay && in.captureTimed && in.receiver.valid;

    // An LF groundwave source with a sharp edge: DCF77 and Allouis while their
    // phase times the second, MSF on its carrier-off. Every condition that earns
    // it must hold; any one lapsing puts the 10 ms back at once.
    bool lfKnown = false;
    const GeoPoint site = lfSite(in, lfKnown);
    const bool sharpEdge = in.broadcast == Broadcast::Lf60 ? true : in.timedByPhase;
    if (captured && lfKnown && sharpEdge &&
        greatCircleMeters(in.receiver, site) <= kLfGroundwaveServiceM) {
        return std::max(kDelayUncertaintyFloorLfSec, in.delaySec * kDelayUncertaintyFraction);
    }

    if (captured && in.broadcast == Broadcast::Wwv && (in.station == "wwv" || in.station == "wwvh")) {
        const GeoPoint tx = in.station == "wwvh" ? wwvhSite() : wwvSite();
        return skywaveModeSpreadSeconds(greatCircleMeters(in.receiver, tx)) + kWwvDecoderAllowanceSec;
    }

    return std::max(kDelayUncertaintyFloorSec, in.delaySec * kDelayUncertaintyFraction);
}

double weightDispersionSec(double ownSec) {
    return std::max(kWeightDispersionFloorSec, ownSec);
}

} // namespace ubersdr_ntp
