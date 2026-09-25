#pragma once

// Where the time signal came from, and how long it took to get there.
//
// This is the largest term in the delay budget that can actually be computed
// rather than guessed: a 10 MHz signal from Fort Collins to western Europe is
// two or three ionospheric hops and something over 25 ms of flight time. Left
// out, it biases every served timestamp by that amount in the same direction.
//
// The model is a specular reflection at a fixed virtual height, repeated for
// as many hops as the distance needs. That is a caricature of HF propagation —
// the real virtual height moves with frequency, time of day and solar
// activity, between roughly 100 km (E layer) and 400 km (F2 at night) — and it
// is deliberately a caricature, because the alternative is an ionospheric model
// with inputs this daemon does not have.
//
// What it buys: on a 7000 km path the difference between assuming a straight
// line and modelling two F2 hops is about 1.5 ms, and the difference between a
// 250 km and a 350 km virtual height is about 0.6 ms. So the estimate is good
// to a millisecond or two on the geometry, against a term of 25 ms — which is
// the right trade when the receiver's own buffering, which nothing here can
// see, is several times larger than either.

#include <string>

namespace ubersdr_ntp {

struct GeoPoint {
    double lat = 0.0;   // degrees north
    double lon = 0.0;   // degrees east
    bool valid = false;
};

// The NIST transmitter sites.
//
// WWV and WWVB share the Fort Collins site; WWVH is on Kauai. Which one a
// source is hearing is not a configuration item — the decoder reports it on
// every event, and on the four frequencies WWV and WWVH share it can be either
// depending on the hour.
GeoPoint wwvSite();    // Fort Collins, Colorado
GeoPoint wwvhSite();   // Kekaha, Kauai, Hawaii
GeoPoint wwvbSite();   // Fort Collins, Colorado
GeoPoint dcf77Site();  // Mainflingen, Germany
GeoPoint msfSite();    // Anthorn, UK
GeoPoint allouisSite(); // Allouis, France

// Great-circle distance in metres.
double greatCircleMeters(const GeoPoint& a, const GeoPoint& b);

inline constexpr double kGroundwaveLimitM = 300000.0;   // 300 km

// THIS MODEL IS A LOWER BOUND, AND THAT IS WHY IT NEEDED FIXING
//
// Every free choice here used to be made in the direction that shortens the
// path: the fewest hops that can span the distance, at the lowest plausible
// reflection height. A real path is never SHORTER than the shortest geometry
// that can carry it, but it is very often longer -- a higher layer, or one hop
// more than the minimum. So the error was not zero-mean noise, it was a
// one-signed deficit, and it showed: the radio class read LATE against the NTP
// class on every receiver ever measured, never once early.
//
// A delay model wants the MEAN over the modes actually carrying the signal,
// not the floor of their range. The two changes below are both in that
// direction, and neither was fitted to the NTP comparison -- see Selector.h on
// why that comparison must stay independent.
//
// 350 km, not 300. Mid-latitude F2 sits near 250-300 km by day and climbs to
// 350-450 km through the night, and higher still as a path approaches its MUF,
// where the ionogram's nose turns up. 300 km is a fair DAYTIME figure and was
// being used all hours. 350 km is the honest all-hours average for the outlets
// these decoders hear, and it is still comfortably inside the band.
inline constexpr double kVirtualHeightM = 350000.0;

// The lowest takeoff angle a hop is credited with. A ray cannot leave below
// the local horizon, and an HF antenna over real ground has nothing useful
// below about 3 degrees. This is what bounds the hop length below.
inline constexpr double kMinElevationRad = 3.0 * 3.14159265358979323846 / 180.0;

// Longest ground distance one hop can cover, from geometry rather than folklore.
//
// The old code carried a hard-coded 4000 km "usual working number for F2"
// alongside a 300 km height, and those two cannot both be true: at 300 km the
// geometric maximum is 3836 km even at ZERO elevation, and 3225 km at a usable
// 3 degrees. Every path between about 3225 and 4000 km was therefore being
// given one hop where the geometry demands two, and was under-delayed by most
// of a millisecond -- a pure modelling error, always in the same direction.
//
// Derivation. Earth centre O, station A at radius R, reflection point B at
// radius R+h. The ray leaves A at elevation eps above the local horizon, which
// is perpendicular to OA, so the interior angle at A is 90 deg + eps. With
// theta the half-hop arc at O, the remaining angle at B is 90 deg - eps -
// theta, and the sine rule gives
//
//     (R+h) / sin(90 + eps)  =  R / sin(90 - eps - theta)
//     cos(eps + theta)       =  R cos(eps) / (R + h)
//     theta                  =  acos( R cos(eps) / (R+h) ) - eps
//
// and one hop spans 2*R*theta of ground.
double maxHopMeters(double virtualHeightM = kVirtualHeightM,
                    double minElevationRad = kMinElevationRad);

// One-way propagation delay in seconds for a skywave path of this length.
//
// virtualHeightM is the reflection height. Paths under kGroundwaveLimitM are
// treated as groundwave and get the great-circle distance at the speed of
// light. Hop count comes from maxHopMeters at the same height, so the two can
// never again disagree about what the geometry allows.
double skywaveDelaySeconds(double distanceMeters, double virtualHeightM = kVirtualHeightM);

// How far the real path's delay can plausibly sit from skywaveDelaySeconds():
// the largest difference, either sign, over the modes that could be carrying
// it -- a virtual height anywhere from kVirtualHeightLowM to kVirtualHeightHighM,
// and the fewest hops that height can carry or one more. That is the
// geometry's whole contribution to the doubt, and it is what bounds the delay
// model once capture timing has taken the receiver's chain and the network out
// of the path. About 1.4 ms at 7000 km and under 2 ms on most long paths; more
// on short ones, where one extra hop is a larger share of the flight.
inline constexpr double kVirtualHeightLowM = 250000.0;
inline constexpr double kVirtualHeightHighM = 450000.0;
double skywaveModeSpreadSeconds(double distanceMeters);

// WWVB IS NOT AN HF PATH AND MUST NOT USE THE GEOMETRY ABOVE
//
// 60 kHz does not hop off the F layer. It travels in the Earth-ionosphere
// waveguide, turning at the D region -- near 70 km by day, 85-90 km at night --
// and across the continental US the FIRST energy to arrive is the groundwave,
// which follows the surface. That is also what an envelope decoder locks to:
// the leading edge of the second, not whatever arrives later.
//
// Sending it through the F2 geometry put WWVB at a 350 km reflection height and
// over-delayed it by about 0.45 ms at 2500 km -- alone among the errors in this
// file, in the direction that makes the radio read EARLY.
//
// So: great circle, at the speed of light, retarded slightly because the ground
// is not a perfect conductor. 1.0003 is a fair working index over mixed land at
// LF, and it is worth about 1 us per 1000 km -- negligible next to everything
// else in the budget, and carried only because it costs one multiply to be
// right rather than approximately right.
//
// Past roughly 2000 km the skywave begins to matter and the two modes
// interfere. The waveguide path is only about 0.1 ms longer than the surface
// one at those ranges, so the mode choice stays well inside the delay
// uncertainty; the 350 km height did not.
//
// DCF77 at 77.5 kHz is the same kind of path and gets the same model. Its
// service area is inside about 2000 km, where the groundwave is what an edge
// decoder -- or the PM correlator -- locks to; the night-time skywave that
// fades it across Britain arrives off a ~90 km D/E layer, which on a 700 km
// path is some 90 us longer, well inside the budget. That is one hop over a
// curved Earth; the flat-Earth figure of about 75 us understates it, and more
// so with distance -- it falls towards 25 us at 2000 km, where the curved one
// holds near 65 us. The skywave only ever arrives later, so ignoring it
// biases a night-time path late, never early.
inline constexpr double kLfGroundIndex = 1.0003;
double lfDelaySeconds(double distanceMeters);

// A human-readable one-liner for the log: distance, hops and delay.
std::string describePath(const GeoPoint& rx, const GeoPoint& tx,
                         double virtualHeightM = kVirtualHeightM);
std::string describeLfPath(const GeoPoint& rx, const GeoPoint& tx);

} // namespace ubersdr_ntp
