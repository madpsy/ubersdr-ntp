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

// Great-circle distance in metres.
double greatCircleMeters(const GeoPoint& a, const GeoPoint& b);

// One-way propagation delay in seconds for a skywave path of this length.
//
// virtualHeightM is the reflection height; 300 km is a reasonable all-hours
// average for the HF outlets. Paths under kGroundwaveLimitM are treated as
// groundwave and get the great-circle distance at the speed of light.
double skywaveDelaySeconds(double distanceMeters, double virtualHeightM = 300000.0);

inline constexpr double kGroundwaveLimitM = 300000.0;   // 300 km
// Longest ground distance one F-layer hop is taken to cover. The real figure
// depends on the takeoff angle and the layer height; 4000 km is the usual
// working number for F2.
inline constexpr double kMaxHopM = 4000000.0;

// A human-readable one-liner for the log: distance, hops and delay.
std::string describePath(const GeoPoint& rx, const GeoPoint& tx, double virtualHeightM = 300000.0);

} // namespace ubersdr_ntp
