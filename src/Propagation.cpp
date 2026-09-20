#include "Propagation.h"

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace ubersdr_ntp {

namespace {
constexpr double kEarthRadiusM = 6371000.0;
constexpr double kC = 299792458.0;
constexpr double kPi = 3.14159265358979323846;

double rad(double deg) { return deg * kPi / 180.0; }
} // namespace

// Coordinates from NIST Special Publication 250-67 and the WWV/WWVH station
// pages. Precise to well under a kilometre, which is 3 microseconds of flight
// time — four orders of magnitude below anything else in this budget.
GeoPoint wwvSite()  { return GeoPoint{ 40.67806, -105.04667, true }; }
GeoPoint wwvbSite() { return GeoPoint{ 40.67806, -105.04667, true }; }
GeoPoint wwvhSite() { return GeoPoint{ 21.98750, -159.76306, true }; }

double greatCircleMeters(const GeoPoint& a, const GeoPoint& b) {
    if (!a.valid || !b.valid) return 0.0;
    const double p1 = rad(a.lat), p2 = rad(b.lat);
    const double dp = rad(b.lat - a.lat), dl = rad(b.lon - a.lon);
    // Haversine rather than the spherical law of cosines: the latter loses its
    // precision on short paths, and a receiver on the same continent as Fort
    // Collins is a short path.
    const double h = std::sin(dp / 2) * std::sin(dp / 2) +
                     std::cos(p1) * std::cos(p2) * std::sin(dl / 2) * std::sin(dl / 2);
    return 2.0 * kEarthRadiusM * std::asin(std::min(1.0, std::sqrt(h)));
}

// theta = acos( R cos(eps) / (R+h) ) - eps, and the hop spans 2*R*theta.
// See Propagation.h for the triangle this comes out of.
double maxHopMeters(double virtualHeightM, double minElevationRad) {
    const double Rh = kEarthRadiusM + std::max(1.0, virtualHeightM);
    const double eps = std::max(0.0, minElevationRad);
    // R cos(eps) / (R+h) < 1 for any positive height, so the arccos is always
    // defined; the clamp is there for the degenerate h -> 0 case only.
    const double c = std::min(1.0, kEarthRadiusM * std::cos(eps) / Rh);
    const double theta = std::acos(c) - eps;
    return theta > 0.0 ? 2.0 * kEarthRadiusM * theta : 0.0;
}

// How many hops this path is taken to need: the fewest that the geometry can
// actually carry, which is not the same as the fewest that divide the distance.
int hopCount(double distanceMeters, double virtualHeightM) {
    const double maxHop = maxHopMeters(virtualHeightM, kMinElevationRad);
    if (maxHop <= 0.0) return 1;
    return std::max(1, static_cast<int>(std::ceil(distanceMeters / maxHop)));
}

double skywaveDelaySeconds(double distanceMeters, double virtualHeightM) {
    if (distanceMeters <= 0.0) return 0.0;
    if (distanceMeters < kGroundwaveLimitM) return distanceMeters / kC;

    const int hops = hopCount(distanceMeters, virtualHeightM);
    const double groundPerHop = distanceMeters / hops;

    // Each hop is up to the reflection point and back down. Half a hop
    // subtends groundPerHop/2 of arc at the centre of the Earth; the slant
    // range to a point at height h over that arc is the law of cosines on the
    // triangle (centre, ground station, reflection point).
    const double halfArc = (groundPerHop / 2.0) / kEarthRadiusM;   // radians
    const double R = kEarthRadiusM;
    const double Rh = kEarthRadiusM + virtualHeightM;
    const double slant = std::sqrt(R * R + Rh * Rh - 2.0 * R * Rh * std::cos(halfArc));

    return (2.0 * hops * slant) / kC;
}

std::string describePath(const GeoPoint& rx, const GeoPoint& tx, double virtualHeightM) {
    if (!rx.valid || !tx.valid) return "path unknown (no receiver coordinates)";
    const double d = greatCircleMeters(rx, tx);
    const double t = skywaveDelaySeconds(d, virtualHeightM);
    const int hops = d < kGroundwaveLimitM ? 0 : hopCount(d, virtualHeightM);
    char buf[160];
    if (hops == 0) {
        std::snprintf(buf, sizeof buf, "%.0f km groundwave, %.2f ms", d / 1000.0, t * 1000.0);
    } else {
        std::snprintf(buf, sizeof buf, "%.0f km, %d hop%s at %.0f km, %.2f ms",
                      d / 1000.0, hops, hops == 1 ? "" : "s", virtualHeightM / 1000.0, t * 1000.0);
    }
    return buf;
}

} // namespace ubersdr_ntp
