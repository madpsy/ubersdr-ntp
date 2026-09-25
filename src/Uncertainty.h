// What a radio source claims to be worth: how far wrong its delay model could
// be, and the smallest uncertainty it may claim when weighed against the other
// sources. Pure functions, so the rules are tested apart from a live Source.
#pragma once

#include "Config.h"
#include "Propagation.h"

#include <string>

namespace ubersdr_ntp {

// Everything the delay-uncertainty rule looks at, from the source's snapshot.
struct UncertaintyInputs {
    Broadcast broadcast = Broadcast::Wwv;
    std::string station;          // the decoder's tag: "wwv", "wwvh", "msf", "wwvb", ...
    bool autoDelay = true;        // the delay is modelled, not configured verbatim
    bool captureTimed = false;    // capture timing in force and not paused
    bool timedByPhase = false;    // PM locked and timing the second (DCF77, Allouis)
    GeoPoint receiver;            // .valid false when the receiver publishes none
    double delaySec = 0.0;        // the modelled one-way delay
};

// How far wrong the delay model could be for this source, in seconds: the
// term nothing can measure, and so the one that usually dominates dispersion.
double delayUncertaintySec(const UncertaintyInputs& in);

// What a source is worth against the others: its own measured terms (jitter,
// clock residual and slope, rate term), floored. See kWeightDispersionFloorSec.
double weightDispersionSec(double ownSec);

// Exposed for the tests.
extern const double kDelayUncertaintyFloorSec;
extern const double kDelayUncertaintyFloorLfSec;
extern const double kWeightDispersionFloorSec;
extern const double kLfGroundwaveServiceM;

} // namespace ubersdr_ntp
