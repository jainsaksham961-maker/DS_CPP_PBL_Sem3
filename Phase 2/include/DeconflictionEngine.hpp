#pragma once

#include "Types.hpp"
#include "AirwayGraph.hpp"
#include "RoutingEngine.hpp"
#include <vector>

class DeconflictionEngine {
public:
    DeconflictionEngine();

    // Verifies whether current in-flight drone trajectories violate separation minimums
    bool detectMidAirConflict(const DroneTelemetryAgent& droneA, 
                              const DroneTelemetryAgent& droneB, 
                              double safe_distance_m = 50.0) const;

    // Dynamically recalculates trajectory from current airborne coordinates when forward edge blocks
    PathResult executeInFlightReroute(AirwayGraph& graph, 
                                     const RoutingEngine& router, 
                                     DroneTelemetryAgent& drone, 
                                     int blocked_u, 
                                     int blocked_v, 
                                     const WindField& wind);
};
