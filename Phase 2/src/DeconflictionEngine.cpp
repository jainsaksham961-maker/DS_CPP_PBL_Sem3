#include "DeconflictionEngine.hpp"
#include <iostream>

DeconflictionEngine::DeconflictionEngine() {}

bool DeconflictionEngine::detectMidAirConflict(const DroneTelemetryAgent& droneA, 
                                              const DroneTelemetryAgent& droneB, 
                                              double safe_distance_m) const {
    if (!droneA.in_flight || !droneB.in_flight) return false;
    double spatial_separation = droneA.current_position.distanceTo(droneB.current_position);
    return (spatial_separation < safe_distance_m);
}

PathResult DeconflictionEngine::executeInFlightReroute(AirwayGraph& graph, 
                                                     const RoutingEngine& router, 
                                                     DroneTelemetryAgent& drone, 
                                                     int blocked_u, 
                                                     int blocked_v, 
                                                     const WindField& wind) {
    std::cout << "\n\033[1;33m[DECONFLICTION ENGINE] Dynamic Conflict / Corridor Blockage Detected!\033[0m\n"
              << "  Affected Corridor : [" << blocked_u << " <-> " << blocked_v << "]\n"
              << "  UAV Call Sign     : " << drone.callsign << " (SysID: " << static_cast<int>(drone.sysid) << ")\n"
              << "  Current Waypoint  : " << drone.current_waypoint << "\n"
              << "  Destination       : " << drone.target_waypoint << "\n";

    // 1. Invalidate hazardous corridor immediately in the airway graph
    graph.setCorridorStatus(blocked_u, blocked_v, CorridorStatus::BLOCKED_RESTRICTED);

    // 2. Compute dynamic route starting directly from vehicle's current location
    PathResult new_route = router.computeAStarRoute(graph, 
                                                    drone.current_waypoint, 
                                                    drone.target_waypoint, 
                                                    drone.airspeed, 
                                                    2.0, // Payload mass
                                                    wind);

    if (new_route.success) {
        std::cout << "\033[1;32m[DECONFLICTION SUCCESS] Real-Time Dynamic Rerouting Resolved:\033[0m\n"
                  << "  New Corridor Path : ";
        for (size_t i = 0; i < new_route.path.size(); ++i) {
            std::cout << new_route.path[i] << (i + 1 < new_route.path.size() ? " -> " : "");
        }
        std::cout << "\n  Updated Distance  : " << new_route.total_distance << " m\n"
                  << "  Updated Energy    : " << new_route.total_energy << " Wh\n";

        drone.active_route = new_route.path;
        drone.route_step = 0;
    } else {
        std::cout << "\033[1;31m[CRITICAL FAIL] No alternate collision-free corridor found!\033[0m\n";
    }

    return new_route;
}
