#include "Types.hpp"
#include "AirwayGraph.hpp"
#include "RoutingEngine.hpp"
#include "MavlinkBridge.hpp"
#include "DeconflictionEngine.hpp"
#include "BenchmarkScheduler.hpp"
#include <iostream>
#include <iomanip>
#include <vector>

int main() {
    std::cout << "\n============================================================\n"
              << "   AUTONOMOUS DRONE FLEET ROUTING & AIRSPACE MANAGEMENT    \n"
              << "   Team ID: DSCPP-III-2026-T154 | Graphic Era University   \n"
              << "   Complete Integrated Build: Phase 2 + Phase 3 All Deliverables\n"
              << "============================================================\n\n";

    // -------------------------------------------------------------
    // STAGE 1 (PHASE 2): 3D Airspace Graph Setup & Baseline A*
    // -------------------------------------------------------------
    std::cout << "\033[1;34m[STAGE 1 / PHASE 2: 3D GRAPH TOPOLOGY & NOMINAL A* ROUTING]\033[0m\n";
    AirwayGraph graph;

    graph.addWaypoint(0, "Central Depot",     {   0.0,    0.0, 15.0}, true);
    graph.addWaypoint(1, "North Junction",    { 600.0,  900.0, 45.0}, false);
    graph.addWaypoint(2, "East Hub",          {1400.0,  500.0, 30.0}, false);
    graph.addWaypoint(3, "Suburban Dropzone", {2000.0, 1200.0, 20.0}, false);
    graph.addWaypoint(4, "South Skyway",      { 800.0, -400.0, 50.0}, false);
    graph.addWaypoint(5, "Metro Hub Depot",   {1800.0, -200.0, 15.0}, true);

    graph.addCorridor(0, 1);
    graph.addCorridor(0, 4);
    graph.addCorridor(1, 2);
    graph.addCorridor(1, 3);
    graph.addCorridor(2, 3);
    graph.addCorridor(4, 5);
    graph.addCorridor(2, 5);

    graph.printNetwork();

    RoutingEngine router;
    WindField ambient_wind = { 4.5, 45.0 }; // 4.5 m/s wind heading North-East
    std::cout << "[INFO] Dispatching Mission A* Search: Node 0 -> Node 3...\n";
    PathResult mission_route = router.computeAStarRoute(graph, 0, 3, 18.0, 2.5, ambient_wind);

    if (mission_route.success) {
        std::cout << "\033[1;32m[SUCCESS] Nominal Flight Trajectory Computed:\033[0m\n";
        std::cout << "  Flight Path       : ";
        for (size_t i = 0; i < mission_route.path.size(); ++i) {
            std::cout << mission_route.path[i] << (i + 1 < mission_route.path.size() ? " -> " : "");
        }
        std::cout << "\n  Distance          : " << std::fixed << std::setprecision(1) << mission_route.total_distance << " m\n";
        std::cout << "  Estimated Duration: " << mission_route.total_time << " s (" << (mission_route.total_time / 60.0) << " min)\n";
        std::cout << "  Battery Draw      : " << mission_route.total_energy << " Wh\n\n";
        
        graph.renderConsoleRadar(mission_route.path[0], mission_route.path[1]);
    }

    // -------------------------------------------------------------
    // STAGE 2 (PHASE 3, DELIVERABLE 2): SITL / MAVLink v2 Protocol
    // -------------------------------------------------------------
    std::cout << "\033[1;36m[STAGE 2 / PHASE 3: SITL & MAVLINK AUTOPILOT PROTOCOL]\033[0m\n";
    MavlinkBridge mavlink(1, 1); // SysID: 1, CompID: 1

    std::vector<uint8_t> hb_packet = mavlink.packHeartbeat(MavAutopilotState::MAV_STATE_ACTIVE);
    mavlink.printPacketTelemetry(hb_packet);

    Vector3D current_drone_pos = graph.nodes[1].coords;
    std::vector<uint8_t> pos_packet = mavlink.packGlobalPosition(current_drone_pos, 12.5, 12.5, 0.0);
    mavlink.printPacketTelemetry(pos_packet);
    std::cout << "\n";

    // -------------------------------------------------------------
    // STAGE 3 (PHASE 3, DELIVERABLE 1): Live Mid-Air Deconfliction & Reroute
    // -------------------------------------------------------------
    std::cout << "\033[1;36m[STAGE 3 / PHASE 3: DYNAMIC MID-AIR REROUTING & DECONFLICTION]\033[0m\n";
    DeconflictionEngine deconfliction;

    DroneTelemetryAgent droneAlpha;
    droneAlpha.sysid = 1;
    droneAlpha.callsign = "UAV-ALPHA-01";
    droneAlpha.current_waypoint = 1;
    droneAlpha.target_waypoint = 3;
    droneAlpha.current_position = { 600.0, 900.0, 45.0 };
    droneAlpha.in_flight = true;
    droneAlpha.active_route = {0, 1, 3};

    DroneTelemetryAgent droneBravo;
    droneBravo.sysid = 2;
    droneBravo.callsign = "UAV-BRAVO-02";
    droneBravo.current_waypoint = 1;
    droneBravo.target_waypoint = 0;
    droneBravo.current_position = { 625.0, 915.0, 45.0 }; // Proximity hazard (<50m)
    droneBravo.in_flight = true;

    if (deconfliction.detectMidAirConflict(droneAlpha, droneBravo, 50.0)) {
        std::cout << "\033[1;31m[PROXIMITY WARNING] Spatial separation violation between "
                  << droneAlpha.callsign << " and " << droneBravo.callsign 
                  << " (< 50m buffer)!\033[0m\n";
    }

    // Dynamic obstacle: Corridor [1 <-> 3] dynamically closes
    deconfliction.executeInFlightReroute(graph, router, droneAlpha, 1, 3, ambient_wind);
    graph.renderConsoleRadar(droneAlpha.active_route[0], droneAlpha.active_route[1]);

    // -------------------------------------------------------------
    // STAGE 4 (PHASE 3, DELIVERABLE 3): Makespan Benchmarking (EDF vs FIFO)
    // -------------------------------------------------------------
    std::cout << "\033[1;36m[STAGE 4 / PHASE 3: MAKESPAN BENCHMARKING (FIFO vs EDF MIN-HEAP)]\033[0m\n";
    BenchmarkScheduler benchmarker;

    std::vector<DeliveryOrder> batch_orders = {
        {101, 0, 3, 2.0,  0.0, 180.0, 0.0}, // Order 1: Urgent (180s SLA)
        {102, 0, 2, 1.5, 10.0, 600.0, 0.0}, // Order 2: Standard
        {103, 5, 1, 3.0, 20.0, 220.0, 0.0}, // Order 3: Critical Medical (220s SLA)
        {104, 0, 5, 2.5, 30.0, 800.0, 0.0}, // Order 4: Flexible
        {105, 4, 3, 1.8, 40.0, 250.0, 0.0}  // Order 5: High Priority (250s SLA)
    };

    BenchmarkMetrics fifo_res = benchmarker.runFIFOBenchmark(batch_orders, graph, router, ambient_wind);
    BenchmarkMetrics edf_res  = benchmarker.runEDFBenchmark(batch_orders, graph, router, ambient_wind);
    benchmarker.printComparison(fifo_res, edf_res);

    // -------------------------------------------------------------
    // STAGE 5 (PACKAGING & VISUALIZER EXPORT)
    // -------------------------------------------------------------
    graph.exportHtmlVisualizer("airspace_map.html");
    std::cout << "[EXPORT] SVG Airspace Map exported to 'airspace_map.html'.\n";
    std::cout << "\033[1;32m[SYSTEM STATUS] All Phase 2 & Phase 3 Modules Successfully Executed!\033[0m\n\n";

    return 0;
}
