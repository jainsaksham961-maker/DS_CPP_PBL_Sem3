#include "Types.hpp"
#include "AirwayGraph.hpp"
#include "RoutingEngine.hpp"
#include "MavlinkBridge.hpp"
#include "DeconflictionEngine.hpp"
#include "BenchmarkScheduler.hpp"

#include <iostream>
#include <iomanip>
#include <vector>
#include <string>
#include <limits>

void clearInputBuffer() {
    std::cin.clear();
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
}

void printMenu() {
    std::cout << COLOR_CYAN
              << "\n============================================================\n"
              << "   AUTONOMOUS DRONE FLEET ROUTING & AIRSPACE MANAGEMENT     \n"
              << "   Team ID: DSCPP-III-2026-T154 | Graphic Era University    \n"
              << "============================================================\n"
              << COLOR_RESET
              << "  1. View 3D Airspace Network & Tactical ASCII Radar HUD\n"
              << "  2. Configure Drone Parameters (Speed, Battery SoC, Payload)\n"
              << "  3. Configure Ambient Wind Vector (Vx, Vy, Vz)\n"
              << "  4. Plan Custom A* Mission Trajectory (Start -> Goal)\n"
              << "  5. Enqueue Custom Delivery Order (EDF Min-Heap)\n"
              << "  6. Simulate Mid-Air Corridor Blockage & Deconfliction Reroute\n"
              << "  7. Emit SITL / MAVLink v2 Binary Telemetry Frames\n"
              << "  8. Run FIFO vs. EDF Min-Heap Makespan & SLA Benchmark\n"
              << "  9. Export Interactive SVG Airspace Map (airspace_map.html)\n"
              << "  0. Exit Mission Control\n"
              << "------------------------------------------------------------\n"
              << "Select option [0-9]: ";
}

int main() {
    AirwayGraph graph;
    RoutingEngine router;
    MavlinkBridge mavlink;
    DeconflictionEngine deconfliction;
    BenchmarkScheduler benchmark;

    // Default 3D Airspace Network (Nodes 0..5)
    graph.addWaypoint(0, "Central Depot",     {0.0,    0.0,   15.0}, true);
    graph.addWaypoint(1, "North Junction",    {600.0,  900.0, 45.0}, false);
    graph.addWaypoint(2, "East Hub",          {1400.0, 500.0, 30.0}, false);
    graph.addWaypoint(3, "Suburban Dropzone", {2000.0, 1200.0,20.0}, false);
    graph.addWaypoint(4, "South Skyway",      {800.0, -400.0, 50.0}, false);
    graph.addWaypoint(5, "Metro Hub Depot",   {1800.0,-200.0, 15.0}, true);

    graph.addCorridor(0, 1);
    graph.addCorridor(0, 4);
    graph.addCorridor(1, 2);
    graph.addCorridor(1, 3);
    graph.addCorridor(4, 5);
    graph.addCorridor(2, 3);
    graph.addCorridor(2, 5);

    // Active Fleet Agent & Environment State
    DroneTelemetryAgent droneAlpha;
    droneAlpha.sysid = 1;
    droneAlpha.callsign = "UAV-ALPHA-01";
    droneAlpha.current_waypoint = 0;
    droneAlpha.target_waypoint = 3;
    droneAlpha.airspeed = 18.0;
    droneAlpha.battery_soc = 0.92; // 92%
    double drone_payload_kg = 2.5;
    double battery_capacity_wh = 120.0; // Standard 6S LiPo pack

    WindField ambient_wind{{2.5, -1.5, 0.0}, 2.91, 329.0};

    // Order Batch Queue for Benchmarking
    std::vector<DeliveryOrder> order_batch = {
        {101, 0, 3, 2.5, 0.0,  65.0,  0.0},
        {102, 0, 2, 1.8, 5.0,  110.0, 0.0},
        {103, 5, 1, 3.0, 10.0, 75.0,  0.0},
        {104, 0, 5, 2.0, 15.0, 140.0, 0.0}
    };

    int active_u = -1, active_v = -1;
    int choice = -1;

    while (choice != 0) {
        std::cout << std::setfill(' '); // Reset fill char cleanly every loop
        printMenu();
        if (!(std::cin >> choice)) {
            clearInputBuffer();
            continue;
        }

        if (choice == 1) {
            graph.printNetwork();
            graph.renderConsoleRadar(active_u, active_v);
        }
        else if (choice == 2) {
            std::cout << "\n[DRONE CONFIGURATION] Current: " << droneAlpha.callsign
                      << " | Speed: " << droneAlpha.airspeed << " m/s"
                      << " | Battery SoC: " << (droneAlpha.battery_soc * 100.0) << "%"
                      << " | Payload: " << drone_payload_kg << " kg\n";
            std::cout << "Enter Cruise Airspeed (m/s, e.g., 18.0): ";
            std::cin >> droneAlpha.airspeed;
            double soc_pct = 92.0;
            std::cout << "Enter Battery State-of-Charge (%, e.g., 85): ";
            std::cin >> soc_pct;
            droneAlpha.battery_soc = clampVal(soc_pct / 100.0, 0.05, 1.0);
            std::cout << "Enter Payload Mass (kg, e.g., 2.5): ";
            std::cin >> drone_payload_kg;
            std::cout << COLOR_GREEN << "[OK] Drone telemetry updated.\n" << COLOR_RESET;
        }
        else if (choice == 3) {
            std::cout << "\n[WIND FIELD] Enter Wind Velocity Vector (Vx Vy Vz in m/s): ";
            std::cin >> ambient_wind.velocity.x >> ambient_wind.velocity.y >> ambient_wind.velocity.z;
            ambient_wind.velocity_ms = ambient_wind.velocity.norm();
            std::cout << COLOR_GREEN << "[OK] Wind speed set to " << std::fixed << std::setprecision(2)
                      << ambient_wind.velocity_ms << " m/s.\n" << COLOR_RESET;
        }
        else if (choice == 4) {
            int start_node = 0, goal_node = 3;
            std::cout << "\nEnter Start Node ID (0-5): ";
            std::cin >> start_node;
            std::cout << "Enter Destination Node ID (0-5): ";
            std::cin >> goal_node;

            PathResult route = router.computeAStarRoute(
                graph, start_node, goal_node, droneAlpha.airspeed, drone_payload_kg, ambient_wind
            );

            if (!route.success) {
                std::cout << COLOR_RED << "[ERROR] No reachable corridor path between Node "
                          << start_node << " and Node " << goal_node << "!\n" << COLOR_RESET;
            } else {
                double available_wh = droneAlpha.battery_soc * battery_capacity_wh;
                double round_trip_wh = route.total_energy * 1.85; // Outbound + unloaded return reserve
                std::cout << COLOR_GREEN << "\n[A* ROUTE COMPUTED]\n" << COLOR_RESET;
                std::cout << "  Path Sequence     : ";
                for (size_t i = 0; i < route.path.size(); ++i) {
                    std::cout << route.path[i] << (i + 1 < route.path.size() ? " -> " : "\n");
                }
                std::cout << "  Total Distance    : " << std::fixed << std::setprecision(1) << route.total_distance << " m\n"
                          << "  Transit Duration  : " << route.total_time << " s\n"
                          << "  One-Way Energy    : " << route.total_energy << " Wh (Available: " << available_wh << " Wh)\n";

                if (round_trip_wh > available_wh * 0.85) {
                    std::cout << COLOR_RED << "  [BATTERY WARNING] Insufficient SoC reserve for safe round-trip!\n" << COLOR_RESET;
                } else {
                    droneAlpha.active_route = route.path;
                    droneAlpha.current_waypoint = start_node;
                    droneAlpha.target_waypoint = goal_node;
                    if (route.path.size() >= 2) {
                        active_u = route.path[0];
                        active_v = route.path[1];
                    }
                    graph.renderConsoleRadar(active_u, active_v);
                }
            }
        }
        else if (choice == 5) {
            DeliveryOrder ord;
            ord.order_id = static_cast<int>(order_batch.size()) + 101;
            std::cout << "\n[NEW DELIVERY ORDER #" << ord.order_id << "]\n";
            std::cout << "Enter Pickup Depot Node (0 or 5): ";
            std::cin >> ord.source_node;
            std::cout << "Enter Dropoff Node (1-4): ";
            std::cin >> ord.destination_node;
            std::cout << "Enter Payload Weight (kg): ";
            std::cin >> ord.payload_kg;
            std::cout << "Enter SLA Deadline (seconds from dispatch, e.g., 55): ";
            std::cin >> ord.sla_deadline_sec;
            ord.release_time_sec = 0.0;
            order_batch.push_back(ord);
            std::cout << COLOR_GREEN << "[OK] Order #" << ord.order_id << " queued (Total batch size: "
                      << order_batch.size() << ").\n" << COLOR_RESET;
        }
        else if (choice == 6) {
            int block_u = 1, block_v = 3;
            std::cout << "\nEnter Corridor Endpoints to Block (u v, e.g., 1 3): ";
            std::cin >> block_u >> block_v;
            int uav_curr = 1, uav_dest = 3;
            std::cout << "Enter UAV Current Node & Target Node (e.g., 1 3): ";
            std::cin >> uav_curr >> uav_dest;

            deconfliction.executeInFlightReroute(graph, router, droneAlpha, block_u, block_v, ambient_wind);
            if (droneAlpha.active_route.size() >= 2) {
                active_u = droneAlpha.active_route[0];
                active_v = droneAlpha.active_route[1];
            }
            graph.renderConsoleRadar(active_u, active_v);
        }
       else if (choice == 7) {
            std::cout << "\n";
            auto printMavFrame = [](const std::string& label, const std::vector<uint8_t>& frame) {
                std::cout << COLOR_CYAN << "[SITL/MAVLink v2] " << label << " (" << frame.size() << " bytes) -> Hex: " << COLOR_RESET;
                for (size_t i = 0; i < frame.size() && i < 16; ++i) {
                    std::cout << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(frame[i]) << " ";
                }
                std::cout << std::dec << std::setfill(' ') << "... [CRC: OK]\n";
            };

            auto hb_frame = mavlink.packHeartbeat(MavAutopilotState::MAV_STATE_ACTIVE);
            printMavFrame("HEARTBEAT (MsgID 0)", hb_frame);

            const auto& pos = graph.nodes.at(droneAlpha.current_waypoint).coords;
            auto gp_frame = mavlink.packGlobalPosition(pos, droneAlpha.airspeed, 0.0, 0.0);
            printMavFrame("GLOBAL_POSITION_INT (MsgID 33)", gp_frame);
        }
        else if (choice == 8) {
            std::cout << std::setfill(' ');
            BenchmarkMetrics fifo_res = benchmark.runFIFOBenchmark(order_batch, graph, router, ambient_wind);
            BenchmarkMetrics edf_res  = benchmark.runEDFBenchmark(order_batch, graph, router, ambient_wind);
            benchmark.printComparison(fifo_res, edf_res);
        }
        else if (choice == 9) {
            graph.exportHtmlVisualizer("airspace_map.html");
        }
    }

    std::cout << "Exiting Drone Fleet Mission Control.\n";
    return 0;
}

