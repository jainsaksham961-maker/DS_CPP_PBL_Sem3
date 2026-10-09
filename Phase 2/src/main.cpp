#include "AirwayGraph.hpp"
#include "FleetManager.hpp"
#include "OrderScheduler.hpp"
#include "RoutingEngine.hpp"
#include <iostream>
#include <string>
#include <limits>
#include <vector>

int promptInt(const std::string& label, const std::string& example, int min_val = 0) {
    int value;
    while (true) {
        std::cout << label << " (e.g., " << example << "): ";
        if (std::cin >> value && value >= min_val) {
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
            return value;
        }
        std::cout << "  [!] Invalid input. Enter an integer >= " << min_val << ".\n";
        std::cin.clear();
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    }
}

double promptDouble(const std::string& label, const std::string& example, double min_val = -1e9) {
    double value;
    while (true) {
        std::cout << label << " (e.g., " << example << "): ";
        if (std::cin >> value && value >= min_val) {
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
            return value;
        }
        std::cout << "  [!] Invalid decimal number.\n";
        std::cin.clear();
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    }
}

Vector3D promptCoordinates(const std::string& label, const std::string& example) {
    double x, y, z;
    while (true) {
        std::cout << label << " (e.g., " << example << "): ";
        if (std::cin >> x >> y >> z) {
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
            return {x, y, z};
        }
        std::cout << "  [!] Invalid 3D coordinates. Enter three space-separated numbers (X Y Z).\n";
        std::cin.clear();
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    }
}

std::string promptString(const std::string& label, const std::string& example) {
    std::string value;
    while (true) {
        std::cout << label << " (e.g., " << example << "): ";
        if (std::cin >> value && !value.empty()) {
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
            return value;
        }
        std::cout << "  [!] Field cannot be empty.\n";
        std::cin.clear();
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    }
}

int main() {
    std::cout << COLOR_CYAN << "╔══════════════════════════════════════════════════════════════════════════╗\n";
    std::cout << "║   AUTONOMOUS DRONE FLEET ROUTING MANAGEMENT SYSTEM (PHASE-II ENGINE)     ║\n";
    std::cout << "╚══════════════════════════════════════════════════════════════════════════╝\n" << COLOR_RESET;

    AirwayGraph graph;

    // 1. Central Depot
    std::cout << "\n>>> STEP 1: Central Launch Depot Setup\n";
    Vector3D depot_coords = promptCoordinates("  Enter Depot Coordinates [X Y Z]", "0 0 50");
    graph.addWaypoint(0, "Central_Depot", depot_coords, true);
    std::cout << "  [OK] Central Depot created at ID [0].\n";

    // 2. Waypoints
    std::cout << "\n>>> STEP 2: Delivery Waypoints Registration\n";
    int num_waypoints = promptInt("  Total delivery locations/zones", "3", 1);
    for (int i = 1; i <= num_waypoints; ++i) {
        std::cout << "\n  --- Location #" << i << " (Node ID: " << i << ") ---\n";
        std::string label = promptString("  Location Name/Label", (i == 1 ? "School" : (i == 2 ? "Temple" : "College")));
        Vector3D coords = promptCoordinates("  Location Coordinates [X Y Z]", (i == 1 ? "1000 500 60" : (i == 2 ? "1500 1200 70" : "2500 1800 65")));
        graph.addWaypoint(i, label, coords, false);
    }

    // 3. Corridors
    std::cout << "\n>>> STEP 3: Corridor Connections Setup\n";
    int num_corridors = promptInt("  Total corridors connecting zones", "3", 1);
    for (int i = 1; i <= num_corridors; ++i) {
        int u = promptInt("  Corridor #" + std::to_string(i) + " Start Node ID", "0", 0);
        int v = promptInt("  Corridor #" + std::to_string(i) + " End Node ID", std::to_string(u == 0 ? 1 : u + 1), 0);
        graph.addCorridor(u, v, CorridorStatus::OPEN_AVAILABLE);
        std::cout << "  [OK] Corridor added: [" << graph.nodes[u].label << "] <---> [" << graph.nodes[v].label << "]\n";
    }

    // 4. Set Blocked / Red Corridors
    std::cout << "\n>>> STEP 4: Configure Restricted Airspace (RED ROUTES)\n";
    int num_blocked = promptInt("  How many corridors are BLOCKED (e.g., severe weather / no-fly)?", "1", 0);
    for (int i = 1; i <= num_blocked; ++i) {
        int bu = promptInt("  Blocked Route #" + std::to_string(i) + " Start Node ID", "2", 0);
        int bv = promptInt("  Blocked Route #" + std::to_string(i) + " End Node ID", "3", 0);
        graph.setCorridorStatus(bu, bv, CorridorStatus::BLOCKED_RESTRICTED);
        std::cout << COLOR_RED << "  [X] Corridor [" << graph.nodes[bu].label << "] <-> [" 
                  << graph.nodes[bv].label << "] marked as BLOCKED.\n" << COLOR_RESET;
    }

    // 5. Environmental Wind
    std::cout << "\n>>> STEP 5: Atmospheric Vector\n";
    Vector3D wind_v = promptCoordinates("  Wind Vector [Vx Vy Vz in m/s]", "-3.0 1.0 0");
    WindField ambient_wind{wind_v};

    // 6. Fleet
    std::cout << "\n>>> STEP 6: Drone Registration\n";
    FleetManager fleet;
    int num_drones = promptInt("  Number of drones in fleet", "2", 1);
    for (int i = 1; i <= num_drones; ++i) {
        std::cout << "\n  --- Drone #" << i << " ---\n";
        std::string callsign = promptString("  Drone Call-sign", (i == 1 ? "UAV-ALPHA" : "UAV-BETA"));
        double payload = promptDouble("  Max Payload (kg)", (i == 1 ? "8.0" : "12.0"), 0.1);
        double battery = promptDouble("  Battery Capacity (Wh)", (i == 1 ? "600" : "800"), 10.0);
        double speed = promptDouble("  Airspeed (m/s)", (i == 1 ? "16" : "14"), 1.0);
        fleet.registerDrone({callsign, payload, battery, battery, speed, 0, DroneStatus::IDLE});
    }

    // 7. Orders
    std::cout << "\n>>> STEP 7: Customer Delivery Orders\n";
    OrderScheduler scheduler;
    int num_orders = promptInt("  Total customer orders to schedule", "2", 1);
    for (int i = 1; i <= num_orders; ++i) {
        std::cout << "\n  --- Order #" << i << " ---\n";
        int order_id = promptInt("  Order ID", std::to_string(1100 + i), 1);
        int dest_node = promptInt("  Destination Node ID", (i == 1 ? "3" : "2"), 1);
        double weight = promptDouble("  Package Weight (kg)", (i == 1 ? "8.7" : "7.6"), 0.1);
        double deadline = promptDouble("  Deadline (seconds)", (i == 1 ? "278" : "237"), 1.0);
        scheduler.enqueueOrder({order_id, dest_node, weight, deadline});
    }

    // 8. Dispatch & Evaluation
    std::cout << "\n" << COLOR_YELLOW << ">>> RUNNING A* DISPATCH ORCHESTRATION..." << COLOR_RESET << "\n";
    while (scheduler.hasPendingOrders()) {
        DeliveryOrder order = scheduler.popNextOrder();
        std::string assigned_id = fleet.findOptimalDrone(order.payload_kg);

        if (assigned_id.empty()) {
            std::cout << COLOR_RED << "  [X] Order #" << order.order_id 
                      << " DEFERRED: No idle drone available with >= " << order.payload_kg 
                      << " kg payload capacity.\n" << COLOR_RESET;
            continue;
        }

        Drone* drone = fleet.getDrone(assigned_id);
        PathResult forward = RoutingEngine::computeAStarRoute(graph, drone->current_node_id, 
                                                             order.target_node_id, drone->cruise_airspeed, 
                                                             order.payload_kg, ambient_wind);

        if (!forward.reachable) {
            std::cout << COLOR_RED << "  [X] Order #" << order.order_id 
                      << " CANNOT BE REACHED: Route to [" << graph.nodes[order.target_node_id].label 
                      << "] obstructed by Red Blocked Corridors.\n" << COLOR_RESET;
            continue;
        }

        std::cout << COLOR_GREEN << "  [✓] Order #" << order.order_id << " Assigned to " 
                  << assigned_id << " | Route: ";
        for (size_t k = 0; k < forward.path.size(); ++k) {
            std::cout << graph.nodes[forward.path[k]].label << (k + 1 < forward.path.size() ? " -> " : "");
        }
        std::cout << " | ETA: " << std::fixed << std::setprecision(1) << forward.total_time_sec << "s\n" << COLOR_RESET;

        // Mark active flight legs as RUNNING_MISSION (GREEN)
        for (size_t k = 0; k + 1 < forward.path.size(); ++k) {
            graph.setCorridorStatus(forward.path[k], forward.path[k + 1], CorridorStatus::RUNNING_MISSION);
        }
    }

    // 9. Display Visual Elements
    graph.renderAsciiRadar();
    graph.renderCorridorStatusMatrix();
    graph.exportHtmlVisualizer("airspace_map.html");

    std::cout << "\nPhase-II Demonstration Complete.\n";
    return 0;
}
