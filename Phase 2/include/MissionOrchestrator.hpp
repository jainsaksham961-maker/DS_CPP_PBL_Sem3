#ifndef MISSION_ORCHESTRATOR_HPP
#define MISSION_ORCHESTRATOR_HPP

#include "AirwayGraph.hpp"
#include "FleetManager.hpp"
#include "OrderScheduler.hpp"
#include "RoutingEngine.hpp"
#include <iostream>
#include <iomanip>

class MissionOrchestrator {
private:
    AirwayGraph& graph;
    FleetManager& fleet;
    OrderScheduler& scheduler;
    WindField wind;

public:
    MissionOrchestrator(AirwayGraph& g, FleetManager& f, OrderScheduler& s, WindField w)
        : graph(g), fleet(f), scheduler(s), wind(w) {}

    void executeDispatchLoop(double sim_time) {
        std::cout << "\n=======================================================\n";
        std::cout << "  DISPATCH CYCLE EXECUTION (Time: " << sim_time << "s)\n";
        std::cout << "=======================================================\n";

        while (scheduler.hasPendingOrders()) {
            DeliveryOrder order = scheduler.popNextOrder();
            std::cout << "\n[Scheduler] Extracted Order #" << order.order_id 
                      << " | Target Node: " << order.target_node_id
                      << " | Payload: " << order.payload_kg << " kg"
                      << " | Deadline: " << order.deadline_sec << "s\n";

            std::string drone_id = fleet.findOptimalDrone(order.payload_kg);
            if (drone_id.empty()) {
                std::cout << "  [!] REJECTED: No idle drone available with sufficient capacity.\n";
                continue;
            }

            Drone* assigned_drone = fleet.getDrone(drone_id);
            int start_node = assigned_drone->current_node_id;

            PathResult forward_leg = RoutingEngine::computeAStarRoute(graph, start_node, order.target_node_id,
                                                                      assigned_drone->cruise_airspeed,
                                                                      order.payload_kg, wind);
            if (!forward_leg.reachable) {
                std::cout << "  [!] REJECTED: Target node unreachable in airway graph.\n";
                continue;
            }

            PathResult return_leg = RoutingEngine::computeAStarRoute(graph, order.target_node_id, 0,
                                                                     assigned_drone->cruise_airspeed,
                                                                     0.0, wind);

            double total_energy = forward_leg.total_energy_wh + return_leg.total_energy_wh;
            double eta = sim_time + forward_leg.total_time_sec;

            std::cout << "  [Plan] Assigned: " << drone_id << "\n";
            std::cout << "  [Plan] Flight Waypoints: ";
            for (size_t i = 0; i < forward_leg.path.size(); ++i) {
                std::cout << graph.nodes[forward_leg.path[i]].label << (i + 1 < forward_leg.path.size() ? " -> " : "");
            }
            std::cout << "\n";
            std::cout << "  [Plan] ETA: " << eta << "s (Deadline: " << order.deadline_sec << "s)\n";
            std::cout << "  [Plan] Required Energy: " << total_energy << " Wh (Available: " << assigned_drone->current_battery_wh << " Wh)\n";

            if (eta > order.deadline_sec) {
                std::cout << "  [X] INFEASIBLE: Violates deadline window.\n";
                continue;
            }
            if (total_energy > (assigned_drone->current_battery_wh * 0.85)) {
                std::cout << "  [X] INFEASIBLE: Insufficient battery reserves for safe return leg.\n";
                continue;
            }

            fleet.updateTelemetry(drone_id, order.target_node_id, forward_leg.total_energy_wh, DroneStatus::IN_TRANSIT);
            std::cout << "  [✓] MISSION DISPATCHED: Drone " << drone_id << " in transit. Remaining SoC: "
                      << std::fixed << std::setprecision(1) << assigned_drone->getSoC() << "%\n";
        }
    }
};

#endif
