#ifndef FLEET_MANAGER_HPP
#define FLEET_MANAGER_HPP

#include "Types.hpp"
#include <unordered_map>
#include <string>
#include <algorithm>

struct Drone {
    std::string drone_id;
    double max_payload_kg;
    double battery_capacity_wh;
    double current_battery_wh;
    double cruise_airspeed;
    int current_node_id;
    DroneStatus status{DroneStatus::IDLE};

    double getSoC() const {
        return (current_battery_wh / battery_capacity_wh) * 100.0;
    }
};

class FleetManager {
private:
    std::unordered_map<std::string, Drone> fleet_registry;

public:
    void registerDrone(const Drone& drone) {
        fleet_registry[drone.drone_id] = drone;
    }

    Drone* getDrone(const std::string& id) {
        auto it = fleet_registry.find(id);
        if (it != fleet_registry.end()) return &(it->second);
        return nullptr;
    }

    std::string findOptimalDrone(double order_weight) {
        for (auto& entry : fleet_registry) {
            Drone& drone = entry.second;
            if (drone.status == DroneStatus::IDLE && 
                drone.max_payload_kg >= order_weight && 
                drone.getSoC() >= 30.0) {
                return drone.drone_id;
            }
        }
        return "";
    }

    void updateTelemetry(const std::string& id, int node, double energy_spent, DroneStatus st) {
        auto it = fleet_registry.find(id);
        if (it != fleet_registry.end()) {
            it->second.current_node_id = node;
            it->second.current_battery_wh = std::max(0.0, it->second.current_battery_wh - energy_spent);
            it->second.status = st;
        }
    }
};

#endif
