#ifndef AIRWAY_GRAPH_HPP
#define AIRWAY_GRAPH_HPP

#include "Types.hpp"
#include <vector>
#include <unordered_map>
#include <string>

struct WaypointNode { 
    int id;
    std::string label;
    Vector3D coords;
    bool is_depot{false};
};

struct Edge {
    int to_node;
    double base_distance;
    CorridorStatus status{CorridorStatus::OPEN_AVAILABLE};
};

class AirwayGraph {
public:
    std::unordered_map<int, WaypointNode> nodes;
    std::unordered_map<int, std::vector<Edge>> adj_list;

    void addWaypoint(int id, const std::string& label, const Vector3D& coords, bool is_depot = false);
    void addCorridor(int u, int v, CorridorStatus initial_status = CorridorStatus::OPEN_AVAILABLE);
    void setCorridorStatus(int u, int v, CorridorStatus new_status);
    
    double computeFlightCost(int u, int v, double airspeed, double payload_mass, 
                             const WindField& wind, double& out_time, double& out_energy) const;

    // Advanced Visualizers
    void renderAsciiRadar(int width = 50, int height = 18) const;
    void renderCorridorStatusMatrix() const;
    void exportHtmlVisualizer(const std::string& filename = "airspace_map.html") const;
};

#endif // AIRWAY_GRAPH_HPP
