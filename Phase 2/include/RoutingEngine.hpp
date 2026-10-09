#ifndef ROUTING_ENGINE_HPP
#define ROUTING_ENGINE_HPP

#include "AirwayGraph.hpp"
#include <vector>
 
struct PathResult {
    bool reachable{false};
    std::vector<int> path;
    double total_energy_wh{0.0};
    double total_time_sec{0.0};
};

class RoutingEngine {
public:
    static PathResult computeAStarRoute(const AirwayGraph& graph, int start, int goal, 
                                        double airspeed, double payload, const WindField& wind);
};

#endif // ROUTING_ENGINE_HPP
