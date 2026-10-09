#include "RoutingEngine.hpp"
#include <queue>
#include <unordered_map>
#include <limits>
#include <algorithm>

PathResult RoutingEngine::computeAStarRoute(const AirwayGraph& graph, int start, int goal, 
                                            double airspeed, double payload, const WindField& wind) {
    PathResult res;
    if (graph.nodes.find(start) == graph.nodes.end() || graph.nodes.find(goal) == graph.nodes.end()) {
        return res;
    }

    using PQElement = std::pair<double, int>;
    std::priority_queue<PQElement, std::vector<PQElement>, std::greater<PQElement>> open_set;

    std::unordered_map<int, double> g_score;
    std::unordered_map<int, double> f_score;
    std::unordered_map<int, int> parent;
    std::unordered_map<int, double> leg_energy;
    std::unordered_map<int, double> leg_time;

    for (const auto& entry : graph.nodes) {
        int node_id = entry.first;
        g_score[node_id] = std::numeric_limits<double>::infinity();
        f_score[node_id] = std::numeric_limits<double>::infinity();
    }

    auto heuristic = [&](int u, int target) {
        double dist = Vector3D::distance(graph.nodes.at(u).coords, graph.nodes.at(target).coords);
        double max_speed = airspeed + wind.velocity.norm();
        return (dist / max_speed) * (220.0 / 3600.0);
    };

    g_score[start] = 0.0;
    f_score[start] = heuristic(start, goal);
    open_set.push({f_score[start], start});

    while (!open_set.empty()) {
        PQElement top = open_set.top();
        double current_f = top.first;
        int curr = top.second;
        open_set.pop();

        if (curr == goal) {
            res.reachable = true;
            int curr_track = goal;
            while (curr_track != start) {
                res.path.push_back(curr_track);
                res.total_energy_wh += leg_energy[curr_track];
                res.total_time_sec += leg_time[curr_track];
                curr_track = parent[curr_track];
            }
            res.path.push_back(start);
            std::reverse(res.path.begin(), res.path.end());
            return res;
        }

        if (current_f > f_score[curr]) continue;

        auto adj_it = graph.adj_list.find(curr);
        if (adj_it == graph.adj_list.end()) continue;

        for (const auto& edge : adj_it->second) {
            // CRITICAL CHECK: Ignore blocked corridors entirely
            if (edge.status == CorridorStatus::BLOCKED_RESTRICTED) {
                continue;
            }

            int nxt = edge.to_node;
            double time_cost = 0.0, energy_cost = 0.0;
            graph.computeFlightCost(curr, nxt, airspeed, payload, wind, time_cost, energy_cost);

            double tentative_g = g_score[curr] + energy_cost;
            if (tentative_g < g_score[nxt]) {
                parent[nxt] = curr;
                g_score[nxt] = tentative_g;
                leg_energy[nxt] = energy_cost;
                leg_time[nxt] = time_cost;
                f_score[nxt] = tentative_g + heuristic(nxt, goal);
                open_set.push({f_score[nxt], nxt});
            }
        }
    }
    return res;
}
