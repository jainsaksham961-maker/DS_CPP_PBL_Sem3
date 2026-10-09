#include "AirwayGraph.hpp"
#include <iostream>
#include <iomanip>
#include <cmath>
#include <fstream>
#include <algorithm>

void AirwayGraph::addWaypoint(int id, const std::string& label, const Vector3D& coords, bool is_depot) {
    nodes[id] = {id, label, coords, is_depot};
}

void AirwayGraph::addCorridor(int u, int v, CorridorStatus initial_status) {
    if (nodes.find(u) == nodes.end() || nodes.find(v) == nodes.end()) return;
    double dist = Vector3D::distance(nodes[u].coords, nodes[v].coords);
    adj_list[u].push_back({v, dist, initial_status});
    adj_list[v].push_back({u, dist, initial_status});
}

void AirwayGraph::setCorridorStatus(int u, int v, CorridorStatus new_status) {
    auto update_edge = [&](int from, int to) {
        if (adj_list.find(from) != adj_list.end()) {
            for (auto& edge : adj_list[from]) {
                if (edge.to_node == to) {
                    edge.status = new_status;
                }
            }
        }
    };
    update_edge(u, v);
    update_edge(v, u);
}

double AirwayGraph::computeFlightCost(int u, int v, double airspeed, double payload_mass, 
                                     const WindField& wind, double& out_time, double& out_energy) const {
    Vector3D p1 = nodes.at(u).coords;
    Vector3D p2 = nodes.at(v).coords;
    Vector3D direction = (p2 - p1).normalized();
    double dist = Vector3D::distance(p1, p2);

    Vector3D v_ground = (direction * airspeed) + wind.velocity;
    double ground_speed = std::max(1.0, v_ground.norm());

    out_time = dist / ground_speed;
    double total_mass = 4.5 + payload_mass;
    double power_w = 220.0 + (28.0 * total_mass) + (0.35 * std::pow(airspeed, 3));

    out_energy = (power_w * out_time) / 3600.0;
    return out_energy;
}

// ----------------------------------------------------------------------------
// 1. 2D Top-Down Airspace Radar Grid
// ----------------------------------------------------------------------------
void AirwayGraph::renderAsciiRadar(int width, int height) const {
    if (nodes.empty()) return;

    double min_x = 1e9, max_x = -1e9, min_y = 1e9, max_y = -1e9;
    for (const auto& [id, node] : nodes) {
        min_x = std::min(min_x, node.coords.x);
        max_x = std::max(max_x, node.coords.x);
        min_y = std::min(min_y, node.coords.y);
        max_y = std::max(max_y, node.coords.y);
    }

    if (max_x - min_x < 1.0) max_x += 10.0;
    if (max_y - min_y < 1.0) max_y += 10.0;

    std::vector<std::string> grid(height, std::string(width, '.'));

    auto to_grid = [&](double x, double y) -> std::pair<int, int> {
        int gx = static_cast<int>(((x - min_x) / (max_x - min_x)) * (width - 4)) + 2;
        int gy = static_cast<int>(((y - min_y) / (max_y - min_y)) * (height - 3)) + 1;
        gy = height - 1 - gy; // Invert Y for screen projection
        return {std::clamp(gx, 0, width - 1), std::clamp(gy, 0, height - 1)};
    };

    // Draw lines between nodes
    for (const auto& [u, edges] : adj_list) {
        for (const auto& edge : edges) {
            if (u > edge.to_node) continue;
            auto [x1, y1] = to_grid(nodes.at(u).coords.x, nodes.at(u).coords.y);
            auto [x2, y2] = to_grid(nodes.at(edge.to_node).coords.x, nodes.at(edge.to_node).coords.y);

            char fill_char = '-';
            if (edge.status == CorridorStatus::RUNNING_MISSION) fill_char = '=';
            else if (edge.status == CorridorStatus::BLOCKED_RESTRICTED) fill_char = 'x';

            int steps = std::max(std::abs(x2 - x1), std::abs(y2 - y1));
            for (int s = 1; s < steps; ++s) {
                int px = x1 + (x2 - x1) * s / steps;
                int py = y1 + (y2 - y1) * s / steps;
                if (grid[py][px] == '.') grid[py][px] = fill_char;
            }
        }
    }

    std::cout << "\n" << COLOR_CYAN << "╔══════════════════════════════════════════════════════════════════════════╗\n";
    std::cout << "║            AUTONOMOUS AIRSPACE RADAR DISPLAY (X-Y PROJECTION)            ║\n";
    std::cout << "╚══════════════════════════════════════════════════════════════════════════╝" << COLOR_RESET << "\n";
    std::cout << "  LEGEND: "
              << COLOR_GREEN << "==== Active Mission" << COLOR_RESET << "  |  "
              << COLOR_RED   << "xxxx Blocked / Hazard" << COLOR_RESET << "  |  "
              << "---- Open Airway\n\n";

    for (int y = 0; y < height; ++y) {
        std::cout << "  │ ";
        for (int x = 0; x < width; ++x) {
            // Check if a node is at (x, y)
            bool is_node = false;
            for (const auto& [id, node] : nodes) {
                auto [nx, ny] = to_grid(node.coords.x, node.coords.y);
                if (nx == x && ny == y) {
                    if (node.is_depot) std::cout << COLOR_BOLD << COLOR_YELLOW << "[D]" << COLOR_RESET;
                    else std::cout << COLOR_BOLD << "[" << id << "]" << COLOR_RESET;
                    is_node = true;
                    break;
                }
            }
            if (!is_node) {
                char c = grid[y][x];
                if (c == '=') std::cout << COLOR_GREEN << "=" << COLOR_RESET;
                else if (c == 'x') std::cout << COLOR_RED << "x" << COLOR_RESET;
                else if (c == '.') std::cout << "\033[90m·\033[0m";
                else std::cout << c;
            }
        }
        std::cout << " │\n";
    }
    std::cout << "  └" << std::string(width + 2, '─') << "┘\n";
}

// ----------------------------------------------------------------------------
// 2. Tabular Flight Corridor Matrix
// ----------------------------------------------------------------------------
void AirwayGraph::renderCorridorStatusMatrix() const {
    std::cout << "\n" << COLOR_CYAN << "┌────────────────────────────────────────────────────────────────────────┐\n";
    std::cout << "│                       CORRIDOR TELEMETRY STATUS                        │\n";
    std::cout << "└────────────────────────────────────────────────────────────────────────┘" << COLOR_RESET << "\n";
    std::cout << std::left << std::setw(8)  << " ID" 
              << std::setw(28) << "Corridor Vector" 
              << std::setw(14) << "Distance" 
              << std::setw(20) << "Current Status" << "\n";
    std::cout << "────────────────────────────────────────────────────────────────────────\n";

    int index = 1;
    for (const auto& [u, edges] : adj_list) {
        for (const auto& edge : edges) {
            if (u > edge.to_node) continue;
            std::string route_str = nodes.at(u).label + " <-> " + nodes.at(edge.to_node).label;
            std::cout << " [" << std::setw(3) << index++ << "] " 
                      << std::left << std::setw(28) << route_str
                      << std::setw(12) << (std::to_string((int)edge.base_distance) + " m");

            if (edge.status == CorridorStatus::RUNNING_MISSION) {
                std::cout << COLOR_GREEN << "● RUNNING MISSION" << COLOR_RESET << "\n";
            } else if (edge.status == CorridorStatus::BLOCKED_RESTRICTED) {
                std::cout << COLOR_RED << "✖ BLOCKED (NO-FLY)" << COLOR_RESET << "\n";
            } else {
                std::cout << "\033[37m○ OPEN / IDLE\033[0m\n";
            }
        }
    }
    std::cout << "────────────────────────────────────────────────────────────────────────\n";
}

// ----------------------------------------------------------------------------
// 3. Export to Interactive HTML/SVG for Browser Demonstrations
// ----------------------------------------------------------------------------
void AirwayGraph::exportHtmlVisualizer(const std::string& filename) const {
    std::ofstream file(filename);
    if (!file.is_open()) return;

    double min_x = 1e9, max_x = -1e9, min_y = 1e9, max_y = -1e9;
    for (const auto& [id, node] : nodes) {
        min_x = std::min(min_x, node.coords.x);
        max_x = std::max(max_x, node.coords.x);
        min_y = std::min(min_y, node.coords.y);
        max_y = std::max(max_y, node.coords.y);
    }
    if (max_x - min_x < 1.0) max_x += 10.0;
    if (max_y - min_y < 1.0) max_y += 10.0;

    auto svg_x = [&](double x) { return ((x - min_x) / (max_x - min_x)) * 600.0 + 100.0; };
    auto svg_y = [&](double y) { return 450.0 - (((y - min_y) / (max_y - min_y)) * 320.0); };

    file << "<!DOCTYPE html>\n<html>\n<head>\n"
         << "<title>Autonomous Drone Airspace Radar</title>\n"
         << "<style>\n"
         << "  body { background: #0c1017; color: #e6edf3; font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, sans-serif; text-align: center; margin: 0; padding: 20px; }\n"
         << "  .panel { background: #161b22; border: 1px solid #30363d; border-radius: 12px; width: 850px; margin: 0 auto; padding: 20px; box-shadow: 0 8px 24px rgba(0,0,0,0.5); }\n"
         << "  h2 { color: #58a6ff; margin-top: 0; letter-spacing: 1px; }\n"
         << "  svg { background: #0d1117; border: 1px solid #21262d; border-radius: 8px; }\n"
         << "  .legend { display: flex; justify-content: center; gap: 20px; margin-top: 15px; font-size: 14px; }\n"
         << "  .legend-item { display: flex; align-items: center; gap: 8px; }\n"
         << "  .dot { width: 14px; height: 14px; border-radius: 50%; display: inline-block; }\n"
         << "</style>\n</head>\n<body>\n"
         << "<div class='panel'>\n"
         << "  <h2>Autonomous Drone Fleet Airspace Visualizer</h2>\n"
         << "  <p style='color: #8b949e;'>Phase-II Project Evaluation | Team ID: DSCPP-III-2026-T154</p>\n"
         << "  <svg width='800' height='500'>\n";

    // Draw Corridors
    for (const auto& [u, edges] : adj_list) {
        for (const auto& edge : edges) {
            if (u > edge.to_node) continue;
            double x1 = svg_x(nodes.at(u).coords.x);
            double y1 = svg_y(nodes.at(u).coords.y);
            double x2 = svg_x(nodes.at(edge.to_node).coords.x);
            double y2 = svg_y(nodes.at(edge.to_node).coords.y);

            std::string stroke = "#30363d";
            std::string stroke_width = "2";
            std::string stroke_dash = "4";

            if (edge.status == CorridorStatus::RUNNING_MISSION) {
                stroke = "#2ea043"; // Green
                stroke_width = "4";
                stroke_dash = "0";
            } else if (edge.status == CorridorStatus::BLOCKED_RESTRICTED) {
                stroke = "#da3633"; // Red
                stroke_width = "3";
                stroke_dash = "6";
            }

            file << "    <line x1='" << x1 << "' y1='" << y1 << "' x2='" << x2 << "' y2='" << y2 
                 << "' stroke='" << stroke << "' stroke-width='" << stroke_width << "' stroke-dasharray='" << stroke_dash << "' />\n";
        }
    }

    // Draw Nodes
    for (const auto& [id, node] : nodes) {
        double nx = svg_x(node.coords.x);
        double ny = svg_y(node.coords.y);
        std::string fill = node.is_depot ? "#d29922" : "#58a6ff";

        file << "    <circle cx='" << nx << "' cy='" << ny << "' r='14' fill='" << fill << "' stroke='#ffffff' stroke-width='2' />\n"
             << "    <text x='" << nx << "' y='" << (ny + 5) << "' fill='#0d1117' font-weight='bold' font-size='11' text-anchor='middle'>" << id << "</text>\n"
             << "    <text x='" << nx << "' y='" << (ny - 20) << "' fill='#e6edf3' font-size='12' font-weight='500' text-anchor='middle'>" << node.label << "</text>\n";
    }

    file << "  </svg>\n"
         << "  <div class='legend'>\n"
         << "    <div class='legend-item'><span class='dot' style='background: #2ea043;'></span> Active Mission Route</div>\n"
         << "    <div class='legend-item'><span class='dot' style='background: #da3633;'></span> Blocked / No-Fly Corridor</div>\n"
         << "    <div class='legend-item'><span class='dot' style='background: #30363d;'></span> Available Open Airway</div>\n"
         << "    <div class='legend-item'><span class='dot' style='background: #d29922;'></span> Central Depot</div>\n"
         << "  </div>\n"
         << "</div>\n</body>\n</html>\n";

    file.close();
    std::cout << "\n" << COLOR_GREEN << "  [✓] Interactive Map exported to 'airspace_map.html' (Double-click to view in browser)" << COLOR_RESET << "\n";
}
