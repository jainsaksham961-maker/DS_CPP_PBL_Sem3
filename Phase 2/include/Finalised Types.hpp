#pragma once

#include <iostream>
#include <string>
#include <vector>
#include <cmath>
#include <cstdint>
#include <algorithm>

// ANSI COLOR Constants
const std::string COLOR_RESET  = "\033[0m";
const std::string COLOR_RED    = "\033[1;31m";
const std::string COLOR_GREEN  = "\033[1;32m";
const std::string COLOR_YELLOW = "\033[1;33m";
const std::string COLOR_CYAN   = "\033[1;36m";

template <typename T>
inline T clampVal(T val, T lo, T hi) {
    return (val < lo) ? lo : (val > hi) ? hi : val;
}

enum class CorridorStatus : uint8_t {
    OPEN_AVAILABLE,
    RUNNING_MISSION,
    BLOCKED_RESTRICTED
};

enum class MavAutopilotState : uint8_t {
    MAV_STATE_BOOT = 0,
    MAV_STATE_STANDBY = 3,
    MAV_STATE_ACTIVE = 4,
    MAV_STATE_EMERGENCY = 6
};

struct Vector3D {
    double x{0.0};
    double y{0.0};
    double z{0.0};

    Vector3D operator+(const Vector3D& o) const { return {x + o.x, y + o.y, z + o.z}; }
    Vector3D operator-(const Vector3D& o) const { return {x - o.x, y - o.y, z - o.z}; }
    Vector3D operator*(double s) const { return {x * s, y * s, z * s}; }

    double norm() const { return std::sqrt(x * x + y * y + z * z); }

    Vector3D normalized() const {
        double n = norm();
        return (n > 1e-9) ? Vector3D{x / n, y / n, z / n} : Vector3D{0.0, 0.0, 0.0};
    }

    double distanceTo(const Vector3D& other) const {
        return (*this - other).norm();
    }

    static double distance(const Vector3D& a, const Vector3D& b) {
        return (a - b).norm();
    }
};

struct WindField {
    Vector3D velocity{0.0, 0.0, 0.0};
    double velocity_ms{0.0};
    double heading_deg{0.0};
};

#pragma pack(push, 1)
struct MavlinkHeader {
    uint8_t magic{0xFD};
    uint8_t len{0};
    uint8_t incompat_flags{0};
    uint8_t compat_flags{0};
    uint8_t seq{0};
    uint8_t sysid{1};
    uint8_t compid{1};
    uint32_t msgid{0};
};

struct MavlinkHeartbeatPayload {
    uint32_t custom_mode{0};
    uint8_t type{2};
    uint8_t autopilot{3};
    uint8_t base_mode{209};
    uint8_t system_status{4};
    uint8_t mavlink_version{3};
};

struct MavlinkGlobalPositionPayload {
    uint32_t time_boot_ms{0};
    int32_t lat{0};
    int32_t lon{0};
    int32_t alt{0};
    int32_t relative_alt{0};
    int16_t vx{0};
    int16_t vy{0};
    int16_t vz{0};
    uint16_t hdg{0};
};
#pragma pack(pop)

struct Waypoint {
    int id{0};
    std::string label;
    Vector3D coords;
    bool is_depot{false};
};

struct Edge {
    int to_node{0};
    double base_distance{0.0};
    CorridorStatus status{CorridorStatus::OPEN_AVAILABLE};
};

struct PathResult {
    bool success{false};
    bool reachable{false};
    std::vector<int> path;
    double total_distance{0.0};
    double total_time{0.0};
    double total_energy{0.0};
};

struct Drone {
    std::string id;
    int current_node_id{0};
    double max_payload_kg{5.0};
    double cruise_airspeed{18.0};
    double battery_soc{1.0};
    bool is_available{true};
};

struct DroneTelemetryAgent {
    uint8_t sysid{1};
    std::string callsign{"UAV-ALPHA-01"};
    int current_waypoint{0};
    int target_waypoint{3};
    Vector3D current_position{0.0, 0.0, 15.0};
    double airspeed{18.0};
    double battery_soc{0.92};
    bool in_flight{false};
    std::vector<int> active_route;
    size_t route_step{0};
};

struct DeliveryOrder {
    int order_id{0};
    int source_node{0};
    int destination_node{0};
    double payload_kg{2.0};
    double release_time_sec{0.0};
    double sla_deadline_sec{0.0};
    double flight_duration_sec{0.0};
    int target_node_id{0};
    double deadline_sec{0.0};

    bool operator>(const DeliveryOrder& other) const {
        return sla_deadline_sec > other.sla_deadline_sec;
    }
};

struct BenchmarkMetrics {
    std::string algorithm_name;
    double total_makespan_sec{0.0};
    double avg_turnaround_time_sec{0.0};
    int sla_violations{0};
};
