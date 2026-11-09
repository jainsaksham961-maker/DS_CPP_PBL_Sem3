#pragma once

#include <iostream>
#include <string>
#include <vector>
#include <cmath>
#include <cstdint>

// 1. Scoped Enumerations (1-byte memory footprint)
enum class CorridorStatus : uint8_t {
    OPEN_AVAILABLE,
    BLOCKED_RESTRICTED
};

enum class MavAutopilotState : uint8_t {
    MAV_STATE_BOOT = 0,
    MAV_STATE_STANDBY = 3,
    MAV_STATE_ACTIVE = 4,
    MAV_STATE_EMERGENCY = 6
};

// 2. 3D Spatial Vector & Metric Geometry
struct Vector3D {
    double x{0.0};
    double y{0.0};
    double z{0.0};

    double distanceTo(const Vector3D& other) const {
        double dx = x - other.x;
        double dy = y - other.y;
        double dz = z - other.z;
        return std::sqrt(dx * dx + dy * dy + dz * dz);
    }
};

// 3. Environmental Wind Vectors
struct WindField {
    double velocity_ms{0.0};
    double heading_deg{0.0};
};

// 4. MAVLink / SITL Autopilot Telemetry Packets (Phase 3 Roadmap)
#pragma pack(push, 1)
struct MavlinkHeader {
    uint8_t magic{0xFD};      // MAVLink v2 magic byte
    uint8_t len{0};           // Payload length
    uint8_t incompat_flags{0};
    uint8_t compat_flags{0};
    uint8_t seq{0};           // Packet sequence
    uint8_t sysid{1};         // Vehicle System ID
    uint8_t compid{1};        // Autopilot Component ID
    uint32_t msgid{0};        // 24-bit Message ID
};

struct MavlinkHeartbeatPayload {
    uint32_t custom_mode{0};
    uint8_t type{2};          // MAV_TYPE_QUADROTOR
    uint8_t autopilot{3};     // MAV_AUTOPILOT_ARDUPILOTMEGA / PX4
    uint8_t base_mode{209};   // Armed + Guided mode
    uint8_t system_status{4}; // MAV_STATE_ACTIVE
    uint8_t mavlink_version{3};
};

struct MavlinkGlobalPositionPayload {
    uint32_t time_boot_ms{0};
    int32_t lat{0};           // degE7
    int32_t lon{0};           // degE7
    int32_t alt{0};           // mm (AMSL)
    int32_t relative_alt{0};  // mm (above home)
    int16_t vx{0};            // cm/s ground speed X
    int16_t vy{0};            // cm/s ground speed Y
    int16_t vz{0};            // cm/s ground speed Z
    uint16_t hdg{0};          // cdeg heading
};
#pragma pack(pop)

// 5. Waypoints and Corridors
struct Waypoint {
    int id{0};
    std::string label;
    Vector3D coords;
    bool is_depot{false};
};

struct Edge {
    int v{0};
    CorridorStatus status{CorridorStatus::OPEN_AVAILABLE};
};

// 6. Path Result Container
struct PathResult {
    bool success{false};
    std::vector<int> path;
    double total_distance{0.0};
    double total_time{0.0};
    double total_energy{0.0};
};

// 7. Live Drone Agent for SITL Tracking & Dynamic Rerouting
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

// 8. Makespan Benchmarking Metric Struct (Phase 3 Roadmap)
struct DeliveryOrder {
    int order_id{0};
    int source_node{0};
    int destination_node{0};
    double payload_kg{2.0};
    double release_time_sec{0.0};
    double sla_deadline_sec{0.0};
    double flight_duration_sec{0.0};
};

struct BenchmarkMetrics {
    std::string algorithm_name;
    double total_makespan_sec{0.0};
    double avg_turnaround_time_sec{0.0};
    int sla_violations{0};
};
