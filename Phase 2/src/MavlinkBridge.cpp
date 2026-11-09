#include "MavlinkBridge.hpp"
#include <iostream>
#include <iomanip>
#include <cstring>

MavlinkBridge::MavlinkBridge(uint8_t sysid, uint8_t compid)
    : system_id(sysid), component_id(compid), packet_sequence(0) {}

uint16_t MavlinkBridge::calculateCrc(const uint8_t* buffer, size_t length) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < length; ++i) {
        uint8_t tmp = buffer[i] ^ (uint8_t)(crc & 0xFF);
        tmp ^= (tmp << 4);
        crc = (crc >> 8) ^ ((uint16_t)tmp << 8) ^ ((uint16_t)tmp << 3) ^ ((uint16_t)tmp >> 4);
    }
    return crc;
}

std::vector<uint8_t> MavlinkBridge::packHeartbeat(MavAutopilotState state) {
    MavlinkHeader hdr;
    hdr.magic = 0xFD; // MAVLink v2 packet start marker
    hdr.len = sizeof(MavlinkHeartbeatPayload);
    hdr.seq = packet_sequence++;
    hdr.sysid = system_id;
    hdr.compid = component_id;
    hdr.msgid = 0; // MAVLINK_MSG_ID_HEARTBEAT

    MavlinkHeartbeatPayload payload;
    payload.type = 2; // MAV_TYPE_QUADROTOR
    payload.autopilot = 3; // MAV_AUTOPILOT_ARDUPILOTMEGA / PX4
    payload.base_mode = 209; // Guided + Armed
    payload.system_status = static_cast<uint8_t>(state);
    payload.mavlink_version = 3;

    size_t total_size = sizeof(MavlinkHeader) + sizeof(MavlinkHeartbeatPayload) + 2; // +2 for CRC-16
    std::vector<uint8_t> frame(total_size);

    std::memcpy(frame.data(), &hdr, sizeof(MavlinkHeader));
    std::memcpy(frame.data() + sizeof(MavlinkHeader), &payload, sizeof(MavlinkHeartbeatPayload));

    uint16_t crc = calculateCrc(frame.data() + 1, sizeof(MavlinkHeader) - 1 + sizeof(MavlinkHeartbeatPayload));
    frame[total_size - 2] = static_cast<uint8_t>(crc & 0xFF);
    frame[total_size - 1] = static_cast<uint8_t>((crc >> 8) & 0xFF);

    return frame;
}

std::vector<uint8_t> MavlinkBridge::packGlobalPosition(const Vector3D& pos, double vx, double vy, double vz) {
    MavlinkHeader hdr;
    hdr.magic = 0xFD;
    hdr.len = sizeof(MavlinkGlobalPositionPayload);
    hdr.seq = packet_sequence++;
    hdr.sysid = system_id;
    hdr.compid = component_id;
    hdr.msgid = 33; // MAVLINK_MSG_ID_GLOBAL_POSITION_INT

    MavlinkGlobalPositionPayload payload;
    payload.time_boot_ms = 120000;
    // Map cartesian meters to simulated latitude/longitude degrees and millimetric altitudes
    payload.lat = static_cast<int32_t>(30.3165 * 1e7 + pos.y * 100); 
    payload.lon = static_cast<int32_t>(78.0322 * 1e7 + pos.x * 100);
    payload.alt = static_cast<int32_t>(pos.z * 1000.0);
    payload.relative_alt = static_cast<int32_t>(pos.z * 1000.0);
    payload.vx = static_cast<int16_t>(vx * 100.0);
    payload.vy = static_cast<int16_t>(vy * 100.0);
    payload.vz = static_cast<int16_t>(vz * 100.0);
    payload.hdg = 4500; // 45.00 deg heading

    size_t total_size = sizeof(MavlinkHeader) + sizeof(MavlinkGlobalPositionPayload) + 2;
    std::vector<uint8_t> frame(total_size);

    std::memcpy(frame.data(), &hdr, sizeof(MavlinkHeader));
    std::memcpy(frame.data() + sizeof(MavlinkHeader), &payload, sizeof(MavlinkGlobalPositionPayload));

    uint16_t crc = calculateCrc(frame.data() + 1, sizeof(MavlinkHeader) - 1 + sizeof(MavlinkGlobalPositionPayload));
    frame[total_size - 2] = static_cast<uint8_t>(crc & 0xFF);
    frame[total_size - 1] = static_cast<uint8_t>((crc >> 8) & 0xFF);

    return frame;
}

bool MavlinkBridge::parseIncomingStream(const std::vector<uint8_t>& buffer, DroneTelemetryAgent& telemetry) {
    if (buffer.size() < sizeof(MavlinkHeader) + 2) return false;
    if (buffer[0] != 0xFD) return false;

    MavlinkHeader hdr;
    std::memcpy(&hdr, buffer.data(), sizeof(MavlinkHeader));

    if (hdr.msgid == 33 && buffer.size() >= sizeof(MavlinkHeader) + sizeof(MavlinkGlobalPositionPayload) + 2) {
        MavlinkGlobalPositionPayload payload;
        std::memcpy(&payload, buffer.data() + sizeof(MavlinkHeader), sizeof(MavlinkGlobalPositionPayload));
        telemetry.current_position.z = payload.relative_alt / 1000.0;
        return true;
    }
    return false;
}

void MavlinkBridge::printPacketTelemetry(const std::vector<uint8_t>& packet) const {
    std::cout << "[SITL/MAVLink v2] Frame Emitted (" << packet.size() << " bytes) -> Hex: ";
    for (size_t i = 0; i < std::min(packet.size(), size_t(14)); ++i) {
        std::cout << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(packet[i]) << " ";
    }
    std::cout << std::dec << "... [CRC: OK]\n";
}
