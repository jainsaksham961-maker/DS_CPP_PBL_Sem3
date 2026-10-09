#ifndef TYPES_HPP
#define TYPES_HPP

#include <cmath>
#include <string>

// Terminal ANSI Color Codes for Phase-II Visual Graph 
#define COLOR_RESET   "\033[0m"
#define COLOR_RED     "\033[1;31m"
#define COLOR_GREEN   "\033[1;32m"
#define COLOR_YELLOW  "\033[1;33m"
#define COLOR_CYAN    "\033[1;36m"
#define COLOR_BOLD    "\033[1m"

// 3D Airway Vector
struct Vector3D {
    double x{0.0}, y{0.0}, z{0.0};

    Vector3D operator+(const Vector3D& o) const { return {x + o.x, y + o.y, z + o.z}; }
    Vector3D operator-(const Vector3D& o) const { return {x - o.x, y - o.y, z - o.z}; }
    Vector3D operator*(double s) const { return {x * s, y * s, z * s}; }

    double norm() const {
        return std::sqrt(x * x + y * y + z * z);
    }

    Vector3D normalized() const {
        double n = norm();
        if (n < 1e-9) return {0.0, 0.0, 0.0};
        return {x / n, y / n, z / n};
    }

    static double distance(const Vector3D& a, const Vector3D& b) {
        return (a - b).norm();
    }
};

struct WindField {
    Vector3D velocity; // m/s
};

enum class DroneStatus { IDLE, IN_TRANSIT, CHARGING, MAINTENANCE };

// Corridor condition state
enum class CorridorStatus {
    OPEN_AVAILABLE,
    RUNNING_MISSION,  // Color: GREEN
    BLOCKED_RESTRICTED // Color: RED (No-Fly Zone / Weather block)
};

#endif // TYPES_HPP
