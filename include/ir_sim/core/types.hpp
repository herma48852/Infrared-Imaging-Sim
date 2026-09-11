#pragma once

#include <cmath>
#include <cstdint>
#include <string>
#include <string_view>
#include <vector>

namespace ir_sim::core {

// 3D Vector with common geometric operations
struct Vec3 {
    float x{0.0f};
    float y{0.0f};
    float z{0.0f};

    constexpr Vec3() = default;
    constexpr Vec3(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}

    [[nodiscard]] constexpr Vec3 operator+(const Vec3& o) const { return {x + o.x, y + o.y, z + o.z}; }
    [[nodiscard]] constexpr Vec3 operator-(const Vec3& o) const { return {x - o.x, y - o.y, z - o.z}; }
    [[nodiscard]] constexpr Vec3 operator*(float s) const { return {x * s, y * s, z * s}; }
    [[nodiscard]] constexpr Vec3 operator/(float s) const { return {x / s, y / s, z / s}; }

    constexpr Vec3& operator+=(const Vec3& o) { x += o.x; y += o.y; z += o.z; return *this; }
    constexpr Vec3& operator-=(const Vec3& o) { x -= o.x; y -= o.y; z -= o.z; return *this; }
    constexpr Vec3& operator*=(float s) { x *= s; y *= s; z *= s; return *this; }

    [[nodiscard]] constexpr float dot(const Vec3& o) const { return x * o.x + y * o.y + z * o.z; }
    
    [[nodiscard]] constexpr Vec3 cross(const Vec3& o) const {
        return {
            y * o.z - z * o.y,
            z * o.x - x * o.z,
            x * o.y - y * o.x
        };
    }

    [[nodiscard]] float norm_sq() const { return x * x + y * y + z * z; }
    [[nodiscard]] float norm() const { return std::sqrt(norm_sq()); }

    [[nodiscard]] Vec3 normalized() const {
        const float n = norm();
        return (n > 1.0e-8f) ? (*this / n) : Vec3{0.0f, 0.0f, 0.0f};
    }
};

// Quaternion for 3D orientation without gimbal lock
struct Quat {
    float w{1.0f};
    float x{0.0f};
    float y{0.0f};
    float z{0.0f};

    constexpr Quat() = default;
    constexpr Quat(float w_, float x_, float y_, float z_) : w(w_), x(x_), y(y_), z(z_) {}

    [[nodiscard]] static Quat from_euler(float roll, float pitch, float yaw) {
        const float cr = std::cos(roll * 0.5f);
        const float sr = std::sin(roll * 0.5f);
        const float cp = std::cos(pitch * 0.5f);
        const float sp = std::sin(pitch * 0.5f);
        const float cy = std::cos(yaw * 0.5f);
        const float sy = std::sin(yaw * 0.5f);

        return {
            cr * cp * cy + sr * sp * sy,
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy
        };
    }

    [[nodiscard]] Quat normalized() const {
        const float n = std::sqrt(w * w + x * x + y * y + z * z);
        return (n > 1.0e-8f) ? Quat{w / n, x / n, y / n, z / n} : Quat{1.0f, 0.0f, 0.0f, 0.0f};
    }

    [[nodiscard]] constexpr Quat conjugate() const { return {w, -x, -y, -z}; }

    [[nodiscard]] constexpr Quat operator*(const Quat& o) const {
        return {
            w * o.w - x * o.x - y * o.y - z * o.z,
            w * o.x + x * o.w + y * o.z - z * o.y,
            w * o.y - x * o.z + y * o.w + z * o.x,
            w * o.z + x * o.y - y * o.x + z * o.w
        };
    }

    // Rotate a 3D vector by this quaternion: v' = q * (0, v) * q^-1
    [[nodiscard]] Vec3 rotate(const Vec3& v) const {
        const Vec3 u{x, y, z};
        const float s = w;
        return u * (2.0f * u.dot(v)) + v * (s * s - u.dot(u)) + u.cross(v) * (2.0f * s);
    }

    [[nodiscard]] float roll_rad() const noexcept {
        return std::atan2(2.0f * (w * x + y * z), 1.0f - 2.0f * (x * x + y * y));
    }
    [[nodiscard]] float pitch_rad() const noexcept {
        const float sinp = 2.0f * (w * y - z * x);
        return (std::abs(sinp) >= 1.0f) ? std::copysign(3.14159265f / 2.0f, sinp) : std::asin(sinp);
    }
    [[nodiscard]] float yaw_rad() const noexcept {
        return std::atan2(2.0f * (w * z + x * y), 1.0f - 2.0f * (y * y + z * z));
    }
    [[nodiscard]] float roll_deg() const noexcept { return roll_rad() * (180.0f / 3.14159265f); }
    [[nodiscard]] float pitch_deg() const noexcept { return pitch_rad() * (180.0f / 3.14159265f); }
    [[nodiscard]] float yaw_deg() const noexcept { return yaw_rad() * (180.0f / 3.14159265f); }
};

// 2D Pixel coordinate on Focal Plane Array
struct PixelCoord {
    int32_t x{0};
    int32_t y{0};
};

// Global Geographic coordinate
struct GeoCoordinate {
    double latitude_deg{0.0};
    double longitude_deg{0.0};
    double altitude_msl_m{0.0};
};

// Bounding Box in image coordinates
struct BoundingBox {
    float x{0.0f};      // Top-left x [pixels]
    float y{0.0f};      // Top-left y [pixels]
    float width{0.0f};  // Box width [pixels]
    float height{0.0f}; // Box height [pixels]
    float score{0.0f};  // Confidence score [0 - 1]
    uint32_t class_id{0};
};

// Detection result with radiometric Signal-to-Clutter Ratio
struct Detection {
    BoundingBox bbox;
    float peak_intensity{0.0f}; // Peak raw ADC or radiance
    float scr{0.0f};            // Signal-to-Clutter Ratio: (I_peak - mu_clutter) / sigma_clutter
    uint32_t target_id{0};
    GeoCoordinate estimated_geo;
};

// Infrared Spectral Band Selection
enum class SpectralBandType {
    SWIR,    // 0.9 - 1.7 um
    MWIR,    // 3.0 - 5.0 um
    LWIR,    // 8.0 - 14.0 um
    Custom
};

struct SpectralBand {
    SpectralBandType type{SpectralBandType::LWIR};
    double lambda_min_um{8.0};
    double lambda_max_um{14.0};

    static constexpr SpectralBand lwir() {
        return {SpectralBandType::LWIR, 8.0, 14.0};
    }

    static constexpr SpectralBand mwir() {
        return {SpectralBandType::MWIR, 3.0, 5.0};
    }

    static constexpr SpectralBand swir() {
        return {SpectralBandType::SWIR, 0.9, 1.7};
    }
};

// Drone 6-DOF Pose
struct DronePose {
    Vec3 position_enu_m{0.0f, 0.0f, 100.0f}; // East-North-Up coordinate [m]
    Vec3 velocity_mps{0.0f, 0.0f, 0.0f};
    Quat attitude{};                         // Orientation body-to-world
    Vec3 angular_velocity_radps{0.0f, 0.0f, 0.0f};
};

// Gimbal Attitude and Line of Sight
struct GimbalPose {
    float pitch_rad{-0.785398f}; // Default -45 deg look-down
    float yaw_rad{0.0f};
    float roll_rad{0.0f};
    Vec3 look_at_vector_world{0.0f, 0.7071f, -0.7071f};
    Vec3 jitter_offset_rad{0.0f, 0.0f, 0.0f};
};

// Flight Telemetry Packet for UI HUD and logging
struct TelemetryPacket {
    double timestamp_sec{0.0};
    GeoCoordinate drone_geo;
    float altitude_agl_m{0.0f};
    float airspeed_mps{0.0f};
    float ground_speed_mps{0.0f};
    float roll_deg{0.0f};
    float pitch_deg{0.0f};
    float yaw_deg{0.0f};
    float gimbal_pitch_deg{0.0f};
    float gimbal_yaw_deg{0.0f};
    float jitter_rms_mrad{0.0f};
    uint32_t active_tracks_count{0};
};

} // namespace ir_sim::core
