#include "ir_sim/platform/gimbal.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>

namespace ir_sim::platform {

Gimbal::Gimbal() {
    current_pitch_rad_ = -static_cast<float>(std::numbers::pi) * 0.5f; // Nadir
    current_yaw_rad_ = 0.0f;
}

void Gimbal::compute_geolock_angles(const core::Vec3& drone_pos, float& out_pitch, float& out_yaw) const {
    const core::Vec3 delta = target_location_ - drone_pos;
    const float horizontal_dist = std::sqrt(delta.x * delta.x + delta.y * delta.y);

    out_pitch = std::atan2(delta.z, horizontal_dist); // Negative when drone is above target
    out_yaw = std::atan2(delta.x, delta.y);           // 0 is North (+Y), positive clockwise toward East (+X)
}

float Gimbal::current_pitch_deg() const noexcept {
    return current_pitch_rad_ * (180.0f / static_cast<float>(std::numbers::pi));
}

float Gimbal::current_yaw_deg() const noexcept {
    return current_yaw_rad_ * (180.0f / static_cast<float>(std::numbers::pi));
}

void Gimbal::update(const core::DronePose& drone_pose, float dt) {
    if (dt <= 0.0f) return;
    elapsed_time_sec_ += dt;

    float desired_pitch = current_pitch_rad_;
    float desired_yaw = current_yaw_rad_;

    switch (mode_) {
        case GimbalMode::Nadir:
            desired_pitch = -static_cast<float>(std::numbers::pi) * 0.5f; // -90 deg
            desired_yaw = 0.0f;
            break;

        case GimbalMode::GeoLock:
            compute_geolock_angles(drone_pose.position_enu_m, desired_pitch, desired_yaw);
            break;

        case GimbalMode::FixedLook: {
            // Fixed relative to drone yaw
            // Extract drone yaw from quaternion
            const float q0 = drone_pose.attitude.w;
            const float q3 = drone_pose.attitude.z;
            const float drone_yaw = 2.0f * std::atan2(q3, q0);

            desired_pitch = fixed_pitch_rad_;
            desired_yaw = drone_yaw + fixed_yaw_rad_;
            break;
        }

        case GimbalMode::ManualSlew:
            desired_pitch = current_pitch_rad_ + pitch_rate_cmd_ * dt;
            desired_yaw = current_yaw_rad_ + yaw_rate_cmd_ * dt;
            break;
    }

    // Limit pitch angle to prevent gimbal flip [-90 deg to +15 deg]
    constexpr float min_pitch = -static_cast<float>(std::numbers::pi) * 0.5f;
    constexpr float max_pitch = 0.2618f; // +15 deg
    desired_pitch = std::clamp(desired_pitch, min_pitch, max_pitch);

    // Apply servo slew rate limits
    const float max_step = max_slew_rate_radps_ * dt;
    const float pitch_err = desired_pitch - current_pitch_rad_;
    current_pitch_rad_ += std::clamp(pitch_err, -max_step, max_step);

    // Yaw angle error with wrap-around
    float yaw_err = desired_yaw - current_yaw_rad_;
    constexpr float pi = static_cast<float>(std::numbers::pi);
    while (yaw_err > pi) yaw_err -= 2.0f * pi;
    while (yaw_err < -pi) yaw_err += 2.0f * pi;
    current_yaw_rad_ += std::clamp(yaw_err, -max_step, max_step);

    // High-Frequency Structural Vibration / Motor Harmonics
    float jitter_p = 0.0f;
    float jitter_y = 0.0f;
    if (jitter_enabled_ && jitter_amplitude_rad_ > 1.0e-8f) {
        const double t = elapsed_time_sec_;
        // Multi-frequency harmonic synthesis (50 Hz motor pole, 120 Hz blade pass, 200 Hz harmonic)
        const double harm1 = 0.60 * std::sin(2.0 * std::numbers::pi * 50.0 * t);
        const double harm2 = 0.30 * std::sin(2.0 * std::numbers::pi * 120.0 * t + 1.2);
        const double harm3 = 0.10 * std::sin(2.0 * std::numbers::pi * 200.0 * t + 2.4);

        const double harm_y1 = 0.50 * std::cos(2.0 * std::numbers::pi * 55.0 * t);
        const double harm_y2 = 0.35 * std::sin(2.0 * std::numbers::pi * 115.0 * t + 0.8);
        const double harm_y3 = 0.15 * std::cos(2.0 * std::numbers::pi * 210.0 * t);

        jitter_p = static_cast<float>((harm1 + harm2 + harm3) * jitter_amplitude_rad_);
        jitter_y = static_cast<float>((harm_y1 + harm_y2 + harm_y3) * jitter_amplitude_rad_);
    }

    const float effective_pitch = current_pitch_rad_ + jitter_p;
    const float effective_yaw = current_yaw_rad_ + jitter_y;

    // Unit Line-of-Sight (LOS) vector in ENU coordinates:
    // X = East, Y = North, Z = Up
    pose_.look_at_vector_world = {
        std::cos(effective_pitch) * std::sin(effective_yaw),
        std::cos(effective_pitch) * std::cos(effective_yaw),
        std::sin(effective_pitch)
    };

    pose_.pitch_rad = effective_pitch;
    pose_.yaw_rad = effective_yaw;
    pose_.roll_rad = 0.0f; // 3-axis gimbal keeps camera level with horizon
    pose_.jitter_offset_rad = {jitter_p, jitter_y, 0.0f};

    // Construct orthonormal basis in ENU world frame:
    // +Z_cam = Forward (along line of sight)
    const core::Vec3 f = pose_.look_at_vector_world.normalized();

    // Reference world up: [0, 0, 1]
    // If looking nearly straight down (Nadir), use North [0, 1, 0] as secondary reference
    core::Vec3 r;
    if (std::abs(f.z) > 0.999f) {
        // Looking Nadir or Zenith: +X_cam points East, +Y_cam points South/North
        r = core::Vec3{1.0f, 0.0f, 0.0f};
    } else {
        // +X_cam points horizontally to the right
        r = core::Vec3{f.y, -f.x, 0.0f}.normalized();
    }
    // +Y_cam = Down (f x r)
    const core::Vec3 d = f.cross(r).normalized();

    // Convert 3x3 rotation matrix [r, d, f] to quaternion
    // R = [ r.x  d.x  f.x ]
    //     [ r.y  d.y  f.y ]
    //     [ r.z  d.z  f.z ]
    const float trace = r.x + d.y + f.z;
    if (trace > 0.0f) {
        const float s = 0.5f / std::sqrt(trace + 1.0f);
        camera_quat_.w = 0.25f / s;
        camera_quat_.x = (d.z - f.y) * s;
        camera_quat_.y = (f.x - r.z) * s;
        camera_quat_.z = (r.y - d.x) * s;
    } else {
        if (r.x > d.y && r.x > f.z) {
            const float s = 2.0f * std::sqrt(1.0f + r.x - d.y - f.z);
            camera_quat_.w = (d.z - f.y) / s;
            camera_quat_.x = 0.25f * s;
            camera_quat_.y = (r.y + d.x) / s;
            camera_quat_.z = (r.z + f.x) / s;
        } else if (d.y > f.z) {
            const float s = 2.0f * std::sqrt(1.0f + d.y - r.x - f.z);
            camera_quat_.w = (f.x - r.z) / s;
            camera_quat_.x = (r.y + d.x) / s;
            camera_quat_.y = 0.25f * s;
            camera_quat_.z = (d.z + f.y) / s;
        } else {
            const float s = 2.0f * std::sqrt(1.0f + f.z - r.x - d.y);
            camera_quat_.w = (r.y - d.x) / s;
            camera_quat_.x = (r.z + f.x) / s;
            camera_quat_.y = (d.z + f.y) / s;
            camera_quat_.z = 0.25f * s;
        }
    }
    camera_quat_ = camera_quat_.normalized();
}

} // namespace ir_sim::platform
