#pragma once

#include "ir_sim/core/types.hpp"

namespace ir_sim::platform {

enum class GimbalMode {
    Nadir,       // Look straight down [0, 0, -1]
    FixedLook,   // Fixed pitch/yaw relative to drone body
    GeoLock,     // Continuously tracks 3D ground target coordinate
    ManualSlew   // Slew by commanded pan/tilt rates
};

/**
 * @brief 2-Axis / 3-Axis Stabilized Gimbal Rig with High-Frequency Jitter.
 * 
 * Compensates for airframe roll, pitch, and yaw oscillations to maintain
 * Line of Sight (LOS) stabilization, while injecting realistic motor/propeller
 * harmonic vibrations (50 - 250 Hz) to test tracker stability.
 */
class Gimbal {
public:
    Gimbal();

    /**
     * @brief Updates gimbal attitude and stabilization given current drone pose.
     * @param drone_pose Current 6-DOF drone state.
     * @param dt Simulation time step [s].
     */
    void update(const core::DronePose& drone_pose, float dt);

    // Mode Configuration
    void set_mode(GimbalMode mode) noexcept { mode_ = mode; }
    void set_target_location(const core::Vec3& target_enu_m) noexcept {
        target_location_ = target_enu_m;
        mode_ = GimbalMode::GeoLock;
    }
    void snap_to_target(const core::Vec3& drone_pos) noexcept {
        compute_geolock_angles(drone_pos, current_pitch_rad_, current_yaw_rad_);
    }
    void set_fixed_angles(float pitch_rad, float yaw_rad) noexcept {
        fixed_pitch_rad_ = pitch_rad;
        fixed_yaw_rad_ = yaw_rad;
    }

    // Manual Slew commands
    void set_slew_rates(float pitch_rate_radps, float yaw_rate_radps) noexcept {
        pitch_rate_cmd_ = pitch_rate_radps;
        yaw_rate_cmd_ = yaw_rate_radps;
    }

    // Vibration Jitter configuration
    void set_jitter_enabled(bool enabled) noexcept { jitter_enabled_ = enabled; }
    void set_jitter_amplitude_mrad(float amplitude_mrad) noexcept {
        jitter_amplitude_rad_ = amplitude_mrad * 1.0e-3f;
    }

    // Output State
    [[nodiscard]] const core::GimbalPose& pose() const noexcept { return pose_; }
    [[nodiscard]] core::Vec3 line_of_sight() const noexcept { return pose_.look_at_vector_world; }
    [[nodiscard]] core::Quat camera_orientation() const noexcept { return camera_quat_; }
    [[nodiscard]] GimbalMode mode() const noexcept { return mode_; }
    [[nodiscard]] float current_pitch_deg() const noexcept;
    [[nodiscard]] float current_yaw_deg() const noexcept;
    [[nodiscard]] float jitter_rms_mrad() const noexcept { return jitter_amplitude_rad_ * 1000.0f * 0.707f; }

private:
    void compute_geolock_angles(const core::Vec3& drone_pos, float& out_pitch, float& out_yaw) const;

    GimbalMode mode_{GimbalMode::Nadir};
    core::GimbalPose pose_{};
    core::Quat camera_quat_{};

    core::Vec3 target_location_{0.0f, 0.0f, 0.0f};
    float fixed_pitch_rad_{-0.785398f}; // -45 deg
    float fixed_yaw_rad_{0.0f};

    float current_pitch_rad_{-1.570796f}; // Nadir (-90 deg)
    float current_yaw_rad_{0.0f};

    float pitch_rate_cmd_{0.0f};
    float yaw_rate_cmd_{0.0f};
    float max_slew_rate_radps_{1.57f}; // 90 deg/s max slew

    // Jitter & Vibration Parameters
    bool jitter_enabled_{true};
    float jitter_amplitude_rad_{0.15e-3f}; // 0.15 mrad default RMS jitter
    double elapsed_time_sec_{0.0};
};

} // namespace ir_sim::platform
