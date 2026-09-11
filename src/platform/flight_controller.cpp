#include "ir_sim/platform/flight_controller.hpp"

#include <cmath>
#include <numbers>

namespace ir_sim::platform {

// -----------------------------------------------------------------------------
// WaypointFlightController
// -----------------------------------------------------------------------------

WaypointFlightController::WaypointFlightController(std::vector<core::Vec3> waypoints,
                                                   float cruise_speed_mps,
                                                   float acceptance_radius_m,
                                                   bool loop)
    : waypoints_(std::move(waypoints)),
      cruise_speed_mps_(cruise_speed_mps),
      acceptance_radius_m_(acceptance_radius_m),
      loop_(loop) {}

void WaypointFlightController::reset() {
    current_idx_ = 0;
    completed_ = false;
}

ControlCommand WaypointFlightController::update(const core::DronePose& current_pose, float /*dt*/) {
    ControlCommand cmd{};

    if (waypoints_.empty() || completed_) {
        cmd.desired_velocity_mps = {0.0f, 0.0f, 0.0f};
        cmd.hold_position = true;
        return cmd;
    }

    const core::Vec3& target_wp = waypoints_[current_idx_];
    const core::Vec3 to_target = target_wp - current_pose.position_enu_m;
    const float dist = to_target.norm();

    // Check if waypoint reached
    if (dist <= acceptance_radius_m_) {
        if (current_idx_ + 1 < waypoints_.size()) {
            current_idx_++;
        } else if (loop_) {
            current_idx_ = 0;
        } else {
            completed_ = true;
            cmd.desired_velocity_mps = {0.0f, 0.0f, 0.0f};
            cmd.hold_position = true;
            return cmd;
        }
    }

    // Direct unit vector toward waypoint
    const core::Vec3 dir = (dist > 1.0e-4f) ? (to_target / dist) : core::Vec3{0.0f, 0.0f, 0.0f};
    cmd.desired_velocity_mps = dir * cruise_speed_mps_;

    // Set desired yaw aligned with horizontal velocity
    if (std::abs(dir.x) > 0.01f || std::abs(dir.y) > 0.01f) {
        cmd.desired_yaw_rad = std::atan2(dir.y, dir.x);
    }

    return cmd;
}

// -----------------------------------------------------------------------------
// OrbitFlightController
// -----------------------------------------------------------------------------

OrbitFlightController::OrbitFlightController(core::Vec3 center_enu_m,
                                             float radius_m,
                                             float altitude_m,
                                             float airspeed_mps,
                                             bool clockwise)
    : center_enu_m_(center_enu_m),
      radius_m_(radius_m),
      altitude_m_(altitude_m),
      airspeed_mps_(airspeed_mps),
      clockwise_(clockwise) {}

ControlCommand OrbitFlightController::update(const core::DronePose& current_pose, float /*dt*/) {
    ControlCommand cmd{};

    const float dx = current_pose.position_enu_m.x - center_enu_m_.x;
    const float dy = current_pose.position_enu_m.y - center_enu_m_.y;
    const float current_radius = std::sqrt(dx * dx + dy * dy);
    const float current_angle = std::atan2(dy, dx);

    // Tangential direction angle
    const float direction_sign = clockwise_ ? -1.0f : 1.0f;
    const float tangent_angle = current_angle + direction_sign * (static_cast<float>(std::numbers::pi) * 0.5f);

    // Tangential velocity vector
    core::Vec3 vel_tangent{
        std::cos(tangent_angle) * airspeed_mps_,
        std::sin(tangent_angle) * airspeed_mps_,
        0.0f
    };

    // Radial correction (P controller driving radius error to 0)
    constexpr float kp_radial = 0.5f;
    const float radial_error = radius_m_ - current_radius;
    const core::Vec3 vel_radial{
        std::cos(current_angle) * (-radial_error * kp_radial),
        std::sin(current_angle) * (-radial_error * kp_radial),
        0.0f
    };

    // Altitude correction
    constexpr float kp_alt = 1.0f;
    const float alt_error = altitude_m_ - current_pose.position_enu_m.z;
    const float vel_z = std::clamp(alt_error * kp_alt, -3.0f, 3.0f);

    cmd.desired_velocity_mps = vel_tangent + vel_radial;
    cmd.desired_velocity_mps.z = vel_z;

    // Point drone nose tangentially along the circle
    cmd.desired_yaw_rad = std::atan2(cmd.desired_velocity_mps.y, cmd.desired_velocity_mps.x);

    return cmd;
}

// -----------------------------------------------------------------------------
// ManualFlightController
// -----------------------------------------------------------------------------

ControlCommand ManualFlightController::update(const core::DronePose& /*current_pose*/, float dt) {
    ControlCommand cmd{};

    current_yaw_ += yaw_rate_radps_ * dt;
    cmd.desired_yaw_rad = current_yaw_;

    // Translate pitch/roll inputs in body frame to world ENU frame
    const float cy = std::cos(current_yaw_);
    const float sy = std::sin(current_yaw_);

    const float forward_speed = pitch_cmd_ * max_speed_mps_;
    const float right_speed = roll_cmd_ * max_speed_mps_;

    cmd.desired_velocity_mps.x = cy * forward_speed + sy * right_speed;
    cmd.desired_velocity_mps.y = sy * forward_speed - cy * right_speed;
    cmd.desired_velocity_mps.z = climb_rate_mps_;

    return cmd;
}

} // namespace ir_sim::platform
