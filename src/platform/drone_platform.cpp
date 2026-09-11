#include "ir_sim/platform/drone_platform.hpp"

#include <algorithm>
#include <cmath>

namespace ir_sim::platform {

DronePlatform::DronePlatform() {
    pose_.position_enu_m = {0.0f, 0.0f, 100.0f}; // Default 100 m AGL
    pose_.velocity_mps = {0.0f, 0.0f, 0.0f};
    pose_.attitude = core::Quat{1.0f, 0.0f, 0.0f, 0.0f};
    pose_.angular_velocity_radps = {0.0f, 0.0f, 0.0f};
}

DronePlatform::DronePlatform(const core::DronePose& initial_pose)
    : pose_(initial_pose) {}

float DronePlatform::ground_speed_mps() const noexcept {
    return std::sqrt(pose_.velocity_mps.x * pose_.velocity_mps.x +
                     pose_.velocity_mps.y * pose_.velocity_mps.y);
}

float DronePlatform::airspeed_mps() const noexcept {
    const core::Vec3 v_rel = pose_.velocity_mps - steady_wind_mps_;
    return v_rel.norm();
}

DronePlatform::Derivative DronePlatform::evaluate(const core::DronePose& initial_pose,
                                                  float dt,
                                                  const Derivative& d,
                                                  const ControlCommand& cmd) {
    core::DronePose state;
    state.position_enu_m = initial_pose.position_enu_m + d.d_pos * dt;
    state.velocity_mps = initial_pose.velocity_mps + d.d_vel * dt;

    core::Quat q = initial_pose.attitude;
    q.w += d.d_quat.w * dt;
    q.x += d.d_quat.x * dt;
    q.y += d.d_quat.y * dt;
    q.z += d.d_quat.z * dt;
    state.attitude = q.normalized();

    // 1. Position rate is velocity
    Derivative output;
    output.d_pos = state.velocity_mps;

    // 2. Velocity command tracking acceleration (PID feedforward)
    const core::Vec3 vel_error = cmd.desired_velocity_mps - state.velocity_mps;
    core::Vec3 accel_cmd = vel_error * kp_vel_;

    // Limit maximum horizontal acceleration for realistic flight envelope (e.g. 5 m/s^2)
    const float max_accel_horiz = 5.0f;
    const float horiz_accel_norm = std::sqrt(accel_cmd.x * accel_cmd.x + accel_cmd.y * accel_cmd.y);
    if (horiz_accel_norm > max_accel_horiz) {
        accel_cmd.x = (accel_cmd.x / horiz_accel_norm) * max_accel_horiz;
        accel_cmd.y = (accel_cmd.y / horiz_accel_norm) * max_accel_horiz;
    }

    // Limit vertical climb/descent acceleration
    accel_cmd.z = std::clamp(accel_cmd.z, -3.0f, 4.0f);

    // 3. Aerodynamic drag: F_drag = 0.5 * rho * Cd * A * v_rel^2
    const core::Vec3 v_rel = state.velocity_mps - steady_wind_mps_;
    const float v_rel_norm = v_rel.norm();
    const core::Vec3 drag_force = v_rel * (-0.5f * air_density_ * drag_cd_ * frontal_area_m2_ * v_rel_norm);
    const core::Vec3 drag_accel = drag_force / mass_kg_;

    output.d_vel = accel_cmd + drag_accel;

    // 4. Attitude dynamics: compute bank angle from horizontal acceleration
    constexpr float g = 9.80665f;
    const float pitch = std::clamp(accel_cmd.y / g, -max_tilt_angle_rad_, max_tilt_angle_rad_);
    const float roll = std::clamp(-accel_cmd.x / g, -max_tilt_angle_rad_, max_tilt_angle_rad_);
    const float yaw = cmd.desired_yaw_rad;

    const core::Quat target_quat = core::Quat::from_euler(roll, pitch, yaw).normalized();

    // Slew rate toward target orientation (angular rate integration)
    constexpr float attitude_tau = 0.20f; // 200ms time constant
    output.d_quat.w = (target_quat.w - state.attitude.w) / attitude_tau;
    output.d_quat.x = (target_quat.x - state.attitude.x) / attitude_tau;
    output.d_quat.y = (target_quat.y - state.attitude.y) / attitude_tau;
    output.d_quat.z = (target_quat.z - state.attitude.z) / attitude_tau;

    return output;
}

void DronePlatform::step(float dt) {
    if (dt <= 0.0f) return;

    // 1. Update guidance controller command
    if (controller_) {
        current_cmd_ = controller_->update(pose_, dt);
    }

    // 2. 4th-Order Runge-Kutta (RK4) numerical integration
    const Derivative zero_d{};
    const Derivative a = evaluate(pose_, 0.0f, zero_d, current_cmd_);
    const Derivative b = evaluate(pose_, dt * 0.5f, a, current_cmd_);
    const Derivative c = evaluate(pose_, dt * 0.5f, b, current_cmd_);
    const Derivative d = evaluate(pose_, dt, c, current_cmd_);

    // Update position
    pose_.position_enu_m += (a.d_pos + (b.d_pos + c.d_pos) * 2.0f + d.d_pos) * (dt / 6.0f);

    // Prevent subterranean flight (clamp to ground level z >= 0)
    if (pose_.position_enu_m.z < 0.0f) {
        pose_.position_enu_m.z = 0.0f;
        pose_.velocity_mps.z = std::max(0.0f, pose_.velocity_mps.z);
    }

    // Update velocity
    pose_.velocity_mps += (a.d_vel + (b.d_vel + c.d_vel) * 2.0f + d.d_vel) * (dt / 6.0f);

    // Update attitude quaternion
    pose_.attitude.w += (a.d_quat.w + (b.d_quat.w + c.d_quat.w) * 2.0f + d.d_quat.w) * (dt / 6.0f);
    pose_.attitude.x += (a.d_quat.x + (b.d_quat.x + c.d_quat.x) * 2.0f + d.d_quat.x) * (dt / 6.0f);
    pose_.attitude.y += (a.d_quat.y + (b.d_quat.y + c.d_quat.y) * 2.0f + d.d_quat.y) * (dt / 6.0f);
    pose_.attitude.z += (a.d_quat.z + (b.d_quat.z + c.d_quat.z) * 2.0f + d.d_quat.z) * (dt / 6.0f);

    pose_.attitude = pose_.attitude.normalized();
    sim_time_sec_ += dt;
}

} // namespace ir_sim::platform
