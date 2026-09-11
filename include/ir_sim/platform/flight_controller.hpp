#pragma once

#include "ir_sim/platform/drone_platform.hpp"

#include <vector>

namespace ir_sim::platform {

/**
 * @brief Autonomous Waypoint Follower for perimeter surveillance and grid searches.
 */
class WaypointFlightController : public IFlightController {
public:
    WaypointFlightController() = default;
    explicit WaypointFlightController(std::vector<core::Vec3> waypoints,
                                     float cruise_speed_mps = 12.0f,
                                     float acceptance_radius_m = 8.0f,
                                     bool loop = true);

    ControlCommand update(const core::DronePose& current_pose, float dt) override;
    void reset() override;

    void set_waypoints(std::vector<core::Vec3> waypoints) {
        waypoints_ = std::move(waypoints);
        current_idx_ = 0;
    }

    [[nodiscard]] size_t current_waypoint_index() const noexcept { return current_idx_; }
    [[nodiscard]] size_t total_waypoints() const noexcept { return waypoints_.size(); }
    [[nodiscard]] bool is_completed() const noexcept { return completed_; }

private:
    std::vector<core::Vec3> waypoints_;
    float cruise_speed_mps_{12.0f};
    float acceptance_radius_m_{8.0f};
    bool loop_{true};
    size_t current_idx_{0};
    bool completed_{false};
};

/**
 * @brief Standoff Orbit / Loiter Controller for continuous target surveillance.
 */
class OrbitFlightController : public IFlightController {
public:
    OrbitFlightController() = default;
    OrbitFlightController(core::Vec3 center_enu_m,
                          float radius_m = 150.0f,
                          float altitude_m = 100.0f,
                          float airspeed_mps = 14.0f,
                          bool clockwise = true);

    ControlCommand update(const core::DronePose& current_pose, float dt) override;

    void set_center(const core::Vec3& center) noexcept { center_enu_m_ = center; }
    void set_radius(float r) noexcept { radius_m_ = r; }
    void set_altitude(float alt) noexcept { altitude_m_ = alt; }
    void set_speed(float speed) noexcept { airspeed_mps_ = speed; }

    [[nodiscard]] const core::Vec3& center() const noexcept { return center_enu_m_; }
    [[nodiscard]] float radius() const noexcept { return radius_m_; }

private:
    core::Vec3 center_enu_m_{0.0f, 0.0f, 0.0f};
    float radius_m_{150.0f};
    float altitude_m_{100.0f};
    float airspeed_mps_{14.0f};
    bool clockwise_{true};
};

/**
 * @brief Manual Virtual Stick Controller for interactive teleoperation from the UI.
 */
class ManualFlightController : public IFlightController {
public:
    ManualFlightController() = default;

    ControlCommand update(const core::DronePose& current_pose, float dt) override;

    // Direct UI input setters [-1.0f to 1.0f]
    void set_pitch_input(float pitch) noexcept { pitch_cmd_ = pitch; }
    void set_roll_input(float roll) noexcept { roll_cmd_ = roll; }
    void set_climb_rate(float climb) noexcept { climb_rate_mps_ = climb; }
    void set_yaw_rate(float yaw_rate) noexcept { yaw_rate_radps_ = yaw_rate; }

private:
    float pitch_cmd_{0.0f};       // Forward/back
    float roll_cmd_{0.0f};        // Left/right strafe
    float climb_rate_mps_{0.0f};  // Climb/descend speed [m/s]
    float yaw_rate_radps_{0.0f};  // Heading rate [rad/s]
    float current_yaw_{0.0f};
    float max_speed_mps_{15.0f};
};

} // namespace ir_sim::platform
