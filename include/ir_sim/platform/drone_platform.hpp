#pragma once

#include "ir_sim/core/types.hpp"

#include <memory>
#include <vector>

namespace ir_sim::platform {

// Control command sent from guidance layer to drone actuator/dynamics model
struct ControlCommand {
    core::Vec3 desired_velocity_mps{0.0f, 0.0f, 0.0f};
    float desired_yaw_rad{0.0f};
    bool hold_position{false};
};

// Abstract interface for flight guidance algorithms
class IFlightController {
public:
    virtual ~IFlightController() = default;
    virtual ControlCommand update(const core::DronePose& current_pose, float dt) = 0;
    virtual void reset() {}
};

/**
 * @brief 6-DOF Rigid-Body Drone Kinematics and Dynamics Simulator.
 * 
 * Uses 4th-Order Runge-Kutta (RK4) integration at 100 Hz for high numerical stability.
 * Incorporates aerodynamic drag, gravitational acceleration, and steady/gusting wind fields.
 */
class DronePlatform {
public:
    DronePlatform();
    explicit DronePlatform(const core::DronePose& initial_pose);

    /**
     * @brief Steps the 6-DOF kinematics forward by dt seconds using RK4.
     * @param dt Time step in seconds (typical: 0.01s = 100 Hz).
     */
    void step(float dt);

    // Flight Controller assignment
    void set_flight_controller(std::shared_ptr<IFlightController> controller) noexcept {
        controller_ = std::move(controller);
    }

    // Environmental Wind
    void set_wind(const core::Vec3& steady_wind_mps, float turbulence_intensity = 0.0f) noexcept {
        steady_wind_mps_ = steady_wind_mps;
        turbulence_intensity_ = turbulence_intensity;
    }

    // State Accessors
    [[nodiscard]] const core::DronePose& pose() const noexcept { return pose_; }
    [[nodiscard]] core::DronePose& pose() noexcept { return pose_; }
    [[nodiscard]] double simulation_time_sec() const noexcept { return sim_time_sec_; }
    [[nodiscard]] float altitude_agl_m() const noexcept { return pose_.position_enu_m.z; }
    [[nodiscard]] float ground_speed_mps() const noexcept;
    [[nodiscard]] float airspeed_mps() const noexcept;

    // Physical UAV specs
    void set_mass_kg(float mass_kg) noexcept { mass_kg_ = mass_kg; }
    void set_drag_coefficient(float cd) noexcept { drag_cd_ = cd; }
    void set_cross_section_area_m2(float area_m2) noexcept { frontal_area_m2_ = area_m2; }

    void set_kp_velocity(float kp) noexcept { kp_vel_ = kp; }
    void set_kp_position(float kp) noexcept { kp_pos_ = kp; }
    void set_max_airspeed(float max_v) noexcept { max_airspeed_mps_ = max_v; }

    [[nodiscard]] float kp_position() const noexcept { return kp_pos_; }
    [[nodiscard]] float max_airspeed() const noexcept { return max_airspeed_mps_; }

private:
    struct Derivative {
        core::Vec3 d_pos;
        core::Vec3 d_vel;
        core::Quat d_quat;
    };

    Derivative evaluate(const core::DronePose& initial_pose,
                        float dt,
                        const Derivative& d,
                        const ControlCommand& cmd);

    core::DronePose pose_;
    double sim_time_sec_{0.0};

    // Guidance
    std::shared_ptr<IFlightController> controller_{nullptr};
    ControlCommand current_cmd_{};

    // Aerodynamics and mass properties (representative medium quadcopter / tactical UAV)
    float mass_kg_{4.5f};              // 4.5 kg tactical UAV (e.g. Ghost class)
    float drag_cd_{0.65f};             // Drag coefficient
    float frontal_area_m2_{0.12f};      // Frontal cross-section
    float air_density_{1.225f};        // Sea level air density [kg/m^3]

    // PID tuning for position-to-acceleration command loop
    float kp_vel_{2.5f};
    float kp_pos_{1.2f};
    float max_tilt_angle_rad_{0.45f};  // ~25 deg maximum pitch/roll bank angle
    float max_airspeed_mps_{25.0f};    // 25 m/s (~50 knots) max cruise speed

    // Wind environment
    core::Vec3 steady_wind_mps_{0.0f, 0.0f, 0.0f};
    float turbulence_intensity_{0.0f};
};

} // namespace ir_sim::platform
