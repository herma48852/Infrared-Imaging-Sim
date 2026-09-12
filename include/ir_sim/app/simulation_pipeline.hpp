#pragma once

#include "ir_sim/core/frame_buffer.hpp"
#include "ir_sim/core/types.hpp"
#include "ir_sim/detection/cfar_2d.hpp"
#include "ir_sim/detection/kalman_tracker.hpp"
#include "ir_sim/detection/point_target_lcm.hpp"
#include "ir_sim/platform/camera_model.hpp"
#include "ir_sim/platform/drone_platform.hpp"
#include "ir_sim/platform/flight_controller.hpp"
#include "ir_sim/platform/gimbal.hpp"
#include "ir_sim/scene/scenario_manifest.hpp"
#include "ir_sim/scene/synthetic_scene.hpp"
#include "ir_sim/sensor/fpa_sensor.hpp"
#include "ir_sim/sensor/isp_pipeline.hpp"

#include <chrono>
#include <deque>
#include <memory>
#include <vector>

namespace ir_sim::app {

struct BeamRecord {
    double sim_time_sec{0.0};
    double az_deg{0.0};
    double el_deg{0.0};
};

enum class FlightMode {
    Waypoint,
    Orbit,
    Manual,
    Pursuit
};

struct StageLatency {
    float scene_ms{0.0f};
    float fpa_ms{0.0f};
    float isp_ms{0.0f};
    float cfar_ms{0.0f};
    float tracker_ms{0.0f};
    float total_ms{0.0f};
};

struct PipelineTelemetry {
    float sim_time_sec{0.0f};
    core::DronePose drone_pose{};
    core::GimbalPose gimbal_pose{};
    core::GeoCoordinate drone_gps{};
    float ground_speed_mps{0.0f};
    float altitude_agl_m{0.0f};
    StageLatency latency{};
    size_t ground_truth_count{0};
    size_t detection_count{0};
    size_t confirmed_track_count{0};
};

/**
 * @brief Unified Simulation Pipeline Coordinator.
 * 
 * Manages 6-DOF kinematics, gimbal pointing, synthetic radiance rendering,
 * FPA sensor degradation, embedded ISP processing, tactical detection, and
 * multi-target Kalman tracking with zero heap allocations on the hot frame loop.
 */
class SimulationPipeline {
public:
    explicit SimulationPipeline(
        scene::ScenarioManifest manifest,
        uint32_t width = 640,
        uint32_t height = 512
    );

    /**
     * @brief Advances the complete end-to-end simulation by dt seconds.
     * @param dt Frame delta time in seconds (e.g. 0.0333s = 30 Hz).
     */
    void step(float dt);

    /**
     * @brief Performs two-point NUC calibration using uniform blackbody frames.
     */
    void calibrate_nuc(float temp_cold_k = 288.15f, float temp_hot_k = 308.15f);

    // Flight mode control
    void set_flight_mode(FlightMode mode);
    [[nodiscard]] FlightMode flight_mode() const noexcept { return current_flight_mode_; }

    // Manual teleoperation stick commands (normalized -1.0 to +1.0)
    void set_manual_sticks(float roll_stick, float pitch_stick, float yaw_rate_stick, float throttle_stick);

    // Orbit controller parameters
    void set_orbit_params(const core::Vec3& center, float radius_m, float speed_mps);

    // Gimbal controls
    void set_gimbal_nadir();
    void set_gimbal_geolock(const core::Vec3& ground_target_m);
    void set_gimbal_manual_rates(float pitch_rate_radps, float yaw_rate_radps);
    void set_gimbal_sector_scan(
        float az_min_deg = 0.0f,
        float az_max_deg = 90.0f,
        float sweep_period_sec = 0.45f,
        std::vector<float> elevation_bars_deg = {25.0f, 3.0f, 14.0f}
    );

    // Component accessors
    [[nodiscard]] platform::DronePlatform& drone() noexcept { return drone_; }
    [[nodiscard]] const platform::DronePlatform& drone() const noexcept { return drone_; }

    [[nodiscard]] platform::Gimbal& gimbal() noexcept { return gimbal_; }
    [[nodiscard]] const platform::Gimbal& gimbal() const noexcept { return gimbal_; }

    [[nodiscard]] const platform::CameraModel& camera() const noexcept { return camera_; }
    [[nodiscard]] scene::SyntheticScene& scene() noexcept { return scene_; }
    [[nodiscard]] const scene::SyntheticScene& scene() const noexcept { return scene_; }

    [[nodiscard]] sensor::FPASensor& fpa() noexcept { return fpa_; }
    [[nodiscard]] const sensor::FPASensor& fpa() const noexcept { return fpa_; }

    [[nodiscard]] sensor::ISPPipeline& isp() noexcept { return isp_; }
    [[nodiscard]] const sensor::ISPPipeline& isp() const noexcept { return isp_; }

    [[nodiscard]] detection::CFAR2D& cfar() noexcept { return cfar_; }
    [[nodiscard]] const detection::CFAR2D& cfar() const noexcept { return cfar_; }

    [[nodiscard]] detection::MultiTargetTracker& tracker() noexcept { return tracker_; }
    [[nodiscard]] const detection::MultiTargetTracker& tracker() const noexcept { return tracker_; }

    // Frame buffer accessors (zero-copy)
    [[nodiscard]] const core::RadianceFrame& ground_truth_radiance() const noexcept { return gt_radiance_; }
    [[nodiscard]] const core::RawFrame14Bit& raw_fpa_frame() const noexcept { return raw_fpa_; }
    [[nodiscard]] const core::RawFrame14Bit& calibrated_nuc_frame() const noexcept { return nuc_frame_; }
    [[nodiscard]] const core::FrameBuffer<uint8_t>& display_frame() const noexcept { return display_8bit_; }

    // Detections & Tracks
    [[nodiscard]] const std::vector<core::Detection>& ground_truth_detections() const noexcept { return gt_detections_; }
    [[nodiscard]] const std::vector<core::Detection>& current_detections() const noexcept { return detections_; }
    [[nodiscard]] const std::vector<detection::Track>& active_tracks() const noexcept { return active_tracks_; }

    // Telemetry & Latency
    [[nodiscard]] const PipelineTelemetry& telemetry() const noexcept { return telemetry_; }
    [[nodiscard]] const std::deque<BeamRecord>& beam_history() const noexcept { return beam_history_; }
    [[nodiscard]] core::Vec3 primary_target_position() const noexcept {
        if (!manifest_.targets.empty()) {
            return manifest_.targets.front().evaluate_at_time(elapsed_sim_time_sec_).position;
        }
        return {100.0f, 200.0f, manifest_.terrain.base_elevation_m};
    }

    // Options
    bool enable_lcm_filter{true};
    bool enable_bpr{true};

private:
    scene::ScenarioManifest manifest_;
    uint32_t width_;
    uint32_t height_;
    float elapsed_sim_time_sec_{0.0f};

    // Subsystems
    platform::DronePlatform drone_;
    platform::Gimbal gimbal_;
    platform::CameraModel camera_;
    scene::SyntheticScene scene_;
    sensor::FPASensor fpa_;
    sensor::ISPPipeline isp_;
    detection::CFAR2D cfar_;
    detection::PointTargetLCM lcm_;
    detection::MultiTargetTracker tracker_;

    // Flight Controllers
    FlightMode current_flight_mode_{FlightMode::Orbit};
    std::shared_ptr<platform::WaypointFlightController> waypoint_fc_;
    std::shared_ptr<platform::OrbitFlightController> orbit_fc_;
    std::shared_ptr<platform::ManualFlightController> manual_fc_;

    // Pre-allocated frame buffers
    core::RadianceFrame gt_radiance_;
    core::RawFrame14Bit raw_fpa_;
    core::RawFrame14Bit nuc_frame_;
    core::FrameBuffer<uint8_t> display_8bit_;
    std::vector<float> lcm_saliency_;

    // Detections & Tracks
    std::vector<core::Detection> gt_detections_;
    std::vector<core::Detection> detections_;
    std::vector<detection::Track> active_tracks_;

    // Telemetry
    PipelineTelemetry telemetry_{};
    std::deque<BeamRecord> beam_history_{};
};

} // namespace ir_sim::app
