#include "ir_sim/app/simulation_pipeline.hpp"

#include <algorithm>

namespace ir_sim::app {

SimulationPipeline::SimulationPipeline(
    scene::ScenarioManifest manifest,
    uint32_t width,
    uint32_t height
)
    : manifest_(std::move(manifest)),
      width_(width),
      height_(height),
      scene_(manifest_),
      fpa_(sensor::FPASpecs{.width = width, .height = height}),
      isp_(width, height),
      cfar_(width, height),
      lcm_(width, height),
      gt_radiance_(width, height),
      raw_fpa_(width, height),
      nuc_frame_(width, height),
      display_8bit_(width, height),
      lcm_saliency_(width * height, 0.0f)
{
    // Setup camera specs
    platform::CameraSpecs cam_specs{};
    cam_specs.width_pixels = width;
    cam_specs.height_pixels = height;
    cam_specs.focal_length_m = 0.050f;     // 50 mm tactical lens
    cam_specs.pixel_pitch_m = 12.0e-6f;    // 12 um pixel pitch
    camera_ = platform::CameraModel(cam_specs);

    // Lock payload onto primary target
    core::Vec3 tgt_center{100.0f, 200.0f, manifest_.terrain.base_elevation_m};
    if (!manifest_.targets.empty() && !manifest_.targets.front().waypoints.empty()) {
        tgt_center = manifest_.targets.front().waypoints.front().position;
    }

    const float flight_alt = manifest_.terrain.base_elevation_m + 350.0f;
    drone_.pose().position_enu_m = {tgt_center.x - 100.0f, tgt_center.y - 100.0f, flight_alt};
    drone_.pose().velocity_mps = {15.0f, 0.0f, 0.0f};

    // Initialize flight controllers centered on tactical objective
    std::vector<core::Vec3> waypoints = {
        {tgt_center.x - 150.0f, tgt_center.y - 150.0f, flight_alt},
        {tgt_center.x + 150.0f, tgt_center.y - 150.0f, flight_alt},
        {tgt_center.x + 150.0f, tgt_center.y + 150.0f, flight_alt},
        {tgt_center.x - 150.0f, tgt_center.y + 150.0f, flight_alt}
    };
    waypoint_fc_ = std::make_shared<platform::WaypointFlightController>(waypoints, 18.0f, 15.0f, true);
    orbit_fc_ = std::make_shared<platform::OrbitFlightController>(
        core::Vec3{tgt_center.x, tgt_center.y, flight_alt},
        160.0f, 18.0f, true
    );
    manual_fc_ = std::make_shared<platform::ManualFlightController>();

    set_flight_mode(FlightMode::Orbit);

    // Gimbal pointing: GeoLock onto tactical targets centroid
    gimbal_.set_target_location(tgt_center);
    gimbal_.snap_to_target(drone_.pose().position_enu_m);
    gimbal_.update(drone_.pose(), 0.001f);
    gimbal_.set_jitter_enabled(true);

    // Configure CFAR parameters for high-fidelity tactical detection
    cfar_.params().guard_half_size = 3;
    cfar_.params().training_half_size = 8;
    cfar_.params().threshold_factor = 5.0f;
    cfar_.params().min_scr = 3.5f;
    cfar_.params().min_peak_value = 5000.0f;

    // Pre-calibrate NUC at startup so video is clear from frame 0
    calibrate_nuc(288.15f, 308.15f);
}

void SimulationPipeline::calibrate_nuc(float temp_cold_k, float temp_hot_k) {
    core::RawFrame14Bit cold_frame(width_, height_);
    core::RawFrame14Bit hot_frame(width_, height_);

    fpa_.generate_flat_field(temp_cold_k, cold_frame.as_span(), false);
    fpa_.generate_flat_field(temp_hot_k, hot_frame.as_span(), false);

    isp_.calibrate_2point(cold_frame.as_span(), hot_frame.as_span(), temp_cold_k, temp_hot_k);
    isp_.set_bad_pixel_mask(fpa_.dead_pixel_mask());
}

void SimulationPipeline::set_flight_mode(FlightMode mode) {
    current_flight_mode_ = mode;
    switch (mode) {
        case FlightMode::Waypoint:
            drone_.set_flight_controller(waypoint_fc_);
            break;
        case FlightMode::Orbit:
            drone_.set_flight_controller(orbit_fc_);
            break;
        case FlightMode::Manual:
            drone_.set_flight_controller(manual_fc_);
            break;
        case FlightMode::Pursuit:
            drone_.set_flight_controller(orbit_fc_);
            break;
    }
}

void SimulationPipeline::set_manual_sticks(
    float roll_stick,
    float pitch_stick,
    float yaw_rate_stick,
    float throttle_stick
) {
    if (manual_fc_) {
        manual_fc_->set_roll_input(roll_stick);
        manual_fc_->set_pitch_input(pitch_stick);
        manual_fc_->set_yaw_rate(yaw_rate_stick * 0.5f);
        manual_fc_->set_climb_rate(throttle_stick * 5.0f);
    }
}

void SimulationPipeline::set_orbit_params(const core::Vec3& center, float radius_m, float speed_mps) {
    if (orbit_fc_) {
        orbit_fc_->set_center(center);
        orbit_fc_->set_radius(radius_m);
        orbit_fc_->set_speed(speed_mps);
    }
}

void SimulationPipeline::set_gimbal_nadir() {
    gimbal_.set_mode(platform::GimbalMode::Nadir);
}

void SimulationPipeline::set_gimbal_geolock(const core::Vec3& ground_target_m) {
    gimbal_.set_target_location(ground_target_m);
}

void SimulationPipeline::set_gimbal_manual_rates(float pitch_rate_radps, float yaw_rate_radps) {
    gimbal_.set_mode(platform::GimbalMode::ManualSlew);
    gimbal_.set_slew_rates(pitch_rate_radps, yaw_rate_radps);
}

void SimulationPipeline::set_gimbal_sector_scan(
    float az_min_deg,
    float az_max_deg,
    float sweep_period_sec,
    std::vector<float> elevation_bars_deg
) {
    gimbal_.set_sector_scan(az_min_deg, az_max_deg, sweep_period_sec, std::move(elevation_bars_deg));
}

void SimulationPipeline::step(float dt) {
    dt = std::clamp(dt, 0.001f, 0.1f);
    elapsed_sim_time_sec_ += dt;

    // 1. Step Platform Kinematics & Controllers
    if (gimbal_.mode() == platform::GimbalMode::GeoLock && !manifest_.targets.empty()) {
        const core::Vec3 live_target = primary_target_position();
        gimbal_.set_target_location(live_target);
        if (current_flight_mode_ == FlightMode::Orbit && orbit_fc_) {
            const float flight_alt = manifest_.terrain.base_elevation_m + 350.0f;
            orbit_fc_->set_center({live_target.x, live_target.y, flight_alt});
        }
    } else if (current_flight_mode_ == FlightMode::Pursuit && !active_tracks_.empty()) {
        // Pursuit mode: Lock gimbal onto highest confidence track
        const auto& best_track = active_tracks_.front();
        if (best_track.state == detection::TrackState::Confirmed) {
            // Repoint gimbal towards target estimated ground location
            const auto hit_opt = camera_.raycast_pixel_to_ground(
                best_track.bbox.x + best_track.bbox.width * 0.5f,
                best_track.bbox.y + best_track.bbox.height * 0.5f,
                drone_.pose().position_enu_m,
                gimbal_.camera_orientation(),
                manifest_.terrain.base_elevation_m
            );
            if (hit_opt) {
                gimbal_.set_target_location(*hit_opt);
            }
        }
    }

    drone_.step(dt);

    // Multi-sample gimbal kinematics at 400 Hz equivalent (12 sub-samples per frame)
    constexpr int SUBSTEPS = 12;
    const float sub_dt = dt / static_cast<float>(SUBSTEPS);
    for (int s = 0; s < SUBSTEPS; ++s) {
        gimbal_.update(drone_.pose(), sub_dt);

        BeamRecord rec;
        rec.sim_time_sec = elapsed_sim_time_sec_ - dt + static_cast<double>(s + 1) * static_cast<double>(sub_dt);

        const float az = gimbal_.current_yaw_deg();
        const float el = gimbal_.current_pitch_deg();

        if (gimbal_.mode() == platform::GimbalMode::SectorScan) {
            rec.az_deg = az;
            rec.el_deg = el;
        } else {
            // General display: azimuth in [0, 360) and depression elevation in [0, 90]
            rec.az_deg = std::fmod(az + 360.0f, 360.0f);
            rec.el_deg = std::clamp(-el, 0.0f, 90.0f);
        }

        beam_history_.push_back(rec);
        if (beam_history_.size() > 2000) {
            beam_history_.pop_front();
        }
    }

    scene_.update(elapsed_sim_time_sec_);

    // 2. Stage 1: Radiative Transfer & Scene Rendering
    const auto t0 = std::chrono::steady_clock::now();
    scene_.render_at_aperture(drone_, gimbal_, camera_, gt_radiance_, gt_detections_);
    const auto t1 = std::chrono::steady_clock::now();

    // 3. Stage 2: FPA Sensor Degradation (FPN, NETD noise, 14-bit ADC)
    fpa_.process(gt_radiance_.as_span(), raw_fpa_.as_span());
    const auto t2 = std::chrono::steady_clock::now();

    // 4. Stage 3: Embedded ISP Pipeline (2-Point NUC, BPR, Plateau AGC / CLAHE)
    isp_.options().enable_bpr = enable_bpr;
    isp_.correct_nuc_and_bpr(raw_fpa_.as_span(), nuc_frame_.as_span());
    isp_.enhance_dynamic_range(nuc_frame_.as_span(), display_8bit_.as_span());
    const auto t3 = std::chrono::steady_clock::now();

    // 5. Stage 4: Tactical Target Detection (CFAR + LCM)
    detections_ = cfar_.detect(nuc_frame_.as_span());
    if (enable_lcm_filter && detections_.empty()) {
        // Multi-scale LCM fallback for dim point targets or extended targets exceeding CFAR guard cells
        lcm_.compute_saliency_map(nuc_frame_.as_span(), lcm_saliency_, 7);
        float max_saliency = 0.0f;
        size_t max_idx = 0;
        for (size_t i = 0; i < lcm_saliency_.size(); ++i) {
            if (lcm_saliency_[i] > max_saliency) {
                max_saliency = lcm_saliency_[i];
                max_idx = i;
            }
        }
        if (max_saliency > 20.0f) {
            const float cx = static_cast<float>(max_idx % width_);
            const float cy = static_cast<float>(max_idx / width_);
            core::Detection lcm_det{};
            lcm_det.bbox.x = cx - 12.0f;
            lcm_det.bbox.y = cy - 12.0f;
            lcm_det.bbox.width = 24.0f;
            lcm_det.bbox.height = 24.0f;
            lcm_det.scr = max_saliency / 10.0f;
            lcm_det.peak_intensity = static_cast<float>(nuc_frame_(static_cast<size_t>(cx), static_cast<size_t>(cy)));
            detections_.push_back(lcm_det);
        }
    }
    const auto t4 = std::chrono::steady_clock::now();

    // 6. Stage 5: Multi-Target Kalman Tracking
    active_tracks_ = tracker_.update(detections_, dt);

    // Compute ground geodetic GPS coordinates for each track
    for (auto& track : active_tracks_) {
        const float cx = track.bbox.x + track.bbox.width * 0.5f;
        const float cy = track.bbox.y + track.bbox.height * 0.5f;
        const auto hit_opt = camera_.raycast_pixel_to_ground(
            cx, cy,
            drone_.pose().position_enu_m,
            gimbal_.camera_orientation(),
            manifest_.terrain.base_elevation_m
        );
        if (hit_opt) {
            track.estimated_geo = platform::CameraModel::enu_to_geodetic(
                *hit_opt,
                core::GeoCoordinate{manifest_.terrain.origin_lat, manifest_.terrain.origin_lon, manifest_.terrain.base_elevation_m}
            );
        }
    }
    const auto t5 = std::chrono::steady_clock::now();

    // 7. Aggregate Latencies & Telemetry
    telemetry_.sim_time_sec = elapsed_sim_time_sec_;
    telemetry_.drone_pose = drone_.pose();
    telemetry_.gimbal_pose = gimbal_.pose();
    telemetry_.ground_speed_mps = drone_.ground_speed_mps();
    telemetry_.altitude_agl_m = drone_.altitude_agl_m();
    telemetry_.drone_gps = platform::CameraModel::enu_to_geodetic(
        drone_.pose().position_enu_m,
        core::GeoCoordinate{manifest_.terrain.origin_lat, manifest_.terrain.origin_lon, manifest_.terrain.base_elevation_m}
    );

    telemetry_.latency.scene_ms = std::chrono::duration<float, std::milli>(t1 - t0).count();
    telemetry_.latency.fpa_ms = std::chrono::duration<float, std::milli>(t2 - t1).count();
    telemetry_.latency.isp_ms = std::chrono::duration<float, std::milli>(t3 - t2).count();
    telemetry_.latency.cfar_ms = std::chrono::duration<float, std::milli>(t4 - t3).count();
    telemetry_.latency.tracker_ms = std::chrono::duration<float, std::milli>(t5 - t4).count();
    telemetry_.latency.total_ms = std::chrono::duration<float, std::milli>(t5 - t0).count();

    telemetry_.ground_truth_count = gt_detections_.size();
    telemetry_.detection_count = detections_.size();
    telemetry_.confirmed_track_count = tracker_.confirmed_track_count();
}

} // namespace ir_sim::app
