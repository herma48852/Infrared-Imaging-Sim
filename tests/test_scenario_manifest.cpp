#include "test_main.hpp"
#include "ir_sim/core/frame_buffer.hpp"
#include "ir_sim/platform/camera_model.hpp"
#include "ir_sim/platform/drone_platform.hpp"
#include "ir_sim/platform/gimbal.hpp"
#include "ir_sim/scene/scenario_manifest.hpp"
#include "ir_sim/scene/synthetic_scene.hpp"

#include <cmath>

using namespace ir_sim::core;
using namespace ir_sim::scene;
using namespace ir_sim::platform;

TEST_CASE(scenario_json_roundtrip_fidelity) {
    ScenarioManifest m;
    m.scenario_name = "test_recon_flight";
    m.description = "Test scenario for JSON roundtrip verification";
    m.duration_sec = 45.0f;
    m.environment.ambient_temp_k = 293.15f;
    m.environment.solar_irradiance_w_m2 = 450.0f;
    m.environment.visibility_km = 18.0f;
    m.environment.time_of_day_hours = 11.5f;

    m.terrain.terrain_type = "woodland";
    m.terrain.origin_lat = 32.7767;
    m.terrain.origin_lon = -96.7970;
    m.terrain.base_elevation_m = 150.0f;

    TargetDefinition t1;
    t1.id = 42;
    t1.name = "test_truck";
    t1.type = TargetType::MilitaryVehicle;
    t1.dimensions = {6.0f, 2.4f, 2.5f};
    t1.thermal_zones = {
        {"engine", 350.0f, 0.92f, 0.3f},
        {"body", 298.0f, 0.90f, 0.7f}
    };
    t1.waypoints = {
        {0.0f, {10.0f, 20.0f, 150.0f}, 12.0f, 0.0f},
        {10.0f, {130.0f, 20.0f, 150.0f}, 12.0f, 0.0f}
    };
    m.targets.push_back(t1);

    // Serialize to JSON
    const std::string json_str = m.to_json();
    REQUIRE(!json_str.empty());

    // Deserialize back
    const auto loaded = ScenarioManifest::from_json(json_str);
    REQUIRE(loaded.has_value());

    REQUIRE(loaded->scenario_name == "test_recon_flight");
    REQUIRE_NEAR(loaded->duration_sec, 45.0f, 0.01f);
    REQUIRE_NEAR(loaded->environment.ambient_temp_k, 293.15f, 0.01f);
    REQUIRE_NEAR(loaded->environment.solar_irradiance_w_m2, 450.0f, 0.01f);
    REQUIRE_NEAR(loaded->terrain.origin_lat, 32.7767, 0.0001);
    REQUIRE(loaded->terrain.terrain_type == "woodland");

    REQUIRE(loaded->targets.size() == 1);
    const auto& lt = loaded->targets[0];
    REQUIRE(lt.id == 42);
    REQUIRE(lt.name == "test_truck");
    REQUIRE(lt.type == TargetType::MilitaryVehicle);
    REQUIRE_NEAR(lt.dimensions.x, 6.0f, 0.01f);
    REQUIRE(lt.thermal_zones.size() == 2);
    REQUIRE_NEAR(lt.thermal_zones[0].temp_k, 350.0f, 0.01f);
    REQUIRE(lt.waypoints.size() == 2);
    REQUIRE_NEAR(lt.waypoints[1].position.x, 130.0f, 0.01f);
}

TEST_CASE(target_kinematic_trajectory_interpolation) {
    TargetDefinition target;
    target.id = 1;
    target.name = "drone";
    target.type = TargetType::DroneQuadcopter;
    target.thermal_zones = {
        {"motors", 330.0f, 0.85f, 0.5f},
        {"frame", 290.0f, 0.85f, 0.5f}
    };

    target.waypoints = {
        {0.0f,  {0.0f,   0.0f, 100.0f}, 10.0f, 0.0f},
        {10.0f, {100.0f, 0.0f, 100.0f}, 10.0f, 0.0f},
        {20.0f, {100.0f, 100.0f, 120.0f}, 10.0f, 90.0f}
    };

    // Composite temperature: 0.5 * 330 + 0.5 * 290 = 310 K
    const auto s0 = target.evaluate_at_time(0.0f);
    REQUIRE_NEAR(s0.composite_temp_k, 310.0f, 0.1f);
    REQUIRE_NEAR(s0.position.x, 0.0f, 0.01f);

    // Halfway through segment 1: t = 5.0s -> x = 50m
    const auto s5 = target.evaluate_at_time(5.0f);
    REQUIRE_NEAR(s5.position.x, 50.0f, 0.01f);
    REQUIRE_NEAR(s5.position.y, 0.0f, 0.01f);
    REQUIRE_NEAR(s5.position.z, 100.0f, 0.01f);
    REQUIRE_NEAR(s5.velocity.x, 10.0f, 0.01f);
    REQUIRE_NEAR(s5.velocity.y, 0.0f, 0.01f);

    // Halfway through segment 2: t = 15.0s -> x = 100m, y = 50m, z = 110m
    const auto s15 = target.evaluate_at_time(15.0f);
    REQUIRE_NEAR(s15.position.x, 100.0f, 0.01f);
    REQUIRE_NEAR(s15.position.y, 50.0f, 0.01f);
    REQUIRE_NEAR(s15.position.z, 110.0f, 0.01f);
    REQUIRE_NEAR(s15.velocity.y, 10.0f, 0.01f);

    // Clamping beyond endpoints
    const auto s_past = target.evaluate_at_time(30.0f);
    REQUIRE_NEAR(s_past.position.x, 100.0f, 0.01f);
    REQUIRE_NEAR(s_past.position.y, 100.0f, 0.01f);
}

TEST_CASE(synthetic_scene_subpixel_psf_radiant_flux_conservation) {
    ScenarioManifest manifest;
    manifest.scenario_name = "subpixel_test";
    manifest.environment.ambient_temp_k = 288.15f;
    manifest.environment.visibility_km = 20.0f;
    manifest.terrain.base_elevation_m = 0.0f;
    manifest.terrain.terrain_type = "desert";

    // Sub-pixel bird target at (0, 600, 0)
    TargetDefinition bird;
    bird.id = 99;
    bird.name = "subpixel_bird";
    bird.type = TargetType::Bird;
    bird.dimensions = {0.25f, 0.25f, 0.25f}; // 25 cm target
    bird.thermal_zones = {{"body", 318.0f, 0.95f, 1.0f}};
    bird.waypoints = {{0.0f, {0.0f, 600.0f, 0.0f}, 0.0f, 0.0f}};
    manifest.targets.push_back(bird);

    SyntheticScene scene(manifest);

    // Place drone directly above bird at (0, 600m, 500m)
    DronePlatform drone;
    drone.pose().position_enu_m = {0.0f, 600.0f, 500.0f};

    Gimbal gimbal;
    gimbal.set_mode(GimbalMode::Nadir);
    gimbal.set_jitter_enabled(false);
    for (int i = 0; i < 50; ++i) gimbal.update(drone.pose(), 0.01f);

    CameraSpecs specs;
    specs.width_pixels = 64;
    specs.height_pixels = 64;
    specs.focal_length_m = 0.050f;
    specs.pixel_pitch_m = 12.0e-6f;
    CameraModel camera(specs);

    RadianceFrame radiance(64, 64);
    std::vector<Detection> ground_truth;

    scene.render_at_aperture(drone, gimbal, camera, radiance, ground_truth);

    // Target must be in ground truth
    REQUIRE(ground_truth.size() == 1);
    REQUIRE(ground_truth[0].scr > 0.0f);

    // Target center is at optical center (32, 32)
    const uint32_t cx = 32;
    const uint32_t cy = 32;

    const float center_radiance = radiance(cy, cx);
    const float background_radiance = radiance(10, 10);

    // Center radiance must exhibit positive thermal contrast
    REQUIRE(center_radiance > background_radiance);

    // Energy conservation check across 3x3 PSF neighborhood:
    // Total integrated contrast in 3x3 window should exceed 0
    float integrated_contrast = 0.0f;
    for (int dy = -1; dy <= 1; ++dy) {
        for (int dx = -1; dx <= 1; ++dx) {
            const auto py = static_cast<uint32_t>(static_cast<int>(cy) + dy);
            const auto px = static_cast<uint32_t>(static_cast<int>(cx) + dx);
            integrated_contrast += (radiance(py, px) - background_radiance);
        }
    }

    REQUIRE(integrated_contrast > 0.01f);
}
