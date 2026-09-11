#include "test_main.hpp"
#include "ir_sim/platform/camera_model.hpp"
#include "ir_sim/platform/gimbal.hpp"

using namespace ir_sim::core;
using namespace ir_sim::platform;

TEST_CASE(camera_optical_metrics) {
    CameraSpecs specs;
    specs.width_pixels = 640;
    specs.height_pixels = 512;
    specs.pixel_pitch_m = 12.0e-6f;   // 12 um
    specs.focal_length_m = 0.050f;     // 50 mm

    CameraModel cam(specs);

    // IFOV = 12e-6 / 0.050 = 0.00024 rad = 0.24 mrad
    REQUIRE_NEAR(cam.ifov_mrad(), 0.24f, 1e-4);

    // GSD at 100m range: 100 * 0.24e-3 = 0.024m = 2.4 cm/pixel
    REQUIRE_NEAR(cam.gsd_m(100.0f), 0.024f, 1e-4);

    // GSD at 500m range: 500 * 0.24e-3 = 0.12m = 12 cm/pixel (bird/drone scale!)
    REQUIRE_NEAR(cam.gsd_m(500.0f), 0.12f, 1e-4);
}

TEST_CASE(camera_nadir_projection_and_raycast_roundtrip) {
    CameraModel cam;
    Gimbal gimbal;
    DronePose drone;
    drone.position_enu_m = {0.0f, 0.0f, 100.0f};

    gimbal.set_mode(GimbalMode::Nadir);
    gimbal.set_jitter_enabled(false);
    for (int i = 0; i < 50; ++i) gimbal.update(drone, 0.01f);

    const Quat cam_quat = gimbal.camera_orientation();

    // 1. Target directly below drone on ground (0, 0, 0)
    const Vec3 target_sub_drone{0.0f, 0.0f, 0.0f};
    const auto proj_center = cam.project_world_to_pixel(target_sub_drone, drone.position_enu_m, cam_quat);
    REQUIRE(proj_center.has_value());

    // Should project directly onto principal point (center of image: 319.5, 255.5)
    REQUIRE_NEAR(proj_center->x, 319.5f, 0.5f);
    REQUIRE_NEAR(proj_center->y, 255.5f, 0.5f);
    REQUIRE_NEAR(proj_center->z, 100.0f, 0.1f); // Slant range = 100m

    // 2. Off-center target: 5 meters East, 5 meters North
    const Vec3 target_offset{5.0f, 5.0f, 0.0f};
    const auto proj_offset = cam.project_world_to_pixel(target_offset, drone.position_enu_m, cam_quat);
    REQUIRE(proj_offset.has_value());

    // 3. Raycast back from pixel to ground: must recover target_offset (5, 5, 0)
    const auto ray_ground = cam.raycast_pixel_to_ground(
        proj_offset->x, proj_offset->y, drone.position_enu_m, cam_quat, 0.0f);
    REQUIRE(ray_ground.has_value());

    REQUIRE_NEAR(ray_ground->x, target_offset.x, 0.05f);
    REQUIRE_NEAR(ray_ground->y, target_offset.y, 0.05f);
    REQUIRE_NEAR(ray_ground->z, 0.0f, 0.01f);
}

TEST_CASE(camera_geodetic_gps_translation) {
    GeoCoordinate datum{37.7749, -122.4194, 10.0}; // San Francisco datum
    Vec3 enu_offset{100.0f, 200.0f, 50.0f};       // 100m East, 200m North, 50m Up

    const GeoCoordinate gps = CameraModel::enu_to_geodetic(enu_offset, datum);

    REQUIRE(gps.latitude_deg > datum.latitude_deg);   // North increases latitude
    REQUIRE(gps.longitude_deg > datum.longitude_deg); // East increases longitude
    REQUIRE_NEAR(gps.altitude_msl_m, 60.0, 0.01);
}
