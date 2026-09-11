#include "test_main.hpp"
#include "ir_sim/platform/gimbal.hpp"

using namespace ir_sim::core;
using namespace ir_sim::platform;

TEST_CASE(gimbal_nadir_pointing) {
    Gimbal gimbal;
    DronePose drone;
    drone.position_enu_m = {0.0f, 0.0f, 100.0f};

    gimbal.set_mode(GimbalMode::Nadir);
    gimbal.set_jitter_enabled(false);

    // Update for 1 second to settle
    for (int i = 0; i < 100; ++i) {
        gimbal.update(drone, 0.01f);
    }

    const Vec3 los = gimbal.line_of_sight();
    // Nadir look vector should point straight down [0, 0, -1]
    REQUIRE_NEAR(los.x, 0.0f, 1e-4);
    REQUIRE_NEAR(los.y, 0.0f, 1e-4);
    REQUIRE_NEAR(los.z, -1.0f, 1e-4);
}

TEST_CASE(gimbal_geolock_tracking) {
    Gimbal gimbal;
    DronePose drone;
    drone.position_enu_m = {0.0f, 0.0f, 100.0f}; // Drone at (0, 0, 100)

    const Vec3 target{100.0f, 0.0f, 0.0f}; // Target 100m East on ground
    gimbal.set_target_location(target);
    gimbal.set_jitter_enabled(false);

    // Let gimbal slew toward target for 2 seconds
    for (int i = 0; i < 200; ++i) {
        gimbal.update(drone, 0.01f);
    }

    const Vec3 los = gimbal.line_of_sight();
    // Expected unit vector: (target - drone) / dist = [100, 0, -100] / sqrt(20000) = [0.7071, 0, -0.7071]
    REQUIRE_NEAR(los.x, 0.7071f, 0.02);
    REQUIRE_NEAR(los.y, 0.0f, 0.02);
    REQUIRE_NEAR(los.z, -0.7071f, 0.02);
}

TEST_CASE(gimbal_vibration_jitter) {
    Gimbal gimbal;
    DronePose drone;
    drone.position_enu_m = {0.0f, 0.0f, 100.0f};

    gimbal.set_mode(GimbalMode::Nadir);
    gimbal.set_jitter_enabled(true);
    gimbal.set_jitter_amplitude_mrad(0.20f); // 0.2 mrad

    float max_jitter_p = 0.0f;
    for (int i = 0; i < 200; ++i) {
        gimbal.update(drone, 0.01f);
        const float j = std::abs(gimbal.pose().jitter_offset_rad.x);
        if (j > max_jitter_p) max_jitter_p = j;
    }

    // Peak jitter should be positive and bounded by amplitude
    REQUIRE(max_jitter_p > 0.0f);
    REQUIRE(max_jitter_p <= 0.25e-3f);
}

TEST_CASE(gimbal_sector_scan) {
    Gimbal gimbal;
    DronePose drone;
    drone.position_enu_m = {0.0f, 0.0f, 100.0f};

    gimbal.set_jitter_enabled(false);
    gimbal.set_sector_scan(0.0f, 90.0f, 0.45f, {25.0f, 3.0f, 14.0f});

    REQUIRE(gimbal.mode() == GimbalMode::SectorScan);

    // Initial update
    gimbal.update(drone, 0.001f);
    REQUIRE_NEAR(gimbal.current_yaw_deg(), 0.0f, 1.0);
    REQUIRE_NEAR(gimbal.current_pitch_deg(), 25.0f, 1e-3);

    // Halfway through bar 0 (t = 0.225s): az should be near 45 deg, el = 25 deg
    for (int i = 0; i < 22; ++i) {
        gimbal.update(drone, 0.01f);
    }
    REQUIRE_NEAR(gimbal.current_yaw_deg(), 45.0f, 3.0);
    REQUIRE_NEAR(gimbal.current_pitch_deg(), 25.0f, 1e-3);

    // Advance to bar 1 (t = 0.50s): az should reset and el should step to 3.0 deg
    for (int i = 0; i < 28; ++i) {
        gimbal.update(drone, 0.01f);
    }
    REQUIRE_NEAR(gimbal.current_pitch_deg(), 3.0f, 1e-3);

    // Advance to bar 2 (t = 0.95s): el should step to 14.0 deg
    for (int i = 0; i < 45; ++i) {
        gimbal.update(drone, 0.01f);
    }
    REQUIRE_NEAR(gimbal.current_pitch_deg(), 14.0f, 1e-3);
}
