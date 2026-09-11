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
