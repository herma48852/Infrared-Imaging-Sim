#include "test_main.hpp"
#include "ir_sim/platform/drone_platform.hpp"
#include "ir_sim/platform/flight_controller.hpp"

using namespace ir_sim::core;
using namespace ir_sim::platform;

TEST_CASE(drone_rk4_hover_stability) {
    DronePlatform drone;
    drone.pose().position_enu_m = {0.0f, 0.0f, 100.0f};
    drone.pose().velocity_mps = {0.0f, 0.0f, 0.0f};

    // Step for 2 seconds with zero command
    for (int i = 0; i < 200; ++i) {
        drone.step(0.01f);
    }

    // Altitude should remain near 100m
    REQUIRE_NEAR(drone.altitude_agl_m(), 100.0f, 0.01f);
    REQUIRE(drone.ground_speed_mps() < 0.01f);
}

TEST_CASE(drone_waypoint_controller) {
    DronePlatform drone;
    drone.pose().position_enu_m = {0.0f, 0.0f, 50.0f};

    std::vector<Vec3> waypoints = {
        {50.0f, 0.0f, 50.0f},
        {50.0f, 50.0f, 50.0f},
        {0.0f, 0.0f, 50.0f}
    };

    auto wp_controller = std::make_shared<WaypointFlightController>(waypoints, 10.0f, 5.0f, false);
    drone.set_flight_controller(wp_controller);

    REQUIRE(wp_controller->current_waypoint_index() == 0);

    // Fly toward waypoint 0 for 6 seconds (distance 50m at 10 m/s)
    for (int i = 0; i < 600; ++i) {
        drone.step(0.01f);
    }

    // Drone should have reached waypoint 0 and transitioned to waypoint 1
    REQUIRE(wp_controller->current_waypoint_index() >= 1);
    REQUIRE(drone.pose().position_enu_m.x > 35.0f);
}

TEST_CASE(drone_orbit_controller) {
    DronePlatform drone;
    drone.pose().position_enu_m = {150.0f, 0.0f, 80.0f}; // Start on orbit circumference

    const Vec3 center{0.0f, 0.0f, 0.0f};
    const float radius = 150.0f;
    const float alt = 80.0f;

    auto orbit_controller = std::make_shared<OrbitFlightController>(center, radius, alt, 12.0f, true);
    drone.set_flight_controller(orbit_controller);

    // Fly orbit for 5 seconds
    for (int i = 0; i < 500; ++i) {
        drone.step(0.01f);
    }

    // Verify distance from center remains close to 150m
    const float current_r = std::sqrt(drone.pose().position_enu_m.x * drone.pose().position_enu_m.x +
                                      drone.pose().position_enu_m.y * drone.pose().position_enu_m.y);
    REQUIRE_NEAR(current_r, radius, 10.0f);
    REQUIRE_NEAR(drone.altitude_agl_m(), alt, 5.0f);
}
