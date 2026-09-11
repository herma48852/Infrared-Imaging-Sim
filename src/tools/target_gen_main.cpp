#include "ir_sim/scene/scenario_manifest.hpp"

#include <filesystem>
#include <iostream>
#include <string>
#include <string_view>

using namespace ir_sim::scene;

namespace {

ScenarioManifest create_desert_convoy(float duration = 60.0f, float ambient_k = 303.15f) {
    ScenarioManifest m;
    m.scenario_name = "desert_convoy_day";
    m.description = "Tactical military supply convoy traversing desert highway in high ambient temperature";
    m.duration_sec = duration;

    m.environment.ambient_temp_k = ambient_k; // 30°C desert day
    m.environment.solar_irradiance_w_m2 = 850.0f; // High midday sun
    m.environment.visibility_km = 20.0f;
    m.environment.wind_vector_mps = {4.0f, 2.0f, 0.0f};
    m.environment.time_of_day_hours = 13.5f;

    m.terrain.terrain_type = "desert";
    m.terrain.size_x_m = 3000.0f;
    m.terrain.size_y_m = 3000.0f;
    m.terrain.base_elevation_m = 120.0f;

    // Target 1: Lead Military 6x6 Heavy Truck
    {
        TargetDefinition t;
        t.id = 101;
        t.name = "lead_truck_6x6";
        t.type = TargetType::MilitaryVehicle;
        t.dimensions = {7.5f, 2.5f, 2.8f};
        t.thermal_zones = {
            {"hood_engine", 348.0f, 0.92f, 0.25f},
            {"exhaust_pipe", 475.0f, 0.88f, 0.08f},
            {"tires", 322.0f, 0.94f, 0.22f},
            {"chassis_painted", 308.0f, 0.91f, 0.45f}
        };
        // Driving along road at y=200m from x=0 to x=900m (15 m/s = 54 km/h)
        t.waypoints = {
            {0.0f,  {50.0f, 200.0f, 120.0f}, 15.0f, 90.0f},
            {30.0f, {500.0f, 200.0f, 120.0f}, 15.0f, 90.0f},
            {60.0f, {950.0f, 200.0f, 120.0f}, 15.0f, 90.0f}
        };
        m.targets.push_back(t);
    }

    // Target 2: Trailing Supply Truck (70m trailing distance)
    {
        TargetDefinition t;
        t.id = 102;
        t.name = "supply_truck_2";
        t.type = TargetType::MilitaryVehicle;
        t.dimensions = {7.0f, 2.5f, 2.7f};
        t.thermal_zones = {
            {"hood_engine", 342.0f, 0.92f, 0.25f},
            {"exhaust_pipe", 460.0f, 0.88f, 0.08f},
            {"tires", 320.0f, 0.94f, 0.22f},
            {"chassis_painted", 307.0f, 0.91f, 0.45f}
        };
        t.waypoints = {
            {0.0f,  {-20.0f, 200.0f, 120.0f}, 15.0f, 90.0f},
            {30.0f, {430.0f, 200.0f, 120.0f}, 15.0f, 90.0f},
            {60.0f, {880.0f, 200.0f, 120.0f}, 15.0f, 90.0f}
        };
        m.targets.push_back(t);
    }

    // Target 3: Fast Escort Technical Vehicle
    {
        TargetDefinition t;
        t.id = 103;
        t.name = "escort_technical";
        t.type = TargetType::CivilianVehicle;
        t.dimensions = {4.8f, 1.9f, 1.8f};
        t.thermal_zones = {
            {"hood_engine", 355.0f, 0.93f, 0.30f},
            {"exhaust", 490.0f, 0.89f, 0.10f},
            {"tires", 325.0f, 0.94f, 0.25f},
            {"body", 310.0f, 0.92f, 0.35f}
        };
        t.waypoints = {
            {0.0f,  {140.0f, 200.0f, 120.0f}, 18.0f, 90.0f},
            {30.0f, {680.0f, 200.0f, 120.0f}, 18.0f, 90.0f},
            {60.0f, {1220.0f, 200.0f, 120.0f}, 18.0f, 90.0f}
        };
        m.targets.push_back(t);
    }

    return m;
}

ScenarioManifest create_woodland_birds(float duration = 60.0f, float ambient_k = 285.15f) {
    ScenarioManifest m;
    m.scenario_name = "woodland_bird_swarm";
    m.description = "Sub-pixel point targets: flock of wild birds flying over temperate forest canopy";
    m.duration_sec = duration;

    m.environment.ambient_temp_k = ambient_k; // 12°C morning
    m.environment.solar_irradiance_w_m2 = 120.0f;
    m.environment.visibility_km = 12.0f;
    m.environment.wind_vector_mps = {3.0f, 1.5f, 0.0f};
    m.environment.time_of_day_hours = 8.5f;

    m.terrain.terrain_type = "woodland";
    m.terrain.size_x_m = 2000.0f;
    m.terrain.size_y_m = 2000.0f;
    m.terrain.base_elevation_m = 50.0f;

    // Generate 4 birds flying at 40-70m altitude with 12-16 m/s flight speed
    // At 600m - 1000m drone slant range, these subtend ~0.3 - 0.7 pixels!
    const float offsets[4][2] = {
        {0.0f, 0.0f}, {15.0f, -10.0f}, {-12.0f, 20.0f}, {25.0f, 15.0f}
    };

    for (int i = 0; i < 4; ++i) {
        TargetDefinition t;
        t.id = static_cast<uint32_t>(201 + i);
        t.name = "bird_" + std::to_string(i + 1);
        t.type = TargetType::Bird;
        t.dimensions = {0.35f, 0.65f, 0.20f}; // 65cm wingspan
        t.thermal_zones = {
            {"plumage_body", 314.5f, 0.95f, 0.85f}, // Avian core body temp ~41°C
            {"head_beak", 310.0f, 0.92f, 0.15f}
        };

        const float ox = offsets[i][0];
        const float oy = offsets[i][1];
        const float alt = 90.0f + static_cast<float>(i) * 5.0f;

        t.waypoints = {
            {0.0f,  {200.0f + ox, 300.0f + oy, alt}, 14.0f, 45.0f},
            {30.0f, {500.0f + ox, 600.0f + oy, alt + 10.0f}, 14.0f, 45.0f},
            {60.0f, {800.0f + ox, 900.0f + oy, alt + 5.0f}, 14.0f, 45.0f}
        };
        m.targets.push_back(t);
    }

    return m;
}

ScenarioManifest create_micro_drone_incursion(float duration = 60.0f, float ambient_k = 289.15f) {
    ScenarioManifest m;
    m.scenario_name = "micro_drone_incursion";
    m.description = "Tactical micro-UAV quadcopters hovering and performing low-altitude reconnaissance";
    m.duration_sec = duration;

    m.environment.ambient_temp_k = ambient_k; // 16°C dusk
    m.environment.solar_irradiance_w_m2 = 60.0f;
    m.environment.visibility_km = 16.0f;
    m.environment.wind_vector_mps = {1.5f, 0.5f, 0.0f};
    m.environment.time_of_day_hours = 18.0f;

    m.terrain.terrain_type = "urban";
    m.terrain.size_x_m = 2000.0f;
    m.terrain.size_y_m = 2000.0f;
    m.terrain.base_elevation_m = 80.0f;

    // Drone 1: Quadcopter loitering near perimeter
    {
        TargetDefinition t;
        t.id = 301;
        t.name = "recon_quadcopter_1";
        t.type = TargetType::DroneQuadcopter;
        t.dimensions = {0.30f, 0.30f, 0.15f}; // 30cm sub-pixel target
        t.thermal_zones = {
            {"electric_motors", 328.0f, 0.85f, 0.25f}, // Hot brushless motors
            {"battery_pack", 318.0f, 0.90f, 0.35f},    // Warm LiPo
            {"carbon_frame", 295.0f, 0.82f, 0.40f}
        };
        t.waypoints = {
            {0.0f,  {350.0f, 250.0f, 110.0f}, 5.0f, 120.0f},
            {20.0f, {410.0f, 310.0f, 115.0f}, 4.0f, 90.0f},
            {40.0f, {430.0f, 350.0f, 112.0f}, 3.0f, 45.0f},
            {60.0f, {480.0f, 390.0f, 118.0f}, 5.0f, 45.0f}
        };
        m.targets.push_back(t);
    }

    // Drone 2: High-speed sprint micro-drone
    {
        TargetDefinition t;
        t.id = 302;
        t.name = "sprint_micro_uav";
        t.type = TargetType::DroneQuadcopter;
        t.dimensions = {0.25f, 0.25f, 0.12f};
        t.thermal_zones = {
            {"electric_motors", 335.0f, 0.85f, 0.30f},
            {"battery_pack", 322.0f, 0.90f, 0.30f},
            {"frame", 294.0f, 0.82f, 0.40f}
        };
        t.waypoints = {
            {0.0f,  {200.0f, 100.0f, 95.0f}, 18.0f, 60.0f},
            {30.0f, {550.0f, 400.0f, 105.0f}, 18.0f, 60.0f},
            {60.0f, {900.0f, 700.0f, 115.0f}, 18.0f, 60.0f}
        };
        m.targets.push_back(t);
    }

    return m;
}

void print_usage(const char* prog) {
    std::cout << "Usage: " << prog << " [options]\n\n"
              << "Options:\n"
              << "  --preset <convoy|birds|drones>  Select built-in scenario preset\n"
              << "  --out <filepath>               Output scenario JSON file path\n"
              << "  --duration <seconds>           Scenario duration in seconds (default: 60)\n"
              << "  --ambient <kelvin>             Ambient temperature in Kelvin\n"
              << "  --all                          Generate all standard presets into scenarios/\n"
              << "  --help                         Show this help message\n";
}

} // namespace

int main(int argc, char* argv[]) {
    std::string preset = "convoy";
    std::string out_path = "";
    float duration = 60.0f;
    float ambient_k = -1.0f;
    bool gen_all = (argc == 1); // Default without args generates all

    for (int i = 1; i < argc; ++i) {
        std::string_view arg = argv[i];
        if (arg == "--help" || arg == "-h") {
            print_usage(argv[0]);
            return 0;
        } else if (arg == "--all") {
            gen_all = true;
        } else if (arg == "--preset" && i + 1 < argc) {
            preset = argv[++i];
        } else if (arg == "--out" && i + 1 < argc) {
            out_path = argv[++i];
        } else if (arg == "--duration" && i + 1 < argc) {
            duration = std::stof(argv[++i]);
        } else if (arg == "--ambient" && i + 1 < argc) {
            ambient_k = std::stof(argv[++i]);
        }
    }

    std::filesystem::create_directories("scenarios");

    if (gen_all) {
        std::cout << "[target_gen] Generating all standard tactical scenarios...\n";

        auto s1 = create_desert_convoy(60.0f);
        if (!s1.save_to_file("scenarios/desert_convoy_day.json")) {
            std::cerr << "Warning: Failed to save desert convoy scenario\n";
        }
        std::cout << "  -> Generated: scenarios/desert_convoy_day.json (" << s1.targets.size() << " targets)\n";

        auto s2 = create_woodland_birds(60.0f);
        if (!s2.save_to_file("scenarios/woodland_bird_swarm.json")) {
            std::cerr << "Warning: Failed to save woodland birds scenario\n";
        }
        std::cout << "  -> Generated: scenarios/woodland_bird_swarm.json (" << s2.targets.size() << " point targets)\n";

        auto s3 = create_micro_drone_incursion(60.0f);
        if (!s3.save_to_file("scenarios/micro_drone_incursion.json")) {
            std::cerr << "Warning: Failed to save micro drone scenario\n";
        }
        std::cout << "  -> Generated: scenarios/micro_drone_incursion.json (" << s3.targets.size() << " micro-drones)\n";

        std::cout << "[target_gen] Successfully generated all 3 benchmark scenarios.\n";
        return 0;
    }

    ScenarioManifest manifest;
    if (preset == "convoy") {
        manifest = create_desert_convoy(duration, (ambient_k > 0.0f) ? ambient_k : 303.15f);
    } else if (preset == "birds") {
        manifest = create_woodland_birds(duration, (ambient_k > 0.0f) ? ambient_k : 285.15f);
    } else if (preset == "drones") {
        manifest = create_micro_drone_incursion(duration, (ambient_k > 0.0f) ? ambient_k : 289.15f);
    } else {
        std::cerr << "Error: Unknown preset '" << preset << "'. Choose convoy, birds, or drones.\n";
        return 1;
    }

    if (out_path.empty()) {
        out_path = "scenarios/" + manifest.scenario_name + ".json";
    }

    if (!manifest.save_to_file(out_path)) {
        std::cerr << "Error: Failed to write scenario to " << out_path << "\n";
        return 1;
    }

    std::cout << "[target_gen] Scenario successfully written to: " << out_path << "\n"
              << "  Name:     " << manifest.scenario_name << "\n"
              << "  Duration: " << manifest.duration_sec << " s\n"
              << "  Ambient:  " << manifest.environment.ambient_temp_k << " K\n"
              << "  Targets:  " << manifest.targets.size() << "\n";

    return 0;
}
