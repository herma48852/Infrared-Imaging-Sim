#pragma once

#include "ir_sim/core/types.hpp"

#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace ir_sim::scene {

enum class TargetType {
    MilitaryVehicle,
    CivilianVehicle,
    Bird,
    DroneQuadcopter,
    Personnel
};

[[nodiscard]] std::string to_string(TargetType type);
[[nodiscard]] TargetType target_type_from_string(std::string_view str);

struct ThermalZone {
    std::string name{"body"};
    float temp_k{295.0f};
    float emissivity{0.90f};
    float relative_area{1.0f}; // Fraction of visible surface area (sum to ~1.0)
};

struct Waypoint {
    float time_sec{0.0f};
    core::Vec3 position{0.0f, 0.0f, 0.0f}; // World coordinates [m] (X=East, Y=North, Z=Up)
    float speed_mps{0.0f};
    float yaw_deg{0.0f};
};

struct TargetKinematicState {
    core::Vec3 position{0.0f, 0.0f, 0.0f};
    core::Vec3 velocity{0.0f, 0.0f, 0.0f};
    float yaw_deg{0.0f};
    float composite_temp_k{295.0f};
    float composite_emissivity{0.90f};
};

struct TargetDefinition {
    uint32_t id{1};
    std::string name{"target_1"};
    TargetType type{TargetType::MilitaryVehicle};
    core::Vec3 dimensions{4.5f, 2.2f, 2.0f}; // [length, width, height] in meters
    std::vector<ThermalZone> thermal_zones;
    std::vector<Waypoint> waypoints;

    [[nodiscard]] TargetKinematicState evaluate_at_time(float time_sec) const;
};

struct EnvironmentParams {
    float ambient_temp_k{288.15f};       // 15°C
    float solar_irradiance_w_m2{200.0f}; // Solar flux
    float visibility_km{15.0f};          // MODTRAN aerosol visibility
    core::Vec3 wind_vector_mps{2.0f, 1.0f, 0.0f};
    float time_of_day_hours{14.0f};      // 14:00 (2 PM)
};

struct TerrainParams {
    double origin_lat{34.0522};          // Reference GPS WGS84
    double origin_lon{-118.2437};
    float size_x_m{2000.0f};             // 2 km x 2 km AOI
    float size_y_m{2000.0f};
    float base_elevation_m{100.0f};
    std::string terrain_type{"desert"};  // "desert", "woodland", "urban"
};

struct ScenarioManifest {
    std::string scenario_name{"tactical_scenario"};
    std::string description{"IR drone surveillance scenario"};
    float duration_sec{60.0f};
    EnvironmentParams environment{};
    TerrainParams terrain{};
    std::vector<TargetDefinition> targets;

    [[nodiscard]] std::string to_json() const;
    [[nodiscard]] bool save_to_file(const std::string& filepath) const;

    [[nodiscard]] static std::optional<ScenarioManifest> from_json(std::string_view json_str);
    [[nodiscard]] static std::optional<ScenarioManifest> load_from_file(const std::string& filepath);
};

} // namespace ir_sim::scene
