#include "ir_sim/physics/materials.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>

namespace ir_sim::physics {

MaterialDatabase::MaterialDatabase() {
    default_material_ = MaterialProperties{
        .name = "default_soil",
        .emissivity_lwir = 0.92f,
        .emissivity_mwir = 0.85f,
        .solar_absorptance = 0.75f,
        .thermal_inertia = 1200.0f,
        .base_temperature_k = 293.15f
    };
    populate_default_materials();
}

void MaterialDatabase::populate_default_materials() {
    // 1. Asphalt (Roadways)
    register_material({
        .name = "asphalt",
        .emissivity_lwir = 0.95f,
        .emissivity_mwir = 0.90f,
        .solar_absorptance = 0.92f,
        .thermal_inertia = 1600.0f,
        .base_temperature_k = 293.15f
    });

    // 2. Concrete (Runways, Buildings)
    register_material({
        .name = "concrete",
        .emissivity_lwir = 0.92f,
        .emissivity_mwir = 0.88f,
        .solar_absorptance = 0.65f,
        .thermal_inertia = 1800.0f,
        .base_temperature_k = 293.15f
    });

    // 3. Dry Soil / Dirt
    register_material({
        .name = "dry_soil",
        .emissivity_lwir = 0.92f,
        .emissivity_mwir = 0.82f,
        .solar_absorptance = 0.78f,
        .thermal_inertia = 1000.0f,
        .base_temperature_k = 293.15f
    });

    // 4. Grass / Vegetation
    register_material({
        .name = "grass",
        .emissivity_lwir = 0.98f,
        .emissivity_mwir = 0.95f,
        .solar_absorptance = 0.80f,
        .thermal_inertia = 1400.0f,
        .base_temperature_k = 291.15f
    });

    // 5. Water (Lakes, Rivers, Ocean)
    register_material({
        .name = "water",
        .emissivity_lwir = 0.98f,
        .emissivity_mwir = 0.96f,
        .solar_absorptance = 0.94f,
        .thermal_inertia = 4200.0f,
        .base_temperature_k = 289.15f
    });

    // 6. Painted Military Steel (CARC paint / Armor)
    register_material({
        .name = "painted_steel",
        .emissivity_lwir = 0.91f,
        .emissivity_mwir = 0.87f,
        .solar_absorptance = 0.88f,
        .thermal_inertia = 2200.0f,
        .base_temperature_k = 293.15f
    });

    // 7. Bare Polished Aluminum (Low emissivity, high thermal specular reflection)
    register_material({
        .name = "bare_aluminum",
        .emissivity_lwir = 0.12f,
        .emissivity_mwir = 0.08f,
        .solar_absorptance = 0.25f,
        .thermal_inertia = 2400.0f,
        .base_temperature_k = 293.15f
    });

    // 8. Rubber (Vehicle Tires)
    register_material({
        .name = "rubber_tire",
        .emissivity_lwir = 0.94f,
        .emissivity_mwir = 0.90f,
        .solar_absorptance = 0.92f,
        .thermal_inertia = 1100.0f,
        .base_temperature_k = 300.15f // Warm from friction
    });

    // 9. Vehicle Glass (Windshield)
    register_material({
        .name = "vehicle_glass",
        .emissivity_lwir = 0.86f,
        .emissivity_mwir = 0.80f,
        .solar_absorptance = 0.15f,
        .thermal_inertia = 1500.0f,
        .base_temperature_k = 292.15f
    });

    // 10. Human Skin / Clothing
    register_material({
        .name = "human_skin",
        .emissivity_lwir = 0.98f,
        .emissivity_mwir = 0.97f,
        .solar_absorptance = 0.65f,
        .thermal_inertia = 1000.0f,
        .base_temperature_k = 305.15f // 32 deg C surface
    });

    // 11. Bird Plumage / Feathers (Core body 41 C)
    register_material({
        .name = "bird_plumage",
        .emissivity_lwir = 0.95f,
        .emissivity_mwir = 0.93f,
        .solar_absorptance = 0.70f,
        .thermal_inertia = 800.0f,
        .base_temperature_k = 312.15f // ~39 C surface with feather insulation
    });

    // 12. Carbon Fiber Composite (Drone Frame)
    register_material({
        .name = "carbon_fiber",
        .emissivity_lwir = 0.92f,
        .emissivity_mwir = 0.88f,
        .solar_absorptance = 0.85f,
        .thermal_inertia = 1300.0f,
        .base_temperature_k = 293.15f
    });
}

const MaterialProperties& MaterialDatabase::get(std::string_view name) const {
    const auto it = materials_.find(std::string(name));
    if (it != materials_.end()) {
        return it->second;
    }
    return default_material_;
}

bool MaterialDatabase::contains(std::string_view name) const noexcept {
    return materials_.contains(std::string(name));
}

void MaterialDatabase::register_material(MaterialProperties mat) {
    materials_[mat.name] = std::move(mat);
}

float MaterialDatabase::compute_diurnal_temperature(const MaterialProperties& mat,
                                                    float time_of_day_hours,
                                                    float ambient_temp_k,
                                                    float solar_irradiance_w_m2) noexcept {
    // Diurnal cycle: temperature peaks around 14:00 (2 PM) due to thermal lag
    constexpr float peak_hour = 14.0f;
    const float omega = (2.0f * static_cast<float>(std::numbers::pi)) / 24.0f;
    const float phase_cos = std::cos(omega * (time_of_day_hours - peak_hour));

    // Solar thermal elevation delta T: proportional to absorptance, inversely proportional to thermal inertia
    // High inertia (water) changes slowly; low inertia (thin metal/soil) changes rapidly
    const float solar_factor = (mat.solar_absorptance * solar_irradiance_w_m2) / (mat.thermal_inertia * 0.05f);
    const float delta_t = std::max(0.0f, solar_factor * (phase_cos + 1.0f) * 0.5f);

    return ambient_temp_k + delta_t;
}

} // namespace ir_sim::physics
