#pragma once

#include "ir_sim/core/types.hpp"

#include <string>
#include <unordered_map>

namespace ir_sim::physics {

// Thermal and Optical Properties of a Surface Material
struct MaterialProperties {
    std::string name;
    float emissivity_lwir{0.90f};     // 8 - 14 um
    float emissivity_mwir{0.85f};     // 3 - 5 um
    float solar_absorptance{0.80f};   // Visible/solar spectrum absorption
    float thermal_inertia{1200.0f};   // [J / (m^2 * K * s^0.5)] Resistance to temp changes
    float base_temperature_k{293.15f};

    [[nodiscard]] float emissivity(const core::SpectralBand& band) const noexcept {
        return (band.lambda_min_um >= 7.0) ? emissivity_lwir : emissivity_mwir;
    }
};

/**
 * @brief Material Thermal Properties Database & Diurnal Heating Model.
 */
class MaterialDatabase {
public:
    MaterialDatabase();

    [[nodiscard]] const MaterialProperties& get(std::string_view name) const;
    [[nodiscard]] bool contains(std::string_view name) const noexcept;

    void register_material(MaterialProperties mat);

    /**
     * @brief Computes apparent surface temperature given diurnal time of day and solar irradiance.
     * @param mat Material properties.
     * @param time_of_day_hours Local time [0.0 - 24.0].
     * @param ambient_temp_k Current ambient air temperature [K].
     * @param solar_irradiance_w_m2 Current solar flux [W / m^2].
     * @return Physical surface temperature [K].
     */
    [[nodiscard]] static float compute_diurnal_temperature(const MaterialProperties& mat,
                                                           float time_of_day_hours,
                                                           float ambient_temp_k,
                                                           float solar_irradiance_w_m2) noexcept;

private:
    void populate_default_materials();

    std::unordered_map<std::string, MaterialProperties> materials_;
    MaterialProperties default_material_;
};

} // namespace ir_sim::physics
