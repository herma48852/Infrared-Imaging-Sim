#pragma once

#include "ir_sim/core/frame_buffer.hpp"
#include "ir_sim/core/types.hpp"
#include "ir_sim/physics/atmosphere.hpp"
#include "ir_sim/physics/materials.hpp"
#include "ir_sim/physics/planck.hpp"
#include "ir_sim/platform/camera_model.hpp"
#include "ir_sim/platform/drone_platform.hpp"
#include "ir_sim/platform/gimbal.hpp"
#include "ir_sim/scene/scenario_manifest.hpp"

#include <memory>
#include <vector>

namespace ir_sim::scene {

/**
 * @brief Synthetic Scene & Radiative Transfer Renderer.
 * 
 * Simulates 2.5D Digital Elevation Model (DEM), terrain surface material radiometry,
 * dynamic tactical targets, and accurate at-aperture radiance calculation including
 * sub-pixel optical PSF energy conservation for point targets (birds & micro-drones).
 */
class SyntheticScene {
public:
    explicit SyntheticScene(ScenarioManifest manifest);

    /**
     * @brief Steps simulation time forward and updates all target kinematics.
     * @param sim_time_sec Current elapsed scenario time [s].
     */
    void update(float sim_time_sec);

    /**
     * @brief Renders high-fidelity at-aperture spectral radiance map [W/(m^2 sr)]
     *        seen through the drone camera optics.
     * 
     * @param drone Current 6-DOF drone platform state.
     * @param gimbal Current gimbal pointing pose.
     * @param camera Optical camera model.
     * @param out_radiance Destination radiance frame buffer.
     * @param out_ground_truth Output list of ground-truth target detections in view.
     */
    void render_at_aperture(
        const platform::DronePlatform& drone,
        const platform::Gimbal& gimbal,
        const platform::CameraModel& camera,
        core::RadianceFrame& out_radiance,
        std::vector<core::Detection>& out_ground_truth
    ) const;

    [[nodiscard]] const ScenarioManifest& manifest() const noexcept { return manifest_; }
    [[nodiscard]] float current_time() const noexcept { return current_time_sec_; }
    [[nodiscard]] size_t target_count() const noexcept { return manifest_.targets.size(); }

    enum class SurfaceType : uint8_t {
        Asphalt = 0,
        Concrete,
        DrySoil,
        Grass,
        Water
    };

    /**
     * @brief Returns terrain elevation [m] at world coordinates (x, y).
     */
    [[nodiscard]] float get_elevation(float x, float y) const noexcept;

    /**
     * @brief Returns terrain surface material at world coordinates (x, y).
     */
    [[nodiscard]] SurfaceType get_material(float x, float y) const noexcept;

private:
    ScenarioManifest manifest_;
    float current_time_sec_{0.0f};
    physics::MaterialDatabase material_db_;
    physics::Atmosphere atmosphere_;

    // Precomputed background temperatures and radiances per surface material
    struct MaterialRadianceCache {
        float temp_k{288.15f};
        float in_band_radiance{25.0f}; // W / (m^2 sr) in LWIR
    };
    std::vector<MaterialRadianceCache> material_cache_;

    // Evaluated target states at current_time_sec_
    struct EvaluatedTarget {
        uint32_t id{0};
        std::string name;
        TargetType type{TargetType::MilitaryVehicle};
        core::Vec3 dimensions{4.0f, 2.0f, 2.0f};
        TargetKinematicState state;
        float source_radiance{30.0f};
    };
    std::vector<EvaluatedTarget> active_targets_;

    void update_material_radiance_cache();
    void update_target_kinematics();
};

} // namespace ir_sim::scene
