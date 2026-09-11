#include "ir_sim/scene/synthetic_scene.hpp"

#include <algorithm>
#include <cmath>

namespace ir_sim::scene {

SyntheticScene::SyntheticScene(ScenarioManifest manifest)
    : manifest_(std::move(manifest)) {
    atmosphere_.set_ground_temp(manifest_.environment.ambient_temp_k);
    update_material_radiance_cache();
    update_target_kinematics();
}

void SyntheticScene::update_material_radiance_cache() {
    material_cache_.clear();
    material_cache_.resize(5);

    const char* mat_names[5] = {"asphalt", "concrete", "dry_soil", "grass", "water"};

    for (size_t i = 0; i < 5; ++i) {
        const auto& mat = material_db_.get(mat_names[i]);

        const float temp = physics::MaterialDatabase::compute_diurnal_temperature(
            mat,
            manifest_.environment.time_of_day_hours,
            manifest_.environment.ambient_temp_k,
            manifest_.environment.solar_irradiance_w_m2
        );

        const float l_bb = static_cast<float>(physics::Planck::integrate_band_radiance(8.0, 14.0, temp));
        const float l_ambient = static_cast<float>(physics::Planck::integrate_band_radiance(
            8.0, 14.0, manifest_.environment.ambient_temp_k
        ));

        const float in_band_radiance = mat.emissivity_lwir * l_bb + (1.0f - mat.emissivity_lwir) * l_ambient;

        material_cache_[i] = {temp, in_band_radiance};
    }
}

void SyntheticScene::update_target_kinematics() {
    active_targets_.clear();
    active_targets_.reserve(manifest_.targets.size());

    const float l_ambient = static_cast<float>(physics::Planck::integrate_band_radiance(
        8.0, 14.0, manifest_.environment.ambient_temp_k
    ));

    for (const auto& t_def : manifest_.targets) {
        EvaluatedTarget target{};
        target.id = t_def.id;
        target.name = t_def.name;
        target.type = t_def.type;
        target.dimensions = t_def.dimensions;
        target.state = t_def.evaluate_at_time(current_time_sec_);

        const float l_bb = static_cast<float>(physics::Planck::integrate_band_radiance(
            8.0, 14.0, target.state.composite_temp_k
        ));
        target.source_radiance = target.state.composite_emissivity * l_bb +
                                 (1.0f - target.state.composite_emissivity) * l_ambient;

        active_targets_.push_back(std::move(target));
    }
}

void SyntheticScene::update(float sim_time_sec) {
    current_time_sec_ = sim_time_sec;
    update_target_kinematics();
}

float SyntheticScene::get_elevation(float x, float y) const noexcept {
    const float base = manifest_.terrain.base_elevation_m;

    if (manifest_.terrain.terrain_type == "desert") {
        return base + 2.5f * std::sin(x * 0.005f) * std::cos(y * 0.005f);
    } else if (manifest_.terrain.terrain_type == "woodland") {
        return base + 8.0f * std::sin(x * 0.003f) + 6.0f * std::cos(y * 0.004f);
    }

    return base;
}

SyntheticScene::SurfaceType SyntheticScene::get_material(float x, float y) const noexcept {
    (void)x;
    // Check for roadway
    if (std::abs(y - 200.0f) <= 4.0f) {
        return SurfaceType::Asphalt;
    }
    if (std::abs(y - 200.0f) <= 7.0f) {
        return SurfaceType::DrySoil;
    }

    if (manifest_.terrain.terrain_type == "desert") {
        return SurfaceType::DrySoil;
    } else if (manifest_.terrain.terrain_type == "woodland") {
        return SurfaceType::Grass;
    }

    return SurfaceType::Concrete;
}

void SyntheticScene::render_at_aperture(
    const platform::DronePlatform& drone,
    const platform::Gimbal& gimbal,
    const platform::CameraModel& camera,
    core::RadianceFrame& out_radiance,
    std::vector<core::Detection>& out_ground_truth
) const {
    const uint32_t width = static_cast<uint32_t>(camera.width());
    const uint32_t height = static_cast<uint32_t>(camera.height());
    const auto drone_pos = drone.pose().position_enu_m;
    const auto gimbal_quat = gimbal.camera_orientation();

    const auto band = core::SpectralBand::lwir();
    const float ambient_t = manifest_.environment.ambient_temp_k;
    const float sky_radiance = static_cast<float>(
        physics::Planck::integrate_band_radiance(8.0, 14.0, ambient_t - 25.0f)
    );
    const float base_elev = manifest_.terrain.base_elevation_m;
    const double blackbody_path = physics::Planck::integrate_band_radiance(8.0, 14.0, ambient_t);
    const double beta = atmosphere_.extinction_coeff(band);

    // 1. Render Terrain Background Radiance
    for (uint32_t v = 0; v < height; ++v) {
        for (uint32_t u = 0; u < width; ++u) {
            const auto hit_opt = camera.raycast_pixel_to_ground(
                static_cast<float>(u), static_cast<float>(v),
                drone_pos, gimbal_quat, base_elev
            );

            if (!hit_opt) {
                out_radiance(u, v) = sky_radiance;
            } else {
                const auto& hit = *hit_opt;
                const float slant_range = (hit - drone_pos).norm();

                const auto mat_type = get_material(hit.x, hit.y);
                const auto& cache = material_cache_[static_cast<size_t>(mat_type)];

                const double tau = std::exp(-beta * slant_range);
                const double l_path = (1.0 - tau) * blackbody_path;

                out_radiance(u, v) = static_cast<float>(tau * cache.in_band_radiance + l_path);
            }
        }
    }

    out_ground_truth.clear();

    // 2. Render Tactical Targets & Apply Sub-Pixel PSF Integration
    for (const auto& target : active_targets_) {
        const auto& tgt_pos = target.state.position;
        const float slant_range = (tgt_pos - drone_pos).norm();
        if (slant_range < 1.0f) continue;

        // Project target center onto camera image plane
        const auto proj_opt = camera.project_world_to_pixel(tgt_pos, drone_pos, gimbal_quat);
        if (!proj_opt) continue;

        const float u_tgt = proj_opt->x;
        const float v_tgt = proj_opt->y;

        // Apparent target size in pixels: angular_size / IFOV
        const float ifov = camera.ifov_rad();
        const float w_px = std::max(0.1f, (target.dimensions.y / slant_range) / ifov);
        const float h_px = std::max(0.1f, (target.dimensions.z / slant_range) / ifov);

        // Discard targets that are outside the camera field of view
        if (u_tgt < -w_px || u_tgt >= static_cast<float>(width) + w_px ||
            v_tgt < -h_px || v_tgt >= static_cast<float>(height) + h_px) {
            continue;
        }

        // Compute at-aperture target radiance
        const double tau_tgt = std::exp(-beta * slant_range);
        const double l_path_tgt = (1.0 - tau_tgt) * blackbody_path;
        const float tgt_aperture_radiance = static_cast<float>(tau_tgt * target.source_radiance + l_path_tgt);

        // Record ground truth detection
        core::Detection gt{};
        gt.bbox.x = u_tgt - std::max(1.0f, w_px * 0.5f);
        gt.bbox.y = v_tgt - std::max(1.0f, h_px * 0.5f);
        gt.bbox.width = std::max(2.0f, w_px);
        gt.bbox.height = std::max(2.0f, h_px);
        gt.bbox.score = 1.0f;

        // Estimate local background radiance for SCR recording
        const auto iu = static_cast<uint32_t>(std::clamp(u_tgt, 0.0f, static_cast<float>(width - 1)));
        const auto iv = static_cast<uint32_t>(std::clamp(v_tgt, 0.0f, static_cast<float>(height - 1)));
        const float bg_radiance = out_radiance(iu, iv);
        gt.scr = std::abs(tgt_aperture_radiance - bg_radiance) / std::max(0.01f, bg_radiance * 0.05f);

        out_ground_truth.push_back(gt);

        // Check if target is extended or sub-pixel
        if (w_px >= 1.5f && h_px >= 1.5f) {
            // Extended target: Render filled ellipse
            const int u_min = std::max(0, static_cast<int>(u_tgt - w_px * 0.5f));
            const int u_max = std::min(static_cast<int>(width - 1), static_cast<int>(u_tgt + w_px * 0.5f));
            const int v_min = std::max(0, static_cast<int>(v_tgt - h_px * 0.5f));
            const int v_max = std::min(static_cast<int>(height - 1), static_cast<int>(v_tgt + h_px * 0.5f));

            const float inv_a2 = 1.0f / (w_px * w_px * 0.25f);
            const float inv_b2 = 1.0f / (h_px * h_px * 0.25f);

            for (int v = v_min; v <= v_max; ++v) {
                const float dvy = static_cast<float>(v) - v_tgt;
                const float term_y = dvy * dvy * inv_b2;
                for (int u = u_min; u <= u_max; ++u) {
                    const float dux = static_cast<float>(u) - u_tgt;
                    if (dux * dux * inv_a2 + term_y <= 1.0f) {
                        out_radiance(static_cast<uint32_t>(u), static_cast<uint32_t>(v)) = tgt_aperture_radiance;
                    }
                }
            }
        } else {
            // Sub-pixel point target (Bird or Micro-Drone at range)
            // Optical PSF Gaussian distribution with total radiant flux conservation
            const float effective_area = w_px * h_px;
            const float delta_phi = (tgt_aperture_radiance - bg_radiance) * effective_area;

            constexpr float sigma = 0.65f; // Optics PSF blur radius in pixels
            constexpr float two_sigma_sq = 2.0f * sigma * sigma;

            // Compute 3x3 PSF weights
            float sum_weights = 0.0f;
            float weights[3][3];

            for (int dv = -1; dv <= 1; ++dv) {
                for (int du = -1; du <= 1; ++du) {
                    const float dist_sq = static_cast<float>(du * du + dv * dv);
                    weights[dv + 1][du + 1] = std::exp(-dist_sq / two_sigma_sq);
                    sum_weights += weights[dv + 1][du + 1];
                }
            }

            const float inv_sum = 1.0f / sum_weights;

            for (int dv = -1; dv <= 1; ++dv) {
                const int v = static_cast<int>(v_tgt) + dv;
                if (v < 0 || v >= static_cast<int>(height)) continue;

                for (int du = -1; du <= 1; ++du) {
                    const int u = static_cast<int>(u_tgt) + du;
                    if (u < 0 || u >= static_cast<int>(width)) continue;

                    const float w = weights[dv + 1][du + 1] * inv_sum;
                    out_radiance(static_cast<uint32_t>(u), static_cast<uint32_t>(v)) += delta_phi * w;
                }
            }
        }
    }
}

} // namespace ir_sim::scene
