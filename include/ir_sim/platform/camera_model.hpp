#pragma once

#include "ir_sim/core/types.hpp"

#include <optional>

namespace ir_sim::platform {

struct CameraSpecs {
    size_t width_pixels{640};
    size_t height_pixels{512};
    float pixel_pitch_m{12.0e-6f};    // 12 um tactical microbolometer pitch
    float focal_length_m{0.050f};      // 50 mm lens
    float f_number{1.2f};             // Fast IR optics (F/1.2)
    float optical_transmission{0.88f}; // Germanium lens transmission
};

/**
 * @brief Pinhole Camera Projection, Optical Sizing, and Raycast Geo-Localization.
 */
class CameraModel {
public:
    CameraModel();
    explicit CameraModel(CameraSpecs specs);

    // Derived Optical Quantities
    [[nodiscard]] float ifov_rad() const noexcept { return specs_.pixel_pitch_m / specs_.focal_length_m; }
    [[nodiscard]] float ifov_mrad() const noexcept { return ifov_rad() * 1000.0f; }
    [[nodiscard]] float hfov_rad() const noexcept;
    [[nodiscard]] float vfov_rad() const noexcept;
    [[nodiscard]] float hfov_deg() const noexcept;
    [[nodiscard]] float vfov_deg() const noexcept;
    [[nodiscard]] float focal_length_pixels() const noexcept { return specs_.focal_length_m / specs_.pixel_pitch_m; }
    [[nodiscard]] float gsd_m(float slant_range_m) const noexcept { return slant_range_m * ifov_rad(); }

    /**
     * @brief Projects 3D world coordinate into 2D camera pixel coordinates.
     * @param world_pos 3D position in ENU coordinates [m].
     * @param camera_pos Drone/camera position in ENU coordinates [m].
     * @param camera_quat Camera orientation quaternion in world frame.
     * @return 2D pixel coordinates (u, v) and slant range [m] if in front of camera.
     */
    [[nodiscard]] std::optional<core::Vec3> project_world_to_pixel(
        const core::Vec3& world_pos,
        const core::Vec3& camera_pos,
        const core::Quat& camera_quat) const noexcept;

    /**
     * @brief Raycasts from camera through pixel (u, v) to ground plane (z = ground_z).
     * @return 3D ground intersection in ENU coordinates [m].
     */
    [[nodiscard]] std::optional<core::Vec3> raycast_pixel_to_ground(
        float u, float v,
        const core::Vec3& camera_pos,
        const core::Quat& camera_quat,
        float ground_z = 0.0f) const noexcept;

    /**
     * @brief Converts local ENU coordinates to GPS latitude and longitude relative to datum origin.
     */
    [[nodiscard]] static core::GeoCoordinate enu_to_geodetic(
        const core::Vec3& enu_pos,
        const core::GeoCoordinate& datum_origin) noexcept;

    // Accessors
    [[nodiscard]] const CameraSpecs& specs() const noexcept { return specs_; }
    [[nodiscard]] size_t width() const noexcept { return specs_.width_pixels; }
    [[nodiscard]] size_t height() const noexcept { return specs_.height_pixels; }

private:
    CameraSpecs specs_;
    float fx_{0.0f};
    float fy_{0.0f};
    float cx_{0.0f};
    float cy_{0.0f};
};

} // namespace ir_sim::platform
