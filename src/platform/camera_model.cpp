#include "ir_sim/platform/camera_model.hpp"

#include <cmath>
#include <numbers>

namespace ir_sim::platform {

CameraModel::CameraModel()
    : CameraModel(CameraSpecs{}) {}

CameraModel::CameraModel(CameraSpecs specs)
    : specs_(specs) {
    fx_ = specs_.focal_length_m / specs_.pixel_pitch_m;
    fy_ = fx_;
    cx_ = (static_cast<float>(specs_.width_pixels) - 1.0f) * 0.5f;
    cy_ = (static_cast<float>(specs_.height_pixels) - 1.0f) * 0.5f;
}

float CameraModel::hfov_rad() const noexcept {
    const float w_sensor = static_cast<float>(specs_.width_pixels) * specs_.pixel_pitch_m;
    return 2.0f * std::atan(w_sensor / (2.0f * specs_.focal_length_m));
}

float CameraModel::vfov_rad() const noexcept {
    const float h_sensor = static_cast<float>(specs_.height_pixels) * specs_.pixel_pitch_m;
    return 2.0f * std::atan(h_sensor / (2.0f * specs_.focal_length_m));
}

float CameraModel::hfov_deg() const noexcept {
    return hfov_rad() * (180.0f / static_cast<float>(std::numbers::pi));
}

float CameraModel::vfov_deg() const noexcept {
    return vfov_rad() * (180.0f / static_cast<float>(std::numbers::pi));
}

std::optional<core::Vec3> CameraModel::project_world_to_pixel(
    const core::Vec3& world_pos,
    const core::Vec3& camera_pos,
    const core::Quat& camera_quat) const noexcept {

    // Relative vector from camera to world point in ENU frame
    const core::Vec3 delta_world = world_pos - camera_pos;

    // Transform into camera frame using inverse (conjugate) of camera quaternion
    // Camera frame: +X right, +Y down, +Z forward along optical axis
    const core::Vec3 delta_cam = camera_quat.conjugate().rotate(delta_world);

    // If point is behind or directly on the camera plane, it cannot be projected
    if (delta_cam.z <= 0.10f) {
        return std::nullopt;
    }

    const float u = fx_ * (delta_cam.x / delta_cam.z) + cx_;
    const float v = fy_ * (delta_cam.y / delta_cam.z) + cy_;
    const float slant_range = delta_world.norm();

    return core::Vec3{u, v, slant_range};
}

std::optional<core::Vec3> CameraModel::raycast_pixel_to_ground(
    float u, float v,
    const core::Vec3& camera_pos,
    const core::Quat& camera_quat,
    float ground_z) const noexcept {

    // 1. Ray direction in camera coordinate frame
    const core::Vec3 ray_cam{
        (u - cx_) / fx_,
        (v - cy_) / fy_,
        1.0f
    };
    const core::Vec3 ray_cam_unit = ray_cam.normalized();

    // 2. Rotate ray into ENU world frame
    const core::Vec3 ray_world = camera_quat.rotate(ray_cam_unit);

    // 3. Intersect with ground plane z = ground_z
    // ray_world.z must be negative (pointing downward towards the earth)
    if (ray_world.z >= -1.0e-5f) {
        return std::nullopt; // Ray points horizontally or into the sky
    }

    const float t = (ground_z - camera_pos.z) / ray_world.z;
    if (t < 0.0f) {
        return std::nullopt;
    }

    return camera_pos + ray_world * t;
}

core::GeoCoordinate CameraModel::enu_to_geodetic(
    const core::Vec3& enu_pos,
    const core::GeoCoordinate& datum_origin) noexcept {

    constexpr double earth_radius_m = 6'378'137.0; // WGS84 equatorial radius
    constexpr double rad2deg = 180.0 / std::numbers::pi;
    constexpr double deg2rad = std::numbers::pi / 180.0;

    const double lat_rad = datum_origin.latitude_deg * deg2rad;

    // Coordinate offsets
    const double d_lat = (enu_pos.y / earth_radius_m) * rad2deg;
    const double d_lon = (enu_pos.x / (earth_radius_m * std::cos(lat_rad))) * rad2deg;

    return {
        datum_origin.latitude_deg + d_lat,
        datum_origin.longitude_deg + d_lon,
        datum_origin.altitude_msl_m + enu_pos.z
    };
}

} // namespace ir_sim::platform
