#pragma once

#include "ir_sim/core/frame_buffer.hpp"

#include <span>
#include <vector>

namespace ir_sim::detection {

struct LCMParams {
    int cell_size{3};          // Size of sub-cells (k x k, e.g. 3x3 pixels)
    float min_saliency{1.2f};  // Saliency threshold: m0^2 / max(mi) > threshold
};

/**
 * @brief Multiscale Local Contrast Measure (LCM / MPCM) Point-Target Saliency Engine.
 * 
 * Specifically designed for dim, sub-pixel, and point target detection (1x1 to 5x5 pixels)
 * against complex terrestrial thermal clutter (clouds, roads, tree edges).
 * 
 * Divides local neighborhood into central sub-cell T and 8 surrounding directional cells {B1..B8}.
 * Computes contrast ratio: LCM(x, y) = min_i (m0^2 / mi)
 */
class PointTargetLCM {
public:
    PointTargetLCM(size_t width, size_t height);

    /**
     * @brief Computes 2D LCM saliency contrast map from a 14-bit or 8-bit frame.
     * @param in_frame Input image buffer (e.g. cleaned 14-bit counts).
     * @param out_saliency Output floating-point contrast saliency map.
     * @param cell_size Patch scale (default: 3).
     */
    void compute_saliency_map(std::span<const uint16_t> in_frame,
                              std::span<float> out_saliency,
                              int cell_size = 3);

    /**
     * @brief Computes Multiscale LCM (MLCM) over scales k in {3, 5, 7}, taking maximum response.
     */
    void compute_multiscale_saliency(std::span<const uint16_t> in_frame,
                                     std::span<float> out_saliency);

    [[nodiscard]] size_t width() const noexcept { return width_; }
    [[nodiscard]] size_t height() const noexcept { return height_; }

private:
    size_t width_{640};
    size_t height_{512};
    size_t total_pixels_{640 * 512};

    // Pre-allocated integral image buffer for fast O(1) rectangular box sums
    std::vector<uint64_t> integral_image_;
};

} // namespace ir_sim::detection
