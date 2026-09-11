#pragma once

#include "ir_sim/core/types.hpp"

#include <span>
#include <vector>

namespace ir_sim::detection {

struct CFARParams {
    int guard_half_size{1};     // Guard window: (2*G + 1) -> 3x3
    int training_half_size{4};  // Training window: (2*T + 1) -> 9x9
    float threshold_factor{3.5f}; // Alpha multiplier: Threshold = mu + alpha * sigma
    float min_scr{2.0f};        // Minimum Signal-to-Clutter Ratio
    float min_peak_value{100.0f};
    int max_cluster_distance{4};// Centroid merge radius for NMS
};

/**
 * @brief 2D Cell-Averaging Constant False Alarm Rate (CA-CFAR) Detector.
 * 
 * Uses dual-window integral images (intensity and intensity^2) for O(1) computation
 * of local background clutter mean (mu) and variance (sigma^2). Dynamically sets
 * detection threshold to maintain a fixed false alarm rate in non-stationary clutter.
 */
class CFAR2D {
public:
    CFAR2D(size_t width, size_t height);
    CFAR2D(size_t width, size_t height, CFARParams params);

    /**
     * @brief Runs 2D CA-CFAR detection on a 14-bit thermal image or saliency map.
     * @param in_frame Input image buffer.
     * @return List of detected targets with bounding boxes, peak intensities, and SCR values.
     */
    [[nodiscard]] std::vector<core::Detection> detect(std::span<const uint16_t> in_frame);

    /**
     * @brief Computes local Signal-to-Clutter Ratio (SCR) map for visualization and diagnostics.
     */
    void compute_scr_map(std::span<const uint16_t> in_frame, std::span<float> out_scr_map);

    // Configuration
    CFARParams& params() noexcept { return params_; }
    [[nodiscard]] const CFARParams& params() const noexcept { return params_; }

private:
    std::vector<core::Detection> cluster_detections(const std::vector<core::Detection>& candidates) const;

    size_t width_{640};
    size_t height_{512};
    size_t total_pixels_{640 * 512};
    CFARParams params_{};

    // Pre-allocated integral images (sum and sum-of-squares)
    std::vector<uint64_t> integral_sum_;
    std::vector<double> integral_sq_sum_;
};

} // namespace ir_sim::detection
