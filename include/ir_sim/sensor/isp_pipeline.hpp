#pragma once

#include "ir_sim/core/frame_buffer.hpp"

#include <span>
#include <vector>

namespace ir_sim::sensor {

struct ISPOptions {
    bool enable_nuc{true};
    bool enable_bpr{true};
    bool enable_clahe{true};
    float plateau_clip_factor{2.5f}; // Histogram plateau limit factor
    float percentile_low{0.005f};     // 0.5% lower black clipping limit
    float percentile_high{0.995f};    // 99.5% upper white clipping limit
};

/**
 * @brief Real-Time Embedded Image Signal Processor (ISP) Pipeline.
 * 
 * Implements hardware-grade thermal camera processing:
 * 1. Two-Point Non-Uniformity Correction (NUC): Normalizes detector gain and offset.
 * 2. Bad Pixel Replacement (BPR): 8-neighbor median filtering for defective pixels.
 * 3. Dynamic Range Compression (Plateau-Equalized Histogram AGC / CLAHE): 14-to-8 bit display.
 */
class ISPPipeline {
public:
    ISPPipeline(size_t width, size_t height);

    /**
     * @brief Performs Two-Point NUC calibration from cold and hot uniform reference frames.
     * @param cold_frame Flat-field frame at temperature T_cold (e.g. 290 K).
     * @param hot_frame Flat-field frame at temperature T_hot (e.g. 310 K).
     * @param temp_cold_k Temperature of cold reference [K].
     * @param temp_hot_k Temperature of hot reference [K].
     */
    void calibrate_2point(std::span<const uint16_t> cold_frame,
                          std::span<const uint16_t> hot_frame,
                          float temp_cold_k, float temp_hot_k);

    /**
     * @brief Sets external bad pixel mask (e.g. from FPA sensor model).
     */
    void set_bad_pixel_mask(const std::vector<uint8_t>& mask);

    /**
     * @brief Applies NUC and Bad Pixel Replacement (BPR) to a raw 14-bit frame.
     * @param raw_in Input raw frame with FPN, stripe noise, and dead pixels.
     * @param corrected_out Output linearized, corrected 14-bit frame.
     */
    void correct_nuc_and_bpr(std::span<const uint16_t> raw_in,
                             std::span<uint16_t> corrected_out);

    /**
     * @brief Compresses 14-bit dynamic range to 8-bit display video via Plateau AGC / CLAHE.
     * @param corrected_in Calibrated 14-bit input frame.
     * @param display_out Contrast-enhanced 8-bit display frame [0 - 255].
     */
    void enhance_dynamic_range(std::span<const uint16_t> corrected_in,
                               std::span<uint8_t> display_out);

    /**
     * @brief Calculates Non-Uniformity metric NU [%] on a flat-field frame:
     *        NU = (sigma_counts / mean_counts) * 100%.
     */
    [[nodiscard]] static float calculate_non_uniformity_percent(std::span<const uint16_t> frame) noexcept;

    // Configuration
    ISPOptions& options() noexcept { return options_; }
    [[nodiscard]] const ISPOptions& options() const noexcept { return options_; }
    [[nodiscard]] bool is_nuc_calibrated() const noexcept { return is_calibrated_; }
    [[nodiscard]] size_t bad_pixel_count() const noexcept { return num_bad_pixels_; }

private:
    void apply_bpr(std::span<uint16_t> frame);

    size_t width_{640};
    size_t height_{512};
    size_t total_pixels_{640 * 512};
    ISPOptions options_{};

    bool is_calibrated_{false};
    std::vector<float> nuc_gain_;    // G(i,j)
    std::vector<float> nuc_offset_;  // B(i,j)
    std::vector<uint8_t> bad_pixel_mask_;
    size_t num_bad_pixels_{0};

    // Pre-allocated histogram buffer for zero-allocation AGC
    std::vector<uint32_t> histogram_;
    std::vector<uint8_t> lut_8bit_;
};

} // namespace ir_sim::sensor
