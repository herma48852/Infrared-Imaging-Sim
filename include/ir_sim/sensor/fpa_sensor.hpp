#pragma once

#include "ir_sim/core/frame_buffer.hpp"
#include "ir_sim/core/types.hpp"
#include "ir_sim/physics/constants.hpp"

#include <random>
#include <span>
#include <vector>

namespace ir_sim::sensor {

struct FPASpecs {
    size_t width{640};
    size_t height{512};
    float pixel_pitch_m{12.0e-6f};       // 12 um pixel pitch
    float netd_k{0.040f};                // 40 mK Noise Equivalent Temperature Difference
    float fpn_gain_sigma{0.025f};        // 2.5% spatial gain non-uniformity
    float fpn_offset_sigma_counts{80.0f};// Offset variation [ADC counts]
    float column_fpn_sigma_counts{45.0f};// Column amplifier vertical stripe noise
    float dead_pixel_ratio{0.002f};      // 0.2% defective pixels
    float f_number{1.2f};                // Lens F-number
    float optical_transmission{0.88f};   // Lens transmission
    uint32_t adc_bits{14};               // 14-bit ADC (0 - 16383 counts)
    float base_counts_at_293k{6000.0f};  // Digital counts for 20 C uniform scene
    float counts_per_kelvin{120.0f};     // System responsivity dCounts/dT
};

/**
 * @brief Focal Plane Array (FPA) Sensor Transduction & Degradation Engine.
 * 
 * Simulates microbolometer physical response, including:
 * - Spatial Fixed-Pattern Noise (FPN) gain/offset matrices and column striping.
 * - Temporal Gaussian thermal fluctuations (NETD noise).
 * - Defective (dead and hot) pixel clustering.
 * - 14-bit ADC quantization and saturation.
 */
class FPASensor {
public:
    FPASensor();
    explicit FPASensor(FPASpecs specs, uint64_t random_seed = 42);

    /**
     * @brief Converts pristine at-aperture spectral radiance into degraded 14-bit raw ADC counts.
     * @param aperture_radiance Input at-aperture radiance map [W/(m^2*sr)].
     * @param out_raw_frame Output 14-bit ADC frame buffer [0 - 16383].
     */
    void process(std::span<const float> aperture_radiance,
                 std::span<uint16_t> out_raw_frame);

    /**
     * @brief Generates a synthetic flat-field blackbody calibration frame at uniform temperature.
     */
    void generate_flat_field(double uniform_temp_k,
                             std::span<uint16_t> out_frame,
                             bool include_temporal_noise = false);

    // Defective Pixel Mask (true = defective pixel)
    [[nodiscard]] const std::vector<uint8_t>& dead_pixel_mask() const noexcept { return dead_pixel_mask_; }
    [[nodiscard]] size_t dead_pixel_count() const noexcept { return num_dead_pixels_; }

    // Configuration
    void set_netd_k(float netd_k) noexcept { specs_.netd_k = netd_k; }
    void set_fpn_gain_sigma(float sigma) noexcept { specs_.fpn_gain_sigma = sigma; generate_fpn_matrices(); }
    void set_temporal_noise_enabled(bool enabled) noexcept { temporal_noise_enabled_ = enabled; }

    [[nodiscard]] const FPASpecs& specs() const noexcept { return specs_; }
    [[nodiscard]] size_t width() const noexcept { return specs_.width; }
    [[nodiscard]] size_t height() const noexcept { return specs_.height; }
    [[nodiscard]] uint32_t max_adc_count() const noexcept { return (1u << specs_.adc_bits) - 1u; }
    [[nodiscard]] uint64_t random_seed() const noexcept { return seed_; }

private:
    void generate_fpn_matrices();
    void generate_dead_pixels();

    FPASpecs specs_;
    uint64_t seed_{42};
    std::mt19937_64 rng_;

    bool temporal_noise_enabled_{true};

    // Pre-computed spatial FPN matrices
    std::vector<float> gain_matrix_;          // Per-pixel gain non-uniformity g(i,j) ~ N(1, sigma_g)
    std::vector<float> offset_matrix_;        // Per-pixel offset non-uniformity o(i,j)
    std::vector<float> column_offset_matrix_; // Column stripe noise c(j)
    std::vector<uint8_t> dead_pixel_mask_;    // 1 = dead (0 count), 2 = hot (saturated count), 0 = valid
    size_t num_dead_pixels_{0};

    // Conversion factor from radiance to nominal counts
    float radiance_to_counts_scale_{1.0f};
};

} // namespace ir_sim::sensor
