#include "ir_sim/sensor/fpa_sensor.hpp"
#include "ir_sim/physics/planck.hpp"

#include <algorithm>
#include <cmath>

namespace ir_sim::sensor {

FPASensor::FPASensor()
    : FPASensor(FPASpecs{}, 42) {}

FPASensor::FPASensor(FPASpecs specs, uint64_t random_seed)
    : specs_(specs),
      seed_(random_seed),
      rng_(random_seed) {

    // Calibration of Radiance-to-Counts scaling:
    // Reference 20 C (293.15 K) in LWIR 8-14 um
    const auto lwir = core::SpectralBand::lwir();
    const double l_ref = physics::Planck::in_band_radiance(lwir, physics::TEMP_AMBIENT_STD, 1.0);
    const double l_ref_plus_1k = physics::Planck::in_band_radiance(lwir, physics::TEMP_AMBIENT_STD + 1.0, 1.0);
    const double dL_dT = l_ref_plus_1k - l_ref; // [W/(m^2*sr*K)]

    // scale = dCounts / dL = (dCounts / dT) / (dL / dT)
    radiance_to_counts_scale_ = static_cast<float>(specs_.counts_per_kelvin / dL_dT);

    generate_fpn_matrices();
    generate_dead_pixels();
}

void FPASensor::generate_fpn_matrices() {
    const size_t total_pixels = specs_.width * specs_.height;
    gain_matrix_.resize(total_pixels);
    offset_matrix_.resize(total_pixels);
    column_offset_matrix_.resize(specs_.width);

    std::normal_distribution<float> gain_dist(1.0f, specs_.fpn_gain_sigma);
    std::normal_distribution<float> offset_dist(0.0f, specs_.fpn_offset_sigma_counts);
    std::normal_distribution<float> col_dist(0.0f, specs_.column_fpn_sigma_counts);

    // Column amplifier striping (applied across all rows of column x)
    for (size_t x = 0; x < specs_.width; ++x) {
        column_offset_matrix_[x] = col_dist(rng_);
    }

    // Per-pixel gain and offset non-uniformities
    for (size_t i = 0; i < total_pixels; ++i) {
        gain_matrix_[i] = std::max(0.1f, gain_dist(rng_));
        offset_matrix_[i] = offset_dist(rng_);
    }
}

void FPASensor::generate_dead_pixels() {
    const size_t total_pixels = specs_.width * specs_.height;
    dead_pixel_mask_.assign(total_pixels, 0);
    num_dead_pixels_ = 0;

    std::uniform_real_distribution<float> uniform_dist(0.0f, 1.0f);

    for (size_t i = 0; i < total_pixels; ++i) {
        const float r = uniform_dist(rng_);
        if (r < specs_.dead_pixel_ratio) {
            // Half dead (0 count), half stuck hot (max count)
            dead_pixel_mask_[i] = (r < specs_.dead_pixel_ratio * 0.5f) ? 1 : 2;
            num_dead_pixels_++;
        }
    }
}

void FPASensor::process(std::span<const float> aperture_radiance,
                        std::span<uint16_t> out_raw_frame) {
    const size_t total_pixels = specs_.width * specs_.height;
    assert(aperture_radiance.size() >= total_pixels && "Input radiance span too small");
    assert(out_raw_frame.size() >= total_pixels && "Output raw frame span too small");

    const float l_ref = static_cast<float>(
        physics::Planck::in_band_radiance(core::SpectralBand::lwir(), physics::TEMP_AMBIENT_STD, 1.0));
    const float base_counts = specs_.base_counts_at_293k;
    const float max_adc = static_cast<float>(max_adc_count());

    // Temporal noise std dev in ADC counts: sigma = NETD * (counts_per_kelvin)
    const float netd_sigma_counts = specs_.netd_k * specs_.counts_per_kelvin;
    std::normal_distribution<float> temporal_dist(0.0f, netd_sigma_counts);

    for (size_t y = 0; y < specs_.height; ++y) {
        const size_t row_start = y * specs_.width;
        for (size_t x = 0; x < specs_.width; ++x) {
            const size_t idx = row_start + x;

            // Check defective pixel mask first
            const uint8_t mask = dead_pixel_mask_[idx];
            if (mask == 1) {
                out_raw_frame[idx] = 0; // Dead pixel
                continue;
            } else if (mask == 2) {
                out_raw_frame[idx] = static_cast<uint16_t>(max_adc); // Hot saturated pixel
                continue;
            }

            // 1. Pristine radiance converted to nominal counts
            const float rad = aperture_radiance[idx];
            const float nominal_counts = base_counts + (rad - l_ref) * radiance_to_counts_scale_;

            // 2. Apply Fixed Pattern Noise (gain, offset, and column stripe)
            const float gain = gain_matrix_[idx];
            const float offset = offset_matrix_[idx];
            const float col_offset = column_offset_matrix_[x];
            float count = nominal_counts * gain + offset + col_offset;

            // 3. Apply Temporal NETD Noise
            if (temporal_noise_enabled_ && netd_sigma_counts > 0.0f) {
                count += temporal_dist(rng_);
            }

            // 4. Quantize and clamp to 14-bit ADC limits [0 - 16383]
            count = std::clamp(count, 0.0f, max_adc);
            out_raw_frame[idx] = static_cast<uint16_t>(std::round(count));
        }
    }
}

void FPASensor::generate_flat_field(double uniform_temp_k,
                                    std::span<uint16_t> out_frame,
                                    bool include_temporal_noise) {
    const size_t total_pixels = specs_.width * specs_.height;
    std::vector<float> uniform_radiance(total_pixels);

    const float radiance = static_cast<float>(
        physics::Planck::in_band_radiance(core::SpectralBand::lwir(), uniform_temp_k, 1.0));
    std::fill(uniform_radiance.begin(), uniform_radiance.end(), radiance);

    const bool prev_temporal = temporal_noise_enabled_;
    temporal_noise_enabled_ = include_temporal_noise;
    process(uniform_radiance, out_frame);
    temporal_noise_enabled_ = prev_temporal;
}

} // namespace ir_sim::sensor
