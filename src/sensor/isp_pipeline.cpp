#include "ir_sim/sensor/isp_pipeline.hpp"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <numeric>

namespace ir_sim::sensor {

ISPPipeline::ISPPipeline(size_t width, size_t height)
    : width_(width),
      height_(height),
      total_pixels_(width * height) {
    nuc_gain_.assign(total_pixels_, 1.0f);
    nuc_offset_.assign(total_pixels_, 0.0f);
    bad_pixel_mask_.assign(total_pixels_, 0);
    histogram_.assign(16384, 0);
    lut_8bit_.assign(16384, 0);
}

void ISPPipeline::set_bad_pixel_mask(const std::vector<uint8_t>& mask) {
    assert(mask.size() >= total_pixels_ && "Bad pixel mask size mismatch");
    bad_pixel_mask_ = mask;
    num_bad_pixels_ = 0;
    for (uint8_t m : bad_pixel_mask_) {
        if (m != 0) num_bad_pixels_++;
    }
}

void ISPPipeline::calibrate_2point(std::span<const uint16_t> cold_frame,
                                   std::span<const uint16_t> hot_frame,
                                   float /*temp_cold_k*/, float /*temp_hot_k*/) {
    assert(cold_frame.size() >= total_pixels_ && "Cold calibration frame size mismatch");
    assert(hot_frame.size() >= total_pixels_ && "Hot calibration frame size mismatch");

    // 1. Calculate means of valid pixels in cold and hot frames
    double sum_cold = 0.0;
    double sum_hot = 0.0;
    size_t valid_count = 0;

    for (size_t i = 0; i < total_pixels_; ++i) {
        if (bad_pixel_mask_[i] == 0) {
            sum_cold += cold_frame[i];
            sum_hot += hot_frame[i];
            valid_count++;
        }
    }

    if (valid_count == 0) return;

    const float mean_cold = static_cast<float>(sum_cold / static_cast<double>(valid_count));
    const float mean_hot = static_cast<float>(sum_hot / static_cast<double>(valid_count));
    const float delta_target = mean_hot - mean_cold;

    // 2. Compute per-pixel gain G(i) and offset B(i)
    num_bad_pixels_ = 0;
    for (size_t i = 0; i < total_pixels_; ++i) {
        const float c_cold = static_cast<float>(cold_frame[i]);
        const float c_hot = static_cast<float>(hot_frame[i]);
        const float delta_pixel = c_hot - c_cold;

        // Check responsiveness (threshold to detect dead or unresponsive microbolometer bridges)
        if (delta_pixel > 15.0f && c_cold > 50.0f && c_hot < 16300.0f) {
            const float g = delta_target / delta_pixel;
            const float b = mean_cold - g * c_cold;
            nuc_gain_[i] = g;
            nuc_offset_[i] = b;
            bad_pixel_mask_[i] = 0;
        } else {
            // Flag as defective pixel
            nuc_gain_[i] = 1.0f;
            nuc_offset_[i] = 0.0f;
            bad_pixel_mask_[i] = 1;
            num_bad_pixels_++;
        }
    }

    is_calibrated_ = true;
}

void ISPPipeline::apply_bpr(std::span<uint16_t> frame) {
    if (num_bad_pixels_ == 0) return;

    std::vector<uint16_t> neighbors;
    neighbors.reserve(8);

    const int w = static_cast<int>(width_);
    const int h = static_cast<int>(height_);

    for (int y = 0; y < h; ++y) {
        const int row_idx = y * w;
        for (int x = 0; x < w; ++x) {
            const size_t idx = static_cast<size_t>(row_idx + x);
            if (bad_pixel_mask_[idx] == 0) continue;

            // Collect valid neighbors in 3x3 window
            neighbors.clear();
            for (int dy = -1; dy <= 1; ++dy) {
                const int ny = y + dy;
                if (ny < 0 || ny >= h) continue;
                const int nrow = ny * w;

                for (int dx = -1; dx <= 1; ++dx) {
                    if (dx == 0 && dy == 0) continue;
                    const int nx = x + dx;
                    if (nx < 0 || nx >= w) continue;

                    const size_t nidx = static_cast<size_t>(nrow + nx);
                    if (bad_pixel_mask_[nidx] == 0) {
                        neighbors.push_back(frame[nidx]);
                    }
                }
            }

            // Replace with median of valid neighbors
            if (!neighbors.empty()) {
                const size_t mid = neighbors.size() / 2;
                std::nth_element(neighbors.begin(), neighbors.begin() + static_cast<long>(mid), neighbors.end());
                frame[idx] = neighbors[mid];
            }
        }
    }
}

void ISPPipeline::correct_nuc_and_bpr(std::span<const uint16_t> raw_in,
                                      std::span<uint16_t> corrected_out) {
    assert(raw_in.size() >= total_pixels_ && "Input raw frame size mismatch");
    assert(corrected_out.size() >= total_pixels_ && "Output frame size mismatch");

    // 1. Two-Point NUC Gain and Offset correction
    if (options_.enable_nuc && is_calibrated_) {
        for (size_t i = 0; i < total_pixels_; ++i) {
            const float raw = static_cast<float>(raw_in[i]);
            const float corrected = nuc_gain_[i] * raw + nuc_offset_[i];
            corrected_out[i] = static_cast<uint16_t>(std::clamp(corrected, 0.0f, 16383.0f));
        }
    } else {
        std::copy(raw_in.begin(), raw_in.begin() + static_cast<long>(total_pixels_), corrected_out.begin());
    }

    // 2. Bad Pixel Replacement
    if (options_.enable_bpr) {
        apply_bpr(corrected_out);
    }
}

void ISPPipeline::enhance_dynamic_range(std::span<const uint16_t> corrected_in,
                                        std::span<uint8_t> display_out) {
    assert(corrected_in.size() >= total_pixels_ && "Input corrected frame size mismatch");
    assert(display_out.size() >= total_pixels_ && "Output display frame size mismatch");

    // 1. Clear histogram
    std::fill(histogram_.begin(), histogram_.end(), 0);

    // 2. Populate 14-bit histogram
    for (size_t i = 0; i < total_pixels_; ++i) {
        const uint16_t val = corrected_in[i];
        if (val < 16384) {
            histogram_[val]++;
        }
    }

    // 3. Percentile clipping (ignore extreme outliers)
    const uint32_t count_low = static_cast<uint32_t>(static_cast<float>(total_pixels_) * options_.percentile_low);
    const uint32_t count_high = static_cast<uint32_t>(static_cast<float>(total_pixels_) * options_.percentile_high);

    uint32_t cumulative = 0;
    size_t bin_low = 0;
    size_t bin_high = 16383;
    bool found_low = false;

    for (size_t i = 0; i < 16384; ++i) {
        cumulative += histogram_[i];
        if (!found_low && cumulative >= count_low) {
            bin_low = i;
            found_low = true;
        }
        if (cumulative >= count_high) {
            bin_high = i;
            break;
        }
    }

    if (bin_high <= bin_low) {
        bin_high = bin_low + 1;
    }

    // Minimum dynamic range span: If scene thermal contrast is virtually uniform (< 40 counts, ~1.5 K),
    // clamp minimum bin span so that micro-Kelvin slant-range gradients do not stretch into
    // discrete concentric circle quantization bands.
    constexpr size_t min_bin_span = 40;
    if (bin_high - bin_low < min_bin_span) {
        const size_t mid = (bin_low + bin_high) / 2;
        bin_low = (mid >= min_bin_span / 2) ? (mid - min_bin_span / 2) : 0;
        bin_high = std::min<size_t>(16383, bin_low + min_bin_span);
    }

    // 4. Plateau Equalization (clip histogram peaks to prevent noise blowing out)
    const size_t active_bins = bin_high - bin_low + 1;
    const float avg_count_per_bin = static_cast<float>(total_pixels_) / static_cast<float>(active_bins);
    const uint32_t plateau_limit = std::max(1u, static_cast<uint32_t>(avg_count_per_bin * options_.plateau_clip_factor));

    uint32_t clipped_total = 0;
    for (size_t i = bin_low; i <= bin_high; ++i) {
        clipped_total += std::min(histogram_[i], plateau_limit);
    }

    if (clipped_total == 0) clipped_total = 1;

    // 5. Build 14-bit to 8-bit mapping LUT
    for (size_t i = 0; i < bin_low; ++i) {
        lut_8bit_[i] = 0;
    }

    uint32_t running_sum = 0;
    const float scale_to_255 = 255.0f / static_cast<float>(clipped_total);
    for (size_t i = bin_low; i <= bin_high; ++i) {
        running_sum += std::min(histogram_[i], plateau_limit);
        const float val_8bit = static_cast<float>(running_sum) * scale_to_255;
        lut_8bit_[i] = static_cast<uint8_t>(std::clamp(val_8bit, 0.0f, 255.0f));
    }

    for (size_t i = bin_high + 1; i < 16384; ++i) {
        lut_8bit_[i] = 255;
    }

    // 6. Fast vector-friendly table mapping
    for (size_t i = 0; i < total_pixels_; ++i) {
        display_out[i] = lut_8bit_[corrected_in[i]];
    }
}

float ISPPipeline::calculate_non_uniformity_percent(std::span<const uint16_t> frame) noexcept {
    if (frame.empty()) return 0.0f;

    double sum = 0.0;
    for (uint16_t val : frame) {
        sum += val;
    }
    const double mean = sum / static_cast<double>(frame.size());
    if (mean <= 1.0e-4) return 0.0f;

    double variance_sum = 0.0;
    for (uint16_t val : frame) {
        const double diff = static_cast<double>(val) - mean;
        variance_sum += diff * diff;
    }
    const double std_dev = std::sqrt(variance_sum / static_cast<double>(frame.size()));

    return static_cast<float>((std_dev / mean) * 100.0);
}

} // namespace ir_sim::sensor
