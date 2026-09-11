#include "ir_sim/detection/point_target_lcm.hpp"

#include <algorithm>
#include <cassert>
#include <cmath>

namespace ir_sim::detection {

PointTargetLCM::PointTargetLCM(size_t width, size_t height)
    : width_(width),
      height_(height),
      total_pixels_(width * height) {
    // Integral image is (width + 1) x (height + 1)
    integral_image_.assign((width + 1) * (height + 1), 0);
}

void PointTargetLCM::compute_saliency_map(std::span<const uint16_t> in_frame,
                                          std::span<float> out_saliency,
                                          int cell_size) {
    assert(in_frame.size() >= total_pixels_ && "Input frame size mismatch");
    assert(out_saliency.size() >= total_pixels_ && "Output saliency map size mismatch");

    const int w = static_cast<int>(width_);
    const int h = static_cast<int>(height_);
    const int k = std::max(1, cell_size);
    const int margin = k + (k / 2);

    // 1. Build 2D Integral Image for O(1) rectangular box sums
    const int int_stride = w + 1;
    for (int y = 0; y < h; ++y) {
        uint64_t row_sum = 0;
        const int in_row = y * w;
        const int int_curr_row = (y + 1) * int_stride;
        const int int_prev_row = y * int_stride;

        for (int x = 0; x < w; ++x) {
            row_sum += in_frame[static_cast<size_t>(in_row + x)];
            integral_image_[static_cast<size_t>(int_curr_row + x + 1)] =
                integral_image_[static_cast<size_t>(int_prev_row + x + 1)] + row_sum;
        }
    }

    // Lambda to get box sum in O(1)
    auto get_box_sum = [&](int x1, int y1, int x2, int y2) -> uint64_t {
        x1 = std::clamp(x1, 0, w - 1);
        y1 = std::clamp(y1, 0, h - 1);
        x2 = std::clamp(x2, 0, w - 1);
        y2 = std::clamp(y2, 0, h - 1);

        const uint64_t a = integral_image_[static_cast<size_t>(y2 + 1) * static_cast<size_t>(int_stride) + static_cast<size_t>(x2 + 1)];
        const uint64_t b = integral_image_[static_cast<size_t>(y1) * static_cast<size_t>(int_stride) + static_cast<size_t>(x2 + 1)];
        const uint64_t c = integral_image_[static_cast<size_t>(y2 + 1) * static_cast<size_t>(int_stride) + static_cast<size_t>(x1)];
        const uint64_t d = integral_image_[static_cast<size_t>(y1) * static_cast<size_t>(int_stride) + static_cast<size_t>(x1)];

        return a - b - c + d;
    };

    std::fill(out_saliency.begin(), out_saliency.end(), 0.0f);

    const float inv_cell_area = 1.0f / static_cast<float>(k * k);
    const int half_k = k / 2;

    // 2. Iterate through pixels outside the margin
    for (int y = margin; y < h - margin; ++y) {
        const int row_idx = y * w;
        for (int x = margin; x < w - margin; ++x) {
            const size_t center_idx = static_cast<size_t>(row_idx + x);

            // Center cell bounds [x0, y0] to [x1, y1]
            const int cx0 = x - half_k;
            const int cy0 = y - half_k;
            const int cx1 = cx0 + k - 1;
            const int cy1 = cy0 + k - 1;

            // Find peak intensity m0 in center cell T
            float m0 = 0.0f;
            for (int cy = cy0; cy <= cy1; ++cy) {
                const int crow = cy * w;
                for (int cx = cx0; cx <= cx1; ++cx) {
                    const float val = static_cast<float>(in_frame[static_cast<size_t>(crow + cx)]);
                    if (val > m0) m0 = val;
                }
            }

            // Surrounding 8 directional neighborhood offsets in cell units:
            // 0:TL(-k,-k), 1:T(0,-k), 2:TR(k,-k), 3:R(k,0), 4:BR(k,k), 5:B(0,k), 6:BL(-k,k), 7:L(-k,0)
            const int dx_offsets[8] = {-k,  0,  k, k, k, 0, -k, -k};
            const int dy_offsets[8] = {-k, -k, -k, 0, k, k,  k,  0};

            float min_contrast = 1.0e9f;
            bool is_local_maximum = true;

            for (int i = 0; i < 8; ++i) {
                const int bx0 = cx0 + dx_offsets[i];
                const int by0 = cy0 + dy_offsets[i];
                const int bx1 = bx0 + k - 1;
                const int by1 = by0 + k - 1;

                const uint64_t box_sum = get_box_sum(bx0, by0, bx1, by1);
                const float m_i = static_cast<float>(box_sum) * inv_cell_area;

                if (m0 <= m_i) {
                    is_local_maximum = false;
                    break;
                }

                // Relative contrast measure: C_n = (m0 - m_i) * m0 / (m_i + epsilon)
                const float contrast = ((m0 - m_i) * m0) / (m_i + 1.0f);
                if (contrast < min_contrast) {
                    min_contrast = contrast;
                }
            }

            if (is_local_maximum && min_contrast < 1.0e8f) {
                out_saliency[center_idx] = min_contrast;
            }
        }
    }
}

void PointTargetLCM::compute_multiscale_saliency(std::span<const uint16_t> in_frame,
                                                 std::span<float> out_saliency) {
    std::vector<float> temp_scale(total_pixels_, 0.0f);
    std::fill(out_saliency.begin(), out_saliency.end(), 0.0f);

    const int scales[] = {3, 5, 7};
    for (int s : scales) {
        compute_saliency_map(in_frame, temp_scale, s);
        for (size_t i = 0; i < total_pixels_; ++i) {
            if (temp_scale[i] > out_saliency[i]) {
                out_saliency[i] = temp_scale[i];
            }
        }
    }
}

} // namespace ir_sim::detection
