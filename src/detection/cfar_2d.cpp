#include "ir_sim/detection/cfar_2d.hpp"

#include <algorithm>
#include <cassert>
#include <cmath>

namespace ir_sim::detection {

CFAR2D::CFAR2D(size_t width, size_t height)
    : CFAR2D(width, height, CFARParams{}) {}

CFAR2D::CFAR2D(size_t width, size_t height, CFARParams params)
    : width_(width),
      height_(height),
      total_pixels_(width * height),
      params_(params) {
    const size_t int_size = (width + 1) * (height + 1);
    integral_sum_.assign(int_size, 0);
    integral_sq_sum_.assign(int_size, 0.0);
}

std::vector<core::Detection> CFAR2D::detect(std::span<const uint16_t> in_frame) {
    assert(in_frame.size() >= total_pixels_ && "Input frame size mismatch");

    const int w = static_cast<int>(width_);
    const int h = static_cast<int>(height_);
    const int int_stride = w + 1;

    // 1. Compute 2D Integral Images for sum and sum-of-squares in O(N)
    for (int y = 0; y < h; ++y) {
        uint64_t row_sum = 0;
        double row_sq_sum = 0.0;
        const int in_row = y * w;
        const int curr_row = (y + 1) * int_stride;
        const int prev_row = y * int_stride;

        for (int x = 0; x < w; ++x) {
            const double val = static_cast<double>(in_frame[static_cast<size_t>(in_row + x)]);
            row_sum += static_cast<uint64_t>(val);
            row_sq_sum += val * val;

            const size_t c_idx = static_cast<size_t>(curr_row + x + 1);
            const size_t p_idx = static_cast<size_t>(prev_row + x + 1);

            integral_sum_[c_idx] = integral_sum_[p_idx] + row_sum;
            integral_sq_sum_[c_idx] = integral_sq_sum_[p_idx] + row_sq_sum;
        }
    }

    auto get_box = [&](int x1, int y1, int x2, int y2, uint64_t& out_sum, double& out_sq_sum) {
        x1 = std::clamp(x1, 0, w - 1);
        y1 = std::clamp(y1, 0, h - 1);
        x2 = std::clamp(x2, 0, w - 1);
        y2 = std::clamp(y2, 0, h - 1);

        const size_t idx_br = static_cast<size_t>(y2 + 1) * static_cast<size_t>(int_stride) + static_cast<size_t>(x2 + 1);
        const size_t idx_tr = static_cast<size_t>(y1) * static_cast<size_t>(int_stride) + static_cast<size_t>(x2 + 1);
        const size_t idx_bl = static_cast<size_t>(y2 + 1) * static_cast<size_t>(int_stride) + static_cast<size_t>(x1);
        const size_t idx_tl = static_cast<size_t>(y1) * static_cast<size_t>(int_stride) + static_cast<size_t>(x1);

        out_sum = integral_sum_[idx_br] - integral_sum_[idx_tr] - integral_sum_[idx_bl] + integral_sum_[idx_tl];
        out_sq_sum = integral_sq_sum_[idx_br] - integral_sq_sum_[idx_tr] - integral_sq_sum_[idx_bl] + integral_sq_sum_[idx_tl];
    };

    const int t_sz = params_.training_half_size;
    const int g_sz = params_.guard_half_size;
    const int outer_area = (2 * t_sz + 1) * (2 * t_sz + 1);
    const int guard_area = (2 * g_sz + 1) * (2 * g_sz + 1);
    const double n_train = static_cast<double>(outer_area - guard_area);
    const double inv_n_train = 1.0 / n_train;

    std::vector<core::Detection> candidate_hits;
    candidate_hits.reserve(128);

    // 2. Iterate through pixels with valid training window
    for (int y = t_sz; y < h - t_sz; ++y) {
        const int row_idx = y * w;
        for (int x = t_sz; x < w - t_sz; ++x) {
            const float cut_val = static_cast<float>(in_frame[static_cast<size_t>(row_idx + x)]);
            if (cut_val < params_.min_peak_value) continue;

            uint64_t sum_outer = 0;
            double sq_outer = 0.0;
            get_box(x - t_sz, y - t_sz, x + t_sz, y + t_sz, sum_outer, sq_outer);

            uint64_t sum_guard = 0;
            double sq_guard = 0.0;
            get_box(x - g_sz, y - g_sz, x + g_sz, y + g_sz, sum_guard, sq_guard);

            const double sum_ref = static_cast<double>(sum_outer - sum_guard);
            const double sq_ref = sq_outer - sq_guard;

            const double mu = sum_ref * inv_n_train;
            const double variance = std::max(1.0, (sq_ref * inv_n_train) - (mu * mu));
            const double sigma = std::sqrt(variance);

            const double threshold = mu + static_cast<double>(params_.threshold_factor) * sigma;
            const double scr = (static_cast<double>(cut_val) - mu) / sigma;

            if (static_cast<double>(cut_val) > threshold && scr >= static_cast<double>(params_.min_scr)) {
                core::Detection det{};
                det.bbox.x = static_cast<float>(x);
                det.bbox.y = static_cast<float>(y);
                det.bbox.width = 1.0f;
                det.bbox.height = 1.0f;
                det.bbox.score = static_cast<float>(std::clamp(scr / 10.0, 0.1, 1.0));
                det.peak_intensity = cut_val;
                det.scr = static_cast<float>(scr);
                candidate_hits.push_back(det);
            }
        }
    }

    // 3. Cluster and merge adjacent candidate hits into unified bounding boxes
    return cluster_detections(candidate_hits);
}

std::vector<core::Detection> CFAR2D::cluster_detections(const std::vector<core::Detection>& candidates) const {
    if (candidates.empty()) return {};

    std::vector<core::Detection> clusters;
    clusters.reserve(candidates.size());

    const float max_dist = static_cast<float>(params_.max_cluster_distance);

    for (const auto& hit : candidates) {
        bool merged = false;
        for (auto& cl : clusters) {
            const float cx = cl.bbox.x + cl.bbox.width * 0.5f;
            const float cy = cl.bbox.y + cl.bbox.height * 0.5f;
            const float dx = hit.bbox.x - cx;
            const float dy = hit.bbox.y - cy;

            if (std::sqrt(dx * dx + dy * dy) <= max_dist) {
                // Expand bounding box
                const float x_min = std::min(cl.bbox.x, hit.bbox.x);
                const float y_min = std::min(cl.bbox.y, hit.bbox.y);
                const float x_max = std::max(cl.bbox.x + cl.bbox.width, hit.bbox.x + hit.bbox.width);
                const float y_max = std::max(cl.bbox.y + cl.bbox.height, hit.bbox.y + hit.bbox.height);

                cl.bbox.x = x_min;
                cl.bbox.y = y_min;
                cl.bbox.width = x_max - x_min;
                cl.bbox.height = y_max - y_min;

                if (hit.peak_intensity > cl.peak_intensity) {
                    cl.peak_intensity = hit.peak_intensity;
                    cl.scr = hit.scr;
                    cl.bbox.score = hit.bbox.score;
                }
                merged = true;
                break;
            }
        }

        if (!merged) {
            clusters.push_back(hit);
        }
    }

    return clusters;
}

void CFAR2D::compute_scr_map(std::span<const uint16_t> in_frame, std::span<float> out_scr_map) {
    assert(in_frame.size() >= total_pixels_ && "Input frame size mismatch");
    assert(out_scr_map.size() >= total_pixels_ && "Output SCR map size mismatch");

    std::fill(out_scr_map.begin(), out_scr_map.end(), 0.0f);

    const int w = static_cast<int>(width_);
    const int h = static_cast<int>(height_);
    const int int_stride = w + 1;

    for (int y = 0; y < h; ++y) {
        uint64_t row_sum = 0;
        double row_sq_sum = 0.0;
        const int in_row = y * w;
        const int curr_row = (y + 1) * int_stride;
        const int prev_row = y * int_stride;

        for (int x = 0; x < w; ++x) {
            const double val = static_cast<double>(in_frame[static_cast<size_t>(in_row + x)]);
            row_sum += static_cast<uint64_t>(val);
            row_sq_sum += val * val;

            const size_t c_idx = static_cast<size_t>(curr_row + x + 1);
            const size_t p_idx = static_cast<size_t>(prev_row + x + 1);

            integral_sum_[c_idx] = integral_sum_[p_idx] + row_sum;
            integral_sq_sum_[c_idx] = integral_sq_sum_[p_idx] + row_sq_sum;
        }
    }

    auto get_box = [&](int x1, int y1, int x2, int y2, uint64_t& out_sum, double& out_sq_sum) {
        x1 = std::clamp(x1, 0, w - 1);
        y1 = std::clamp(y1, 0, h - 1);
        x2 = std::clamp(x2, 0, w - 1);
        y2 = std::clamp(y2, 0, h - 1);

        const size_t idx_br = static_cast<size_t>(y2 + 1) * static_cast<size_t>(int_stride) + static_cast<size_t>(x2 + 1);
        const size_t idx_tr = static_cast<size_t>(y1) * static_cast<size_t>(int_stride) + static_cast<size_t>(x2 + 1);
        const size_t idx_bl = static_cast<size_t>(y2 + 1) * static_cast<size_t>(int_stride) + static_cast<size_t>(x1);
        const size_t idx_tl = static_cast<size_t>(y1) * static_cast<size_t>(int_stride) + static_cast<size_t>(x1);

        out_sum = integral_sum_[idx_br] - integral_sum_[idx_tr] - integral_sum_[idx_bl] + integral_sum_[idx_tl];
        out_sq_sum = integral_sq_sum_[idx_br] - integral_sq_sum_[idx_tr] - integral_sq_sum_[idx_bl] + integral_sq_sum_[idx_tl];
    };

    const int t_sz = params_.training_half_size;
    const int g_sz = params_.guard_half_size;
    const double n_train = static_cast<double>((2 * t_sz + 1) * (2 * t_sz + 1) - (2 * g_sz + 1) * (2 * g_sz + 1));
    const double inv_n_train = 1.0 / n_train;

    for (int y = t_sz; y < h - t_sz; ++y) {
        const int row_idx = y * w;
        for (int x = t_sz; x < w - t_sz; ++x) {
            const float cut_val = static_cast<float>(in_frame[static_cast<size_t>(row_idx + x)]);

            uint64_t sum_outer = 0;
            double sq_outer = 0.0;
            get_box(x - t_sz, y - t_sz, x + t_sz, y + t_sz, sum_outer, sq_outer);

            uint64_t sum_guard = 0;
            double sq_guard = 0.0;
            get_box(x - g_sz, y - g_sz, x + g_sz, y + g_sz, sum_guard, sq_guard);

            const double sum_ref = static_cast<double>(sum_outer - sum_guard);
            const double sq_ref = sq_outer - sq_guard;

            const double mu = sum_ref * inv_n_train;
            const double variance = std::max(1.0, (sq_ref * inv_n_train) - (mu * mu));
            const double sigma = std::sqrt(variance);

            const float scr = static_cast<float>((static_cast<double>(cut_val) - mu) / sigma);
            out_scr_map[static_cast<size_t>(row_idx + x)] = std::max(0.0f, scr);
        }
    }
}

} // namespace ir_sim::detection
