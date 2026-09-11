#include "ir_sim/physics/planck.hpp"

#include <algorithm>
#include <cmath>

namespace ir_sim::physics {

double Planck::spectral_exitance(double lambda_um, double temp_k) noexcept {
    if (lambda_um <= 0.0 || temp_k <= 0.0) {
        return 0.0;
    }

    // Exponent: c2 / (lambda * T)
    const double exponent = RADIATION_C2_UM / (lambda_um * temp_k);

    // Prevent floating point overflow for very large exponents (very cold or very short lambda)
    if (exponent > 700.0) {
        return 0.0;
    }

    const double denom_exp = std::expm1(exponent); // exp(x) - 1 with high precision for small x
    if (denom_exp <= 0.0) {
        return 0.0;
    }

    const double lambda2 = lambda_um * lambda_um;
    const double lambda5 = lambda2 * lambda2 * lambda_um;

    return RADIATION_C1_UM / (lambda5 * denom_exp);
}

double Planck::spectral_radiance(double lambda_um, double temp_k) noexcept {
    return spectral_exitance(lambda_um, temp_k) / std::numbers::pi;
}

double Planck::integrate_band_exitance(double lambda_min_um,
                                      double lambda_max_um,
                                      double temp_k,
                                      int num_steps) noexcept {
    if (lambda_min_um >= lambda_max_um || temp_k <= 0.0) {
        return 0.0;
    }

    // Ensure num_steps is a positive multiple of 3 for Simpson's 3/8 rule
    if (num_steps < 3) num_steps = 3;
    if (num_steps % 3 != 0) {
        num_steps += (3 - (num_steps % 3));
    }

    const double h = (lambda_max_um - lambda_min_um) / static_cast<double>(num_steps);
    double sum = spectral_exitance(lambda_min_um, temp_k) + spectral_exitance(lambda_max_um, temp_k);

    for (int i = 1; i < num_steps; ++i) {
        const double lambda = lambda_min_um + static_cast<double>(i) * h;
        const double val = spectral_exitance(lambda, temp_k);
        if (i % 3 == 0) {
            sum += 2.0 * val;
        } else {
            sum += 3.0 * val;
        }
    }

    return (3.0 * h / 8.0) * sum;
}

double Planck::integrate_band_radiance(double lambda_min_um,
                                      double lambda_max_um,
                                      double temp_k,
                                      int num_steps) noexcept {
    return integrate_band_exitance(lambda_min_um, lambda_max_um, temp_k, num_steps) / std::numbers::pi;
}

double Planck::in_band_radiance(const core::SpectralBand& band,
                               double temp_k,
                               double emissivity) noexcept {
    const double blackbody_l = integrate_band_radiance(band.lambda_min_um, band.lambda_max_um, temp_k);
    return emissivity * blackbody_l;
}

// ---------------------------------------------------------------------------
// RadianceLUT Implementation
// ---------------------------------------------------------------------------

Planck::RadianceLUT::RadianceLUT(double lambda_min_um, double lambda_max_um,
                                double t_min_k, double t_max_k, double step_k)
    : lambda_min_um_(lambda_min_um),
      lambda_max_um_(lambda_max_um),
      t_min_k_(t_min_k),
      t_max_k_(t_max_k),
      step_k_(step_k),
      inv_step_k_(1.0 / step_k) {
    const size_t num_entries = static_cast<size_t>(std::ceil((t_max_k - t_min_k) / step_k)) + 1;
    table_.resize(num_entries);

    for (size_t i = 0; i < num_entries; ++i) {
        const double t = t_min_k + static_cast<double>(i) * step_k;
        table_[i] = static_cast<float>(Planck::integrate_band_radiance(lambda_min_um, lambda_max_um, t, 60));
    }
}

double Planck::RadianceLUT::lookup(double temp_k) const noexcept {
    if (table_.empty()) {
        return 0.0;
    }
    if (temp_k <= t_min_k_) {
        return table_.front();
    }
    if (temp_k >= t_max_k_) {
        return table_.back();
    }

    const double float_idx = (temp_k - t_min_k_) * inv_step_k_;
    const size_t idx = static_cast<size_t>(float_idx);
    const double frac = float_idx - static_cast<double>(idx);

    if (idx + 1 < table_.size()) {
        return (1.0 - frac) * table_[idx] + frac * table_[idx + 1];
    }
    return table_[idx];
}

double Planck::RadianceLUT::inverse_lookup(double radiance) const noexcept {
    if (table_.empty() || radiance <= table_.front()) {
        return t_min_k_;
    }
    if (radiance >= table_.back()) {
        return t_max_k_;
    }

    const auto it = std::lower_bound(table_.begin(), table_.end(), static_cast<float>(radiance));
    if (it == table_.end()) {
        return t_max_k_;
    }

    const size_t idx = static_cast<size_t>(std::distance(table_.begin(), it));
    if (idx == 0) {
        return t_min_k_;
    }

    const double r0 = table_[idx - 1];
    const double r1 = table_[idx];
    const double frac = (r1 > r0) ? (radiance - r0) / (r1 - r0) : 0.0;

    return t_min_k_ + (static_cast<double>(idx - 1) + frac) * step_k_;
}

} // namespace ir_sim::physics
