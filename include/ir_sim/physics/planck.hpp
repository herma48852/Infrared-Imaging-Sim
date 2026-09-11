#pragma once

#include "ir_sim/core/types.hpp"
#include "ir_sim/physics/constants.hpp"

#include <vector>

namespace ir_sim::physics {

/**
 * @brief High-precision Planck Blackbody Radiometry Engine.
 * 
 * Provides spectral exitance, spectral radiance, bandpass numerical integration,
 * Stefan-Boltzmann total radiance, Wien's peak calculation, and fast LUT lookups
 * for real-time sensor simulation.
 */
class Planck {
public:
    Planck() = default;

    /**
     * @brief Computes blackbody spectral radiant exitance M(lambda, T) via Planck's Law.
     * @param lambda_um Wavelength in micrometers [um].
     * @param temp_k Physical temperature in Kelvin [K].
     * @return Spectral radiant exitance [W / (m^2 * um)].
     */
    [[nodiscard]] static double spectral_exitance(double lambda_um, double temp_k) noexcept;

    /**
     * @brief Computes blackbody spectral radiance L(lambda, T) for a Lambertian surface: L = M / pi.
     * @param lambda_um Wavelength in micrometers [um].
     * @param temp_k Physical temperature in Kelvin [K].
     * @return Spectral radiance [W / (m^2 * sr * um)].
     */
    [[nodiscard]] static double spectral_radiance(double lambda_um, double temp_k) noexcept;

    /**
     * @brief Numerically integrates Planck exitance over [lambda_min_um, lambda_max_um]
     *        using Composite Simpson's 3/8 Quadrature.
     * @param lambda_min_um Lower wavelength limit [um].
     * @param lambda_max_um Upper wavelength limit [um].
     * @param temp_k Temperature in Kelvin [K].
     * @param num_steps Number of integration steps (must be multiple of 3, default: 60).
     * @return In-band radiant exitance [W / m^2].
     */
    [[nodiscard]] static double integrate_band_exitance(double lambda_min_um,
                                                        double lambda_max_um,
                                                        double temp_k,
                                                        int num_steps = 60) noexcept;

    /**
     * @brief Numerically integrates Planck radiance over [lambda_min_um, lambda_max_um].
     * @return In-band radiance [W / (m^2 * sr)].
     */
    [[nodiscard]] static double integrate_band_radiance(double lambda_min_um,
                                                        double lambda_max_um,
                                                        double temp_k,
                                                        int num_steps = 60) noexcept;

    /**
     * @brief Computes in-band radiance for a specific standard or custom spectral band.
     */
    [[nodiscard]] static double in_band_radiance(const core::SpectralBand& band,
                                                 double temp_k,
                                                 double emissivity = 1.0) noexcept;

    /**
     * @brief Analytical total hemispherical radiant exitance via Stefan-Boltzmann law: M = sigma * T^4.
     * @param temp_k Temperature in Kelvin [K].
     * @return Total radiant exitance [W / m^2].
     */
    [[nodiscard]] static constexpr double stefan_boltzmann_exitance(double temp_k) noexcept {
        const double t2 = temp_k * temp_k;
        return STEFAN_BOLTZMANN * t2 * t2;
    }

    /**
     * @brief Analytical total radiance for a Lambertian blackbody: L = (sigma * T^4) / pi.
     */
    [[nodiscard]] static constexpr double stefan_boltzmann_radiance(double temp_k) noexcept {
        return stefan_boltzmann_exitance(temp_k) / std::numbers::pi;
    }

    /**
     * @brief Wien's Displacement Law: Computes peak emission wavelength lambda_max.
     * @param temp_k Temperature in Kelvin [K].
     * @return Peak wavelength in micrometers [um].
     */
    [[nodiscard]] static constexpr double wien_peak_wavelength_um(double temp_k) noexcept {
        return (temp_k > 1.0e-4) ? (WIEN_DISPLACEMENT / temp_k) : 0.0;
    }

    /**
     * @brief Fast Radiance Look-Up Table (LUT) for inner loops.
     */
    class RadianceLUT {
    public:
        RadianceLUT() = default;
        RadianceLUT(double lambda_min_um, double lambda_max_um,
                    double t_min_k = 150.0, double t_max_k = 800.0, double step_k = 0.25);

        [[nodiscard]] double lookup(double temp_k) const noexcept;
        [[nodiscard]] double inverse_lookup(double radiance) const noexcept;

        [[nodiscard]] double lambda_min_um() const noexcept { return lambda_min_um_; }
        [[nodiscard]] double lambda_max_um() const noexcept { return lambda_max_um_; }

    private:
        double lambda_min_um_{8.0};
        double lambda_max_um_{14.0};
        double t_min_k_{150.0};
        double t_max_k_{800.0};
        double step_k_{0.25};
        double inv_step_k_{4.0};
        std::vector<float> table_;
    };
};

} // namespace ir_sim::physics
