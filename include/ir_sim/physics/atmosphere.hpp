#pragma once

#include "ir_sim/core/types.hpp"
#include "ir_sim/physics/constants.hpp"

namespace ir_sim::physics {

// Standard Atmospheric Weather Conditions
enum class WeatherCondition {
    ClearSummer,    // Low aerosol, high humidity
    ClearWinter,    // Very dry, high transmission
    HazyHaze,       // Industrial/urban aerosols, moderate extinction
    LightFog,       // Radiation fog, high extinction
    ModerateRain    // Rain drops, scattering across LWIR/MWIR
};

/**
 * @brief Atmospheric Radiative Transfer and Slant-Range Extinction Engine.
 * 
 * Computes Beer-Lambert transmission tau(R) = exp(-beta * R) and path radiance
 * L_path(R) = (1 - tau) * L_bb(T_air) over slant ranges up to 10 km.
 */
class Atmosphere {
public:
    Atmosphere() = default;

    explicit Atmosphere(WeatherCondition condition,
                        double ground_temp_k = TEMP_AMBIENT_STD,
                        double relative_humidity = 0.50);

    /**
     * @brief Computes band-averaged atmospheric extinction coefficient beta [1/m].
     */
    [[nodiscard]] double extinction_coeff(const core::SpectralBand& band) const noexcept;

    /**
     * @brief Computes spectral transmittance over a given slant range: tau = exp(-beta * R).
     * @param slant_range_m Slant distance between drone and target [m].
     * @param band Spectral band of interest.
     * @return Transmittance [0.0 - 1.0].
     */
    [[nodiscard]] double transmittance(double slant_range_m,
                                       const core::SpectralBand& band) const noexcept;

    /**
     * @brief Computes atmospheric path radiance along the line-of-sight.
     * @param slant_range_m Slant range [m].
     * @param band Spectral band.
     * @param path_temp_k Mean air temperature along the slant path [K].
     * @return In-band path radiance [W / (m^2 * sr)].
     */
    [[nodiscard]] double path_radiance(double slant_range_m,
                                       const core::SpectralBand& band,
                                       double path_temp_k) const noexcept;

    /**
     * @brief Full Radiative Transfer: Computes total at-aperture radiance arriving at the drone sensor.
     * 
     * L_aperture = tau * [ eps * L_bb(T_surf) + (1 - eps) * L_ambient ] + L_path
     * 
     * @param surface_temp_k Target or ground surface physical temperature [K].
     * @param emissivity Target spectral emissivity [0 - 1].
     * @param ambient_temp_k Ambient/sky reflected temperature [K].
     * @param slant_range_m Distance from drone to target [m].
     * @param band Spectral band (LWIR / MWIR).
     * @param path_temp_k Mean air temperature along line-of-sight [K].
     * @return Total at-aperture radiance [W / (m^2 * sr)].
     */
    [[nodiscard]] double at_aperture_radiance(double surface_temp_k,
                                              double emissivity,
                                              double ambient_temp_k,
                                              double slant_range_m,
                                              const core::SpectralBand& band,
                                              double path_temp_k) const noexcept;

    /**
     * @brief Standard tropospheric temperature at altitude z via lapse rate: T(z) = T_0 - Gamma * z.
     * @param altitude_agl_m Altitude above ground [m].
     * @return Air temperature in Kelvin [K].
     */
    [[nodiscard]] double temperature_at_altitude(double altitude_agl_m) const noexcept;

    // Getters / Setters for environmental parameters
    void set_weather(WeatherCondition cond) noexcept { condition_ = cond; }
    void set_ground_temp(double t_k) noexcept { ground_temp_k_ = t_k; }
    void set_relative_humidity(double rh) noexcept { relative_humidity_ = rh; }

    [[nodiscard]] WeatherCondition weather() const noexcept { return condition_; }
    [[nodiscard]] double ground_temp() const noexcept { return ground_temp_k_; }
    [[nodiscard]] double relative_humidity() const noexcept { return relative_humidity_; }

private:
    WeatherCondition condition_{WeatherCondition::ClearSummer};
    double ground_temp_k_{TEMP_AMBIENT_STD};
    double relative_humidity_{0.50};
    double lapse_rate_k_per_m_{0.0065}; // Standard 6.5 K / km
};

} // namespace ir_sim::physics
