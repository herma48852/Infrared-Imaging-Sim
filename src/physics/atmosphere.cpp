#include "ir_sim/physics/atmosphere.hpp"
#include "ir_sim/physics/planck.hpp"

#include <algorithm>
#include <cmath>

namespace ir_sim::physics {

Atmosphere::Atmosphere(WeatherCondition condition,
                       double ground_temp_k,
                       double relative_humidity)
    : condition_(condition),
      ground_temp_k_(ground_temp_k),
      relative_humidity_(relative_humidity) {}

double Atmosphere::extinction_coeff(const core::SpectralBand& band) const noexcept {
    // Extinction coefficients derived from MODTRAN rural/urban standard models [1 / km] -> converted to [1 / m]
    // beta = beta_molecular_absorption + beta_aerosol_scattering
    double beta_per_km = 0.10;

    const bool is_lwir = (band.lambda_min_um >= 7.0);

    switch (condition_) {
        case WeatherCondition::ClearWinter:
            // Cold, low absolute humidity: exceptionally low LWIR water vapor absorption
            beta_per_km = is_lwir ? 0.06 : 0.08;
            break;

        case WeatherCondition::ClearSummer:
            // Warm, higher absolute humidity: moderate water continuum absorption in LWIR
            beta_per_km = is_lwir ? 0.12 : 0.14;
            // Scale slightly with relative humidity
            beta_per_km *= (0.7 + 0.6 * relative_humidity_);
            break;

        case WeatherCondition::HazyHaze:
            // Sub-micron aerosol particles: higher scattering in MWIR, moderate in LWIR
            beta_per_km = is_lwir ? 0.28 : 0.42;
            break;

        case WeatherCondition::LightFog:
            // Liquid water droplets (2 - 10 um): heavy extinction in both bands
            beta_per_km = is_lwir ? 1.85 : 2.10;
            break;

        case WeatherCondition::ModerateRain:
            // Rain drops (0.5 - 3 mm): geometry-dominated scattering
            beta_per_km = is_lwir ? 0.95 : 1.05;
            break;
    }

    // Convert from [1 / km] to [1 / m]
    return beta_per_km * 1.0e-3;
}

double Atmosphere::transmittance(double slant_range_m,
                                 const core::SpectralBand& band) const noexcept {
    if (slant_range_m <= 0.0) {
        return 1.0;
    }
    const double beta = extinction_coeff(band);
    return std::exp(-beta * slant_range_m);
}

double Atmosphere::path_radiance(double slant_range_m,
                                 const core::SpectralBand& band,
                                 double path_temp_k) const noexcept {
    const double tau = transmittance(slant_range_m, band);
    // Kirchhoff's law along the line-of-sight: emissivity of path = 1 - transmittance
    const double blackbody_path = Planck::in_band_radiance(band, path_temp_k, 1.0);
    return (1.0 - tau) * blackbody_path;
}

double Atmosphere::at_aperture_radiance(double surface_temp_k,
                                       double emissivity,
                                       double ambient_temp_k,
                                       double slant_range_m,
                                       const core::SpectralBand& band,
                                       double path_temp_k) const noexcept {
    // 1. Surface thermal self-emission
    const double l_bb_surf = Planck::in_band_radiance(band, surface_temp_k, 1.0);
    const double l_emitted = emissivity * l_bb_surf;

    // 2. Reflected ambient / diffuse sky thermal radiance
    const double l_bb_ambient = Planck::in_band_radiance(band, ambient_temp_k, 1.0);
    const double l_reflected = (1.0 - emissivity) * l_bb_ambient;

    // 3. Total surface exitance
    const double l_surface = l_emitted + l_reflected;

    // 4. Slant-range atmospheric transmission
    const double tau = transmittance(slant_range_m, band);

    // 5. In-scattering & atmospheric path radiance
    const double l_path = path_radiance(slant_range_m, band, path_temp_k);

    return tau * l_surface + l_path;
}

double Atmosphere::temperature_at_altitude(double altitude_agl_m) const noexcept {
    return std::max(180.0, ground_temp_k_ - lapse_rate_k_per_m_ * altitude_agl_m);
}

} // namespace ir_sim::physics
