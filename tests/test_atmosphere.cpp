#include "test_main.hpp"
#include "ir_sim/physics/atmosphere.hpp"

using namespace ir_sim::physics;

TEST_CASE(beer_lambert_transmittance_decay) {
    Atmosphere atmos(WeatherCondition::ClearSummer);
    const auto lwir = ir_sim::core::SpectralBand::lwir();

    // Zero distance has 100% transmission
    REQUIRE_NEAR(atmos.transmittance(0.0, lwir), 1.0, 1e-6);

    const double tau_100m = atmos.transmittance(100.0, lwir);
    const double tau_500m = atmos.transmittance(500.0, lwir);
    const double tau_2000m = atmos.transmittance(2000.0, lwir);

    // Monotonic decay with distance
    REQUIRE(tau_100m < 1.0);
    REQUIRE(tau_500m < tau_100m);
    REQUIRE(tau_2000m < tau_500m);

    // Light fog must have significantly lower transmission than clear weather
    Atmosphere fog(WeatherCondition::LightFog);
    const double tau_fog_500m = fog.transmittance(500.0, lwir);
    REQUIRE(tau_fog_500m < tau_500m * 0.5);
}

TEST_CASE(path_radiance_growth) {
    Atmosphere atmos(WeatherCondition::ClearSummer);
    const auto lwir = ir_sim::core::SpectralBand::lwir();
    const double air_temp = 290.0;

    const double l_path_100m = atmos.path_radiance(100.0, lwir, air_temp);
    const double l_path_2000m = atmos.path_radiance(2000.0, lwir, air_temp);

    // Path radiance must grow as optical depth increases
    REQUIRE(l_path_100m > 0.0);
    REQUIRE(l_path_2000m > l_path_100m);
}

TEST_CASE(at_aperture_radiance_composition) {
    Atmosphere atmos(WeatherCondition::ClearSummer);
    const auto lwir = ir_sim::core::SpectralBand::lwir();

    const double surf_temp = 320.0;     // Warm engine hood
    const double ambient_temp = 280.0;  // Cold ambient sky
    const double air_temp = 290.0;
    const double range_m = 500.0;

    // Blackbody target (emissivity = 1.0)
    const double l_aperture_bb = atmos.at_aperture_radiance(
        surf_temp, 1.0, ambient_temp, range_m, lwir, air_temp);

    // Specular reflecting target (emissivity = 0.1, reflects cold sky)
    const double l_aperture_refl = atmos.at_aperture_radiance(
        surf_temp, 0.1, ambient_temp, range_m, lwir, air_temp);

    // Low-emissivity target reflecting cold sky must appear significantly cooler (lower radiance)
    REQUIRE(l_aperture_refl < l_aperture_bb);
}

TEST_CASE(atmospheric_lapse_rate) {
    Atmosphere atmos(WeatherCondition::ClearSummer, 300.0);

    const double t_ground = atmos.temperature_at_altitude(0.0);
    const double t_1000m = atmos.temperature_at_altitude(1000.0);

    REQUIRE_NEAR(t_ground, 300.0, 1e-4);
    // Standard lapse rate is 6.5 K / km -> at 1000 m, temp should be ~ 293.5 K
    REQUIRE_NEAR(t_1000m, 293.5, 0.1);
}
