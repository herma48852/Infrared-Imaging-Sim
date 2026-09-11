#include "test_main.hpp"
#include "ir_sim/physics/planck.hpp"

using namespace ir_sim::physics;

TEST_CASE(wien_displacement_peak) {
    const double temp_k = 300.0;
    const double lambda_peak = Planck::wien_peak_wavelength_um(temp_k);
    
    // lambda_max = 2897.77 / 300 ~= 9.659 um
    REQUIRE_NEAR(lambda_peak, 9.6592, 0.01);

    const double m_peak = Planck::spectral_exitance(lambda_peak, temp_k);
    const double m_lower = Planck::spectral_exitance(lambda_peak - 1.5, temp_k);
    const double m_upper = Planck::spectral_exitance(lambda_peak + 1.5, temp_k);

    // Peak exitance must be strictly greater than adjacent wavelengths
    REQUIRE(m_peak > m_lower);
    REQUIRE(m_peak > m_upper);
}

TEST_CASE(stefan_boltzmann_convergence) {
    const double temp_k = 300.0;
    const double analytical_total = Planck::stefan_boltzmann_exitance(temp_k);

    // sigma * 300^4 ~= 459.300 W/m^2
    REQUIRE_NEAR(analytical_total, 459.30, 0.1);

    // Wide-band numerical integration [0.5, 250 um] should capture > 99.8% of total blackbody radiation
    const double numerical_wide = Planck::integrate_band_exitance(0.5, 250.0, temp_k, 900);
    const double fraction = numerical_wide / analytical_total;

    REQUIRE(fraction > 0.998);
    REQUIRE(fraction <= 1.001);
}

TEST_CASE(lwir_radiance_values) {
    const double temp_ambient = 293.15; // 20 C
    const auto lwir = ir_sim::core::SpectralBand::lwir();

    const double radiance = Planck::in_band_radiance(lwir, temp_ambient, 1.0);

    // Standard LWIR 8-14 um blackbody at 20 C is ~ 48 - 52 W / (m^2 * sr)
    REQUIRE(radiance > 45.0);
    REQUIRE(radiance < 55.0);

    // Monotonicity: Higher temperature must produce higher radiance
    const double radiance_hot = Planck::in_band_radiance(lwir, temp_ambient + 10.0, 1.0);
    REQUIRE(radiance_hot > radiance);

    // Emissivity scaling: Radiance must scale linearly with emissivity
    const double radiance_half_eps = Planck::in_band_radiance(lwir, temp_ambient, 0.5);
    REQUIRE_NEAR(radiance_half_eps, radiance * 0.5, 1e-4);
}

TEST_CASE(radiance_lut_accuracy_and_inversion) {
    Planck::RadianceLUT lut(8.0, 14.0, 200.0, 400.0, 0.25);

    // Test lookup accuracy at arbitrary non-grid temperatures
    const double test_temps[] = {253.2, 288.75, 310.15, 365.42};

    for (double t : test_temps) {
        const double direct_val = Planck::integrate_band_radiance(8.0, 14.0, t, 60);
        const double lut_val = lut.lookup(t);

        // LUT linear interpolation relative error must be < 0.05%
        const double rel_err = std::abs(direct_val - lut_val) / direct_val;
        REQUIRE(rel_err < 0.0005);

        // Inverse lookup must recover temperature within 0.05 K
        const double recovered_t = lut.inverse_lookup(lut_val);
        REQUIRE_NEAR(recovered_t, t, 0.05);
    }
}
