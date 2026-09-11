#include "test_main.hpp"
#include "ir_sim/sensor/fpa_sensor.hpp"
#include "ir_sim/sensor/isp_pipeline.hpp"

using namespace ir_sim::core;
using namespace ir_sim::sensor;

TEST_CASE(isp_2point_nuc_calibration_recovery) {
    FPASpecs specs;
    specs.width = 128;
    specs.height = 128;
    specs.fpn_gain_sigma = 0.035f;        // 3.5% spatial gain non-uniformity
    specs.fpn_offset_sigma_counts = 75.0f;
    specs.column_fpn_sigma_counts = 50.0f;// Column striping
    specs.dead_pixel_ratio = 0.0f;        // Zero dead pixels for pure NUC metric check

    FPASensor sensor(specs, 777);
    ISPPipeline isp(128, 128);

    // 1. Generate calibration frames: Cold (288 K) and Hot (308 K)
    RawFrame14Bit cold_frame(128, 128);
    RawFrame14Bit hot_frame(128, 128);
    sensor.generate_flat_field(288.0, cold_frame.as_span(), false);
    sensor.generate_flat_field(308.0, hot_frame.as_span(), false);

    // 2. Generate an intermediate test frame at 298 K (uncalibrated)
    RawFrame14Bit uncalibrated_test(128, 128);
    sensor.generate_flat_field(298.0, uncalibrated_test.as_span(), false);

    const float nu_before_nuc = ISPPipeline::calculate_non_uniformity_percent(uncalibrated_test.as_span());

    // Before NUC, Non-Uniformity should be significant (> 2.5%)
    REQUIRE(nu_before_nuc > 2.5f);

    // 3. Calibrate Two-Point NUC
    isp.calibrate_2point(cold_frame.as_span(), hot_frame.as_span(), 288.0f, 308.0f);
    REQUIRE(isp.is_nuc_calibrated());

    // 4. Correct the test frame
    RawFrame14Bit corrected_test(128, 128);
    isp.correct_nuc_and_bpr(uncalibrated_test.as_span(), corrected_test.as_span());

    const float nu_after_nuc = ISPPipeline::calculate_non_uniformity_percent(corrected_test.as_span());

    // After NUC, Non-Uniformity should be dramatically suppressed (< 0.15%)
    REQUIRE(nu_after_nuc < 0.15f);
    REQUIRE(nu_after_nuc < nu_before_nuc * 0.05f); // > 95% reduction in fixed-pattern noise!
}

TEST_CASE(isp_bad_pixel_replacement) {
    ISPPipeline isp(20, 20);

    RawFrame14Bit frame(20, 20, 5000); // Baseline counts: 5000

    // Inject a dead pixel at (10, 10) with value 0, and a hot pixel at (5, 5) with value 16383
    frame(10, 10) = 0;
    frame(5, 5) = 16383;

    std::vector<uint8_t> bad_mask(400, 0);
    bad_mask[10 * 20 + 10] = 1; // dead
    bad_mask[5 * 20 + 5] = 2;  // hot
    isp.set_bad_pixel_mask(bad_mask);

    RawFrame14Bit corrected(20, 20);
    isp.correct_nuc_and_bpr(frame.as_span(), corrected.as_span());

    // Dead and hot pixels must be replaced with neighbor value (5000)
    REQUIRE(corrected(10, 10) == 5000);
    REQUIRE(corrected(5, 5) == 5000);
}

TEST_CASE(isp_plateau_agc_clahe_dynamic_range) {
    ISPPipeline isp(64, 64);

    RawFrame14Bit raw_14bit(64, 64);
    // Gradient ramp from 4500 to 7500 counts
    for (size_t y = 0; y < 64; ++y) {
        for (size_t x = 0; x < 64; ++x) {
            raw_14bit(x, y) = static_cast<uint16_t>(4500 + (x + y) * 23);
        }
    }

    DisplayFrame8Bit display_8bit(64, 64);
    isp.enhance_dynamic_range(raw_14bit.as_span(), display_8bit.as_span());

    auto [min_val, max_val] = display_8bit.min_max();

    // 8-bit dynamic range should span full [0 - 255] display range
    REQUIRE(min_val <= 5);
    REQUIRE(max_val >= 250);

    // Monotonicity: Top-right corner must have higher 8-bit value than bottom-left
    REQUIRE(display_8bit(63, 63) > display_8bit(0, 0));
}
