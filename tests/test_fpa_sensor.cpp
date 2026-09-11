#include "test_main.hpp"
#include "ir_sim/sensor/fpa_sensor.hpp"

using namespace ir_sim::core;
using namespace ir_sim::sensor;

TEST_CASE(fpa_transduction_and_adc_quantization) {
    FPASpecs specs;
    specs.width = 64;
    specs.height = 64;
    specs.adc_bits = 14; // max count = 16383

    FPASensor sensor(specs, 12345);

    REQUIRE(sensor.width() == 64);
    REQUIRE(sensor.height() == 64);
    REQUIRE(sensor.max_adc_count() == 16383);

    RawFrame14Bit raw_frame(64, 64);
    sensor.generate_flat_field(293.15, raw_frame.as_span(), true);

    auto [min_val, max_val] = raw_frame.min_max();

    // Counts should be valid within 14-bit range
    REQUIRE(max_val <= 16383);
    // Baseline around 6000 counts
    REQUIRE(min_val >= 0);
    REQUIRE(max_val > 5000);
}

TEST_CASE(fpa_spatial_fpn_and_column_striping) {
    FPASpecs specs;
    specs.width = 128;
    specs.height = 128;
    specs.fpn_gain_sigma = 0.03f;         // 3% gain non-uniformity
    specs.column_fpn_sigma_counts = 60.0f;// Strong column striping
    specs.dead_pixel_ratio = 0.0f;        // Disable dead pixels for clean stats

    FPASensor sensor(specs, 999);

    RawFrame14Bit flat_frame(128, 128);
    sensor.generate_flat_field(295.0, flat_frame.as_span(), false);

    // Compute column-wise averages to verify vertical column striping variance
    std::vector<double> col_averages(128, 0.0);
    for (size_t y = 0; y < 128; ++y) {
        for (size_t x = 0; x < 128; ++x) {
            col_averages[x] += flat_frame(x, y);
        }
    }
    for (size_t x = 0; x < 128; ++x) {
        col_averages[x] /= 128.0;
    }

    // Measure variance across columns: must be non-zero due to column FPN
    double col_mean = 0.0;
    for (double c : col_averages) col_mean += c;
    col_mean /= 128.0;

    double col_var = 0.0;
    for (double c : col_averages) col_var += (c - col_mean) * (c - col_mean);
    col_var /= 128.0;

    REQUIRE(std::sqrt(col_var) > 20.0); // Column stripe standard deviation
}

TEST_CASE(fpa_defective_pixel_mask) {
    FPASpecs specs;
    specs.width = 100;
    specs.height = 100;
    specs.dead_pixel_ratio = 0.005f; // 0.5% dead pixels

    FPASensor sensor(specs, 42);

    REQUIRE(sensor.dead_pixel_count() > 0);
    REQUIRE(sensor.dead_pixel_count() < 100);

    const auto& mask = sensor.dead_pixel_mask();
    REQUIRE(mask.size() == 10000);

    RawFrame14Bit raw_frame(100, 100);
    sensor.generate_flat_field(293.15, raw_frame.as_span(), false);

    // Verify that every flagged dead pixel in mask produces 0 or 16383
    for (size_t i = 0; i < 10000; ++i) {
        if (mask[i] == 1) {
            REQUIRE(raw_frame.as_span()[i] == 0);
        } else if (mask[i] == 2) {
            REQUIRE(raw_frame.as_span()[i] == 16383);
        }
    }
}
