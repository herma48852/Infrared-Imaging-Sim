#include "test_main.hpp"
#include "ir_sim/physics/materials.hpp"

using namespace ir_sim::physics;

TEST_CASE(material_database_queries) {
    MaterialDatabase db;

    REQUIRE(db.contains("asphalt"));
    REQUIRE(db.contains("water"));
    REQUIRE(db.contains("bare_aluminum"));
    REQUIRE(db.contains("human_skin"));
    REQUIRE(db.contains("bird_plumage"));

    const auto& alum = db.get("bare_aluminum");
    const auto& water = db.get("water");
    const auto& bird = db.get("bird_plumage");

    // Bare aluminum is highly reflective in IR (low emissivity)
    REQUIRE(alum.emissivity_lwir < 0.20f);

    // Water is nearly a blackbody in LWIR (high emissivity)
    REQUIRE(water.emissivity_lwir > 0.95f);

    // Bird plumage has higher base body surface temp
    REQUIRE(bird.base_temperature_k > 310.0f);
}

TEST_CASE(diurnal_solar_heating) {
    MaterialDatabase db;
    const auto& asphalt = db.get("asphalt");

    const float ambient_temp = 295.0f;
    const float solar_flux = 800.0f; // Strong daylight sun [W/m^2]

    const float temp_night = MaterialDatabase::compute_diurnal_temperature(
        asphalt, 2.0f, ambient_temp, 0.0f); // 02:00 night
    const float temp_afternoon = MaterialDatabase::compute_diurnal_temperature(
        asphalt, 14.0f, ambient_temp, solar_flux); // 14:00 peak

    // Afternoon solar-heated asphalt must be significantly hotter than ambient and night
    REQUIRE(temp_afternoon > ambient_temp);
    REQUIRE(temp_afternoon > temp_night + 5.0f);
}
