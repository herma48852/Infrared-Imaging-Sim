#pragma once

#include <numbers>

namespace ir_sim::physics {

// Fundamental Physical Constants (CODATA 2018 Recommended Values)
inline constexpr double SPEED_OF_LIGHT = 299'792'458.0;              // c [m/s]
inline constexpr double PLANCK_CONSTANT = 6.626'070'15e-34;          // h [J*s]
inline constexpr double BOLTZMANN_CONSTANT = 1.380'649e-23;          // k_B [J/K]
inline constexpr double STEFAN_BOLTZMANN = 5.670'374'419e-8;         // sigma [W/(m^2*K^4)]
inline constexpr double WIEN_DISPLACEMENT = 2897.771955;             // b [um*K]

// Derived Radiation Constants for Planck's Law
// c1 = 2 * pi * h * c^2  [W * m^2]
inline constexpr double RADIATION_C1 = 2.0 * std::numbers::pi * PLANCK_CONSTANT * SPEED_OF_LIGHT * SPEED_OF_LIGHT;
// c2 = h * c / k_B       [m * K]
inline constexpr double RADIATION_C2 = (PLANCK_CONSTANT * SPEED_OF_LIGHT) / BOLTZMANN_CONSTANT;

// In micrometer units (convenient for IR calculations where lambda is in um):
// c1_um = 2 * pi * h * c^2 * 10^24 [W * um^4 / m^2]
inline constexpr double RADIATION_C1_UM = RADIATION_C1 * 1.0e24;
// c2_um = c2 * 10^6 [um * K]
inline constexpr double RADIATION_C2_UM = RADIATION_C2 * 1.0e6;

// Standard Infrared Spectral Bands [micrometers]
inline constexpr double SWIR_MIN_UM = 0.9;
inline constexpr double SWIR_MAX_UM = 1.7;

inline constexpr double MWIR_MIN_UM = 3.0;
inline constexpr double MWIR_MAX_UM = 5.0;

inline constexpr double LWIR_MIN_UM = 8.0;
inline constexpr double LWIR_MAX_UM = 14.0;

// Standard Reference Temperatures [Kelvin]
inline constexpr double TEMP_ZERO_CELSIUS = 273.15;
inline constexpr double TEMP_HUMAN_CORE   = 310.15; // 37 C
inline constexpr double TEMP_HUMAN_SKIN   = 305.15; // 32 C
inline constexpr double TEMP_AMBIENT_STD  = 293.15; // 20 C
inline constexpr double TEMP_COLD_SKY     = 240.0;  // Apparent clear-sky cold temp

} // namespace ir_sim::physics
