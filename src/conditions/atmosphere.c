#include "conditions/conditions.h"

#include <math.h>

#define ISA_LAPSE_K_PER_M 0.0065
#define GAS_CONSTANT_AIR 287.058
#define GRAVITY_MS2 9.80665
#define KELVIN_OFFSET 273.15

atmosphere_sample_t atmosphere_at(double sea_level_temp_c, double sea_level_pressure_pa,
                                  double altitude_m) {
    double t0 = sea_level_temp_c + KELVIN_OFFSET;
    double alt = altitude_m < -400.0 ? -400.0 : (altitude_m > 11000.0 ? 11000.0 : altitude_m);
    double t = t0 - ISA_LAPSE_K_PER_M * alt;
    double exponent = GRAVITY_MS2 / (GAS_CONSTANT_AIR * ISA_LAPSE_K_PER_M);
    double p = sea_level_pressure_pa * pow(t / t0, exponent);
    atmosphere_sample_t out;
    out.temp_c = t - KELVIN_OFFSET;
    out.pressure_pa = p;
    out.air_density = p / (GAS_CONSTANT_AIR * t);
    return out;
}
