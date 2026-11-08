#ifndef MUJOCO_DRONES_CONDITIONS_H
#define MUJOCO_DRONES_CONDITIONS_H

#include <stddef.h>
#include <stdint.h>

#include "sensors/noise.h"

#define CONDITIONS_MAX_ROTORS 8

typedef struct {
    const char *name;
    double ambient_temp_c;
    double sea_level_pressure_pa;
    double wind_speed_20ft_ms;
    double wind_toward_rad;
    double turbulence_w20_ms;
    double shear_exponent;
} conditions_profile_t;

typedef struct {
    double temp_c;
    double pressure_pa;
    double air_density;
} atmosphere_sample_t;

atmosphere_sample_t atmosphere_at(double sea_level_temp_c, double sea_level_pressure_pa,
                                  double altitude_m);

typedef struct {
    const conditions_profile_t *profile;
    double gust[3];
    noise_rng_t rng;
} wind_t;

void wind_init(wind_t *wind, const conditions_profile_t *profile, uint64_t seed);
void wind_mean_at(const wind_t *wind, double height_m, double out_ned[3]);
void wind_step(wind_t *wind, double height_m, double airspeed_ms, double dt, double out_ned[3]);

typedef struct {
    double temp_c;
    double time_constant_s;
    double self_heating_k;
} thermal_t;

void thermal_init(thermal_t *thermal, double start_temp_c, double time_constant_s,
                  double self_heating_k);
double thermal_step(thermal_t *thermal, double ambient_c, double dt);

typedef struct {
    int rotors;
    int blades;
    double rated_rpm;
    double imbalance_accel_ms2;
    double blade_pass_ratio;
    double gyro_per_accel;
    double mount_hz;
    double mount_zeta;
    double rotor_scale[CONDITIONS_MAX_ROTORS];
    double phase[CONDITIONS_MAX_ROTORS];
    double mount_pos[3];
    double mount_vel[3];
} vibration_t;

int vibration_init(vibration_t *vib, int rotors, int blades, double rated_rpm,
                   double imbalance_accel_ms2, double mount_hz, double mount_zeta,
                   uint64_t seed);
void vibration_step(vibration_t *vib, const double *rotor_rpm, double dt, double accel_out[3],
                    double gyro_out[3]);

void aero_drag_force(double air_density, double cd_area_m2, const double wind_world[3],
                     const double velocity_world[3], double force_out[3]);
void wind_ned_to_enu(const double ned[3], double enu[3]);

extern const conditions_profile_t CONDITIONS_PRESETS[];
extern const size_t CONDITIONS_PRESET_COUNT;
const conditions_profile_t *conditions_find(const char *name);

#endif
