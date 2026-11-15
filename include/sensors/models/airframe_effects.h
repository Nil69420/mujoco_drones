#ifndef MUJOCO_DRONES_AIRFRAME_EFFECTS_H
#define MUJOCO_DRONES_AIRFRAME_EFFECTS_H

#include <stdint.h>

#include "sensors/models/provenance.h"
#include "sensors/noise.h"

typedef struct {
    double accel_thrust_deficit_mps2;
    double baro_lag_s;
    double camera_drop_probability;
    double camera_time_shift_s;
    provenance_t provenance;
} airframe_effects_params_t;

typedef struct {
    airframe_effects_params_t params;
    noise_rng_t rng;
    double baro_state;
    int baro_primed;
} airframe_effects_t;

#define AIRFRAME_EFFECTS_MAX_DEFICIT_MPS2 20.0
#define AIRFRAME_EFFECTS_MAX_LAG_S 5.0
#define AIRFRAME_EFFECTS_MAX_SHIFT_S 1.0

int airframe_effects_init(airframe_effects_t *fx, const airframe_effects_params_t *params,
                          uint64_t seed);
double airframe_effects_accel_z_offset(const airframe_effects_t *fx, double thrust_fraction);
double airframe_effects_baro(airframe_effects_t *fx, double pressure_pa, double dt);
int airframe_effects_camera_dropped(airframe_effects_t *fx);
double airframe_effects_camera_stamp(const airframe_effects_t *fx, double capture_time_s);

#endif
