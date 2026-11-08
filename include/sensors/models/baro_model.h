#ifndef MUJOCO_DRONES_BARO_MODEL_H
#define MUJOCO_DRONES_BARO_MODEL_H

#include <stddef.h>
#include <stdint.h>

#include "sensors/models/provenance.h"
#include "sensors/noise.h"

typedef struct {
    const char *name;
    const char *grade;
    const char *source;
    provenance_t noise_provenance;
    provenance_t other_provenance;
    double noise_pa;
    double drift_sigma_pa;
    double offset_sigma_pa;
    double tempco_pa_per_k;
    double propwash_pa;
    double resolution_pa;
    double max_rate_hz;
} baro_profile_t;

typedef struct {
    const baro_profile_t *profile;
    noise_rng_t rng;
    double dt;
    double decay;
    double offset;
    double drift;
} baro_model_t;

#define BARO_DRIFT_TAU_S 300.0
#define BARO_REFERENCE_TEMP_C 25.0

int baro_model_init(baro_model_t *model, const baro_profile_t *profile, double rate_hz,
                    uint64_t seed);
double baro_model_step(baro_model_t *model, double true_pressure_pa, double sensor_temp_c,
                       double thrust_fraction);
double baro_pressure_to_altitude(double pressure_pa, double sea_level_pa);

extern const baro_profile_t BARO_PROFILES[];
extern const size_t BARO_PROFILE_COUNT;
const baro_profile_t *baro_profile_find(const char *name);

#endif
