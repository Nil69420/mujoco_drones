#ifndef MUJOCO_DRONES_RANGE_MODEL_H
#define MUJOCO_DRONES_RANGE_MODEL_H

#include <stddef.h>
#include <stdint.h>

#include "sensors/models/provenance.h"
#include "sensors/noise.h"

typedef struct {
    const char *name;
    const char *grade;
    const char *source;
    provenance_t accuracy_provenance;
    provenance_t other_provenance;
    double min_range_m;
    double max_range_90_m;
    double max_range_10_m;
    double sigma_near_m;
    double knee_m;
    double sigma_far_rel;
    double resolution_m;
    double dropout_probability;
    double max_rate_hz;
} range_profile_t;

typedef struct {
    const range_profile_t *profile;
    noise_rng_t rng;
} range_model_t;

int range_model_init(range_model_t *model, const range_profile_t *profile, double rate_hz,
                     uint64_t seed);
double range_effective_max(const range_profile_t *profile, double reflectivity);
int range_model_step(range_model_t *model, double true_range_m, double reflectivity,
                     double *out_range_m);

extern const range_profile_t RANGE_PROFILES[];
extern const size_t RANGE_PROFILE_COUNT;
const range_profile_t *range_profile_find(const char *name);

#endif
