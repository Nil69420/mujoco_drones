#ifndef MUJOCO_DRONES_MAG_MODEL_H
#define MUJOCO_DRONES_MAG_MODEL_H

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
    double noise_nt;
    double hard_iron_sigma_nt;
    double soft_iron_sigma;
    double motor_nt_per_a;
    double tempco_nt_per_k;
    double resolution_nt;
    double range_nt;
    double max_rate_hz;
} mag_profile_t;

typedef struct {
    const mag_profile_t *profile;
    noise_rng_t rng;
    double hard_iron[3];
    double soft_iron[3][3];
    double motor_axis[3];
} mag_model_t;

#define MAG_REFERENCE_TEMP_C 25.0

int mag_model_init(mag_model_t *model, const mag_profile_t *profile, double rate_hz,
                   uint64_t seed);
void mag_model_step(mag_model_t *model, const double field_body_nt[3], double battery_current_a,
                    double sensor_temp_c, double out_nt[3]);

extern const mag_profile_t MAG_PROFILES[];
extern const size_t MAG_PROFILE_COUNT;
const mag_profile_t *mag_profile_find(const char *name);

#endif
