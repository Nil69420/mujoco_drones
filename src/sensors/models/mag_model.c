#include "sensors/models/mag_model.h"

#include <math.h>
#include <string.h>

int mag_model_init(mag_model_t *model, const mag_profile_t *profile, double rate_hz,
                   uint64_t seed) {
    if (profile == NULL || rate_hz <= 0.0 || rate_hz > profile->max_rate_hz) {
        return -1;
    }
    memset(model, 0, sizeof(*model));
    model->profile = profile;
    noise_seed(&model->rng, seed);
    double axis_norm = 0.0;
    for (int i = 0; i < 3; i++) {
        model->hard_iron[i] = noise_gaussian(&model->rng, profile->hard_iron_sigma_nt);
        model->motor_axis[i] = noise_gaussian(&model->rng, 1.0);
        axis_norm += model->motor_axis[i] * model->motor_axis[i];
        for (int j = 0; j < 3; j++) {
            double off = noise_gaussian(&model->rng, profile->soft_iron_sigma);
            model->soft_iron[i][j] = (i == j ? 1.0 : 0.0) + off;
        }
    }
    axis_norm = sqrt(axis_norm);
    for (int i = 0; i < 3; i++) {
        model->motor_axis[i] /= axis_norm > 1e-9 ? axis_norm : 1.0;
    }
    return 0;
}

void mag_model_step(mag_model_t *model, const double field_body_nt[3], double battery_current_a,
                    double sensor_temp_c, double out_nt[3]) {
    const mag_profile_t *prof = model->profile;
    double temp = prof->tempco_nt_per_k * (sensor_temp_c - MAG_REFERENCE_TEMP_C);
    for (int i = 0; i < 3; i++) {
        double acc = 0.0;
        for (int j = 0; j < 3; j++) {
            acc += model->soft_iron[i][j] * field_body_nt[j];
        }
        acc += model->hard_iron[i] + prof->motor_nt_per_a * battery_current_a * model->motor_axis[i] +
             temp + noise_gaussian(&model->rng, prof->noise_nt);
        if (acc > prof->range_nt) {
            acc = prof->range_nt;
        } else if (acc < -prof->range_nt) {
            acc = -prof->range_nt;
        }
        if (prof->resolution_nt > 0.0) {
            acc = round(acc / prof->resolution_nt) * prof->resolution_nt;
        }
        out_nt[i] = acc;
    }
}

const mag_profile_t *mag_profile_find(const char *name) {
    if (name == NULL) {
        return NULL;
    }
    for (size_t i = 0; i < MAG_PROFILE_COUNT; i++) {
        if (strcmp(MAG_PROFILES[i].name, name) == 0) {
            return &MAG_PROFILES[i];
        }
    }
    return NULL;
}
