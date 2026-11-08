#include "sensors/models/range_model.h"

#include <math.h>
#include <string.h>

#define DROPOUT_ONSET 0.8

int range_model_init(range_model_t *model, const range_profile_t *profile, double rate_hz,
                     uint64_t seed) {
    if (profile == NULL || rate_hz <= 0.0 || rate_hz > profile->max_rate_hz) {
        return -1;
    }
    memset(model, 0, sizeof(*model));
    model->profile = profile;
    noise_seed(&model->rng, seed);
    return 0;
}

double range_effective_max(const range_profile_t *profile, double reflectivity) {
    double refl = reflectivity < 0.1 ? 0.1 : (reflectivity > 0.9 ? 0.9 : reflectivity);
    double f = (refl - 0.1) / 0.8;
    return profile->max_range_10_m + (profile->max_range_90_m - profile->max_range_10_m) * f;
}

int range_model_step(range_model_t *model, double true_range_m, double reflectivity,
                     double *out_range_m) {
    const range_profile_t *prof = model->profile;
    double max_range = range_effective_max(prof, reflectivity);
    if (true_range_m < prof->min_range_m || true_range_m > max_range) {
        *out_range_m = 0.0;
        return 0;
    }
    double edge = true_range_m / max_range;
    if (edge > DROPOUT_ONSET) {
        double chance = prof->dropout_probability * (edge - DROPOUT_ONSET) / (1.0 - DROPOUT_ONSET);
        if (noise_uniform(&model->rng) < chance) {
            *out_range_m = 0.0;
            return 0;
        }
    }
    double sigma = true_range_m <= prof->knee_m ? prof->sigma_near_m : prof->sigma_far_rel * true_range_m;
    double value = true_range_m + noise_gaussian(&model->rng, sigma);
    if (prof->resolution_m > 0.0) {
        value = round(value / prof->resolution_m) * prof->resolution_m;
    }
    *out_range_m = value < 0.0 ? 0.0 : value;
    return 1;
}

const range_profile_t *range_profile_find(const char *name) {
    if (name == NULL) {
        return NULL;
    }
    for (size_t i = 0; i < RANGE_PROFILE_COUNT; i++) {
        if (strcmp(RANGE_PROFILES[i].name, name) == 0) {
            return &RANGE_PROFILES[i];
        }
    }
    return NULL;
}
