#include "sensors/models/baro_model.h"

#include <math.h>
#include <string.h>

#define ISA_EXPONENT 0.190263
#define ISA_T0_K 288.15
#define ISA_LAPSE 0.0065

int baro_model_init(baro_model_t *model, const baro_profile_t *profile, double rate_hz,
                    uint64_t seed) {
    if (profile == NULL || rate_hz <= 0.0 || rate_hz > profile->max_rate_hz) {
        return -1;
    }
    memset(model, 0, sizeof(*model));
    model->profile = profile;
    noise_seed(&model->rng, seed);
    model->dt = 1.0 / rate_hz;
    model->decay = exp(-model->dt / BARO_DRIFT_TAU_S);
    model->offset = noise_gaussian(&model->rng, profile->offset_sigma_pa);
    model->drift = noise_gaussian(&model->rng, profile->drift_sigma_pa);
    return 0;
}

double baro_model_step(baro_model_t *model, double true_pressure_pa, double sensor_temp_c,
                       double thrust_fraction) {
    const baro_profile_t *prof = model->profile;
    double drive = sqrt(1.0 - model->decay * model->decay);
    model->drift = model->decay * model->drift +
                   noise_gaussian(&model->rng, prof->drift_sigma_pa * drive);
    double value = true_pressure_pa + model->offset + model->drift +
                   prof->tempco_pa_per_k * (sensor_temp_c - BARO_REFERENCE_TEMP_C) -
                   prof->propwash_pa * thrust_fraction * thrust_fraction +
                   noise_gaussian(&model->rng, prof->noise_pa);
    if (prof->resolution_pa > 0.0) {
        value = round(value / prof->resolution_pa) * prof->resolution_pa;
    }
    return value;
}

double baro_pressure_to_altitude(double pressure_pa, double sea_level_pa) {
    return (ISA_T0_K / ISA_LAPSE) * (1.0 - pow(pressure_pa / sea_level_pa, ISA_EXPONENT));
}

const baro_profile_t *baro_profile_find(const char *name) {
    if (name == NULL) {
        return NULL;
    }
    for (size_t i = 0; i < BARO_PROFILE_COUNT; i++) {
        if (strcmp(BARO_PROFILES[i].name, name) == 0) {
            return &BARO_PROFILES[i];
        }
    }
    return NULL;
}
