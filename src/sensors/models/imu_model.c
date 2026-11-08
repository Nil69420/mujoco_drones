#include "sensors/models/imu_model.h"

#include <math.h>
#include <string.h>

#define LSB_BITS 15.0

static double clamp(double v, double limit) {
    if (v > limit) {
        return limit;
    }
    if (v < -limit) {
        return -limit;
    }
    return v;
}

static double quantize(double v, double range) {
    double lsb = range / pow(2.0, LSB_BITS);
    return round(v / lsb) * lsb;
}

int imu_model_init(imu_model_t *model, const imu_profile_t *profile, double odr_hz,
                   uint64_t seed) {
    if (profile == NULL || odr_hz <= 0.0 || odr_hz > profile->max_odr_hz) {
        return -1;
    }
    memset(model, 0, sizeof(*model));
    model->profile = profile;
    noise_seed(&model->rng, seed);
    model->dt = 1.0 / odr_hz;
    double bw = profile->bandwidth_hz;
    double nyquist = 0.5 * odr_hz;
    double cutoff = bw < nyquist ? bw : nyquist;
    model->lpf_alpha = 1.0 - exp(-2.0 * M_PI * cutoff * model->dt);
    model->gm_decay = exp(-model->dt / IMU_GM_TAU_S);
    model->gyro_gm_sigma = profile->gyro_bias_instability / IMU_BIAS_TO_GM_SIGMA;
    model->accel_gm_sigma = profile->accel_bias_instability / IMU_BIAS_TO_GM_SIGMA;
    for (int i = 0; i < 3; i++) {
        model->gyro_gm[i] = noise_gaussian(&model->rng, model->gyro_gm_sigma);
        model->accel_gm[i] = noise_gaussian(&model->rng, model->accel_gm_sigma);
        model->gyro_turn_on[i] = noise_gaussian(&model->rng, profile->gyro_turn_on);
        model->accel_turn_on[i] = noise_gaussian(&model->rng, profile->accel_turn_on);
        model->gyro_scale[i] = noise_gaussian(&model->rng, profile->gyro_scale_sigma);
        model->accel_scale[i] = noise_gaussian(&model->rng, profile->accel_scale_sigma);
        for (int j = 0; j < 3; j++) {
            model->misalignment[i][j] =
                i == j ? 0.0 : noise_gaussian(&model->rng, profile->misalignment_sigma);
        }
    }
    return 0;
}

static void channel_step(imu_model_t *m, double noise_density, double walk, double tempco,
                         double gm_sigma, double range, const double truth[3], double gm[3],
                         double rw[3], const double turn_on[3], const double scale[3],
                         double lpf[3], const double extra[3], double temp_delta, double out[3]) {
    double mixed[3];
    for (int i = 0; i < 3; i++) {
        mixed[i] = 0.0;
        for (int j = 0; j < 3; j++) {
            mixed[i] += (i == j ? 1.0 + scale[i] : m->misalignment[i][j]) * truth[j];
        }
    }
    double gm_drive = sqrt(1.0 - m->gm_decay * m->gm_decay);
    double white_sigma = noise_density / sqrt(m->dt);
    for (int i = 0; i < 3; i++) {
        double signal = mixed[i] + extra[i];
        if (!m->lpf_primed) {
            lpf[i] = signal;
        }
        lpf[i] += m->lpf_alpha * (signal - lpf[i]);
        gm[i] = m->gm_decay * gm[i] + noise_gaussian(&m->rng, gm_sigma * gm_drive);
        rw[i] += noise_gaussian(&m->rng, walk * sqrt(m->dt));
        double value = lpf[i] + turn_on[i] + gm[i] + rw[i] + tempco * temp_delta +
                       noise_gaussian(&m->rng, white_sigma);
        out[i] = quantize(clamp(value, range), range);
    }
}

void imu_model_step(imu_model_t *model, const double true_gyro[3], const double true_accel[3],
                    double sensor_temp_c, const double vib_accel[3], const double vib_gyro[3],
                    double gyro_out[3], double accel_out[3]) {
    const imu_profile_t *prof = model->profile;
    double temp_delta = sensor_temp_c - IMU_REFERENCE_TEMP_C;
    double accel_in[3];
    double gyro_extra[3];
    double accel_extra[3];
    for (int i = 0; i < 3; i++) {
        accel_in[i] = true_accel[i];
        accel_extra[i] = vib_accel != NULL ? vib_accel[i] : 0.0;
        gyro_extra[i] = (vib_gyro != NULL ? vib_gyro[i] : 0.0) +
                        prof->gyro_gsens * (true_accel[i] + accel_extra[i]);
    }
    channel_step(model, prof->gyro_noise_density, prof->gyro_bias_walk, prof->gyro_tempco,
                 model->gyro_gm_sigma, prof->gyro_range, true_gyro, model->gyro_gm,
                 model->gyro_walk, model->gyro_turn_on, model->gyro_scale, model->gyro_lpf,
                 gyro_extra, temp_delta, gyro_out);
    channel_step(model, prof->accel_noise_density, prof->accel_bias_walk, prof->accel_tempco,
                 model->accel_gm_sigma, prof->accel_range, accel_in, model->accel_gm,
                 model->accel_walk, model->accel_turn_on, model->accel_scale, model->accel_lpf,
                 accel_extra, temp_delta, accel_out);
    model->lpf_primed = 1;
}

const imu_profile_t *imu_profile_find(const char *name) {
    if (name == NULL) {
        return NULL;
    }
    for (size_t i = 0; i < IMU_PROFILE_COUNT; i++) {
        if (strcmp(IMU_PROFILES[i].name, name) == 0) {
            return &IMU_PROFILES[i];
        }
    }
    return NULL;
}
