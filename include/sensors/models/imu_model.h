#ifndef MUJOCO_DRONES_IMU_MODEL_H
#define MUJOCO_DRONES_IMU_MODEL_H

#include <stddef.h>
#include <stdint.h>

#include "sensors/models/provenance.h"
#include "sensors/noise.h"

typedef struct {
    const char *name;
    const char *grade;
    const char *source;
    provenance_t noise_provenance;
    provenance_t bias_provenance;
    double gyro_noise_density;
    double gyro_bias_instability;
    double gyro_bias_walk;
    double gyro_tempco;
    double gyro_turn_on;
    double gyro_scale_sigma;
    double gyro_range;
    double gyro_gsens;
    double accel_noise_density;
    double accel_bias_instability;
    double accel_bias_walk;
    double accel_tempco;
    double accel_turn_on;
    double accel_scale_sigma;
    double accel_range;
    double misalignment_sigma;
    double bandwidth_hz;
    double max_odr_hz;
} imu_profile_t;

typedef struct {
    const imu_profile_t *profile;
    noise_rng_t rng;
    double dt;
    double lpf_alpha;
    int lpf_primed;
    double gm_decay;
    double gyro_gm_sigma;
    double accel_gm_sigma;
    double gyro_gm[3];
    double accel_gm[3];
    double gyro_walk[3];
    double accel_walk[3];
    double gyro_turn_on[3];
    double accel_turn_on[3];
    double gyro_scale[3];
    double accel_scale[3];
    double misalignment[3][3];
    double gyro_lpf[3];
    double accel_lpf[3];
} imu_model_t;

#define IMU_BIAS_TO_GM_SIGMA 0.617
#define IMU_GM_TAU_S 50.0
#define IMU_REFERENCE_TEMP_C 25.0

int imu_model_init(imu_model_t *model, const imu_profile_t *profile, double odr_hz,
                   uint64_t seed);
void imu_model_step(imu_model_t *model, const double true_gyro[3], const double true_accel[3],
                    double sensor_temp_c, const double vib_accel[3], const double vib_gyro[3],
                    double gyro_out[3], double accel_out[3]);

extern const imu_profile_t IMU_PROFILES[];
extern const size_t IMU_PROFILE_COUNT;
const imu_profile_t *imu_profile_find(const char *name);

#endif
