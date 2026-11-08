#include "sensors/models/imu_model.h"
#include "test_support.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

#define ODR_HZ 50.0
#define SAMPLES 1000000
#define SEEDS 3

static double allan_dev(const double *rate, size_t n, double dt, size_t m) {
    double *theta = malloc((n + 1) * sizeof(double));
    if (theta == NULL) {
        return -1.0;
    }
    theta[0] = 0.0;
    for (size_t i = 0; i < n; i++) {
        theta[i + 1] = theta[i] + rate[i] * dt;
    }
    double tau = (double)m * dt;
    double sum = 0.0;
    size_t count = n + 1 - 2 * m;
    for (size_t k = 0; k < count; k++) {
        double d = theta[k + 2 * m] - 2.0 * theta[k + m] + theta[k];
        sum += d * d;
    }
    free(theta);
    return sqrt(sum / (2.0 * tau * tau * (double)count));
}

static double ensemble_dev(const double *series, size_t seeds, size_t n, double dt, size_t m) {
    double acc = 0.0;
    for (size_t s = 0; s < seeds; s++) {
        double d = allan_dev(series + s * n, n, dt, m);
        acc += d * d;
    }
    return sqrt(acc / (double)seeds);
}

static void validate_profile(const imu_profile_t *original) {
    const double dt = 1.0 / ODR_HZ;
    double gyro_sample_sigma = original->gyro_noise_density / sqrt(dt);
    double accel_sample_sigma = original->accel_noise_density / sqrt(dt);
    imu_profile_t p = *original;
    p.gyro_range = 4096.0 * gyro_sample_sigma;
    p.accel_range = 4096.0 * accel_sample_sigma;
    if (p.gyro_range < 40.0 * p.gyro_bias_instability) {
        p.gyro_range = 40.0 * p.gyro_bias_instability;
    }
    if (p.accel_range < 40.0 * p.accel_bias_instability) {
        p.accel_range = 40.0 * p.accel_bias_instability;
    }
    p.gyro_turn_on = 0.0;
    p.accel_turn_on = 0.0;
    p.gyro_scale_sigma = 0.0;
    p.accel_scale_sigma = 0.0;
    p.misalignment_sigma = 0.0;
    p.gyro_gsens = 0.0;
    p.gyro_tempco = 0.0;
    p.accel_tempco = 0.0;

    double *gyro = malloc((size_t)SEEDS * SAMPLES * sizeof(double));
    double *accel = malloc((size_t)SEEDS * SAMPLES * sizeof(double));
    CHECK(gyro != NULL && accel != NULL);
    if (gyro == NULL || accel == NULL) {
        free(gyro);
        free(accel);
        return;
    }
    const double zero[3] = {0.0, 0.0, 0.0};
    for (size_t s = 0; s < SEEDS; s++) {
        imu_model_t m;
        CHECK(imu_model_init(&m, &p, ODR_HZ, 1000 + s) == 0);
        for (size_t i = 0; i < SAMPLES; i++) {
            double g[3];
            double a[3];
            imu_model_step(&m, zero, zero, IMU_REFERENCE_TEMP_C, NULL, NULL, g, a);
            gyro[s * SAMPLES + i] = g[0];
            accel[s * SAMPLES + i] = a[0];
        }
    }

    const size_t m_white = 2;
    double tau_white = (double)m_white * dt;
    double g_white = ensemble_dev(gyro, SEEDS, SAMPLES, dt, m_white);
    double a_white = ensemble_dev(accel, SEEDS, SAMPLES, dt, m_white);
    CHECK_REL(g_white, original->gyro_noise_density / sqrt(tau_white), 0.06);
    CHECK_REL(a_white, original->accel_noise_density / sqrt(tau_white), 0.06);

    const size_t m_bias = (size_t)(2.0 * IMU_GM_TAU_S * ODR_HZ);
    double tau_bias = (double)m_bias * dt;
    double g_expect = sqrt(original->gyro_bias_instability * original->gyro_bias_instability +
                           original->gyro_noise_density * original->gyro_noise_density / tau_bias +
                           original->gyro_bias_walk * original->gyro_bias_walk * tau_bias / 3.0);
    double a_expect = sqrt(original->accel_bias_instability * original->accel_bias_instability +
                           original->accel_noise_density * original->accel_noise_density / tau_bias +
                           original->accel_bias_walk * original->accel_bias_walk * tau_bias / 3.0);
    CHECK_REL(ensemble_dev(gyro, SEEDS, SAMPLES, dt, m_bias), g_expect, 0.12);
    CHECK_REL(ensemble_dev(accel, SEEDS, SAMPLES, dt, m_bias), a_expect, 0.12);
    free(gyro);
    free(accel);
}

static void test_profiles_match_their_numbers(void) {
    CHECK(IMU_PROFILE_COUNT >= 6);
    for (size_t i = 0; i < IMU_PROFILE_COUNT; i++) {
        validate_profile(&IMU_PROFILES[i]);
    }
}

static void test_registry_and_limits(void) {
    CHECK(imu_profile_find("icm42688p") != NULL);
    CHECK(imu_profile_find("nope") == NULL);
    CHECK(imu_profile_find(NULL) == NULL);
    imu_model_t m;
    const imu_profile_t *p = imu_profile_find("mpu6050");
    CHECK(imu_model_init(&m, p, 1000.0, 1) == 0);
    CHECK(imu_model_init(&m, p, 1001.0, 1) != 0);
    CHECK(imu_model_init(&m, p, 0.0, 1) != 0);
    CHECK(imu_model_init(&m, NULL, 100.0, 1) != 0);
    for (size_t i = 0; i < IMU_PROFILE_COUNT; i++) {
        CHECK(IMU_PROFILES[i].name != NULL && IMU_PROFILES[i].source != NULL);
        CHECK(IMU_PROFILES[i].gyro_noise_density > 0.0 && IMU_PROFILES[i].accel_noise_density > 0.0);
    }
}

static void test_determinism(void) {
    const imu_profile_t *p = imu_profile_find("bmi088");
    imu_model_t a;
    imu_model_t b;
    imu_model_t c;
    CHECK(imu_model_init(&a, p, 1000.0, 77) == 0);
    CHECK(imu_model_init(&b, p, 1000.0, 77) == 0);
    CHECK(imu_model_init(&c, p, 1000.0, 78) == 0);
    const double w[3] = {0.1, -0.2, 0.3};
    const double f[3] = {0.0, 0.0, -9.80665};
    double ga[3];
    double gb[3];
    double gc[3];
    double aa[3];
    double ab[3];
    double ac[3];
    for (int i = 0; i < 500; i++) {
        imu_model_step(&a, w, f, 25.0, NULL, NULL, ga, aa);
        imu_model_step(&b, w, f, 25.0, NULL, NULL, gb, ab);
        imu_model_step(&c, w, f, 25.0, NULL, NULL, gc, ac);
    }
    CHECK(memcmp(ga, gb, sizeof(ga)) == 0 && memcmp(aa, ab, sizeof(aa)) == 0);
    CHECK(memcmp(ga, gc, sizeof(ga)) != 0);
}

static imu_profile_t quiet_profile(void) {
    imu_profile_t p = *imu_profile_find("icm42688p");
    p.gyro_noise_density = 0.0;
    p.accel_noise_density = 0.0;
    p.gyro_bias_instability = 0.0;
    p.accel_bias_instability = 0.0;
    p.gyro_bias_walk = 0.0;
    p.accel_bias_walk = 0.0;
    p.gyro_turn_on = 0.0;
    p.accel_turn_on = 0.0;
    p.gyro_scale_sigma = 0.0;
    p.accel_scale_sigma = 0.0;
    p.misalignment_sigma = 0.0;
    p.gyro_gsens = 0.0;
    p.gyro_tempco = 0.0;
    p.accel_tempco = 0.0;
    p.gyro_range = 100.0;
    p.accel_range = 400.0;
    p.bandwidth_hz = 1000.0;
    return p;
}

static void test_deterministic_effects(void) {
    const double zero[3] = {0.0, 0.0, 0.0};
    double g[3];
    double a[3];

    imu_profile_t p = quiet_profile();
    p.gyro_tempco = 0.002;
    p.accel_tempco = 0.01;
    imu_model_t m;
    CHECK(imu_model_init(&m, &p, 1000.0, 1) == 0);
    imu_model_step(&m, zero, zero, 45.0, NULL, NULL, g, a);
    CHECK_NEAR(g[1], 0.002 * 20.0, 100.0 / 32768.0);
    CHECK_NEAR(a[2], 0.01 * 20.0, 400.0 / 32768.0);

    p = quiet_profile();
    p.gyro_gsens = 0.001;
    CHECK(imu_model_init(&m, &p, 1000.0, 1) == 0);
    const double f[3] = {0.0, 0.0, 10.0};
    for (int i = 0; i < 100; i++) {
        imu_model_step(&m, zero, f, 25.0, NULL, NULL, g, a);
    }
    CHECK_NEAR(g[2], 0.01, 100.0 / 32768.0);
    CHECK_NEAR(a[2], 10.0, 400.0 / 32768.0);

    p = quiet_profile();
    p.gyro_range = 5.0;
    CHECK(imu_model_init(&m, &p, 1000.0, 1) == 0);
    const double big[3] = {50.0, -50.0, 1.0};
    for (int i = 0; i < 2000; i++) {
        imu_model_step(&m, big, zero, 25.0, NULL, NULL, g, a);
    }
    CHECK_NEAR(g[0], 5.0, 1e-9);
    CHECK_NEAR(g[1], -5.0, 1e-9);
    CHECK_NEAR(g[2], 1.0, 5.0 / 32768.0);

    p = quiet_profile();
    p.gyro_range = 3.2768;
    CHECK(imu_model_init(&m, &p, 1000.0, 1) == 0);
    const double odd[3] = {0.12345, 0.0, 0.0};
    for (int i = 0; i < 2000; i++) {
        imu_model_step(&m, odd, zero, 25.0, NULL, NULL, g, a);
    }
    double lsb = 3.2768 / 32768.0;
    CHECK_NEAR(fmod(fabs(g[0]) / lsb + 0.5, 1.0) - 0.5, 0.0, 1e-6);
    CHECK_NEAR(g[0], 0.12345, lsb);

    p = quiet_profile();
    p.bandwidth_hz = 10.0;
    CHECK(imu_model_init(&m, &p, 1000.0, 1) == 0);
    imu_model_step(&m, zero, zero, 25.0, NULL, NULL, g, a);
    const double step[3] = {1.0, 0.0, 0.0};
    int steps_to_63 = 0;
    for (int i = 1; i < 2000; i++) {
        imu_model_step(&m, step, zero, 25.0, NULL, NULL, g, a);
        if (steps_to_63 == 0 && g[0] >= 1.0 - exp(-1.0)) {
            steps_to_63 = i;
        }
    }
    CHECK_NEAR((double)steps_to_63 / 1000.0, 1.0 / (2.0 * M_PI * 10.0), 0.002);

    p = quiet_profile();
    CHECK(imu_model_init(&m, &p, 1000.0, 1) == 0);
    const double vib_a[3] = {0.5, 0.0, 0.0};
    const double vib_g[3] = {0.0, 0.02, 0.0};
    for (int i = 0; i < 200; i++) {
        imu_model_step(&m, zero, zero, 25.0, vib_a, vib_g, g, a);
    }
    CHECK_NEAR(a[0], 0.5, 400.0 / 32768.0);
    CHECK_NEAR(g[1], 0.02, 100.0 / 32768.0);
}

static void test_scale_factor_spread(void) {
    imu_profile_t p = quiet_profile();
    p.gyro_scale_sigma = 0.01;
    p.gyro_range = 100.0;
    const double input[3] = {10.0, 0.0, 0.0};
    const double zero[3] = {0.0, 0.0, 0.0};
    double sum = 0.0;
    double sumsq = 0.0;
    const int trials = 4000;
    for (int s = 0; s < trials; s++) {
        imu_model_t m;
        CHECK(imu_model_init(&m, &p, 1000.0, (uint64_t)(5000 + s)) == 0);
        double g[3];
        double a[3];
        for (int i = 0; i < 5; i++) {
            imu_model_step(&m, input, zero, 25.0, NULL, NULL, g, a);
        }
        double err = g[0] / 10.0 - 1.0;
        sum += err;
        sumsq += err * err;
    }
    double mean = sum / trials;
    double sd = sqrt(sumsq / trials - mean * mean);
    CHECK_NEAR(mean, 0.0, 0.0008);
    CHECK_REL(sd, 0.01, 0.06);
}

int main(void) {
    test_registry_and_limits();
    test_determinism();
    test_deterministic_effects();
    test_scale_factor_spread();
    test_profiles_match_their_numbers();
    if (g_failures != 0) {
        fprintf(stderr, "%d check(s) failed\n", g_failures);
        return 1;
    }
    puts("test_imu_model OK");
    return 0;
}
