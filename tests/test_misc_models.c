#include "sensors/models/baro_model.h"
#include "sensors/models/mag_model.h"
#include "sensors/models/range_model.h"
#include "test_support.h"

#include <math.h>
#include <string.h>

static void test_baro_registry_and_limits(void) {
    baro_model_t m;
    CHECK(baro_profile_find("dps310") != NULL && baro_profile_find("bmp388") != NULL);
    CHECK(baro_profile_find("nope") == NULL && baro_profile_find(NULL) == NULL);
    CHECK(baro_model_init(&m, baro_profile_find("dps310"), 128.0, 1) == 0);
    CHECK(baro_model_init(&m, baro_profile_find("dps310"), 129.0, 1) != 0);
    CHECK(baro_model_init(&m, NULL, 10.0, 1) != 0);
}

static void test_baro_noise_tempco_and_altitude(void) {
    for (size_t k = 0; k < BARO_PROFILE_COUNT; k++) {
        baro_profile_t p = BARO_PROFILES[k];
        if (p.noise_pa == 0.0) {
            continue;
        }
        p.drift_sigma_pa = 0.0;
        p.offset_sigma_pa = 0.0;
        p.propwash_pa = 0.0;
        p.resolution_pa = 0.0;
        p.tempco_pa_per_k = 0.0;
        baro_model_t m;
        CHECK(baro_model_init(&m, &p, 50.0, 100 + k) == 0);
        double sumsq = 0.0;
        const int n = 400000;
        for (int i = 0; i < n; i++) {
            double e = baro_model_step(&m, 101325.0, 25.0, 0.0) - 101325.0;
            sumsq += e * e;
        }
        CHECK_REL(sqrt(sumsq / n), p.noise_pa, 0.01);
    }
    baro_profile_t q = *baro_profile_find("bmp388");
    q.noise_pa = 0.0;
    q.drift_sigma_pa = 0.0;
    q.offset_sigma_pa = 0.0;
    q.propwash_pa = 0.0;
    q.resolution_pa = 0.0;
    baro_model_t m;
    CHECK(baro_model_init(&m, &q, 50.0, 1) == 0);
    CHECK_NEAR(baro_model_step(&m, 100000.0, 45.0, 0.0), 100000.0 + 0.75 * 20.0, 1e-9);
    CHECK_NEAR(baro_model_step(&m, 100000.0, 5.0, 0.0), 100000.0 - 0.75 * 20.0, 1e-9);
    q.propwash_pa = 10.0;
    CHECK(baro_model_init(&m, &q, 50.0, 1) == 0);
    CHECK_NEAR(baro_model_step(&m, 100000.0, 25.0, 1.0), 100000.0 - 10.0, 1e-9);
    CHECK_NEAR(baro_pressure_to_altitude(101325.0, 101325.0), 0.0, 1e-9);
    CHECK_NEAR(baro_pressure_to_altitude(89874.6, 101325.0), 1000.0, 1.0);
    CHECK_NEAR(baro_pressure_to_altitude(101325.0 - 12.0, 101325.0), 1.0, 0.05);
}

static void test_baro_drift_and_offset_statistics(void) {
    baro_profile_t p = *baro_profile_find("dps310");
    p.noise_pa = 0.0;
    p.resolution_pa = 0.0;
    p.propwash_pa = 0.0;
    p.tempco_pa_per_k = 0.0;
    double sum_off = 0.0;
    double sumsq_off = 0.0;
    const int trials = 3000;
    for (int s = 0; s < trials; s++) {
        baro_model_t m;
        CHECK(baro_model_init(&m, &p, 10.0, (uint64_t)(9000 + s)) == 0);
        double e = baro_model_step(&m, 100000.0, 25.0, 0.0) - 100000.0;
        sum_off += e;
        sumsq_off += e * e;
    }
    double sigma_expected = sqrt(p.offset_sigma_pa * p.offset_sigma_pa + p.drift_sigma_pa * p.drift_sigma_pa);
    CHECK_NEAR(sum_off / trials, 0.0, sigma_expected * 0.06);
    CHECK_REL(sqrt(sumsq_off / trials), sigma_expected, 0.06);
}

static void test_mag(void) {
    CHECK(mag_profile_find("rm3100") != NULL && mag_profile_find("nope") == NULL);
    mag_model_t m;
    CHECK(mag_model_init(&m, mag_profile_find("rm3100"), 200.0, 1) == 0);
    CHECK(mag_model_init(&m, mag_profile_find("rm3100"), 201.0, 1) != 0);

    mag_profile_t p = *mag_profile_find("hmc5883l");
    p.hard_iron_sigma_nt = 0.0;
    p.soft_iron_sigma = 0.0;
    p.motor_nt_per_a = 0.0;
    p.tempco_nt_per_k = 0.0;
    p.resolution_nt = 0.0;
    CHECK(mag_model_init(&m, &p, 50.0, 3) == 0);
    const double field[3] = {20000.0, 0.0, 45000.0};
    double out[3];
    double sumsq = 0.0;
    const int n = 200000;
    for (int i = 0; i < n; i++) {
        mag_model_step(&m, field, 0.0, 25.0, out);
        double e = out[0] - field[0];
        sumsq += e * e;
    }
    CHECK_REL(sqrt(sumsq / n), p.noise_nt, 0.01);

    p.noise_nt = 0.0;
    p.motor_nt_per_a = 500.0;
    p.tempco_nt_per_k = 10.0;
    CHECK(mag_model_init(&m, &p, 50.0, 4) == 0);
    double idle[3];
    double loaded[3];
    mag_model_step(&m, field, 0.0, 25.0, idle);
    mag_model_step(&m, field, 20.0, 25.0, loaded);
    double dx = loaded[0] - idle[0];
    double dy = loaded[1] - idle[1];
    double dz = loaded[2] - idle[2];
    CHECK_NEAR(sqrt(dx * dx + dy * dy + dz * dz), 500.0 * 20.0, 1e-6);
    double hot[3];
    mag_model_step(&m, field, 0.0, 45.0, hot);
    CHECK_NEAR(hot[0] - idle[0], 200.0, 1e-6);

    p.hard_iron_sigma_nt = 2000.0;
    p.motor_nt_per_a = 0.0;
    p.tempco_nt_per_k = 0.0;
    CHECK(mag_model_init(&m, &p, 50.0, 5) == 0);
    double a[3];
    double b[3];
    mag_model_step(&m, field, 0.0, 25.0, a);
    mag_model_step(&m, field, 0.0, 25.0, b);
    CHECK(memcmp(a, b, sizeof(a)) == 0);
    CHECK(fabs(a[0] - field[0]) + fabs(a[1]) + fabs(a[2] - field[2]) > 1.0);

    p.hard_iron_sigma_nt = 0.0;
    p.range_nt = 30000.0;
    CHECK(mag_model_init(&m, &p, 50.0, 6) == 0);
    mag_model_step(&m, field, 0.0, 25.0, a);
    CHECK_NEAR(a[2], 30000.0, 1e-9);
}

static void test_range(void) {
    for (size_t i = 0; i < RANGE_PROFILE_COUNT; i++) {
        const range_profile_t *rp = &RANGE_PROFILES[i];
        CHECK(rp->max_range_90_m > rp->min_range_m && rp->max_range_10_m <= rp->max_range_90_m);
        CHECK(rp->max_rate_hz > 0.0 && rp->resolution_m >= 0.0);
    }
    CHECK(range_profile_find("tf02_pro") != NULL && range_profile_find("lidar_lite_v3") != NULL &&
          range_profile_find("sf11_c") != NULL);
    CHECK_NEAR(range_effective_max(range_profile_find("tf02_pro"), 0.1), 13.5, 0.05);
    CHECK_NEAR(range_effective_max(range_profile_find("tf02_pro"), 0.9), 40.0, 0.05);
    const range_profile_t *tf = range_profile_find("tfmini_s");
    CHECK(tf != NULL && range_profile_find("nope") == NULL);
    range_model_t m;
    CHECK(range_model_init(&m, tf, 1000.0, 1) == 0);
    CHECK(range_model_init(&m, tf, 1001.0, 1) != 0);
    CHECK_NEAR(range_effective_max(tf, 0.9), 12.0, 1e-9);
    CHECK_NEAR(range_effective_max(tf, 0.1), 7.0, 1e-9);
    CHECK_NEAR(range_effective_max(tf, 5.0), 12.0, 1e-9);
    CHECK_NEAR(range_effective_max(tf, 0.0), 7.0, 1e-9);

    double out = 1.0;
    CHECK(range_model_init(&m, tf, 100.0, 7) == 0);
    CHECK(range_model_step(&m, 0.05, 0.9, &out) == 0 && out == 0.0);
    CHECK(range_model_step(&m, 13.0, 0.9, &out) == 0 && out == 0.0);
    CHECK(range_model_step(&m, 8.0, 0.1, &out) == 0 && out == 0.0);

    double near_sq = 0.0;
    double far_sq = 0.0;
    int n_near = 0;
    int n_far = 0;
    for (int i = 0; i < 200000; i++) {
        if (range_model_step(&m, 3.0, 0.9, &out)) {
            near_sq += (out - 3.0) * (out - 3.0);
            n_near++;
            double grid = out / tf->resolution_m;
            CHECK(fabs(grid - round(grid)) < 1e-6);
        }
        if (range_model_step(&m, 8.0, 0.9, &out)) {
            far_sq += (out - 8.0) * (out - 8.0);
            n_far++;
        }
    }
    CHECK(n_near == 200000);
    CHECK_REL(sqrt(near_sq / n_near), 0.03, 0.04);
    CHECK_REL(sqrt(far_sq / n_far), 0.005 * 8.0, 0.04);
    CHECK(n_far == 200000);

    int valid = 0;
    const int trials = 200000;
    for (int i = 0; i < trials; i++) {
        valid += range_model_step(&m, 11.9, 0.9, &out);
    }
    double drop = 1.0 - (double)valid / trials;
    double edge = 11.9 / 12.0;
    CHECK_NEAR(drop, 0.5 * (edge - 0.8) / 0.2, 0.01);
}

int main(void) {
    test_baro_registry_and_limits();
    test_baro_noise_tempco_and_altitude();
    test_baro_drift_and_offset_statistics();
    test_mag();
    test_range();
    if (g_failures != 0) {
        fprintf(stderr, "%d check(s) failed\n", g_failures);
        return 1;
    }
    puts("test_misc_models OK");
    return 0;
}
