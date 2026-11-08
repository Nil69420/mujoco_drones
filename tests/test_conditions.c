#include "conditions/conditions.h"
#include "test_support.h"

#include <math.h>
#include <string.h>

#define PI 3.14159265358979323846

static void test_atmosphere(void) {
    atmosphere_sample_t sl = atmosphere_at(15.0, 101325.0, 0.0);
    CHECK_NEAR(sl.pressure_pa, 101325.0, 1e-6);
    CHECK_NEAR(sl.air_density, 1.225, 0.001);
    atmosphere_sample_t km = atmosphere_at(15.0, 101325.0, 1000.0);
    CHECK_REL(km.pressure_pa, 89874.6, 0.001);
    CHECK_NEAR(km.temp_c, 8.5, 0.01);
    atmosphere_sample_t hot = atmosphere_at(45.0, 100500.0, 0.0);
    CHECK(hot.air_density < sl.air_density * 0.93);
    atmosphere_sample_t cold = atmosphere_at(-25.0, 102000.0, 0.0);
    CHECK(cold.air_density > sl.air_density * 1.15);
}

static void test_presets(void) {
    CHECK(conditions_find("clear_calm") != NULL);
    CHECK(conditions_find("hot_desert") != NULL);
    CHECK(conditions_find("nonexistent") == NULL);
    CHECK(conditions_find(NULL) == NULL);
    for (size_t i = 0; i < CONDITIONS_PRESET_COUNT; i++) {
        CHECK(conditions_find(CONDITIONS_PRESETS[i].name) == &CONDITIONS_PRESETS[i]);
    }
}

static void test_wind_shear_and_direction(void) {
    conditions_profile_t p = {"t", 15.0, 101325.0, 5.0, PI / 2.0, 0.0, 1.0 / 7.0};
    wind_t w;
    wind_init(&w, &p, 1);
    double ned[3];
    wind_mean_at(&w, 6.096, ned);
    CHECK_NEAR(ned[0], 0.0, 1e-9);
    CHECK_NEAR(ned[1], 5.0, 1e-9);
    wind_mean_at(&w, 24.384, ned);
    CHECK_REL(ned[1], 5.0 * pow(4.0, 1.0 / 7.0), 1e-9);
}

static void test_dryden_statistics(void) {
    conditions_profile_t p = {"t", 15.0, 101325.0, 0.0, 0.0, 7.7, 1.0 / 7.0};
    wind_t w;
    wind_init(&w, &p, 99);
    const double height = 20.0;
    const double dt = 0.01;
    const int steps = 600000;
    double sum[3] = {0.0, 0.0, 0.0};
    double sumsq[3] = {0.0, 0.0, 0.0};
    double ned[3];
    for (int i = 0; i < steps; i++) {
        wind_step(&w, height, 5.0, dt, ned);
        for (int a = 0; a < 3; a++) {
            sum[a] += ned[a];
            sumsq[a] += ned[a] * ned[a];
        }
    }
    double h_ft = height * 3.28084;
    double scale = 0.177 + 0.000823 * h_ft;
    double sigma_w = 0.1 * 7.7;
    double sigma_uv = sigma_w / pow(scale, 0.4);
    double expect[3] = {sigma_uv, sigma_uv, sigma_w};
    for (int a = 0; a < 3; a++) {
        double mean = sum[a] / steps;
        double var = sumsq[a] / steps - mean * mean;
        CHECK_NEAR(mean, 0.0, 0.15 * expect[a]);
        CHECK_REL(sqrt(var), expect[a], 0.12);
    }
    wind_t a1;
    wind_t a2;
    wind_t b;
    wind_init(&a1, &p, 5);
    wind_init(&a2, &p, 5);
    wind_init(&b, &p, 6);
    double o1[3];
    double o2[3];
    double o3[3];
    for (int i = 0; i < 100; i++) {
        wind_step(&a1, height, 5.0, dt, o1);
        wind_step(&a2, height, 5.0, dt, o2);
        wind_step(&b, height, 5.0, dt, o3);
    }
    CHECK(memcmp(o1, o2, sizeof(o1)) == 0);
    CHECK(memcmp(o1, o3, sizeof(o1)) != 0);
}

static void test_thermal(void) {
    thermal_t t;
    thermal_init(&t, 20.0, 60.0, 15.0);
    const double dt = 0.01;
    for (int i = 0; i < 6000; i++) {
        thermal_step(&t, 20.0, dt);
    }
    CHECK_NEAR(t.temp_c, 20.0 + 15.0 * (1.0 - exp(-1.0)), 0.02);
    for (int i = 0; i < 600000; i++) {
        thermal_step(&t, 20.0, dt);
    }
    CHECK_NEAR(t.temp_c, 35.0, 0.01);
    thermal_step(&t, -10.0, dt);
    CHECK(t.temp_c < 35.0);
}

static double mount_gain(double f, double fn, double zeta) {
    double r = f / fn;
    double re = 1.0 - r * r;
    double im = 2.0 * zeta * r;
    return 1.0 / sqrt(re * re + im * im);
}

static void test_vibration_transfer(void) {
    const double fn = 40.0;
    const double zeta = 0.3;
    const double dt = 0.001;
    const double rated = 6000.0;
    double rpms[3] = {3000.0, 4500.0, 6000.0};
    for (int k = 0; k < 3; k++) {
        vibration_t v;
        CHECK(vibration_init(&v, 1, 2, rated, 20.0, fn, zeta, 7) == 0);
        double rpm[1] = {rpms[k]};
        double acc[3];
        double gyr[3];
        double peak = 0.0;
        for (int i = 0; i < 6000; i++) {
            vibration_step(&v, rpm, dt, acc, gyr);
            if (i > 3000 && fabs(acc[0]) > peak) {
                peak = fabs(acc[0]);
            }
        }
        double f = rpms[k] / 60.0;
        double ratio = rpms[k] / rated;
        double expected = 20.0 * ratio * ratio * v.rotor_scale[0] * mount_gain(f, fn, zeta);
        CHECK_REL(peak, expected, 0.06);
    }
    vibration_t idle;
    CHECK(vibration_init(&idle, 4, 2, rated, 20.0, fn, zeta, 3) == 0);
    double zero[4] = {0.0, 0.0, 0.0, 0.0};
    double acc[3];
    double gyr[3];
    for (int i = 0; i < 1000; i++) {
        vibration_step(&idle, zero, dt, acc, gyr);
    }
    CHECK_NEAR(acc[0], 0.0, 1e-12);
    CHECK_NEAR(acc[2], 0.0, 1e-12);
    vibration_t bad;
    CHECK(vibration_init(&bad, 0, 2, rated, 20.0, fn, zeta, 3) != 0);
    CHECK(vibration_init(&bad, 9, 2, rated, 20.0, fn, zeta, 3) != 0);
    CHECK(vibration_init(&bad, 4, 2, 0.0, 20.0, fn, zeta, 3) != 0);
}

int main(void) {
    test_atmosphere();
    test_presets();
    test_wind_shear_and_direction();
    test_dryden_statistics();
    test_thermal();
    test_vibration_transfer();
    if (g_failures != 0) {
        fprintf(stderr, "%d check(s) failed\n", g_failures);
        return 1;
    }
    puts("test_conditions OK");
    return 0;
}
