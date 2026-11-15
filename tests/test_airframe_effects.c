#include "sensors/models/airframe_effects.h"
#include "test_support.h"

#include <math.h>
#include <string.h>

static airframe_effects_params_t good(void) {
    airframe_effects_params_t params;
    memset(&params, 0, sizeof(params));
    params.accel_thrust_deficit_mps2 = 0.4;
    params.baro_lag_s = 0.25;
    params.camera_drop_probability = 0.02;
    params.camera_time_shift_s = 0.012;
    params.provenance = PROVENANCE_SECONDARY;
    return params;
}

static void test_init_validation(void) {
    airframe_effects_t fx;
    airframe_effects_params_t params = good();
    CHECK(airframe_effects_init(&fx, &params, 1) == 0);
    CHECK(airframe_effects_init(NULL, &params, 1) != 0 && airframe_effects_init(&fx, NULL, 1) != 0);
    params = good();
    params.accel_thrust_deficit_mps2 = -0.1;
    CHECK(airframe_effects_init(&fx, &params, 1) != 0);
    params.accel_thrust_deficit_mps2 = AIRFRAME_EFFECTS_MAX_DEFICIT_MPS2 + 1.0;
    CHECK(airframe_effects_init(&fx, &params, 1) != 0);
    params = good();
    params.baro_lag_s = NAN;
    CHECK(airframe_effects_init(&fx, &params, 1) != 0);
    params.baro_lag_s = AIRFRAME_EFFECTS_MAX_LAG_S + 0.1;
    CHECK(airframe_effects_init(&fx, &params, 1) != 0);
    params = good();
    params.camera_drop_probability = 1.01;
    CHECK(airframe_effects_init(&fx, &params, 1) != 0);
    params.camera_drop_probability = -0.01;
    CHECK(airframe_effects_init(&fx, &params, 1) != 0);
    params = good();
    params.camera_time_shift_s = INFINITY;
    CHECK(airframe_effects_init(&fx, &params, 1) != 0);
    params.camera_time_shift_s = -AIRFRAME_EFFECTS_MAX_SHIFT_S - 0.01;
    CHECK(airframe_effects_init(&fx, &params, 1) != 0);
}

static void test_accel_deficit_follows_thrust(void) {
    airframe_effects_t fx;
    airframe_effects_params_t params = good();
    CHECK(airframe_effects_init(&fx, &params, 1) == 0);
    CHECK_NEAR(airframe_effects_accel_z_offset(&fx, 0.0), 0.0, 1e-15);
    CHECK_NEAR(airframe_effects_accel_z_offset(&fx, 0.5), 0.2, 1e-12);
    CHECK_NEAR(airframe_effects_accel_z_offset(&fx, 1.0), 0.4, 1e-12);
    CHECK_NEAR(airframe_effects_accel_z_offset(&fx, 7.0), 0.4, 1e-12);
    CHECK_NEAR(airframe_effects_accel_z_offset(&fx, -3.0), 0.0, 1e-15);
    CHECK_NEAR(airframe_effects_accel_z_offset(&fx, NAN), 0.0, 1e-15);
}

static void test_baro_lag_is_first_order(void) {
    airframe_effects_t fx;
    airframe_effects_params_t params = good();
    CHECK(airframe_effects_init(&fx, &params, 1) == 0);
    CHECK_NEAR(airframe_effects_baro(&fx, 101325.0, 0.01), 101325.0, 1e-9);
    double out = 0.0;
    double t = 0.0;
    while (t < params.baro_lag_s - 1e-9) {
        out = airframe_effects_baro(&fx, 101225.0, 0.001);
        t += 0.001;
    }
    CHECK_NEAR(101325.0 - out, 100.0 * (1.0 - exp(-1.0)), 0.05);
    for (int i = 0; i < 20000; i++) {
        out = airframe_effects_baro(&fx, 101225.0, 0.001);
    }
    CHECK_NEAR(out, 101225.0, 1e-3);
    double held = airframe_effects_baro(&fx, 101225.0, 0.0);
    CHECK(held == out);
    CHECK(airframe_effects_baro(&fx, 90000.0, NAN) == out);
    CHECK(airframe_effects_baro(&fx, 90000.0, -1.0) == out);
    params.baro_lag_s = 0.0;
    CHECK(airframe_effects_init(&fx, &params, 1) == 0);
    airframe_effects_baro(&fx, 100000.0, 0.01);
    CHECK_NEAR(airframe_effects_baro(&fx, 90000.0, 0.01), 90000.0, 1e-9);
}

static void test_camera_drop_rate_and_stamp(void) {
    airframe_effects_t fx;
    airframe_effects_params_t params = good();
    CHECK(airframe_effects_init(&fx, &params, 7) == 0);
    int dropped = 0;
    const int n = 400000;
    for (int i = 0; i < n; i++) {
        dropped += airframe_effects_camera_dropped(&fx);
    }
    CHECK_REL((double)dropped / n, params.camera_drop_probability, 0.03);
    CHECK_NEAR(airframe_effects_camera_stamp(&fx, 10.0), 10.012, 1e-12);
    params.camera_drop_probability = 0.0;
    CHECK(airframe_effects_init(&fx, &params, 7) == 0);
    for (int i = 0; i < 10000; i++) {
        CHECK(!airframe_effects_camera_dropped(&fx));
    }
    params.camera_drop_probability = 1.0;
    CHECK(airframe_effects_init(&fx, &params, 7) == 0);
    for (int i = 0; i < 10000; i++) {
        CHECK(airframe_effects_camera_dropped(&fx));
    }
}

int main(void) {
    test_init_validation();
    test_accel_deficit_follows_thrust();
    test_baro_lag_is_first_order();
    test_camera_drop_rate_and_stamp();
    if (g_failures != 0) {
        fprintf(stderr, "%d check(s) failed\n", g_failures);
        return 1;
    }
    puts("test_airframe_effects OK");
    return 0;
}
