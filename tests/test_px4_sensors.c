#include "bridge/px4_sensors.h"
#include "test_support.h"

#include <string.h>

static ap_state_t level_state(void) {
    ap_state_t state;
    memset(&state, 0, sizeof(state));
    state.quaternion_wxyz[0] = 1.0;
    state.accel_body_mps2[2] = -9.80665;
    return state;
}

static void test_pressure(void) {
    CHECK_NEAR(px4_isa_pressure_hpa(0.0), 1013.25, 1e-6);
    CHECK_NEAR(px4_isa_pressure_hpa(1000.0), 898.7, 0.2);
    CHECK_NEAR(px4_isa_pressure_hpa(5000.0), 540.2, 0.5);
    CHECK(px4_isa_pressure_hpa(1.0e6) == 0.0);
}

static void test_sensor_fields(void) {
    px4_home_t home = px4_home_default();
    ap_state_t state = level_state();
    state.timestamp_s = 0.004;
    state.gyro_rps[0] = 0.1;
    state.position_ned_m[2] = -10.0;
    px4_hil_sensor_t sensor;
    px4_sensor_from_state(&state, &home, &sensor);
    CHECK(sensor.time_usec == 4000 && sensor.fields_updated == PX4_FIELDS_ALL);
    CHECK_NEAR(sensor.acc_mps2[2], -9.80665, 1e-5);
    CHECK_NEAR(sensor.gyro_rps[0], 0.1, 1e-7);
    CHECK_NEAR(sensor.pressure_alt_m, 498.0, 1e-3);
    CHECK_NEAR(sensor.abs_pressure_hpa, px4_isa_pressure_hpa(498.0), 1e-3);
    CHECK_NEAR(sensor.temperature_c, 15.0 - 0.0065 * 498.0, 1e-3);
    CHECK_NEAR(sensor.mag_gauss[0], 0.21, 1e-6);
    CHECK_NEAR(sensor.mag_gauss[2], 0.43, 1e-6);
}

static void test_mag_rotates_with_attitude(void) {
    px4_home_t home = px4_home_default();
    ap_state_t state = level_state();
    double half = 0.70710678118654752;
    state.quaternion_wxyz[0] = half;
    state.quaternion_wxyz[3] = half;
    px4_hil_sensor_t sensor;
    px4_sensor_from_state(&state, &home, &sensor);
    CHECK_NEAR(sensor.mag_gauss[0], 0.0, 1e-6);
    CHECK_NEAR(sensor.mag_gauss[1], -0.21, 1e-6);
    CHECK_NEAR(sensor.mag_gauss[2], 0.43, 1e-6);
}

static void test_gps(void) {
    px4_home_t home = px4_home_default();
    ap_state_t state = level_state();
    state.timestamp_s = 1.5;
    state.position_ned_m[0] = 111.3195;
    state.position_ned_m[1] = 0.0;
    state.position_ned_m[2] = -20.0;
    state.velocity_ned_mps[0] = 0.0;
    state.velocity_ned_mps[1] = -3.0;
    state.velocity_ned_mps[2] = 0.5;
    px4_hil_gps_t gps;
    px4_gps_from_state(&state, &home, &gps);
    CHECK(gps.time_usec == 1500000 && gps.fix_type == 3 && gps.satellites == 12);
    CHECK_NEAR((double)gps.lat_e7, 473977420.0 + 10000.0, 3.0);
    CHECK_NEAR((double)gps.lon_e7, 85455940.0, 1.0);
    CHECK(gps.alt_mm == 508000);
    CHECK(gps.vn_cms == 0 && gps.ve_cms == -300 && gps.vd_cms == 50 && gps.vel_cms == 300);
    CHECK(gps.cog_cdeg == 27000);
    state.position_ned_m[1] = 164.430576;
    px4_gps_from_state(&state, &home, &gps);
    CHECK_NEAR((double)gps.lon_e7, 85477761.5, 3.0);
    state.velocity_ned_mps[0] = 1.0e6;
    px4_gps_from_state(&state, &home, &gps);
    CHECK(gps.vn_cms == 32767 && gps.vel_cms == 65535);
}

static void test_models_overlay(void) {
    px4_home_t home = px4_home_default();
    ap_state_t state = level_state();
    px4_hil_sensor_t truth;
    px4_sensor_from_state(&state, &home, &truth);

    sensor_imu_t imu;
    memset(&imu, 0, sizeof(imu));
    imu.accel[0] = 0.1;
    imu.accel[1] = 0.2;
    imu.accel[2] = 9.7;
    imu.gyro[0] = 0.01;
    imu.gyro[1] = 0.02;
    imu.gyro[2] = 0.03;
    imu.mag[0] = 21.0;
    imu.mag[1] = 2.0;
    imu.mag[2] = -43.0;
    sensor_baro_t baro;
    memset(&baro, 0, sizeof(baro));
    baro.pressure_pa = 95000.0;
    baro.temperature_c = 12.5;
    px4_model_samples_t samples = {&imu, true, true, &baro, true};
    px4_hil_sensor_t out = truth;
    px4_sensor_apply_models(&out, &samples);
    CHECK_NEAR(out.acc_mps2[0], 0.1, 1e-6);
    CHECK_NEAR(out.acc_mps2[1], -0.2, 1e-6);
    CHECK_NEAR(out.acc_mps2[2], -9.7, 1e-6);
    CHECK_NEAR(out.gyro_rps[2], -0.03, 1e-6);
    CHECK_NEAR(out.mag_gauss[0], 0.21, 1e-6);
    CHECK_NEAR(out.mag_gauss[1], -0.02, 1e-6);
    CHECK_NEAR(out.mag_gauss[2], 0.43, 1e-6);
    CHECK_NEAR(out.abs_pressure_hpa, 950.0, 1e-3);
    CHECK_NEAR(out.temperature_c, 12.5, 1e-6);
    CHECK(out.fields_updated == (PX4_FIELDS_ACCEL | PX4_FIELDS_GYRO | PX4_FIELDS_MAG | PX4_FIELDS_BARO));

    samples.imu_fresh = false;
    samples.baro_fresh = false;
    out = truth;
    px4_sensor_apply_models(&out, &samples);
    CHECK_NEAR(out.acc_mps2[2], -9.80665, 1e-5);
    CHECK_NEAR(out.abs_pressure_hpa, truth.abs_pressure_hpa, 1e-6);
    CHECK(out.fields_updated == (PX4_FIELDS_ACCEL | PX4_FIELDS_GYRO));

    samples.mag_modelled = false;
    samples.baro = NULL;
    out = truth;
    px4_sensor_apply_models(&out, &samples);
    CHECK(out.fields_updated == (PX4_FIELDS_ACCEL | PX4_FIELDS_GYRO | PX4_FIELDS_MAG | PX4_FIELDS_BARO));
    CHECK_NEAR(out.mag_gauss[0], 0.21, 1e-6);
}

int main(void) {
    test_pressure();
    test_sensor_fields();
    test_mag_rotates_with_attitude();
    test_gps();
    test_models_overlay();
    if (g_failures != 0) {
        fprintf(stderr, "%d failures\n", g_failures);
        return 1;
    }
    printf("px4_sensors tests passed\n");
    return 0;
}
