#include "bridge/bf_packets.h"
#include "test_support.h"

#include <string.h>

/* Reference bytes dumped from the fdm_packet, rc_packet and servo_packet structs of Betaflight SITL */
static const uint8_t GOLDEN_FDM[144] = {
    0x58, 0x39, 0xb4, 0xc8, 0x76, 0xbe, 0xf3, 0x3f, 0x9a, 0x99, 0x99, 0x99,
    0x99, 0x99, 0xb9, 0x3f, 0x9a, 0x99, 0x99, 0x99, 0x99, 0x99, 0xc9, 0xbf,
    0x33, 0x33, 0x33, 0x33, 0x33, 0x33, 0xd3, 0x3f, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0xe0, 0x3f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xd0, 0xbf,
    0x05, 0xa3, 0x92, 0x3a, 0x01, 0x9d, 0x23, 0xc0, 0xcd, 0xcc, 0xcc, 0xcc,
    0xcc, 0xcc, 0xec, 0x3f, 0x9a, 0x99, 0x99, 0x99, 0x99, 0x99, 0xb9, 0x3f,
    0x9a, 0x99, 0x99, 0x99, 0x99, 0x99, 0xc9, 0xbf, 0x33, 0x33, 0x33, 0x33,
    0x33, 0x33, 0xd3, 0x3f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf8, 0x3f,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0xc0, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0xe8, 0x3f, 0xa3, 0xc9, 0xc5, 0x18, 0x58, 0x17, 0x21, 0x40,
    0x71, 0x1f, 0xb9, 0x35, 0xe9, 0xb2, 0x47, 0x40, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x28, 0x7f, 0x40, 0x00, 0x00, 0x00, 0x00, 0x80, 0x31, 0xf7, 0x40,
};

static const uint8_t GOLDEN_RC[40] = {
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x40, 0xe8, 0x03, 0x1a, 0x04,
    0x4c, 0x04, 0x7e, 0x04, 0xb0, 0x04, 0xe2, 0x04, 0x14, 0x05, 0x46, 0x05,
    0x78, 0x05, 0xaa, 0x05, 0xdc, 0x05, 0x0e, 0x06, 0x40, 0x06, 0x72, 0x06,
    0xa4, 0x06, 0xd6, 0x06,
};

static const uint8_t GOLDEN_SERVO[16] = {
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3e, 0x00, 0x00, 0x00, 0x3f,
    0x00, 0x00, 0x80, 0x3f,
};
static void test_encoders_match_reference(void) {
    bf_fdm_t fdm;
    memset(&fdm, 0, sizeof(fdm));
    fdm.timestamp_s = 1.234;
    fdm.gyro_frd_rps[0] = 0.1;
    fdm.gyro_frd_rps[1] = -0.2;
    fdm.gyro_frd_rps[2] = 0.3;
    fdm.accel_frd_mps2[0] = 0.5;
    fdm.accel_frd_mps2[1] = -0.25;
    fdm.accel_frd_mps2[2] = -9.80665;
    fdm.quat_wxyz[0] = 0.9;
    fdm.quat_wxyz[1] = 0.1;
    fdm.quat_wxyz[2] = -0.2;
    fdm.quat_wxyz[3] = 0.3;
    fdm.velocity_enu_mps[0] = 1.5;
    fdm.velocity_enu_mps[1] = -2.5;
    fdm.velocity_enu_mps[2] = 0.75;
    fdm.lon_deg = 8.545594;
    fdm.lat_deg = 47.397742;
    fdm.alt_m = 498.5;
    fdm.pressure_pa = 95000.0;
    uint8_t buf[BF_FDM_BYTES];
    CHECK(bf_fdm_encode(&fdm, buf, sizeof(buf)) == BF_FDM_BYTES);
    CHECK(memcmp(buf, GOLDEN_FDM, sizeof(GOLDEN_FDM)) == 0);
    CHECK(bf_fdm_encode(&fdm, buf, sizeof(buf) - 1) == 0);
    CHECK(bf_fdm_encode(NULL, buf, sizeof(buf)) == 0);

    uint16_t channels[BF_RC_CHANNELS];
    for (int i = 0; i < BF_RC_CHANNELS; i++) {
        channels[i] = (uint16_t)(1000 + 50 * i);
    }
    uint8_t rc[BF_RC_BYTES];
    CHECK(bf_rc_encode(2.5, channels, rc, sizeof(rc)) == BF_RC_BYTES);
    CHECK(memcmp(rc, GOLDEN_RC, sizeof(GOLDEN_RC)) == 0);
    CHECK(bf_rc_encode(2.5, channels, rc, 8) == 0);

    float motors[BF_MOTOR_COUNT];
    CHECK(bf_motors_decode(GOLDEN_SERVO, sizeof(GOLDEN_SERVO), motors) == 0);
    CHECK_NEAR(motors[0], 0.0, 1e-9);
    CHECK_NEAR(motors[1], 0.25, 1e-9);
    CHECK_NEAR(motors[2], 0.5, 1e-9);
    CHECK_NEAR(motors[3], 1.0, 1e-9);
    CHECK(bf_motors_decode(GOLDEN_SERVO, sizeof(GOLDEN_SERVO) - 1, motors) < 0);
    CHECK(bf_motors_decode(NULL, 16, motors) < 0);
}

static ap_truth_t base_truth(void) {
    ap_truth_t truth;
    memset(&truth, 0, sizeof(truth));
    truth.rotation_body_to_world[0] = 1.0;
    truth.rotation_body_to_world[4] = 1.0;
    truth.rotation_body_to_world[8] = 1.0;
    return truth;
}

static void test_conversion(void) {
    px4_home_t home = px4_home_default();
    ap_truth_t truth = base_truth();
    bf_fdm_t fdm;
    truth.time_s = 3.5;
    truth.position_enu_m[1] = 111.3195;
    truth.position_enu_m[2] = 7.0;
    truth.origin_enu_m[2] = 2.0;
    truth.velocity_enu_mps[0] = 1.0;
    truth.velocity_enu_mps[1] = 2.0;
    truth.velocity_enu_mps[2] = 3.0;
    truth.gyro_flu_rps[0] = 0.1;
    truth.gyro_flu_rps[1] = 0.2;
    truth.gyro_flu_rps[2] = 0.3;
    truth.accel_flu_mps2[2] = 9.80665;
    bf_fdm_from_truth(&truth, &home, &fdm);
    CHECK_NEAR(fdm.timestamp_s, 3.5, 1e-12);
    CHECK_NEAR(fdm.gyro_frd_rps[1], -0.2, 1e-12);
    CHECK_NEAR(fdm.gyro_frd_rps[2], -0.3, 1e-12);
    CHECK_NEAR(fdm.accel_frd_mps2[2], -9.80665, 1e-12);
    CHECK_NEAR(fdm.velocity_enu_mps[1], 2.0, 1e-12);
    CHECK_NEAR(fdm.lat_deg, 47.397742 - 0.001, 1e-6);
    CHECK_NEAR(fdm.lon_deg, 8.545594, 1e-9);
    CHECK_NEAR(fdm.alt_m, 493.0, 1e-9);
    CHECK_NEAR(fdm.pressure_pa, px4_isa_pressure_hpa(493.0) * 100.0, 1e-6);
    CHECK_NEAR(fdm.quat_wxyz[0], 1.0, 1e-12);

    truth = base_truth();
    truth.position_enu_m[0] = 164.43057607;
    bf_fdm_from_truth(&truth, &home, &fdm);
    CHECK_NEAR(fdm.lon_deg, 8.545594 - 0.0021821452696, 1e-9);
    CHECK_NEAR(fdm.lat_deg, 47.397742, 1e-12);

    truth = base_truth();
    double half = 0.70710678118654752;
    truth.rotation_body_to_world[0] = 0.0;
    truth.rotation_body_to_world[1] = -1.0;
    truth.rotation_body_to_world[3] = 1.0;
    truth.rotation_body_to_world[4] = 0.0;
    bf_fdm_from_truth(&truth, &home, &fdm);
    CHECK_NEAR(fdm.quat_wxyz[0], half, 1e-9);
    CHECK_NEAR(fdm.quat_wxyz[3], -half, 1e-9);

    truth = base_truth();
    truth.rotation_body_to_world[4] = 0.0;
    truth.rotation_body_to_world[5] = -1.0;
    truth.rotation_body_to_world[7] = 1.0;
    truth.rotation_body_to_world[8] = 0.0;
    bf_fdm_from_truth(&truth, &home, &fdm);
    CHECK_NEAR(fdm.quat_wxyz[0], half, 1e-9);
    CHECK_NEAR(fdm.quat_wxyz[1], half, 1e-9);
}

int main(void) {
    test_encoders_match_reference();
    test_conversion();
    if (g_failures != 0) {
        fprintf(stderr, "%d failures\n", g_failures);
        return 1;
    }
    printf("bf_packets tests passed\n");
    return 0;
}
