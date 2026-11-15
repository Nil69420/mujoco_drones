#include "bridge/ap_frames.h"
#include "test_support.h"

#include <string.h>

static void quat_to_rotation(const double quat[4], double rot[9]) {
    double qw = quat[0];
    double qx = quat[1];
    double qy = quat[2];
    double qz = quat[3];
    rot[0] = 1 - 2 * (qy * qy + qz * qz);
    rot[1] = 2 * (qx * qy - qz * qw);
    rot[2] = 2 * (qx * qz + qy * qw);
    rot[3] = 2 * (qx * qy + qz * qw);
    rot[4] = 1 - 2 * (qx * qx + qz * qz);
    rot[5] = 2 * (qy * qz - qx * qw);
    rot[6] = 2 * (qx * qz - qy * qw);
    rot[7] = 2 * (qy * qz + qx * qw);
    rot[8] = 1 - 2 * (qx * qx + qy * qy);
}

static void check_round_trip(const double rot[9]) {
    double quat[4];
    double back[9];
    ap_rotation_to_quaternion(rot, quat);
    CHECK_NEAR(quat[0] * quat[0] + quat[1] * quat[1] + quat[2] * quat[2] + quat[3] * quat[3], 1.0,
               1e-12);
    CHECK(quat[0] >= 0.0);
    quat_to_rotation(quat, back);
    for (int i = 0; i < 9; i++) {
        CHECK_NEAR(back[i], rot[i], 1e-12);
    }
}

static void test_quaternion_branches(void) {
    double identity[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
    double flip_x[9] = {1, 0, 0, 0, -1, 0, 0, 0, -1};
    double flip_y[9] = {-1, 0, 0, 0, 1, 0, 0, 0, -1};
    double flip_z[9] = {-1, 0, 0, 0, -1, 0, 0, 0, 1};
    double tilted[9] = {0.8, -0.6, 0.0, 0.48, 0.64, -0.6, 0.36, 0.48, 0.8};
    check_round_trip(identity);
    check_round_trip(flip_x);
    check_round_trip(flip_y);
    check_round_trip(flip_z);
    check_round_trip(tilted);
}

static void test_imu_overlay(void) {
    ap_state_t state;
    memset(&state, 0, sizeof(state));
    const double gyro[3] = {0.1, 0.2, 0.3};
    const double accel[3] = {1.0, 2.0, 9.5};
    ap_state_overlay_imu(&state, gyro, accel);
    CHECK_NEAR(state.gyro_rps[0], 0.1, 1e-15);
    CHECK_NEAR(state.gyro_rps[1], -0.2, 1e-15);
    CHECK_NEAR(state.gyro_rps[2], -0.3, 1e-15);
    CHECK_NEAR(state.accel_body_mps2[0], 1.0, 1e-15);
    CHECK_NEAR(state.accel_body_mps2[1], -2.0, 1e-15);
    CHECK_NEAR(state.accel_body_mps2[2], -9.5, 1e-15);
    CHECK(state.timestamp_s == 0.0 && state.position_ned_m[2] == 0.0);
}

static void rotation_y(double angle, double rot[9]) {
    const double c = cos(angle);
    const double s = sin(angle);
    const double values[9] = {c, 0, s, 0, 1, 0, -s, 0, c};
    memcpy(rot, values, sizeof(values));
}

static void rotation_x(double angle, double rot[9]) {
    const double c = cos(angle);
    const double s = sin(angle);
    const double values[9] = {1, 0, 0, 0, c, -s, 0, s, c};
    memcpy(rot, values, sizeof(values));
}

static void rotation_z(double angle, double rot[9]) {
    const double c = cos(angle);
    const double s = sin(angle);
    const double values[9] = {c, -s, 0, s, c, 0, 0, 0, 1};
    memcpy(rot, values, sizeof(values));
}

static void test_attitude_degrees(void) {
    double rot[9];
    double att[3];
    const double identity[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
    ap_attitude_degrees(identity, att);
    CHECK_NEAR(att[0], 0.0, 1e-9);
    CHECK_NEAR(att[1], 0.0, 1e-9);
    CHECK_NEAR(att[2], 0.0, 1e-9);
    rotation_y(30.0 * M_PI / 180.0, rot);
    ap_attitude_degrees(rot, att);
    CHECK_NEAR(att[1], -30.0, 1e-9);
    CHECK_NEAR(att[2], 0.0, 1e-9);
    rotation_y(-20.0 * M_PI / 180.0, rot);
    ap_attitude_degrees(rot, att);
    CHECK_NEAR(att[1], 20.0, 1e-9);
    rotation_x(25.0 * M_PI / 180.0, rot);
    ap_attitude_degrees(rot, att);
    CHECK_NEAR(att[2], 25.0, 1e-9);
    CHECK_NEAR(att[1], 0.0, 1e-9);
    rotation_x(-15.0 * M_PI / 180.0, rot);
    ap_attitude_degrees(rot, att);
    CHECK_NEAR(att[2], -15.0, 1e-9);
    rotation_z(90.0 * M_PI / 180.0, rot);
    ap_attitude_degrees(rot, att);
    CHECK_NEAR(att[0], 90.0, 1e-9);
    const double beyond[9] = {1, 0, 0, 0, 1, 0, 1.0000001, 1.0000001, 1};
    ap_attitude_degrees(beyond, att);
    CHECK(isfinite(att[1]) && isfinite(att[2]));
}

static ap_truth_t base_truth(void) {
    ap_truth_t truth;
    memset(&truth, 0, sizeof(truth));
    truth.rotation_body_to_world[0] = 1.0;
    truth.rotation_body_to_world[4] = 1.0;
    truth.rotation_body_to_world[8] = 1.0;
    return truth;
}

static void test_heading_and_vectors(void) {
    ap_truth_t truth = base_truth();
    ap_state_t state;
    memset(&state, 0, sizeof(state));
    truth.time_s = 1.25;
    truth.position_enu_m[0] = 2.0;
    truth.position_enu_m[1] = 3.0;
    truth.position_enu_m[2] = 5.0;
    truth.origin_enu_m[2] = 1.0;
    truth.velocity_enu_mps[0] = 1.0;
    truth.velocity_enu_mps[1] = 2.0;
    truth.velocity_enu_mps[2] = 3.0;
    truth.gyro_flu_rps[0] = 1.0;
    truth.gyro_flu_rps[1] = 2.0;
    truth.gyro_flu_rps[2] = 3.0;
    truth.accel_flu_mps2[2] = 9.80665;
    ap_truth_to_state(&truth, &state);
    CHECK_NEAR(state.timestamp_s, 1.25, 1e-12);
    CHECK_NEAR(state.position_ned_m[0], 3.0, 1e-12);
    CHECK_NEAR(state.position_ned_m[1], 2.0, 1e-12);
    CHECK_NEAR(state.position_ned_m[2], -4.0, 1e-12);
    CHECK_NEAR(state.velocity_ned_mps[0], 2.0, 1e-12);
    CHECK_NEAR(state.velocity_ned_mps[1], 1.0, 1e-12);
    CHECK_NEAR(state.velocity_ned_mps[2], -3.0, 1e-12);
    CHECK_NEAR(state.gyro_rps[1], -2.0, 1e-12);
    CHECK_NEAR(state.gyro_rps[2], -3.0, 1e-12);
    CHECK_NEAR(state.accel_body_mps2[2], -9.80665, 1e-12);
    double half = 0.70710678118654752;
    CHECK_NEAR(state.quaternion_wxyz[0], half, 1e-9);
    CHECK_NEAR(state.quaternion_wxyz[3], half, 1e-9);
    CHECK_NEAR(state.quaternion_wxyz[1], 0.0, 1e-9);
    CHECK_NEAR(state.quaternion_wxyz[2], 0.0, 1e-9);
}

static void test_facing_north_is_identity(void) {
    ap_truth_t truth = base_truth();
    truth.rotation_body_to_world[0] = 0.0;
    truth.rotation_body_to_world[1] = -1.0;
    truth.rotation_body_to_world[3] = 1.0;
    truth.rotation_body_to_world[4] = 0.0;
    ap_state_t state;
    memset(&state, 0, sizeof(state));
    ap_truth_to_state(&truth, &state);
    CHECK_NEAR(state.quaternion_wxyz[0], 1.0, 1e-9);
    CHECK_NEAR(state.quaternion_wxyz[3], 0.0, 1e-9);
}

int main(void) {
    test_quaternion_branches();
    test_imu_overlay();
    test_attitude_degrees();
    test_heading_and_vectors();
    test_facing_north_is_identity();
    if (g_failures != 0) {
        fprintf(stderr, "%d failures\n", g_failures);
        return 1;
    }
    printf("ap_frames tests passed\n");
    return 0;
}
