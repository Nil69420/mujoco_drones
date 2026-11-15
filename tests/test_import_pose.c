#include "test_support.h"
#include "urdf_import_pose.h"

#include <math.h>
#include <stdint.h>
#include <stdio.h>

static double frand(uint64_t *state, double low, double high) {
    *state = *state * 6364136223846793005ULL + 1442695040888963407ULL;
    double unit = (double)(*state >> 11) * 0x1.0p-53;
    return low + (high - low) * unit;
}

static void check_pose_near(const imp_pose_t *a, const imp_pose_t *b, double tol) {
    for (int i = 0; i < 9; i++) {
        CHECK_NEAR(a->rotation[i], b->rotation[i], tol);
    }
    for (int i = 0; i < 3; i++) {
        CHECK_NEAR(a->translation[i], b->translation[i], tol);
    }
}

static void test_parse_and_known_rotations(void) {
    imp_pose_t pose;
    CHECK(imp_pose_parse("1 2 3 0 0 0", &pose) == 0);
    CHECK(imp_pose_is_identity(&pose) == 0);
    CHECK_NEAR(pose.translation[0], 1.0, 1e-15);
    CHECK_NEAR(pose.translation[2], 3.0, 1e-15);
    CHECK(imp_pose_parse("0 0 0 0 0 0", &pose) == 0 && imp_pose_is_identity(&pose));
    CHECK(imp_pose_parse("0 0 0 0 0 1.5707963267948966", &pose) == 0);
    CHECK_NEAR(pose.rotation[1], -1.0, 1e-12);
    CHECK_NEAR(pose.rotation[3], 1.0, 1e-12);
    CHECK(imp_pose_parse("0 0 0 0 1.5707963267948966 0", &pose) == 0);
    CHECK_NEAR(pose.rotation[2], 1.0, 1e-12);
    CHECK_NEAR(pose.rotation[6], -1.0, 1e-12);
    CHECK(imp_pose_parse("0 0 0 1.5707963267948966 0 0", &pose) == 0);
    CHECK_NEAR(pose.rotation[5], -1.0, 1e-12);
    CHECK_NEAR(pose.rotation[7], 1.0, 1e-12);
}

static void test_parse_rejects_bad_text(void) {
    imp_pose_t pose;
    CHECK(imp_pose_parse(NULL, &pose) != 0);
    CHECK(imp_pose_parse("1 2 3", &pose) != 0);
    CHECK(imp_pose_parse("1 2 3 4 5 six", &pose) != 0);
    CHECK(imp_pose_parse("1 2 3 4 5 nan", &pose) != 0);
    CHECK(imp_pose_parse("1 2 3 4 5 inf", &pose) != 0);
    CHECK(imp_pose_parse("", &pose) != 0);
    CHECK(imp_pose_parse("0 0 0 0 0 0", NULL) != 0);
}

static void test_round_trip_and_inverse(void) {
    uint64_t state = 12345;
    for (int trial = 0; trial < 2000; trial++) {
        double values[6] = {frand(&state, -5, 5),    frand(&state, -5, 5),    frand(&state, -5, 5),
                            frand(&state, -3.1, 3.1), frand(&state, -1.55, 1.55), frand(&state, -3.1, 3.1)};
        imp_pose_t pose = imp_pose_from_xyzrpy(values);
        double back[6];
        imp_pose_to_xyzrpy(&pose, back);
        for (int i = 0; i < 6; i++) {
            CHECK_NEAR(back[i], values[i], 1e-9);
        }
        imp_pose_t inverse = imp_pose_inverse(&pose);
        imp_pose_t product = imp_pose_multiply(&pose, &inverse);
        imp_pose_t identity = imp_pose_identity();
        check_pose_near(&product, &identity, 1e-12);
        product = imp_pose_multiply(&inverse, &pose);
        check_pose_near(&product, &identity, 1e-12);
    }
}

static void test_composition_matches_sequential_application(void) {
    imp_pose_t a;
    imp_pose_t b;
    CHECK(imp_pose_parse("1 0 0 0 0 1.5707963267948966", &a) == 0);
    CHECK(imp_pose_parse("2 0 0 0 0 0", &b) == 0);
    imp_pose_t ab = imp_pose_multiply(&a, &b);
    CHECK_NEAR(ab.translation[0], 1.0, 1e-12);
    CHECK_NEAR(ab.translation[1], 2.0, 1e-12);
    imp_pose_t ba = imp_pose_multiply(&b, &a);
    CHECK_NEAR(ba.translation[0], 3.0, 1e-12);
    CHECK_NEAR(ba.translation[1], 0.0, 1e-12);
}

static void test_gimbal_lock_round_trips_the_matrix(void) {
    const double pitches[2] = {1.5707963267948966, -1.5707963267948966};
    for (int i = 0; i < 2; i++) {
        double values[6] = {0.1, 0.2, 0.3, 0.7, pitches[i], -0.4};
        imp_pose_t pose = imp_pose_from_xyzrpy(values);
        double decomposed[6];
        imp_pose_to_xyzrpy(&pose, decomposed);
        imp_pose_t rebuilt = imp_pose_from_xyzrpy(decomposed);
        check_pose_near(&pose, &rebuilt, 1e-9);
        CHECK_NEAR(decomposed[4], pitches[i], 1e-9);
    }
}

static void test_unrotate_expresses_a_vector_in_the_pose_frame(void) {
    imp_pose_t pose;
    double out[3];
    const double along_x[3] = {1.0, 0.0, 0.0};
    const double along_z[3] = {0.0, 0.0, 1.0};
    CHECK(imp_pose_parse("5 6 7 1.5707963267948966 0 0", &pose) == 0);
    imp_pose_unrotate(&pose, along_z, out);
    CHECK_NEAR(out[0], 0.0, 1e-12);
    CHECK_NEAR(out[1], 1.0, 1e-12);
    CHECK_NEAR(out[2], 0.0, 1e-12);
    CHECK(imp_pose_parse("0 0 0 0 1.5707963267948966 0", &pose) == 0);
    imp_pose_unrotate(&pose, along_x, out);
    CHECK_NEAR(out[0], 0.0, 1e-12);
    CHECK_NEAR(out[2], 1.0, 1e-12);
    uint64_t state = 99;
    for (int trial = 0; trial < 500; trial++) {
        double values[6] = {0, 0, 0, frand(&state, -3, 3), frand(&state, -1.5, 1.5), frand(&state, -3, 3)};
        imp_pose_t rotated = imp_pose_from_xyzrpy(values);
        const double vector[3] = {frand(&state, -2, 2), frand(&state, -2, 2), frand(&state, -2, 2)};
        double local[3];
        imp_pose_unrotate(&rotated, vector, local);
        double norm_in = sqrt(vector[0] * vector[0] + vector[1] * vector[1] + vector[2] * vector[2]);
        double norm_out = sqrt(local[0] * local[0] + local[1] * local[1] + local[2] * local[2]);
        CHECK_NEAR(norm_in, norm_out, 1e-12);
        double rebuilt[3];
        for (int row = 0; row < 3; row++) {
            rebuilt[row] = rotated.rotation[3 * row] * local[0] + rotated.rotation[3 * row + 1] * local[1] +
                           rotated.rotation[3 * row + 2] * local[2];
            CHECK_NEAR(rebuilt[row], vector[row], 1e-12);
        }
    }
}

int main(void) {
    test_parse_and_known_rotations();
    test_parse_rejects_bad_text();
    test_round_trip_and_inverse();
    test_composition_matches_sequential_application();
    test_gimbal_lock_round_trips_the_matrix();
    test_unrotate_expresses_a_vector_in_the_pose_frame();
    if (g_failures != 0) {
        fprintf(stderr, "%d check(s) failed\n", g_failures);
        return 1;
    }
    puts("test_import_pose OK");
    return 0;
}
