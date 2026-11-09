#include "bridge/ap_map.h"
#include "test_support.h"

#include <string.h>

static void test_thrust_curve(void) {
    ap_map_t map = ap_map_quad_plus();
    CHECK(ap_map_validate(&map) == 0);
    CHECK_NEAR(ap_map_thrust(&map, 1000), 0.0, 1e-12);
    CHECK_NEAR(ap_map_thrust(&map, 2000), 6.0, 1e-12);
    CHECK_NEAR(ap_map_thrust(&map, 900), 0.0, 1e-12);
    CHECK_NEAR(ap_map_thrust(&map, 2500), 6.0, 1e-12);
    double mid = 0.35 * 0.5 + 0.65 * 0.25;
    CHECK_NEAR(ap_map_thrust(&map, 1500), 6.0 * mid, 1e-12);
    map.thrust_expo = 0.0;
    CHECK_NEAR(ap_map_thrust(&map, 1500), 3.0, 1e-12);
}

static void test_apply_order_and_spin(void) {
    ap_map_t map = ap_map_quad_plus();
    ap_servo_frame_t f;
    memset(&f, 0, sizeof(f));
    f.channels = 16;
    f.pwm[0] = 1100;
    f.pwm[1] = 1200;
    f.pwm[2] = 1300;
    f.pwm[3] = 1400;
    actuator_command_t out;
    ap_map_apply(&map, &f, &out);
    CHECK(out.count == 8);
    CHECK_NEAR(out.value[3], ap_map_thrust(&map, 1100), 1e-12);
    CHECK_NEAR(out.value[1], ap_map_thrust(&map, 1200), 1e-12);
    CHECK_NEAR(out.value[0], ap_map_thrust(&map, 1300), 1e-12);
    CHECK_NEAR(out.value[2], ap_map_thrust(&map, 1400), 1e-12);
    CHECK(out.value[4] < 0.0 && out.value[5] > 0.0 && out.value[6] < 0.0 && out.value[7] > 0.0);
    CHECK_NEAR(out.value[5] * out.value[5] * map.motor_constant, out.value[1], 1e-9);
}

static void test_validate_rejects(void) {
    ap_map_t map = ap_map_quad_plus();
    CHECK(ap_map_validate(NULL) < 0);
    map.motor[0].thrust_channel = 8;
    CHECK(ap_map_validate(&map) < 0);
    map = ap_map_quad_plus();
    map.pwm_max = map.pwm_min;
    CHECK(ap_map_validate(&map) < 0);
    map = ap_map_quad_plus();
    map.max_thrust_n = 0.0;
    CHECK(ap_map_validate(&map) < 0);
    map = ap_map_quad_plus();
    map.thrust_expo = 1.5;
    CHECK(ap_map_validate(&map) < 0);
    map = ap_map_quad_plus();
    map.motor[1].servo_channel = 32;
    CHECK(ap_map_validate(&map) < 0);
    map = ap_map_quad_plus();
    map.motor[2].spin_channel = 8;
    CHECK(ap_map_validate(&map) < 0);
}

static void test_short_frame_is_zero_thrust(void) {
    ap_map_t map = ap_map_quad_plus();
    ap_servo_frame_t f;
    memset(&f, 0, sizeof(f));
    f.channels = 2;
    f.pwm[0] = 2000;
    f.pwm[1] = 2000;
    actuator_command_t out;
    ap_map_apply(&map, &f, &out);
    CHECK_NEAR(out.value[3], 6.0, 1e-12);
    CHECK_NEAR(out.value[1], 6.0, 1e-12);
    CHECK_NEAR(out.value[0], 0.0, 1e-12);
    CHECK_NEAR(out.value[2], 0.0, 1e-12);
}

static void test_fractions(void) {
    ap_map_t map = ap_map_quad_direct();
    CHECK(ap_map_validate(&map) == 0);
    float fractions[16] = {0.0F, 0.5F, 1.0F, 2.0F, 0.0F};
    actuator_command_t out;
    ap_map_apply_fractions(&map, fractions, 16, &out);
    CHECK(out.count == 8);
    CHECK_NEAR(out.value[0], 0.0, 1e-12);
    CHECK_NEAR(out.value[1], 3.0, 1e-12);
    CHECK_NEAR(out.value[2], 6.0, 1e-12);
    CHECK_NEAR(out.value[3], 6.0, 1e-12);
    CHECK(out.value[4] == 0.0 && out.value[5] > 0.0 && out.value[6] < 0.0 && out.value[7] > 0.0);
    fractions[0] = NAN;
    ap_map_apply_fractions(&map, fractions, 2, &out);
    CHECK_NEAR(out.value[0], 0.0, 1e-12);
    CHECK_NEAR(out.value[2], 0.0, 1e-12);
    CHECK_NEAR(out.value[1], 3.0, 1e-12);
}

int main(void) {
    test_thrust_curve();
    test_apply_order_and_spin();
    test_validate_rejects();
    test_short_frame_is_zero_thrust();
    test_fractions();
    if (g_failures != 0) {
        fprintf(stderr, "%d failures\n", g_failures);
        return 1;
    }
    printf("ap_map tests passed\n");
    return 0;
}
