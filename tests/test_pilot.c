#include "pilot/pilot.h"
#include "test_support.h"

#include <linux/input-event-codes.h>
#include <string.h>

static pilot_t make_dualsense(void) {
    pilot_t pilot;
    pilot_profile_t profile;
    CHECK(pilot_profile_for_device(0x054C, 0x0CE6, "", &profile));
    pilot_init(&pilot, &profile);
    pilot_set_axis_range(&pilot, ABS_X, 0, 255);
    pilot_set_axis_range(&pilot, ABS_Y, 0, 255);
    pilot_set_axis_range(&pilot, ABS_RX, 0, 255);
    pilot_set_axis_range(&pilot, ABS_RY, 0, 255);
    return pilot;
}

static void test_profiles(void) {
    pilot_profile_t found;
    CHECK(pilot_profile_for_device(0x054C, 0x0CE6, "anything", &found) && strcmp(found.name, "Sony DualSense") == 0);
    CHECK(pilot_profile_for_device(0x054C, 0x0DF2, "", &found) && strcmp(found.name, "Sony DualSense Edge") == 0);
    CHECK(!pilot_profile_for_device(0x054C, 0x7777, "", &found));
    CHECK(pilot_profile_for_device(0x045E, 0x0B12, "", &found));
    CHECK(pilot_profile_for_device(0x046D, 0xC21D, "", &found));
    CHECK(!pilot_profile_for_device(0x1234, 0x5678, "No Such Pad", &found));
    CHECK(pilot_profile_for_device(0x0483, 0x5710, "RadioMaster GX12 Joystick", &found));
    CHECK(found.throttle_mode == PILOT_THROTTLE_DIRECT && !found.verified);
    CHECK(pilot_profile_for_device(0x0483, 0x5710, "edgetx something", &found));
    CHECK(!pilot_profile_for_device(0x0483, 0x5710, NULL, &found));
    CHECK(!pilot_profile_for_device(0x054C, 0x0CE6, "", NULL));
    CHECK(pilot_profile_count() >= 12 && pilot_profile_at(0, &found) && !pilot_profile_at(-1, &found));
    CHECK(!pilot_profile_at(pilot_profile_count(), &found) && !pilot_profile_at(0, NULL));
    for (int i = 0; i < pilot_profile_count(); i++) {
        CHECK(pilot_profile_at(i, &found) && !found.verified);
    }
    CHECK(pilot_axis_code("ABS_RX") == ABS_RX && pilot_axis_code("3") == 3 && pilot_axis_code("ABS_NOPE") < 0);
    CHECK(pilot_axis_code("200") < 0 && pilot_button_code("BTN_SOUTH") == BTN_SOUTH && pilot_button_code("-1") < 0);
}

static void test_parse_and_format(void) {
    char error[96] = "";
    pilot_profile_t profile;
    const char *text =
        "# my radio\n[controller]\nname = My radio\nvendor = 0x1209\nproduct = 0x4F54\n"
        "axis_roll = ABS_RX\naxis_pitch = ABS_RY\naxis_yaw = ABS_X\naxis_throttle = ABS_Y\n"
        "invert_pitch = true\nthrottle_mode = trigger\ndeadzone = 0.05\nexpo = 0.2\n"
        "button_arm = none\naxis_arm = ABS_Z\naxis_aux2 = ABS_HAT0X\n";
    CHECK(pilot_profile_parse(text, &profile, error, sizeof(error)) == 0);
    CHECK(strcmp(profile.name, "My radio") == 0 && profile.vendor == 0x1209 && profile.product == 0x4F54);
    CHECK(profile.axis_roll == ABS_RX && profile.axis_yaw == ABS_X && profile.invert_pitch);
    CHECK(profile.throttle_mode == PILOT_THROTTLE_TRIGGER && profile.button_arm == PILOT_NONE);
    CHECK(profile.axis_arm == ABS_Z && profile.axis_aux[1] == ABS_HAT0X && profile.axis_aux[0] == PILOT_NONE);

    char formatted[2048];
    CHECK(pilot_profile_format(&profile, formatted, sizeof(formatted)) > 0);
    pilot_profile_t again;
    CHECK(pilot_profile_parse(formatted, &again, error, sizeof(error)) == 0);
    CHECK(strcmp(again.name, profile.name) == 0 && again.vendor == profile.vendor);
    CHECK(again.axis_roll == profile.axis_roll && again.axis_arm == profile.axis_arm);
    CHECK(again.axis_aux[1] == profile.axis_aux[1] && again.invert_pitch == profile.invert_pitch);
    CHECK(again.throttle_mode == profile.throttle_mode && again.expo == profile.expo);
    CHECK(pilot_profile_format(&profile, formatted, 10) < 0 && pilot_profile_format(NULL, formatted, 10) < 0);

    CHECK(pilot_profile_parse("name = ok\nbogus = 1\n", &profile, error, sizeof(error)) < 0);
    CHECK(strstr(error, "line 2") != NULL);
    CHECK(pilot_profile_parse("axis_roll = ABS_NOPE\n", &profile, error, sizeof(error)) < 0);
    CHECK(pilot_profile_parse("deadzone = 2.0\n", &profile, error, sizeof(error)) < 0);
    CHECK(pilot_profile_parse("vendor = 0x12345\n", &profile, error, sizeof(error)) < 0);
    CHECK(pilot_profile_parse("no equals sign\n", &profile, error, sizeof(error)) < 0);
    CHECK(pilot_profile_parse("throttle_mode = sideways\n", &profile, error, sizeof(error)) < 0);
    CHECK(pilot_profile_parse("invert_roll = maybe\n", &profile, error, sizeof(error)) < 0);
    CHECK(pilot_profile_parse("throttle_rate = 3.5\n", &profile, error, sizeof(error)) == 0 && profile.throttle_rate == 3.5);
    CHECK(pilot_profile_parse("x", NULL, error, sizeof(error)) < 0);
    CHECK(pilot_profile_parse("bogus = 1\n", &profile, NULL, 0) < 0);
}

static void test_sticks(void) {
    pilot_t pilot = make_dualsense();
    pilot_state_t state;
    pilot_update(&pilot, 0.01, &state);
    CHECK_NEAR(state.roll, 0.0, 1e-9);
    CHECK_NEAR(state.pitch, 0.0, 1e-9);
    CHECK_NEAR(state.yaw, 0.0, 1e-9);
    pilot_feed(&pilot, EV_ABS, ABS_RX, 255);
    pilot_feed(&pilot, EV_ABS, ABS_RY, 0);
    pilot_feed(&pilot, EV_ABS, ABS_X, 0);
    pilot_update(&pilot, 0.01, &state);
    CHECK_NEAR(state.roll, 1.0, 1e-9);
    CHECK_NEAR(state.pitch, 1.0, 1e-9);
    CHECK_NEAR(state.yaw, -1.0, 1e-9);
    pilot_feed(&pilot, EV_ABS, ABS_RX, 140);
    pilot_update(&pilot, 0.01, &state);
    double raw = 2.0 * 140.0 / 255.0 - 1.0;
    double scaled = (raw - 0.08) / (1.0 - 0.08);
    CHECK_NEAR(state.roll, scaled * 0.7 + scaled * scaled * scaled * 0.3, 1e-9);
    pilot_feed(&pilot, EV_ABS, ABS_RX, 131);
    pilot_update(&pilot, 0.01, &state);
    CHECK_NEAR(state.roll, 0.0, 1e-12);
    pilot_feed(&pilot, EV_ABS, ABS_RX, 9999);
    pilot_update(&pilot, 0.01, &state);
    CHECK_NEAR(state.roll, 1.0, 1e-12);
    pilot_feed(&pilot, EV_ABS, 200, 5);
    pilot_feed(&pilot, EV_KEY, 5000, 1);
    pilot_feed(&pilot, EV_REL, 1, 1);
}

static void test_throttle_modes(void) {
    pilot_t pilot = make_dualsense();
    pilot_state_t state;
    pilot_feed(&pilot, EV_ABS, ABS_Y, 0);
    for (int i = 0; i < 100; i++) {
        pilot_update(&pilot, 0.01, &state);
    }
    CHECK_NEAR(state.throttle, 0.6, 1e-6);
    pilot_feed(&pilot, EV_ABS, ABS_Y, 128);
    pilot_update(&pilot, 0.5, &state);
    CHECK_NEAR(state.throttle, 0.6, 1e-6);
    pilot_feed(&pilot, EV_ABS, ABS_Y, 0);
    pilot_update(&pilot, 5.0, &state);
    CHECK_NEAR(state.throttle, 1.0, 1e-12);
    pilot_feed(&pilot, EV_ABS, ABS_Y, 255);
    pilot_update(&pilot, 10.0, &state);
    CHECK_NEAR(state.throttle, 0.0, 1e-12);

    pilot_profile_t radio;
    CHECK(pilot_profile_for_device(0, 0, "EdgeTX", &radio));
    pilot_t radio_pilot;
    pilot_init(&radio_pilot, &radio);
    pilot_set_axis_range(&radio_pilot, ABS_Z, -1024, 1024);
    pilot_feed(&radio_pilot, EV_ABS, ABS_Z, -1024);
    pilot_update(&radio_pilot, 0.01, &state);
    CHECK_NEAR(state.throttle, 0.0, 1e-12);
    pilot_feed(&radio_pilot, EV_ABS, ABS_Z, 1024);
    pilot_update(&radio_pilot, 0.01, &state);
    CHECK_NEAR(state.throttle, 1.0, 1e-12);
    pilot_feed(&radio_pilot, EV_ABS, ABS_Z, 0);
    pilot_update(&radio_pilot, 0.01, &state);
    CHECK_NEAR(state.throttle, 0.5, 1e-12);

    pilot_profile_t trigger;
    pilot_profile_generic(&trigger);
    trigger.throttle_mode = PILOT_THROTTLE_TRIGGER;
    trigger.axis_throttle = ABS_RZ;
    pilot_t trigger_pilot;
    pilot_init(&trigger_pilot, &trigger);
    pilot_set_axis_range(&trigger_pilot, ABS_RZ, 0, 1023);
    pilot_feed(&trigger_pilot, EV_ABS, ABS_RZ, 0);
    pilot_update(&trigger_pilot, 0.01, &state);
    CHECK_NEAR(state.throttle, 0.0, 1e-12);
    pilot_feed(&trigger_pilot, EV_ABS, ABS_RZ, 1023);
    pilot_update(&trigger_pilot, 0.01, &state);
    CHECK_NEAR(state.throttle, 1.0, 1e-12);
}

static void test_button_arming(void) {
    pilot_t pilot = make_dualsense();
    pilot_state_t state;
    pilot_feed(&pilot, EV_KEY, BTN_SOUTH, 1);
    for (int i = 0; i < 30; i++) {
        pilot_update(&pilot, 0.01, &state);
    }
    CHECK(!state.arm);
    pilot_feed(&pilot, EV_KEY, BTN_SOUTH, 0);
    pilot_update(&pilot, 0.01, &state);
    pilot_feed(&pilot, EV_KEY, BTN_SOUTH, 1);
    for (int i = 0; i < 60; i++) {
        pilot_update(&pilot, 0.01, &state);
    }
    CHECK(state.arm);
    pilot_feed(&pilot, EV_KEY, BTN_SOUTH, 0);
    pilot_update(&pilot, 0.01, &state);
    CHECK(state.arm);
    pilot_feed(&pilot, EV_KEY, BTN_SOUTH, 1);
    pilot_update(&pilot, 0.01, &state);
    CHECK(!state.arm);
    pilot_feed(&pilot, EV_KEY, BTN_SOUTH, 0);
    pilot_update(&pilot, 0.01, &state);

    pilot_feed(&pilot, EV_ABS, ABS_Y, 0);
    pilot_update(&pilot, 1.0, &state);
    pilot_feed(&pilot, EV_ABS, ABS_Y, 128);
    pilot_feed(&pilot, EV_KEY, BTN_SOUTH, 1);
    for (int i = 0; i < 100; i++) {
        pilot_update(&pilot, 0.01, &state);
    }
    CHECK(!state.arm);
}

static void test_mode_button(void) {
    pilot_t pilot = make_dualsense();
    pilot_state_t state;
    pilot_feed(&pilot, EV_KEY, BTN_EAST, 1);
    pilot_update(&pilot, 0.01, &state);
    CHECK(state.mode);
    pilot_update(&pilot, 0.01, &state);
    CHECK(state.mode);
    pilot_feed(&pilot, EV_KEY, BTN_EAST, 0);
    pilot_update(&pilot, 0.01, &state);
    pilot_feed(&pilot, EV_KEY, BTN_EAST, 1);
    pilot_update(&pilot, 0.01, &state);
    CHECK(!state.mode);
}

static void test_switch_arming(void) {
    pilot_profile_t radio;
    CHECK(pilot_profile_for_device(0, 0, "RadioMaster", &radio));
    pilot_t pilot;
    pilot_state_t state;
    pilot_init(&pilot, &radio);
    pilot_set_axis_range(&pilot, ABS_Z, -1024, 1024);
    pilot_set_axis_range(&pilot, ABS_RY, -1024, 1024);
    pilot_set_axis_range(&pilot, ABS_RZ, -1024, 1024);
    pilot_feed(&pilot, EV_ABS, ABS_Z, 1024);
    pilot_feed(&pilot, EV_ABS, ABS_RY, 1024);
    pilot_update(&pilot, 0.01, &state);
    CHECK(!state.arm);
    pilot_feed(&pilot, EV_ABS, ABS_Z, -1024);
    pilot_update(&pilot, 0.01, &state);
    CHECK(state.arm);
    pilot_feed(&pilot, EV_ABS, ABS_Z, 1024);
    pilot_update(&pilot, 0.01, &state);
    CHECK(state.arm);
    pilot_feed(&pilot, EV_ABS, ABS_RY, -1024);
    pilot_update(&pilot, 0.01, &state);
    CHECK(!state.arm);
    pilot_feed(&pilot, EV_ABS, ABS_RZ, 1024);
    pilot_update(&pilot, 0.01, &state);
    CHECK(state.mode);
}

static void test_failsafe_and_rc(void) {
    pilot_t pilot = make_dualsense();
    pilot_state_t state;
    uint16_t channels[PILOT_RC_CHANNELS];
    pilot_feed(&pilot, EV_ABS, ABS_Y, 0);
    pilot_update(&pilot, 1.0, &state);
    pilot_feed(&pilot, EV_KEY, BTN_SOUTH, 1);
    for (int i = 0; i < 60; i++) {
        pilot_update(&pilot, 0.01, &state);
    }
    pilot_feed(&pilot, EV_KEY, BTN_SOUTH, 0);
    pilot_feed(&pilot, EV_ABS, ABS_RX, 255);
    pilot_feed(&pilot, EV_ABS, ABS_RY, 0);
    pilot_update(&pilot, 0.01, &state);
    pilot_to_rc(&state, channels);
    CHECK(channels[0] == 2000 && channels[1] == 2000 && channels[3] == 1500);
    CHECK(channels[2] >= 1000 && channels[2] <= 2000 && channels[4] == 1000 + 1000 * (state.arm ? 1 : 0));
    CHECK(channels[5] == 1000 && channels[15] == 1500);

    pilot_set_connected(&pilot, false);
    pilot_update(&pilot, 0.01, &state);
    CHECK(state.failsafe && !state.arm && state.throttle == 0.0);
    pilot_to_rc(&state, channels);
    CHECK(channels[0] == 1500 && channels[2] == 1000 && channels[4] == 1000 && channels[5] == 1000);
    pilot_set_connected(&pilot, true);
    pilot_feed(&pilot, EV_ABS, ABS_Y, 128);
    pilot_update(&pilot, 0.01, &state);
    CHECK(!state.failsafe && !state.arm && state.throttle == 0.0);
}

static void test_calibration(void) {
    pilot_t pilot;
    pilot_profile_t generic;
    pilot_profile_generic(&generic);
    pilot_init(&pilot, &generic);
    pilot_set_axis_range(&pilot, ABS_X, -32768, 32767);
    pilot_set_axis_range(&pilot, ABS_Y, -32768, 32767);
    pilot_set_axis_range(&pilot, ABS_RX, 0, 255);
    pilot_set_axis_range(&pilot, ABS_Z, 0, 1023);
    int32_t start[PILOT_AXIS_SLOTS];
    int32_t end[PILOT_AXIS_SLOTS];
    memcpy(start, pilot.value, sizeof(start));
    memcpy(end, pilot.value, sizeof(end));
    uint16_t code = 0;
    bool invert = false;
    CHECK(pilot_calibrate_pick(&pilot, start, end, NULL, 0, &code, &invert) < 0);
    end[ABS_X] = 32767;
    end[ABS_Y] = -32768;
    end[ABS_RX] = start[ABS_RX] + 5;
    CHECK(pilot_calibrate_pick(&pilot, start, end, NULL, 0, &code, &invert) == 0);
    CHECK(code == ABS_X && !invert);
    uint16_t used[1] = {ABS_X};
    CHECK(pilot_calibrate_pick(&pilot, start, end, used, 1, &code, &invert) == 0);
    CHECK(code == ABS_Y && invert);
    end[ABS_Z] = 0;
    start[ABS_Z] = 1000;
    uint16_t both[2] = {ABS_X, ABS_Y};
    CHECK(pilot_calibrate_pick(&pilot, start, end, both, 2, &code, &invert) == 0);
    CHECK(code == ABS_Z && invert);
    end[ABS_HAT0X] = 1;
    end[ABS_RY] = 32767;
    uint16_t three[3] = {ABS_X, ABS_Y, ABS_Z};
    CHECK(pilot_calibrate_pick(&pilot, start, end, three, 3, &code, &invert) < 0);
}

int main(void) {
    test_profiles();
    test_parse_and_format();
    test_sticks();
    test_throttle_modes();
    test_button_arming();
    test_mode_button();
    test_switch_arming();
    test_failsafe_and_rc();
    test_calibration();
    if (g_failures != 0) {
        fprintf(stderr, "%d failures\n", g_failures);
        return 1;
    }
    printf("pilot tests passed\n");
    return 0;
}
