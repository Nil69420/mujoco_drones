#include "pilot/pilot_evdev.h"
#include "test_support.h"

#include <errno.h>
#include <fcntl.h>
#include <linux/input.h>
#include <string.h>
#include <unistd.h>

static void write_event(int fd, uint16_t type, uint16_t code, int32_t value) {
    struct input_event event;
    memset(&event, 0, sizeof(event));
    event.type = type;
    event.code = code;
    event.value = value;
    CHECK(write(fd, &event, sizeof(event)) == (ssize_t)sizeof(event));
}

static void test_drain(void) {
    int fds[2];
    CHECK(pipe(fds) == 0);
    CHECK(fcntl(fds[0], F_SETFL, O_NONBLOCK) == 0);
    pilot_t pilot;
    pilot_profile_t profile;
    CHECK(pilot_profile_for_device(0x054C, 0x0CE6, "", &profile));
    pilot_init(&pilot, &profile);
    pilot_set_axis_range(&pilot, ABS_RX, 0, 255);
    int events = -1;
    CHECK(pilot_evdev_drain(fds[0], &pilot, &events) == 0 && events == 0);

    write_event(fds[1], EV_ABS, ABS_RX, 255);
    write_event(fds[1], EV_KEY, BTN_SOUTH, 1);
    write_event(fds[1], EV_SYN, SYN_REPORT, 0);
    CHECK(pilot_evdev_drain(fds[0], &pilot, &events) == 0 && events == 3);
    CHECK(pilot.value[ABS_RX] == 255 && pilot.button[BTN_SOUTH] == 1 && pilot.connected);

    for (int i = 0; i < 300; i++) {
        write_event(fds[1], EV_ABS, ABS_RX, i % 256);
    }
    CHECK(pilot_evdev_drain(fds[0], &pilot, &events) == 0 && events == 300);
    CHECK(pilot.value[ABS_RX] == 299 % 256);

    CHECK(write(fds[1], "abc", 3) == 3);
    CHECK(pilot_evdev_drain(fds[0], &pilot, &events) == 0 && events == 0);

    (void)close(fds[1]);
    CHECK(pilot_evdev_drain(fds[0], &pilot, &events) == -1 && !pilot.connected);
    (void)close(fds[0]);
    pilot_set_connected(&pilot, true);
    CHECK(pilot_evdev_drain(-1, &pilot, NULL) == -1 && !pilot.connected);
}

static void test_classification(void) {
    uint32_t xy = (1U << ABS_X) | (1U << ABS_Y);
    CHECK(pilot_evdev_is_flight_controller(xy, 0, 6, 12));
    CHECK(!pilot_evdev_is_flight_controller(xy, 0, 2, 12));
    CHECK(!pilot_evdev_is_flight_controller(xy, 0, 6, 0));
    CHECK(!pilot_evdev_is_flight_controller(1U << ABS_X, 0, 6, 12));
    CHECK(!pilot_evdev_is_flight_controller(0, 0, 6, 12));
}

static void test_list_and_open(void) {
    pilot_device_info_t devices[8];
    int denied = -1;
    int count = pilot_evdev_list(devices, 8, &denied);
    CHECK(count >= 0 && count <= 8 && denied >= 0);
    for (int i = 0; i < count; i++) {
        CHECK(devices[i].axis_count >= 4 && devices[i].path[0] == '/');
    }
    CHECK(pilot_evdev_list(devices, 0, NULL) == 0);
    pilot_t pilot;
    CHECK(pilot_evdev_open("/dev/input/event_does_not_exist", &pilot, NULL) < 0);
    CHECK(pilot_evdev_open("/dev/null", &pilot, NULL) < 0);
    CHECK(pilot_evdev_apply_ranges(-1, &pilot) == 0);
}

int main(void) {
    test_drain();
    test_classification();
    test_list_and_open();
    if (g_failures != 0) {
        fprintf(stderr, "%d failures\n", g_failures);
        return 1;
    }
    printf("pilot_evdev tests passed\n");
    return 0;
}
