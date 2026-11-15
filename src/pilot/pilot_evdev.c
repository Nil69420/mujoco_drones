#include "pilot/pilot_evdev.h"

#include <errno.h>
#include <fcntl.h>
#include <linux/input.h>
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>

enum { MAX_EVENT_NODES = 256, MIN_AXES = 4, BIT_BYTES = (KEY_MAX / 8) + 1, EVENT_BATCH = 64 };

static bool test_bit(const uint8_t *bits, int bit) {
    return (bits[bit / 8] & (1U << (bit % 8))) != 0;
}

bool pilot_evdev_is_flight_controller(uint32_t axis_bits_low, uint32_t key_bits_low, int axes, int buttons) {
    (void)key_bits_low;
    bool has_xy = (axis_bits_low & ((1U << ABS_X) | (1U << ABS_Y))) == ((1U << ABS_X) | (1U << ABS_Y));
    return has_xy && axes >= MIN_AXES && buttons >= 1;
}

static int count_bits(const uint8_t *bits, int first, int last) {
    int count = 0;
    for (int bit = first; bit <= last; bit++) {
        count += test_bit(bits, bit) ? 1 : 0;
    }
    return count;
}

static int describe(int fd, pilot_device_info_t *info) {
    struct input_id id;
    uint8_t abs_bits[BIT_BYTES];
    uint8_t key_bits[BIT_BYTES];
    memset(abs_bits, 0, sizeof(abs_bits));
    memset(key_bits, 0, sizeof(key_bits));
    if (ioctl(fd, EVIOCGID, &id) < 0 || ioctl(fd, EVIOCGBIT(EV_ABS, sizeof(abs_bits)), abs_bits) < 0 ||
        ioctl(fd, EVIOCGBIT(EV_KEY, sizeof(key_bits)), key_bits) < 0) {
        return -1;
    }
    char name[PILOT_DEVICE_NAME_BYTES];
    memset(name, 0, sizeof(name));
    if (ioctl(fd, EVIOCGNAME(sizeof(name) - 1U), name) < 0) {
        name[0] = '\0';
    }
    (void)snprintf(info->name, sizeof(info->name), "%s", name);
    info->vendor = id.vendor;
    info->product = id.product;
    info->axis_count = count_bits(abs_bits, 0, ABS_MAX);
    info->button_count = count_bits(key_bits, BTN_MISC, KEY_MAX);
    uint32_t axis_low = (uint32_t)abs_bits[0] | ((uint32_t)abs_bits[1] << 8);
    return pilot_evdev_is_flight_controller(axis_low, 0, info->axis_count, info->button_count) ? 0 : 1;
}

int pilot_evdev_list(pilot_device_info_t *out, int max, int *denied) {
    if (denied != NULL) {
        *denied = 0;
    }
    int count = 0;
    for (int index = 0; index < MAX_EVENT_NODES && count < max; index++) {
        char path[PILOT_PATH_BYTES];
        (void)snprintf(path, sizeof(path), "/dev/input/event%d", index);
        int fd = open(path, O_RDONLY | O_NONBLOCK | O_CLOEXEC);
        if (fd < 0) {
            if (errno == EACCES && denied != NULL) {
                (*denied)++;
            }
            continue;
        }
        pilot_device_info_t info;
        memset(&info, 0, sizeof(info));
        (void)snprintf(info.path, sizeof(info.path), "%s", path);
        if (describe(fd, &info) == 0) {
            out[count++] = info;
        }
        (void)close(fd);
    }
    return count;
}

int pilot_evdev_apply_ranges(int fd, pilot_t *pilot) {
    int applied = 0;
    for (int code = 0; code < PILOT_AXIS_SLOTS && code <= ABS_MAX; code++) {
        struct input_absinfo info;
        if (ioctl(fd, EVIOCGABS((unsigned)code), &info) == 0 && info.maximum > info.minimum) {
            pilot_set_axis_range(pilot, (uint16_t)code, info.minimum, info.maximum);
            pilot->value[code] = info.value;
            applied++;
        }
    }
    return applied;
}

int pilot_evdev_open(const char *path, pilot_t *pilot, pilot_device_info_t *info) {
    int fd = open(path, O_RDONLY | O_NONBLOCK | O_CLOEXEC);
    if (fd < 0) {
        return -1;
    }
    pilot_device_info_t local;
    memset(&local, 0, sizeof(local));
    (void)snprintf(local.path, sizeof(local.path), "%s", path);
    if (describe(fd, &local) < 0) {
        (void)close(fd);
        return -1;
    }
    pilot_profile_t profile;
    if (!pilot_profile_for_device(local.vendor, local.product, local.name, &profile)) {
        pilot_profile_generic(&profile);
    }
    pilot_init(pilot, &profile);
    (void)pilot_evdev_apply_ranges(fd, pilot);
    if (info != NULL) {
        *info = local;
    }
    return fd;
}

int pilot_evdev_drain(int fd, pilot_t *pilot, int *events) {
    int handled = 0;
    for (;;) {
        struct input_event batch[EVENT_BATCH];
        ssize_t got = read(fd, batch, sizeof(batch));
        if (got < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                break;
            }
            if (errno == EINTR) {
                continue;
            }
            pilot_set_connected(pilot, false);
            if (events != NULL) {
                *events = handled;
            }
            return -1;
        }
        if (got == 0) {
            pilot_set_connected(pilot, false);
            if (events != NULL) {
                *events = handled;
            }
            return -1;
        }
        size_t count = (size_t)got / sizeof(batch[0]);
        for (size_t i = 0; i < count; i++) {
            pilot_feed(pilot, batch[i].type, batch[i].code, batch[i].value);
            handled++;
        }
    }
    if (events != NULL) {
        *events = handled;
    }
    return 0;
}
