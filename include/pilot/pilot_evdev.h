#ifndef MUJOCO_DRONES_PILOT_PILOT_EVDEV_H
#define MUJOCO_DRONES_PILOT_PILOT_EVDEV_H

#include "pilot/pilot.h"

enum { PILOT_PATH_BYTES = 64, PILOT_DEVICE_NAME_BYTES = 96 };

typedef struct {
    char path[PILOT_PATH_BYTES];
    char name[PILOT_DEVICE_NAME_BYTES];
    uint16_t vendor;
    uint16_t product;
    int axis_count;
    int button_count;
} pilot_device_info_t;

int pilot_evdev_list(pilot_device_info_t *out, int max, int *denied);

int pilot_evdev_open(const char *path, pilot_t *pilot, pilot_device_info_t *info);

int pilot_evdev_apply_ranges(int fd, pilot_t *pilot);

int pilot_evdev_drain(int fd, pilot_t *pilot, int *events);

bool pilot_evdev_is_flight_controller(uint32_t axis_bits_low, uint32_t key_bits_low, int axes, int buttons);

#endif
