#ifndef MUJOCO_DRONES_BRIDGE_PX4_RUNNER_H
#define MUJOCO_DRONES_BRIDGE_PX4_RUNNER_H

#include "bridge/ap_map.h"
#include "bridge/px4_sensors.h"
#include "vehicle/vehicle.h"

#include <mujoco/mujoco.h>
#include <stdint.h>

typedef struct px4_runner px4_runner_t;

typedef void (*px4_tick_fn)(void *ctx);

enum {
    PX4_BRIDGE_DEFAULT_PORT = 4560,
    PX4_RUNNER_EXCHANGED = 1,
    PX4_RUNNER_WAITING = 0,
    PX4_RUNNER_FAILED = -1
};

px4_runner_t *px4_runner_create(const ap_map_t *map, const px4_home_t *home, uint16_t port);

uint16_t px4_runner_port(const px4_runner_t *runner);

int px4_runner_exchange(px4_runner_t *runner, const vehicle_t *vehicle, const mjModel *model,
                        mjData *data, px4_tick_fn tick, void *ctx, int timeout_ms);

void px4_runner_destroy(px4_runner_t *runner);

#endif
