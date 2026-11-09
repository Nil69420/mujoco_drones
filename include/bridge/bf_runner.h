#ifndef MUJOCO_DRONES_BRIDGE_BF_RUNNER_H
#define MUJOCO_DRONES_BRIDGE_BF_RUNNER_H

#include "bridge/ap_map.h"
#include "bridge/px4_sensors.h"
#include "vehicle/vehicle.h"

#include <mujoco/mujoco.h>
#include <stdint.h>

typedef struct bf_runner bf_runner_t;

typedef void (*bf_tick_fn)(void *ctx);

enum {
    BF_BRIDGE_DEFAULT_PORT = 9002,
    BF_RUNNER_EXCHANGED = 1,
    BF_RUNNER_WAITING = 0,
    BF_RUNNER_FAILED = -1
};

bf_runner_t *bf_runner_create(const ap_map_t *map, const px4_home_t *home, uint16_t motors_port,
                              uint16_t fdm_port);

uint16_t bf_runner_port(const bf_runner_t *runner);

int bf_runner_exchange(bf_runner_t *runner, const vehicle_t *vehicle, const mjModel *model,
                       mjData *data, bf_tick_fn tick, void *ctx, int timeout_ms);

void bf_runner_destroy(bf_runner_t *runner);

#endif
