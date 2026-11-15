#ifndef MUJOCO_DRONES_BRIDGE_AP_RUNNER_H
#define MUJOCO_DRONES_BRIDGE_AP_RUNNER_H

#include "bridge/ap_map.h"
#include "vehicle/vehicle.h"

#include <mujoco/mujoco.h>
#include <stdint.h>

typedef struct ap_runner ap_runner_t;

typedef void (*ap_tick_fn)(void *ctx);

enum {
    AP_RUNNER_EXCHANGED = 1,
    AP_RUNNER_WAITING = 0,
    AP_RUNNER_FAILED = -1
};

ap_runner_t *ap_runner_create(const ap_map_t *map, uint16_t port);

uint16_t ap_runner_port(const ap_runner_t *runner);

int ap_runner_exchange(ap_runner_t *runner, const vehicle_t *vehicle, const mjModel *model,
                       mjData *data, ap_tick_fn tick, void *ctx, int timeout_ms);

void ap_runner_destroy(ap_runner_t *runner);

#endif
