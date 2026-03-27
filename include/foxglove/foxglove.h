#ifndef MUJOCO_DRONES_FOXGLOVE_H
#define MUJOCO_DRONES_FOXGLOVE_H

#include "transport/transport.h"
#include <stdbool.h>
#include <stdint.h>

enum {
    FG_DEFAULT_PORT  = 8765,
    FG_MAX_CLIENTS   = 4,
    FG_MAX_CHANNELS  = 16,
};

typedef struct foxglove_bridge foxglove_bridge_t;

foxglove_bridge_t *foxglove_create(transport_t *tp, uint16_t port);
int foxglove_start(foxglove_bridge_t *bridge);
void foxglove_destroy(foxglove_bridge_t *bridge);

#endif
