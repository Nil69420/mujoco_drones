#ifndef MUJOCO_DRONES_BRIDGE_PX4_LINK_H
#define MUJOCO_DRONES_BRIDGE_PX4_LINK_H

#include "bridge/px4_msgs.h"

#include <stdbool.h>
#include <stdint.h>

typedef struct px4_link px4_link_t;

enum {
    PX4_LINK_DEFAULT_PORT = 4560,
    PX4_LINK_READY = 1,
    PX4_LINK_TIMEOUT = 0,
    PX4_LINK_ERROR = -1
};

px4_link_t *px4_link_open(const char *bind_ip, uint16_t port);

uint16_t px4_link_port(const px4_link_t *link);

bool px4_link_connected(const px4_link_t *link);

int px4_link_accept(px4_link_t *link, int timeout_ms);

int px4_link_send(px4_link_t *link, const uint8_t *data, size_t len);

int px4_link_wait_controls(px4_link_t *link, int timeout_ms, px4_actuator_controls_t *out);

void px4_link_close(px4_link_t *link);

#endif
