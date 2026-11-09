#ifndef MUJOCO_DRONES_BRIDGE_BF_LINK_H
#define MUJOCO_DRONES_BRIDGE_BF_LINK_H

#include "bridge/bf_packets.h"

#include <stdbool.h>

typedef struct bf_link bf_link_t;

enum {
    BF_LINK_READY = 1,
    BF_LINK_TIMEOUT = 0,
    BF_LINK_ERROR = -1
};

bf_link_t *bf_link_open(const char *ip, uint16_t motors_port, uint16_t fdm_port);

uint16_t bf_link_port(const bf_link_t *link);

int bf_link_send_fdm(bf_link_t *link, const bf_fdm_t *fdm);

int bf_link_wait_motors(bf_link_t *link, int timeout_ms, float motors[BF_MOTOR_COUNT]);

void bf_link_close(bf_link_t *link);

#endif
