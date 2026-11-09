#ifndef MUJOCO_DRONES_BRIDGE_AP_LINK_H
#define MUJOCO_DRONES_BRIDGE_AP_LINK_H

#include "bridge/ap_json.h"

#include <stdint.h>

typedef struct ap_link ap_link_t;

enum {
    AP_LINK_GOT_FRAME = 1,
    AP_LINK_TIMEOUT = 0,
    AP_LINK_ERROR = -1
};

ap_link_t *ap_link_open(const char *bind_ip, uint16_t port);

uint16_t ap_link_port(const ap_link_t *link);

int ap_link_wait_frame(ap_link_t *link, int timeout_ms, ap_servo_frame_t *frame);

int ap_link_send_state(ap_link_t *link, const ap_state_t *state);

void ap_link_close(ap_link_t *link);

#endif
