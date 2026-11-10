#ifndef MUJOCO_DRONES_BRIDGE_MAV_DIALECT_H
#define MUJOCO_DRONES_BRIDGE_MAV_DIALECT_H

#include <stdint.h>

enum { MAV_NO_TARGET = 255 };

typedef struct {
    uint32_t msgid;
    uint8_t crc_extra;
    uint8_t base_len;
    uint8_t max_len;
    uint8_t target_system;
    uint8_t target_component;
} mav_message_info_t;

extern const mav_message_info_t MAV_DIALECT[];
extern const unsigned MAV_DIALECT_COUNT;

const mav_message_info_t *mav_dialect_find(uint32_t msgid);

#endif
