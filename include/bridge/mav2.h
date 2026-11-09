#ifndef MUJOCO_DRONES_BRIDGE_MAV2_H
#define MUJOCO_DRONES_BRIDGE_MAV2_H

#include <stddef.h>
#include <stdint.h>

enum {
    MAV2_STX = 0xFD,
    MAV2_MAX_PAYLOAD = 255,
    MAV2_HEADER_BYTES = 10,
    MAV2_SIGNATURE_BYTES = 13,
    MAV2_MAX_FRAME_BYTES = 280,
    MAV2_MSG_HEARTBEAT = 0,
    MAV2_MSG_HIL_ACTUATOR_CONTROLS = 93,
    MAV2_MSG_HIL_SENSOR = 107,
    MAV2_MSG_HIL_GPS = 113
};

typedef struct {
    uint8_t seq;
    uint8_t sysid;
    uint8_t compid;
    uint32_t msgid;
    uint8_t len;
    uint8_t payload[MAV2_MAX_PAYLOAD];
} mav2_frame_t;

typedef struct {
    uint8_t buf[MAV2_MAX_FRAME_BYTES];
    size_t have;
    size_t need;
} mav2_parser_t;

int mav2_crc_extra(uint32_t msgid);

size_t mav2_encode(uint8_t *out, size_t cap, const mav2_frame_t *frame);

void mav2_parser_reset(mav2_parser_t *parser);

int mav2_parser_feed(mav2_parser_t *parser, uint8_t byte, mav2_frame_t *out);

#endif
