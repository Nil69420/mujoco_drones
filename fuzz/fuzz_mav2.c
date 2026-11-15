#include "bridge/mav2.h"
#include "bridge/mav_dialect.h"
#include "bridge/px4_msgs.h"

#include <stdint.h>
#include <stdlib.h>
#include <string.h>

int LLVMFuzzerTestOneInput(const uint8_t *data, size_t size);
size_t LLVMFuzzerMutate(uint8_t *data, size_t size, size_t max_size);
size_t LLVMFuzzerCustomMutator(uint8_t *data, size_t size, size_t max_size, unsigned int seed);

static uint16_t crc_of(const uint8_t *data, size_t len, uint8_t extra) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i <= len; i++) {
        uint8_t byte = i < len ? data[i] : extra;
        uint8_t tmp = (uint8_t)(byte ^ (uint8_t)(crc & 0xFFU));
        tmp = (uint8_t)(tmp ^ (uint8_t)(tmp << 4));
        crc = (uint16_t)((crc >> 8) ^ ((uint16_t)tmp << 8) ^ ((uint16_t)tmp << 3) ^ ((uint16_t)tmp >> 4));
    }
    return crc;
}

size_t LLVMFuzzerCustomMutator(uint8_t *data, size_t size, size_t max_size, unsigned int seed) {
    size = LLVMFuzzerMutate(data, size, max_size);
    if (size < MAV2_HEADER_BYTES + 3U || size > MAV2_HEADER_BYTES + MAV2_MAX_PAYLOAD + 2U) {
        return size;
    }
    size_t len = size - MAV2_HEADER_BYTES - 2U;
    data[0] = MAV2_STX;
    data[1] = (uint8_t)len;
    data[2] = 0;
    if (seed % 4U != 0U) {
        uint32_t msgid = MAV_DIALECT[(seed / 4U) % MAV_DIALECT_COUNT].msgid;
        data[7] = (uint8_t)(msgid & 0xFFU);
        data[8] = (uint8_t)((msgid >> 8) & 0xFFU);
        data[9] = (uint8_t)((msgid >> 16) & 0xFFU);
    }
    int extra = mav2_crc_extra((uint32_t)data[7] | ((uint32_t)data[8] << 8) | ((uint32_t)data[9] << 16));
    if (extra >= 0) {
        uint16_t crc = crc_of(data + 1, (size_t)(MAV2_HEADER_BYTES - 1) + len, (uint8_t)extra);
        data[MAV2_HEADER_BYTES + len] = (uint8_t)(crc & 0xFFU);
        data[MAV2_HEADER_BYTES + len + 1U] = (uint8_t)(crc >> 8);
    }
    return size;
}

int LLVMFuzzerTestOneInput(const uint8_t *data, size_t size) {
    mav2_parser_t parser;
    mav2_frame_t frame;
    px4_actuator_controls_t controls;
    mav2_parser_reset(&parser);
    for (size_t i = 0; i < size; i++) {
        if (mav2_parser_feed(&parser, data[i], &frame) == 1) {
            if (frame.msgid == MAV2_MSG_HIL_ACTUATOR_CONTROLS) {
                if (px4_decode_actuator_controls(&frame, &controls) != 0) {
                    abort();
                }
            }
            uint8_t again[MAV2_MAX_FRAME_BYTES];
            size_t wire = mav2_encode(again, sizeof(again), &frame);
            if (wire == 0) {
                abort();
            }
            mav2_parser_t second;
            mav2_frame_t back;
            memset(&back, 0, sizeof(back));
            mav2_parser_reset(&second);
            int parsed = 0;
            for (size_t k = 0; k < wire; k++) {
                parsed = mav2_parser_feed(&second, again[k], &back);
            }
            if (parsed != 1 || back.msgid != frame.msgid || back.sysid != frame.sysid ||
                back.compid != frame.compid || memcmp(back.payload, frame.payload, MAV2_MAX_PAYLOAD) != 0) {
                abort();
            }
        }
    }
    return 0;
}
