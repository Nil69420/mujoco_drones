#include "bridge/mav2.h"
#include "bridge/px4_msgs.h"

#include <stdint.h>
#include <stdlib.h>

int LLVMFuzzerTestOneInput(const uint8_t *data, size_t size);

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
            if (mav2_encode(again, sizeof(again), &frame) == 0) {
                abort();
            }
        }
    }
    return 0;
}
