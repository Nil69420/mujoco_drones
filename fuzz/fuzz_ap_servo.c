#include "bridge/ap_json.h"

#include <stdint.h>
#include <stdlib.h>
#include <string.h>

int LLVMFuzzerTestOneInput(const uint8_t *data, size_t size);

int LLVMFuzzerTestOneInput(const uint8_t *data, size_t size) {
    ap_servo_frame_t frame;
    if (ap_servo_decode(data, size, &frame) == 0) {
        if (frame.channels != 16 && frame.channels != 32) {
            abort();
        }
        if (size != (frame.channels == 16 ? AP_JSON_PACKET_16_BYTES : AP_JSON_PACKET_32_BYTES)) {
            abort();
        }
    }
    return 0;
}
