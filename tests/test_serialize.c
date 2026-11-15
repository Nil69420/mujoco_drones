#include "foxglove/serialize.h"
#include "sensors/sensor_types.h"
#include "test_support.h"

#include <stdlib.h>
#include <string.h>

static int serialize_frame(unsigned channels, char *out, size_t cap) {
    enum { W = 4, H = 2 };
    size_t pixels = (size_t)W * H * channels;
    uint8_t *frame = calloc(1, sizeof(sensor_camera_rgb_hdr_t) + pixels);
    sensor_camera_rgb_hdr_t *hdr = (sensor_camera_rgb_hdr_t *)frame;
    hdr->width = W;
    hdr->height = H;
    hdr->channels = (uint8_t)channels;
    hdr->header.timestamp_ns = 2500000000ULL;
    memset(frame + sizeof(*hdr), 0xAB, pixels);
    int len = fg_serialize_camera_rgb(frame, out, cap);
    free(frame);
    return len;
}

int main(void) {
    char out[4096];
    int mono = serialize_frame(1, out, sizeof(out));
    CHECK(mono > 0);
    CHECK(strstr(out, "\"encoding\":\"mono8\"") != NULL);
    CHECK(strstr(out, "\"step\":4,") != NULL);
    CHECK(strstr(out, "\"sec\":2,\"nsec\":500000000") != NULL);
    int colour = serialize_frame(3, out, sizeof(out));
    CHECK(colour > 0);
    CHECK(strstr(out, "\"encoding\":\"rgb8\"") != NULL);
    CHECK(strstr(out, "\"step\":12,") != NULL);
    char tiny[32];
    CHECK(serialize_frame(1, tiny, sizeof(tiny)) < 0);
    if (g_failures != 0) {
        fprintf(stderr, "%d check(s) failed\n", g_failures);
        return 1;
    }
    puts("test_serialize OK");
    return 0;
}
