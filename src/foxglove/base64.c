#include "foxglove/base64.h"

static const char b64_table[] =
    "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

size_t fg_base64_encode(const uint8_t *in, size_t len, char *out) {
    size_t o = 0;
    for (size_t i = 0; i < len; i += 3) {
        uint32_t v = (uint32_t)in[i] << 16;
        if (i + 1 < len) v |= (uint32_t)in[i+1] << 8;
        if (i + 2 < len) v |= (uint32_t)in[i+2];

        out[o++] = b64_table[(v >> 18) & 0x3FU];
        out[o++] = b64_table[(v >> 12) & 0x3FU];
        out[o++] = (char)((i + 1 < len) ? b64_table[(v >> 6) & 0x3FU] : '=');
        out[o++] = (char)((i + 2 < len) ? b64_table[v & 0x3FU] : '=');
    }
    out[o] = '\0';
    return o;
}
