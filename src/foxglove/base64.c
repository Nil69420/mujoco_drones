#include "foxglove/base64.h"

static const char b64_table[] =
    "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

size_t fg_base64_encode(const uint8_t *in, size_t len, char *out) {
    size_t pos = 0;
    for (size_t i = 0; i < len; i += 3) {
        uint32_t val = (uint32_t)in[i] << 16;
        if (i + 1 < len) val |= (uint32_t)in[i+1] << 8;
        if (i + 2 < len) val |= (uint32_t)in[i+2];

        out[pos++] = b64_table[(val >> 18) & 0x3FU];
        out[pos++] = b64_table[(val >> 12) & 0x3FU];
        out[pos++] = (char)((i + 1 < len) ? b64_table[(val >> 6) & 0x3FU] : '=');
        out[pos++] = (char)((i + 2 < len) ? b64_table[val & 0x3FU] : '=');
    }
    out[pos] = '\0';
    return pos;
}
