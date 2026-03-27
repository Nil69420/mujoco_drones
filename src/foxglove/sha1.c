#include "foxglove/sha1.h"

#include <stddef.h>
#include <string.h>

static void sha1_transform(uint32_t state[5], const uint8_t block[64]) {
    uint32_t w[80];
    for (int i = 0; i < 16; i++) {
        w[i] = ((uint32_t)block[4 * (ptrdiff_t)i] << 24)
             | ((uint32_t)block[4 * (ptrdiff_t)i + 1] << 16)
             | ((uint32_t)block[4 * (ptrdiff_t)i + 2] << 8)
             | ((uint32_t)block[4 * (ptrdiff_t)i + 3]);
    }
    for (int i = 16; i < 80; i++) {
        uint32_t t = w[i-3] ^ w[i-8] ^ w[i-14] ^ w[i-16];
        w[i] = (t << 1) | (t >> 31);
    }

    uint32_t sa = state[0], sb = state[1], sc = state[2];
    uint32_t d = state[3], se = state[4];

    for (int i = 0; i < 80; i++) {
        uint32_t f = 0;
        uint32_t k = 0;
        if      (i < 20) { f = (sb & sc) | (~sb & d);           k = 0x5A827999U; }
        else if (i < 40) { f = sb ^ sc ^ d;                    k = 0x6ED9EBA1U; }
        else if (i < 60) { f = (sb & sc) | (sb & d) | (sc & d);  k = 0x8F1BBCDCU; }
        else              { f = sb ^ sc ^ d;                    k = 0xCA62C1D6U; }

        uint32_t tmp = ((sa << 5) | (sa >> 27)) + f + se + k + w[i];
        se = d; d = sc; sc = (sb << 30) | (sb >> 2); sb = sa; sa = tmp;
    }
    state[0] += sa; state[1] += sb; state[2] += sc;
    state[3] += d; state[4] += se;
}

void fg_sha1(const uint8_t *data, size_t len, uint8_t out[20]) {
    uint32_t state[5] = {
        0x67452301U, 0xEFCDAB89U, 0x98BADCFEU, 0x10325476U, 0xC3D2E1F0U
    };

    size_t i = 0;
    for (; i + 64 <= len; i += 64) {
        sha1_transform(state, data + i);
    }

    uint8_t block[64];
    size_t rem = len - i;
    memcpy(block, data + i, rem);
    block[rem++] = 0x80;

    if (rem > 56) {
        memset(block + rem, 0, 64 - rem);
        sha1_transform(state, block);
        rem = 0;
    }
    memset(block + rem, 0, 56 - rem);

    uint64_t bits = (uint64_t)len * 8;
    for (int j = 0; j < 8; j++) {
        block[56 + j] = (uint8_t)(bits >> (56 - 8*j));
    }

    sha1_transform(state, block);

    for (int j = 0; j < 5; j++) {
        out[4*j+0] = (uint8_t)(state[j] >> 24);
        out[4*j+1] = (uint8_t)(state[j] >> 16);
        out[4*j+2] = (uint8_t)(state[j] >> 8);
        out[4*j+3] = (uint8_t)(state[j]);
    }
}
