#include "foxglove/sha1.h"

#include <string.h>

static void sha1_transform(uint32_t state[5], const uint8_t block[64]) {
    uint32_t w[80];
    for (int i = 0; i < 16; i++) {
        w[i] = ((uint32_t)block[4*i] << 24)
             | ((uint32_t)block[4*i+1] << 16)
             | ((uint32_t)block[4*i+2] << 8)
             | ((uint32_t)block[4*i+3]);
    }
    for (int i = 16; i < 80; i++) {
        uint32_t t = w[i-3] ^ w[i-8] ^ w[i-14] ^ w[i-16];
        w[i] = (t << 1) | (t >> 31);
    }

    uint32_t a = state[0], b = state[1], c = state[2];
    uint32_t d = state[3], e = state[4];

    for (int i = 0; i < 80; i++) {
        uint32_t f = 0;
        uint32_t k = 0;
        if      (i < 20) { f = (b & c) | (~b & d);           k = 0x5A827999U; }
        else if (i < 40) { f = b ^ c ^ d;                    k = 0x6ED9EBA1U; }
        else if (i < 60) { f = (b & c) | (b & d) | (c & d);  k = 0x8F1BBCDCU; }
        else              { f = b ^ c ^ d;                    k = 0xCA62C1D6U; }

        uint32_t tmp = ((a << 5) | (a >> 27)) + f + e + k + w[i];
        e = d; d = c; c = (b << 30) | (b >> 2); b = a; a = tmp;
    }
    state[0] += a; state[1] += b; state[2] += c;
    state[3] += d; state[4] += e;
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
