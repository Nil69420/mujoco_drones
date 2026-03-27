#include "foxglove/ws.h"
#include "foxglove/sha1.h"
#include "foxglove/base64.h"

#include <errno.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>

static const char WS_MAGIC[] = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11";

static int write_all(int fd, const void *buf, size_t len) {
    const uint8_t *p = buf;
    size_t remaining = len;
    while (remaining > 0) {
        ssize_t n = send(fd, p, remaining, 0);
        if (n < 0) {
            if (errno == EINTR) continue;
            return -1;
        }
        p += (size_t)n;
        remaining -= (size_t)n;
    }
    return 0;
}

static ssize_t recv_all(int fd, void *buf, size_t len) {
    uint8_t *p = buf;
    size_t got = 0;
    while (got < len) {
        ssize_t n = recv(fd, p + got, len - got, 0);
        if (n < 0) {
            if (errno == EINTR) continue;
            return -1;
        }
        if (n == 0) return -1;
        got += (size_t)n;
    }
    return (ssize_t)got;
}

int ws_send_frame(int fd, uint8_t opcode, const void *data, size_t len) {
    uint8_t hdr[10];
    size_t hdr_len = 0;

    hdr[0] = (uint8_t)(0x80U | (opcode & 0x0FU));

    if (len < 126) {
        hdr[1] = (uint8_t)len;
        hdr_len = 2;
    } else if (len <= 0xFFFF) {
        hdr[1] = 126;
        hdr[2] = (uint8_t)(len >> 8);
        hdr[3] = (uint8_t)(len);
        hdr_len = 4;
    } else {
        hdr[1] = 127;
        for (int i = 0; i < 8; i++) {
            hdr[2+i] = (uint8_t)(len >> (56 - 8*i));
        }
        hdr_len = 10;
    }

    if (write_all(fd, hdr, hdr_len) != 0) return -1;
    if (data && len > 0) {
        if (write_all(fd, data, len) != 0) return -1;
    }
    return 0;
}

ssize_t ws_recv_frame(int fd, uint8_t *buf, size_t buf_sz, uint8_t *opcode) {
    uint8_t hdr[2];
    if (recv_all(fd, hdr, 2) != 2) return -1;

    *opcode = hdr[0] & 0x0FU;
    bool masked = (hdr[1] & 0x80U) != 0;
    uint64_t plen = hdr[1] & 0x7FU;

    if (plen == 126) {
        uint8_t ext[2];
        if (recv_all(fd, ext, 2) != 2) return -1;
        plen = ((uint64_t)ext[0] << 8) | ext[1];
    } else if (plen == 127) {
        uint8_t ext[8];
        if (recv_all(fd, ext, 8) != 8) return -1;
        plen = 0;
        for (int i = 0; i < 8; i++) {
            plen = (plen << 8) | ext[i];
        }
    }

    uint8_t mask[4] = {0};
    if (masked) {
        if (recv_all(fd, mask, 4) != 4) return -1;
    }

    if (plen > buf_sz) {
        size_t remaining = (size_t)plen;
        while (remaining > 0) {
            uint8_t tmp[4096];
            size_t chunk = remaining < sizeof(tmp) ? remaining : sizeof(tmp);
            ssize_t n = recv(fd, tmp, chunk, 0);
            if (n <= 0) return -1;
            remaining -= (size_t)n;
        }
        return -1;
    }

    size_t payload_len = (size_t)plen;
    if (payload_len > 0) {
        size_t got = 0;
        while (got < payload_len) {
            ssize_t n = recv(fd, buf + got, payload_len - got, 0);
            if (n <= 0) return -1;
            got += (size_t)n;
        }
    }

    if (masked) {
        for (size_t i = 0; i < payload_len; i++) {
            buf[i] ^= mask[i & 3];
        }
    }

    return (ssize_t)payload_len;
}

int ws_accept_handshake(int fd) {
    char req[4096];
    size_t total = 0;
    while (total < sizeof(req) - 1U) {
        ssize_t n = recv(fd, req + total, sizeof(req) - 1U - total, 0);
        if (n <= 0) return -1;
        total += (size_t)n;
        req[total] = '\0';
        if (strstr(req, "\r\n\r\n")) break;
    }

    const char *key_hdr = strstr(req, "Sec-WebSocket-Key: ");
    if (!key_hdr) return -1;
    key_hdr += 19;

    char ws_key[64];
    int ki = 0;
    while (key_hdr[ki] && key_hdr[ki] != '\r' && ki < 60) {
        ws_key[ki] = key_hdr[ki];
        ki++;
    }
    ws_key[ki] = '\0';

    char concat[128];
    snprintf(concat, sizeof(concat), "%s%s", ws_key, WS_MAGIC);

    uint8_t hash[20];
    fg_sha1((const uint8_t *)concat, strlen(concat), hash);

    char accept_b64[32];
    fg_base64_encode(hash, 20, accept_b64);

    char resp[512];
    int rlen = snprintf(resp, sizeof(resp),
        "HTTP/1.1 101 Switching Protocols\r\n"
        "Upgrade: websocket\r\n"
        "Connection: Upgrade\r\n"
        "Sec-WebSocket-Accept: %s\r\n"
        "Sec-WebSocket-Protocol: foxglove.websocket.v1\r\n"
        "\r\n",
        accept_b64);

    return (send(fd, resp, (size_t)rlen, 0) == rlen) ? 0 : -1;
}
