#include "foxglove/ws_proto.h"
#include "sensors/noise.h"
#include "test_support.h"

#include <stdlib.h>
#include <string.h>

static const char VALID_REQUEST[] =
    "GET /chat HTTP/1.1\r\n"
    "Host: server.example.com\r\n"
    "Upgrade: websocket\r\n"
    "Connection: Upgrade\r\n"
    "Sec-WebSocket-Key: dGhlIHNhbXBsZSBub25jZQ==\r\n"
    "Sec-WebSocket-Protocol: foxglove.websocket.v1\r\n"
    "Sec-WebSocket-Version: 13\r\n"
    "\r\n";

static ws_handshake_status_t check_str(const char *s) {
    ws_handshake_t hs;
    return ws_handshake_check(s, strlen(s), &hs);
}

static void test_handshake(void) {
    ws_handshake_t hs;
    CHECK(ws_handshake_check(VALID_REQUEST, strlen(VALID_REQUEST), &hs) == WS_HS_OK);
    CHECK(strcmp(hs.accept, "s3pPLMBiTxaQ9kYGzzhZRbK+xOo=") == 0);
    CHECK(hs.request_len == strlen(VALID_REQUEST));
    CHECK(hs.wants_foxglove_subprotocol);
    char resp[512];
    int n = ws_handshake_response(&hs, resp, sizeof(resp));
    CHECK(n > 0 && strstr(resp, "101 Switching Protocols") != NULL);
    CHECK(strstr(resp, "Sec-WebSocket-Accept: s3pPLMBiTxaQ9kYGzzhZRbK+xOo=") != NULL);
    CHECK(strstr(resp, "foxglove.websocket.v1") != NULL);
    CHECK(ws_handshake_response(&hs, resp, 20) < 0);

    size_t full = strlen(VALID_REQUEST);
    for (size_t i = 0; i < full; i++) {
        ws_handshake_t partial;
        CHECK(ws_handshake_check(VALID_REQUEST, i, &partial) == WS_HS_NEED_MORE);
    }

    CHECK(check_str("get /chat HTTP/1.1\r\nUpgrade: websocket\r\nConnection: Upgrade\r\n"
                    "Sec-WebSocket-Key: dGhlIHNhbXBsZSBub25jZQ==\r\nSec-WebSocket-Version: 13\r\n\r\n") ==
          WS_HS_BAD);
    CHECK(check_str("GET / HTTP/1.1\r\nUPGRADE: WebSocket\r\nCONNECTION: keep-alive, Upgrade\r\n"
                    "sec-websocket-key: dGhlIHNhbXBsZSBub25jZQ==\r\nSEC-WEBSOCKET-VERSION: 13\r\n\r\n") ==
          WS_HS_OK);
    CHECK(check_str("POST / HTTP/1.1\r\nUpgrade: websocket\r\nConnection: Upgrade\r\n"
                    "Sec-WebSocket-Key: dGhlIHNhbXBsZSBub25jZQ==\r\nSec-WebSocket-Version: 13\r\n\r\n") ==
          WS_HS_BAD);
    CHECK(check_str("GET / HTTP/1.0\r\nUpgrade: websocket\r\nConnection: Upgrade\r\n"
                    "Sec-WebSocket-Key: dGhlIHNhbXBsZSBub25jZQ==\r\nSec-WebSocket-Version: 13\r\n\r\n") ==
          WS_HS_BAD);
    CHECK(check_str("GET / HTTP/1.1\r\nConnection: Upgrade\r\n"
                    "Sec-WebSocket-Key: dGhlIHNhbXBsZSBub25jZQ==\r\nSec-WebSocket-Version: 13\r\n\r\n") ==
          WS_HS_BAD);
    CHECK(check_str("GET / HTTP/1.1\r\nUpgrade: websocket\r\n"
                    "Sec-WebSocket-Key: dGhlIHNhbXBsZSBub25jZQ==\r\nSec-WebSocket-Version: 13\r\n\r\n") ==
          WS_HS_BAD);
    CHECK(check_str("GET / HTTP/1.1\r\nUpgrade: websocket\r\nConnection: Upgrade\r\n"
                    "Sec-WebSocket-Key: dGhlIHNhbXBsZSBub25jZQ==\r\nSec-WebSocket-Version: 8\r\n\r\n") ==
          WS_HS_BAD);
    CHECK(check_str("GET / HTTP/1.1\r\nUpgrade: websocket\r\nConnection: Upgrade\r\n"
                    "Sec-WebSocket-Key: dGhlIHNhbXBsZSBub25jZQ=\r\nSec-WebSocket-Version: 13\r\n\r\n") ==
          WS_HS_BAD);
    CHECK(check_str("GET / HTTP/1.1\r\nUpgrade: websocket\r\nConnection: Upgrade\r\n"
                    "Sec-WebSocket-Key: dGhlIHNhbXBsZSBub25jZQ==\r\n"
                    "Sec-WebSocket-Key: dGhlIHNhbXBsZSBub25jZQ==\r\nSec-WebSocket-Version: 13\r\n\r\n") ==
          WS_HS_BAD);
    CHECK(check_str("GET / HTTP/1.1\r\nUpgrade: websocket\r\nConnection: Upgrade\r\n"
                    "Sec-WebSocket-Key: dGhlIHNhbXBsZSBub25jZQ==\r\nnocolonhere\r\n"
                    "Sec-WebSocket-Version: 13\r\n\r\n") == WS_HS_BAD);
    CHECK(check_str("GET / HTTP/1.1\r\nUpgrade: websocket\r\nConnection: Upgrade\r\n"
                    "Sec-WebSocket-Key: dGhlIHNhbXBsZSBub25jZQ==\r\n"
                    "Sec-WebSocket-Version: 13\r\n\r\n") == WS_HS_OK);
    CHECK(check_str("\r\n\r\n") == WS_HS_BAD);
    CHECK(check_str("GET\r\n\r\n") == WS_HS_BAD);

    char big[WS_HANDSHAKE_MAX + 100];
    memset(big, 'A', sizeof(big));
    CHECK(ws_handshake_check(big, sizeof(big), &hs) == WS_HS_BAD);
    CHECK(ws_handshake_check(big, 100, &hs) == WS_HS_NEED_MORE);

    ws_handshake_t plain;
    CHECK(check_str("GET / HTTP/1.1\r\nUpgrade: websocket\r\nConnection: Upgrade\r\n"
                    "Sec-WebSocket-Key: dGhlIHNhbXBsZSBub25jZQ==\r\nSec-WebSocket-Version: 13\r\n\r\n") ==
          WS_HS_OK);
    CHECK(ws_handshake_check("GET / HTTP/1.1\r\nUpgrade: websocket\r\nConnection: Upgrade\r\n"
                             "Sec-WebSocket-Key: dGhlIHNhbXBsZSBub25jZQ==\r\n"
                             "Sec-WebSocket-Version: 13\r\n\r\n",
                             strlen("GET / HTTP/1.1\r\nUpgrade: websocket\r\nConnection: Upgrade\r\n"
                                    "Sec-WebSocket-Key: dGhlIHNhbXBsZSBub25jZQ==\r\n"
                                    "Sec-WebSocket-Version: 13\r\n\r\n"),
                             &plain) == WS_HS_OK);
    CHECK(!plain.wants_foxglove_subprotocol);
    char r2[512];
    CHECK(ws_handshake_response(&plain, r2, sizeof(r2)) > 0);
    CHECK(strstr(r2, "Sec-WebSocket-Protocol") == NULL);
}

static size_t build(uint8_t *out, bool fin, uint8_t opcode, const uint8_t *payload, size_t len,
                    bool masked, uint8_t rsv) {
    static const uint8_t mask[4] = {0x37, 0xFA, 0x21, 0x3D};
    size_t at = 0;
    out[at++] = (uint8_t)((fin ? 0x80 : 0) | (rsv & 0x70) | (opcode & 0x0F));
    uint8_t mbit = masked ? 0x80 : 0;
    if (len < 126) {
        out[at++] = (uint8_t)(mbit | len);
    } else if (len <= 0xFFFF) {
        out[at++] = (uint8_t)(mbit | 126);
        out[at++] = (uint8_t)(len >> 8);
        out[at++] = (uint8_t)len;
    } else {
        out[at++] = (uint8_t)(mbit | 127);
        for (int i = 7; i >= 0; i--) {
            out[at++] = (uint8_t)((uint64_t)len >> (8 * i));
        }
    }
    if (masked) {
        memcpy(out + at, mask, 4);
        at += 4;
    }
    for (size_t i = 0; i < len; i++) {
        out[at + i] = masked ? (uint8_t)(payload[i] ^ mask[i & 3U]) : payload[i];
    }
    return at + len;
}

static ws_event_t feed_all(ws_parser_t *p, const uint8_t *buf, size_t n, size_t chunk) {
    ws_event_t last = {WS_EV_NONE, NULL, 0, 0};
    size_t off = 0;
    while (off < n) {
        size_t take = n - off < chunk ? n - off : chunk;
        ws_event_t ev;
        size_t used = ws_parser_feed(p, buf + off, take, &ev);
        CHECK(used <= take);
        off += used;
        if (ev.kind != WS_EV_NONE) {
            last = ev;
            if (ev.kind == WS_EV_ERROR) {
                return last;
            }
        }
        if (used == 0) {
            break;
        }
    }
    return last;
}

static void test_frames_basic(void) {
    uint8_t buf[70000];
    ws_parser_t p;
    const uint8_t hello[] = "hello";
    size_t n = build(buf, true, 1, hello, 5, true, 0);
    for (size_t chunk = 1; chunk <= n; chunk += (chunk < 4 ? 1 : 3)) {
        ws_parser_init(&p);
        ws_event_t ev = feed_all(&p, buf, n, chunk);
        CHECK(ev.kind == WS_EV_TEXT && ev.len == 5 && memcmp(ev.data, "hello", 5) == 0);
    }

    uint8_t payload[200];
    for (size_t i = 0; i < sizeof(payload); i++) {
        payload[i] = (uint8_t)('a' + i % 26);
    }
    n = build(buf, true, 2, payload, sizeof(payload), true, 0);
    ws_parser_init(&p);
    ws_event_t ev = feed_all(&p, buf, n, n);
    CHECK(ev.kind == WS_EV_BINARY && ev.len == 200 && memcmp(ev.data, payload, 200) == 0);

    n = build(buf, true, 1, (const uint8_t *)"", 0, true, 0);
    ws_parser_init(&p);
    ev = feed_all(&p, buf, n, n);
    CHECK(ev.kind == WS_EV_TEXT && ev.len == 0);

    static uint8_t big[WS_MAX_MESSAGE + 1];
    memset(big, 'x', sizeof(big));
    n = build(buf, true, 1, big, WS_MAX_MESSAGE, true, 0);
    ws_parser_init(&p);
    ev = feed_all(&p, buf, n, 1000);
    CHECK(ev.kind == WS_EV_TEXT && ev.len == WS_MAX_MESSAGE);
    n = build(buf, true, 1, big, WS_MAX_MESSAGE + 1, true, 0);
    ws_parser_init(&p);
    ev = feed_all(&p, buf, n, 1000);
    CHECK(ev.kind == WS_EV_ERROR && ev.close_code == 1009);
}

static void expect_error(const uint8_t *buf, size_t n, uint16_t code, int line) {
    ws_parser_t p;
    ws_parser_init(&p);
    ws_event_t ev = feed_all(&p, buf, n, n);
    if (ev.kind != WS_EV_ERROR || ev.close_code != code) {
        fprintf(stderr, "FAIL %s:%d expected error %u got kind %d code %u\n", __FILE__, line,
                code, ev.kind, ev.close_code);
        g_failures++;
    }
    ws_event_t again;
    (void)ws_parser_feed(&p, buf, n, &again);
    CHECK(again.kind == WS_EV_ERROR);
}

static void test_frame_errors(void) {
    uint8_t buf[300];
    const uint8_t x[] = "x";
    size_t n = build(buf, true, 1, x, 1, false, 0);
    expect_error(buf, n, 1002, __LINE__);
    n = build(buf, true, 1, x, 1, true, 0x40);
    expect_error(buf, n, 1002, __LINE__);
    n = build(buf, true, 3, x, 1, true, 0);
    expect_error(buf, n, 1002, __LINE__);
    n = build(buf, true, 11, x, 1, true, 0);
    expect_error(buf, n, 1002, __LINE__);
    n = build(buf, true, 0, x, 1, true, 0);
    expect_error(buf, n, 1002, __LINE__);
    n = build(buf, false, 9, x, 1, true, 0);
    expect_error(buf, n, 1002, __LINE__);
    uint8_t pl[126];
    memset(pl, 'p', sizeof(pl));
    n = build(buf, true, 9, pl, 126, true, 0);
    expect_error(buf, n, 1002, __LINE__);
    uint8_t huge[14] = {0x81, 0xFF, 0x80, 0, 0, 0, 0, 0, 0, 0, 1, 2, 3, 4};
    expect_error(huge, sizeof(huge), 1002, __LINE__);
    uint8_t huge2[14] = {0x81, 0xFF, 0x7F, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 1, 2, 3, 4};
    expect_error(huge2, sizeof(huge2), 1009, __LINE__);

    uint8_t two[600];
    size_t a = build(two, false, 1, x, 1, true, 0);
    size_t b = build(two + a, true, 1, x, 1, true, 0);
    expect_error(two, a + b, 1002, __LINE__);

    const uint8_t bad_utf8[] = {0xC0, 0x80};
    n = build(buf, true, 1, bad_utf8, 2, true, 0);
    expect_error(buf, n, 1007, __LINE__);
    const uint8_t surrogate[] = {0xED, 0xA0, 0x80};
    n = build(buf, true, 1, surrogate, 3, true, 0);
    expect_error(buf, n, 1007, __LINE__);
    const uint8_t above[] = {0xF4, 0x90, 0x80, 0x80};
    n = build(buf, true, 1, above, 4, true, 0);
    expect_error(buf, n, 1007, __LINE__);
    const uint8_t truncated[] = {0xE2, 0x82};
    n = build(buf, true, 1, truncated, 2, true, 0);
    expect_error(buf, n, 1007, __LINE__);
    const uint8_t good[] = {0xE2, 0x82, 0xAC, 0xF0, 0x9F, 0x98, 0x80, 'o', 'k'};
    ws_parser_t p;
    ws_parser_init(&p);
    n = build(buf, true, 1, good, sizeof(good), true, 0);
    ws_event_t ev = feed_all(&p, buf, n, n);
    CHECK(ev.kind == WS_EV_TEXT && ev.len == sizeof(good));
    n = build(buf, true, 2, bad_utf8, 2, true, 0);
    ws_parser_init(&p);
    ev = feed_all(&p, buf, n, n);
    CHECK(ev.kind == WS_EV_BINARY);
}

static void test_control_and_fragments(void) {
    uint8_t buf[600];
    ws_parser_t p;
    ws_parser_init(&p);
    size_t n = build(buf, true, 9, (const uint8_t *)"pingdata", 8, true, 0);
    ws_event_t ev = feed_all(&p, buf, n, 3);
    CHECK(ev.kind == WS_EV_PING && ev.len == 8 && memcmp(ev.data, "pingdata", 8) == 0);
    n = build(buf, true, 10, (const uint8_t *)"", 0, true, 0);
    ev = feed_all(&p, buf, n, n);
    CHECK(ev.kind == WS_EV_PONG && ev.len == 0);

    uint8_t seq[600];
    size_t at = 0;
    at += build(seq + at, false, 1, (const uint8_t *)"Hel", 3, true, 0);
    size_t ping_at = at;
    at += build(seq + at, true, 9, (const uint8_t *)"p", 1, true, 0);
    size_t ping_end = at;
    at += build(seq + at, true, 0, (const uint8_t *)"lo", 2, true, 0);
    ws_parser_init(&p);
    ev = feed_all(&p, seq, ping_end, ping_end);
    CHECK(ev.kind == WS_EV_PING);
    (void)ping_at;
    ev = feed_all(&p, seq + ping_end, at - ping_end, 1);
    CHECK(ev.kind == WS_EV_TEXT && ev.len == 5 && memcmp(ev.data, "Hello", 5) == 0);

    uint8_t close_ok[2] = {0x03, 0xE8};
    n = build(buf, true, 8, close_ok, 2, true, 0);
    ws_parser_init(&p);
    ev = feed_all(&p, buf, n, n);
    CHECK(ev.kind == WS_EV_CLOSE && ev.close_code == 1000);
    n = build(buf, true, 8, (const uint8_t *)"", 0, true, 0);
    ws_parser_init(&p);
    ev = feed_all(&p, buf, n, n);
    CHECK(ev.kind == WS_EV_CLOSE && ev.close_code == 1005);
    uint8_t one[1] = {0x03};
    n = build(buf, true, 8, one, 1, true, 0);
    expect_error(buf, n, 1002, __LINE__);
    uint8_t bad_code[2] = {0x03, 0xED};
    n = build(buf, true, 8, bad_code, 2, true, 0);
    expect_error(buf, n, 1002, __LINE__);
    uint8_t bad_reason[4] = {0x03, 0xE8, 0xC0, 0x80};
    n = build(buf, true, 8, bad_reason, 4, true, 0);
    expect_error(buf, n, 1007, __LINE__);
}

static void test_random_input(void) {
    ws_parser_t *p = malloc(sizeof(*p));
    CHECK(p != NULL);
    if (p == NULL) {
        return;
    }
    noise_rng_t rng;
    noise_seed(&rng, 12345);
    uint8_t buf[512];
    long events = 0;
    for (int round = 0; round < 3000; round++) {
        ws_parser_init(p);
        for (int burst = 0; burst < 20; burst++) {
            size_t n = (size_t)(noise_uniform(&rng) * (double)sizeof(buf));
            for (size_t i = 0; i < n; i++) {
                buf[i] = (uint8_t)(noise_uniform(&rng) * 256.0);
            }
            if (round % 3 == 0 && n >= 2) {
                buf[1] |= 0x80;
                buf[0] &= 0x8F;
            }
            size_t off = 0;
            while (off < n) {
                size_t take = 1 + (size_t)(noise_uniform(&rng) * 40.0);
                if (take > n - off) {
                    take = n - off;
                }
                ws_event_t ev;
                size_t used = ws_parser_feed(p, buf + off, take, &ev);
                CHECK(used <= take);
                events += ev.kind != WS_EV_NONE;
                if (ev.kind == WS_EV_ERROR) {
                    break;
                }
                off += used == 0 ? take : used;
            }
        }
    }
    CHECK(events > 0);
    free(p);
}

static void test_encoder(void) {
    uint8_t out[70100];
    uint8_t data[70000];
    memset(data, 'z', sizeof(data));
    size_t sizes[] = {0, 1, 125, 126, 127, 65535, 65536, 70000};
    for (size_t k = 0; k < sizeof(sizes) / sizeof(sizes[0]); k++) {
        size_t len = sizes[k];
        size_t n = ws_encode_frame(out, sizeof(out), 2, data, len);
        CHECK(n > 0);
        CHECK((out[0] & 0x80) != 0 && (out[0] & 0x0F) == 2 && (out[1] & 0x80) == 0);
        size_t hdr = len < 126 ? 2 : (len <= 0xFFFF ? 4 : 10);
        CHECK(n == hdr + len);
        uint64_t decoded = out[1] & 0x7FU;
        if (decoded == 126) {
            decoded = ((uint64_t)out[2] << 8) | out[3];
        } else if (decoded == 127) {
            decoded = 0;
            for (int i = 0; i < 8; i++) {
                decoded = (decoded << 8) | out[2 + i];
            }
        }
        CHECK(decoded == len);
        CHECK(ws_encode_frame(out, hdr + len - (len > 0 ? 1U : 0U) - (len == 0 ? 1U : 0U),
                              2, data, len) == 0);
    }
}

int main(void) {
    test_handshake();
    test_frames_basic();
    test_frame_errors();
    test_control_and_fragments();
    test_encoder();
    test_random_input();
    if (g_failures != 0) {
        fprintf(stderr, "%d check(s) failed\n", g_failures);
        return 1;
    }
    puts("test_ws_proto OK");
    return 0;
}
