#include "foxglove/ws_proto.h"

#include "foxglove/base64.h"
#include "foxglove/sha1.h"

#include <stdio.h>
#include <string.h>

static const char WS_GUID[] = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11";
static const char SUBPROTOCOL[] = "foxglove.websocket.v1";

enum { MAX_HEADERS = 64, KEY_LEN = 24 };

static char lower(char c) {
    return (c >= 'A' && c <= 'Z') ? (char)(c + 32) : c;
}

static bool name_equals(const char *a, size_t alen, const char *b) {
    size_t blen = strlen(b);
    if (alen != blen) {
        return false;
    }
    for (size_t i = 0; i < alen; i++) {
        if (lower(a[i]) != lower(b[i])) {
            return false;
        }
    }
    return true;
}

static bool list_has_token(const char *v, size_t len, const char *token) {
    size_t tlen = strlen(token);
    size_t i = 0;
    while (i < len) {
        while (i < len && (v[i] == ' ' || v[i] == '\t' || v[i] == ',')) {
            i++;
        }
        size_t start = i;
        while (i < len && v[i] != ',' && v[i] != ' ' && v[i] != '\t') {
            i++;
        }
        if (i - start == tlen && name_equals(v + start, tlen, token)) {
            return true;
        }
    }
    return false;
}

static bool key_valid(const char *k, size_t len) {
    if (len != KEY_LEN || k[22] != '=' || k[23] != '=') {
        return false;
    }
    for (size_t i = 0; i < 22; i++) {
        char c = k[i];
        bool ok = (c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') || (c >= '0' && c <= '9') ||
                  c == '+' || c == '/';
        if (!ok) {
            return false;
        }
    }
    return true;
}

static const char *find_crlfcrlf(const char *buf, size_t len) {
    if (len < 4) {
        return NULL;
    }
    for (size_t i = 0; i + 3 < len; i++) {
        if (buf[i] == '\r' && buf[i + 1] == '\n' && buf[i + 2] == '\r' && buf[i + 3] == '\n') {
            return buf + i;
        }
    }
    return NULL;
}

ws_handshake_status_t ws_handshake_check(const char *buf, size_t len, ws_handshake_t *out) {
    const char *end = find_crlfcrlf(buf, len > WS_HANDSHAKE_MAX ? WS_HANDSHAKE_MAX : len);
    if (end == NULL) {
        return len >= WS_HANDSHAKE_MAX ? WS_HS_BAD : WS_HS_NEED_MORE;
    }
    size_t head_len = (size_t)(end - buf);
    memset(out, 0, sizeof(*out));
    out->request_len = head_len + 4;

    const char *line = buf;
    const char *stop = buf + head_len;
    const char *eol = memchr(line, '\r', head_len);
    size_t first_len = eol == NULL ? head_len : (size_t)(eol - line);
    if (first_len < 14 || memcmp(line, "GET /", 5) != 0 ||
        memcmp(line + first_len - 9, " HTTP/1.1", 9) != 0) {
        return WS_HS_BAD;
    }
    for (size_t i = 4; i + 9 < first_len; i++) {
        if (line[i] == ' ' || (unsigned char)line[i] < 0x21) {
            return WS_HS_BAD;
        }
    }

    bool upgrade = false;
    bool connection = false;
    bool version = false;
    bool have_key = false;
    char key[KEY_LEN + 1];
    int headers = 0;
    line = eol == NULL ? stop : eol + 2;
    while (line < stop) {
        const char *e = memchr(line, '\r', (size_t)(stop - line));
        size_t llen = e == NULL ? (size_t)(stop - line) : (size_t)(e - line);
        if (++headers > MAX_HEADERS) {
            return WS_HS_BAD;
        }
        const char *colon = memchr(line, ':', llen);
        if (colon == NULL || colon == line) {
            return WS_HS_BAD;
        }
        size_t nlen = (size_t)(colon - line);
        const char *v = colon + 1;
        size_t vlen = llen - nlen - 1;
        while (vlen > 0 && (*v == ' ' || *v == '\t')) {
            v++;
            vlen--;
        }
        while (vlen > 0 && (v[vlen - 1] == ' ' || v[vlen - 1] == '\t')) {
            vlen--;
        }
        if (name_equals(line, nlen, "upgrade")) {
            upgrade = upgrade || list_has_token(v, vlen, "websocket");
        } else if (name_equals(line, nlen, "connection")) {
            connection = connection || list_has_token(v, vlen, "upgrade");
        } else if (name_equals(line, nlen, "sec-websocket-version")) {
            version = vlen == 2 && v[0] == '1' && v[1] == '3';
        } else if (name_equals(line, nlen, "sec-websocket-key")) {
            if (have_key || !key_valid(v, vlen)) {
                return WS_HS_BAD;
            }
            memcpy(key, v, KEY_LEN);
            key[KEY_LEN] = '\0';
            have_key = true;
        } else if (name_equals(line, nlen, "sec-websocket-protocol")) {
            out->wants_foxglove_subprotocol = out->wants_foxglove_subprotocol ||
                                              list_has_token(v, vlen, SUBPROTOCOL);
        }
        line = e == NULL ? stop : e + 2;
    }
    if (!upgrade || !connection || !version || !have_key) {
        return WS_HS_BAD;
    }

    char concat[KEY_LEN + sizeof(WS_GUID)];
    int cl = snprintf(concat, sizeof(concat), "%s%s", key, WS_GUID);
    if (cl <= 0) {
        return WS_HS_BAD;
    }
    uint8_t hash[20];
    fg_sha1((const uint8_t *)concat, (size_t)cl, hash);
    char b64[32];
    fg_base64_encode(hash, 20, b64);
    memcpy(out->accept, b64, WS_ACCEPT_LEN);
    out->accept[WS_ACCEPT_LEN] = '\0';
    return WS_HS_OK;
}

int ws_handshake_response(const ws_handshake_t *hs, char *dst, size_t cap) {
    int n = snprintf(dst, cap,
                     "HTTP/1.1 101 Switching Protocols\r\n"
                     "Upgrade: websocket\r\n"
                     "Connection: Upgrade\r\n"
                     "Sec-WebSocket-Accept: %s\r\n"
                     "%s"
                     "\r\n",
                     hs->accept,
                     hs->wants_foxglove_subprotocol
                         ? "Sec-WebSocket-Protocol: foxglove.websocket.v1\r\n"
                         : "");
    return (n > 0 && (size_t)n < cap) ? n : -1;
}

bool ws_utf8_valid(const uint8_t *s, size_t n) {
    size_t i = 0;
    while (i < n) {
        uint8_t c = s[i];
        if (c < 0x80) {
            i++;
        } else if (c >= 0xC2 && c <= 0xDF) {
            if (i + 1 >= n || (s[i + 1] & 0xC0) != 0x80) {
                return false;
            }
            i += 2;
        } else if (c >= 0xE0 && c <= 0xEF) {
            if (i + 2 >= n || (s[i + 1] & 0xC0) != 0x80 || (s[i + 2] & 0xC0) != 0x80) {
                return false;
            }
            if (c == 0xE0 && s[i + 1] < 0xA0) {
                return false;
            }
            if (c == 0xED && s[i + 1] > 0x9F) {
                return false;
            }
            i += 3;
        } else if (c >= 0xF0 && c <= 0xF4) {
            if (i + 3 >= n || (s[i + 1] & 0xC0) != 0x80 || (s[i + 2] & 0xC0) != 0x80 ||
                (s[i + 3] & 0xC0) != 0x80) {
                return false;
            }
            if (c == 0xF0 && s[i + 1] < 0x90) {
                return false;
            }
            if (c == 0xF4 && s[i + 1] > 0x8F) {
                return false;
            }
            i += 4;
        } else {
            return false;
        }
    }
    return true;
}

void ws_parser_init(ws_parser_t *p) {
    memset(p, 0, sizeof(*p));
    p->head_need = 2;
}

static void fail(ws_parser_t *p, ws_event_t *ev, uint16_t code) {
    p->failed = true;
    ev->kind = WS_EV_ERROR;
    ev->close_code = code;
}

static bool close_code_valid(uint16_t c) {
    return (c >= 1000 && c <= 1003) || (c >= 1007 && c <= 1011) || (c >= 3000 && c <= 4999);
}

static void header_done(ws_parser_t *p, ws_event_t *ev) {
    uint8_t b0 = p->head[0];
    uint8_t b1 = p->head[1];
    p->fin = (b0 & 0x80U) != 0;
    p->opcode = b0 & 0x0FU;
    p->masked = (b1 & 0x80U) != 0;
    uint64_t len = b1 & 0x7FU;
    size_t at = 2;
    if (len == 126) {
        len = ((uint64_t)p->head[2] << 8) | p->head[3];
        at = 4;
    } else if (len == 127) {
        len = 0;
        for (size_t i = 0; i < 8; i++) {
            len = (len << 8) | p->head[2 + i];
        }
        at = 10;
        if ((len >> 63) != 0) {
            fail(p, ev, 1002);
            return;
        }
    }
    if ((b0 & 0x70U) != 0 || !p->masked) {
        fail(p, ev, 1002);
        return;
    }
    memcpy(p->mask, p->head + at, 4);
    bool control = p->opcode >= 8;
    if (p->opcode > 2 && p->opcode < 8) {
        fail(p, ev, 1002);
        return;
    }
    if (p->opcode > 10) {
        fail(p, ev, 1002);
        return;
    }
    if (control) {
        if (!p->fin || len > 125) {
            fail(p, ev, 1002);
            return;
        }
    } else {
        if (p->opcode == 0 && !p->in_fragment) {
            fail(p, ev, 1002);
            return;
        }
        if (p->opcode != 0 && p->in_fragment) {
            fail(p, ev, 1002);
            return;
        }
        if (len > WS_MAX_MESSAGE || p->msg_len + len > WS_MAX_MESSAGE) {
            fail(p, ev, 1009);
            return;
        }
    }
    p->payload_len = len;
    p->payload_got = 0;
    p->control_len = 0;
    p->state = WS_STATE_PAYLOAD;
}

static void frame_done(ws_parser_t *p, ws_event_t *ev) {
    p->state = WS_STATE_HEADER;
    p->head_len = 0;
    p->head_need = 2;
    if (p->opcode >= 8) {
        ev->data = p->control;
        ev->len = p->control_len;
        if (p->opcode == 9) {
            ev->kind = WS_EV_PING;
        } else if (p->opcode == 10) {
            ev->kind = WS_EV_PONG;
        } else {
            ev->kind = WS_EV_CLOSE;
            ev->close_code = 1005;
            if (p->control_len == 1) {
                fail(p, ev, 1002);
                return;
            }
            if (p->control_len >= 2) {
                uint16_t code = (uint16_t)((p->control[0] << 8) | p->control[1]);
                if (!close_code_valid(code)) {
                    fail(p, ev, 1002);
                    return;
                }
                if (!ws_utf8_valid(p->control + 2, p->control_len - 2)) {
                    fail(p, ev, 1007);
                    return;
                }
                ev->close_code = code;
            }
        }
        return;
    }
    if (p->opcode != 0) {
        p->fragment_opcode = p->opcode;
    }
    if (!p->fin) {
        p->in_fragment = true;
        return;
    }
    p->in_fragment = false;
    ev->data = p->msg;
    ev->len = p->msg_len;
    p->msg_len = 0;
    if (p->fragment_opcode == 1) {
        if (!ws_utf8_valid(ev->data, ev->len)) {
            fail(p, ev, 1007);
            return;
        }
        ev->kind = WS_EV_TEXT;
    } else {
        ev->kind = WS_EV_BINARY;
    }
}

size_t ws_parser_feed(ws_parser_t *p, const uint8_t *in, size_t n, ws_event_t *ev) {
    ev->kind = WS_EV_NONE;
    ev->data = NULL;
    ev->len = 0;
    ev->close_code = 0;
    if (p->failed) {
        ev->kind = WS_EV_ERROR;
        ev->close_code = 1002;
        return n;
    }
    size_t used = 0;
    while (used < n && ev->kind == WS_EV_NONE) {
        if (p->state == WS_STATE_HEADER) {
            p->head[p->head_len++] = in[used++];
            if (p->head_len == 2) {
                uint8_t l7 = p->head[1] & 0x7FU;
                size_t ext = l7 == 126 ? 2 : (l7 == 127 ? 8 : 0);
                p->head_need = 2 + ext + ((p->head[1] & 0x80U) != 0 ? 4U : 0U);
            }
            if (p->head_len >= p->head_need && p->head_len >= 2) {
                header_done(p, ev);
                if (ev->kind == WS_EV_NONE && p->payload_len == 0) {
                    frame_done(p, ev);
                }
            }
        } else {
            uint64_t remaining = p->payload_len - p->payload_got;
            size_t take = (uint64_t)(n - used) < remaining ? (n - used) : (size_t)remaining;
            for (size_t i = 0; i < take; i++) {
                uint8_t b = in[used + i] ^ p->mask[(p->payload_got + i) & 3U];
                if (p->opcode >= 8) {
                    p->control[p->control_len++] = b;
                } else {
                    p->msg[p->msg_len++] = b;
                }
            }
            used += take;
            p->payload_got += take;
            if (p->payload_got == p->payload_len) {
                frame_done(p, ev);
            }
        }
    }
    return used;
}

size_t ws_frame_header(uint8_t dst[10], uint8_t opcode, size_t len) {
    dst[0] = (uint8_t)(0x80U | (opcode & 0x0FU));
    if (len < 126) {
        dst[1] = (uint8_t)len;
        return 2;
    }
    if (len <= 0xFFFF) {
        dst[1] = 126;
        dst[2] = (uint8_t)(len >> 8);
        dst[3] = (uint8_t)len;
        return 4;
    }
    dst[1] = 127;
    for (size_t i = 0; i < 8; i++) {
        dst[2 + i] = (uint8_t)((uint64_t)len >> (56 - 8 * i));
    }
    return 10;
}

size_t ws_encode_frame(uint8_t *dst, size_t cap, uint8_t opcode, const void *data, size_t len) {
    uint8_t hdr[10];
    size_t hdr_len = ws_frame_header(hdr, opcode, len);
    if (cap < hdr_len || cap - hdr_len < len) {
        return 0;
    }
    memcpy(dst, hdr, hdr_len);
    if (len > 0 && data != NULL) {
        memcpy(dst + hdr_len, data, len);
    }
    return hdr_len + len;
}
