#include "foxglove/ws_proto.h"

#include "foxglove/base64.h"
#include "foxglove/sha1.h"

#include <stdio.h>
#include <string.h>

static const char WS_GUID[] = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11";
static const char SUBPROTOCOL[] = "foxglove.websocket.v1";

enum { MAX_HEADERS = 64, KEY_LEN = 24 };

static char lower(char ch) {
    if (ch >= 'A' && ch <= 'Z') {
        return (char)((int)ch + 32);
    }
    return ch;
}

static bool name_equals(const char *lhs, size_t lhs_len, const char *rhs) {
    size_t rhs_len = strlen(rhs);
    if (lhs_len != rhs_len) {
        return false;
    }
    for (size_t idx = 0; idx < lhs_len; idx++) {
        if (lower(lhs[idx]) != lower(rhs[idx])) {
            return false;
        }
    }
    return true;
}

static bool list_has_token(const char *value, size_t len, const char *token) {
    size_t token_len = strlen(token);
    size_t pos = 0;
    while (pos < len) {
        while (pos < len && (value[pos] == ' ' || value[pos] == '\t' || value[pos] == ',')) {
            pos++;
        }
        size_t start = pos;
        while (pos < len && value[pos] != ',' && value[pos] != ' ' && value[pos] != '\t') {
            pos++;
        }
        if (pos - start == token_len && name_equals(value + start, token_len, token)) {
            return true;
        }
    }
    return false;
}

static bool key_valid(const char *key, size_t len) {
    if (len != KEY_LEN || key[22] != '=' || key[23] != '=') {
        return false;
    }
    for (size_t idx = 0; idx < 22; idx++) {
        char ch = key[idx];
        bool ok = (ch >= 'A' && ch <= 'Z') || (ch >= 'a' && ch <= 'z') || (ch >= '0' && ch <= '9') ||
                  ch == '+' || ch == '/';
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

typedef struct {
    bool upgrade;
    bool connection;
    bool version;
    bool have_key;
    char key[KEY_LEN + 1];
} header_state_t;

static bool request_line_ok(const char *line, size_t first_len) {
    if (first_len < 14 || memcmp(line, "GET /", 5) != 0 ||
        memcmp(line + first_len - 9, " HTTP/1.1", 9) != 0) {
        return false;
    }
    for (size_t idx = 4; idx + 9 < first_len; idx++) {
        if (line[idx] == ' ' || (unsigned char)line[idx] < 0x21) {
            return false;
        }
    }
    return true;
}

static bool apply_header(header_state_t *state, ws_handshake_t *out, const char *name, size_t name_len,
                         const char *value, size_t value_len) {
    if (name_equals(name, name_len, "upgrade")) {
        state->upgrade = state->upgrade || list_has_token(value, value_len, "websocket");
    } else if (name_equals(name, name_len, "connection")) {
        state->connection = state->connection || list_has_token(value, value_len, "upgrade");
    } else if (name_equals(name, name_len, "sec-websocket-version")) {
        state->version = value_len == 2 && value[0] == '1' && value[1] == '3';
    } else if (name_equals(name, name_len, "sec-websocket-key")) {
        if (state->have_key || !key_valid(value, value_len)) {
            return false;
        }
        memcpy(state->key, value, KEY_LEN);
        state->key[KEY_LEN] = '\0';
        state->have_key = true;
    } else if (name_equals(name, name_len, "sec-websocket-protocol")) {
        out->wants_foxglove_subprotocol =
            out->wants_foxglove_subprotocol || list_has_token(value, value_len, SUBPROTOCOL);
    }
    return true;
}

static bool parse_header_line(header_state_t *state, ws_handshake_t *out, const char *line,
                              size_t line_len) {
    const char *colon = memchr(line, ':', line_len);
    if (colon == NULL || colon == line) {
        return false;
    }
    size_t name_len = (size_t)(colon - line);
    const char *value = colon + 1;
    size_t value_len = line_len - name_len - 1;
    while (value_len > 0 && (*value == ' ' || *value == '\t')) {
        value++;
        value_len--;
    }
    while (value_len > 0 && (value[value_len - 1] == ' ' || value[value_len - 1] == '\t')) {
        value_len--;
    }
    return apply_header(state, out, line, name_len, value, value_len);
}

static bool compute_accept(const header_state_t *state, ws_handshake_t *out) {
    char concat[KEY_LEN + sizeof(WS_GUID)];
    int written = snprintf(concat, sizeof(concat), "%s%s", state->key, WS_GUID);
    if (written <= 0) {
        return false;
    }
    uint8_t hash[20];
    fg_sha1((const uint8_t *)concat, (size_t)written, hash);
    char encoded[32];
    fg_base64_encode(hash, 20, encoded);
    memcpy(out->accept, encoded, WS_ACCEPT_LEN);
    out->accept[WS_ACCEPT_LEN] = '\0';
    return true;
}

ws_handshake_status_t ws_handshake_check(const char *buf, size_t len, ws_handshake_t *out) {
    const char *end = find_crlfcrlf(buf, len > WS_HANDSHAKE_MAX ? WS_HANDSHAKE_MAX : len);
    if (end == NULL) {
        return len >= WS_HANDSHAKE_MAX ? WS_HS_BAD : WS_HS_NEED_MORE;
    }
    size_t head_len = (size_t)(end - buf);
    memset(out, 0, sizeof(*out));
    out->request_len = head_len + 4;

    const char *stop = buf + head_len;
    const char *eol = memchr(buf, '\r', head_len);
    size_t first_len = eol == NULL ? head_len : (size_t)(eol - buf);
    if (!request_line_ok(buf, first_len)) {
        return WS_HS_BAD;
    }
    header_state_t state;
    memset(&state, 0, sizeof(state));
    int headers = 0;
    const char *line = eol == NULL ? stop : eol + 2;
    while (line < stop) {
        const char *line_end = memchr(line, '\r', (size_t)(stop - line));
        size_t line_len = line_end == NULL ? (size_t)(stop - line) : (size_t)(line_end - line);
        if (++headers > MAX_HEADERS || !parse_header_line(&state, out, line, line_len)) {
            return WS_HS_BAD;
        }
        line = line_end == NULL ? stop : line_end + 2;
    }
    if (!state.upgrade || !state.connection || !state.version || !state.have_key) {
        return WS_HS_BAD;
    }
    return compute_accept(&state, out) ? WS_HS_OK : WS_HS_BAD;
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
        uint8_t lead = s[i];
        if (lead < 0x80) {
            i++;
        } else if (lead >= 0xC2 && lead <= 0xDF) {
            if (i + 1 >= n || (s[i + 1] & 0xC0) != 0x80) {
                return false;
            }
            i += 2;
        } else if (lead >= 0xE0 && lead <= 0xEF) {
            if (i + 2 >= n || (s[i + 1] & 0xC0) != 0x80 || (s[i + 2] & 0xC0) != 0x80) {
                return false;
            }
            if (lead == 0xE0 && s[i + 1] < 0xA0) {
                return false;
            }
            if (lead == 0xED && s[i + 1] > 0x9F) {
                return false;
            }
            i += 3;
        } else if (lead >= 0xF0 && lead <= 0xF4) {
            if (i + 3 >= n || (s[i + 1] & 0xC0) != 0x80 || (s[i + 2] & 0xC0) != 0x80 ||
                (s[i + 3] & 0xC0) != 0x80) {
                return false;
            }
            if (lead == 0xF0 && s[i + 1] < 0x90) {
                return false;
            }
            if (lead == 0xF4 && s[i + 1] > 0x8F) {
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

static size_t ws_extended_length_bytes(uint8_t len7) {
    if (len7 == 126) {
        return 2;
    }
    return len7 == 127 ? 8 : 0;
}

static void feed_header_byte(ws_parser_t *prs, uint8_t byte, ws_event_t *ev) {
    prs->head[prs->head_len++] = byte;
    if (prs->head_len == 2) {
        uint8_t len7 = prs->head[1] & 0x7FU;
        size_t ext = ws_extended_length_bytes(len7);
        prs->head_need = 2 + ext + ((prs->head[1] & 0x80U) != 0 ? 4U : 0U);
    }
    if (prs->head_len >= prs->head_need && prs->head_len >= 2) {
        header_done(prs, ev);
        if (ev->kind == WS_EV_NONE && prs->payload_len == 0) {
            frame_done(prs, ev);
        }
    }
}

static size_t feed_payload(ws_parser_t *prs, const uint8_t *in, size_t avail, ws_event_t *ev) {
    uint64_t remaining = prs->payload_len - prs->payload_got;
    size_t take = (uint64_t)avail < remaining ? avail : (size_t)remaining;
    for (size_t idx = 0; idx < take; idx++) {
        uint8_t plain = in[idx] ^ prs->mask[(prs->payload_got + idx) & 3U];
        if (prs->opcode >= 8) {
            prs->control[prs->control_len++] = plain;
        } else {
            prs->msg[prs->msg_len++] = plain;
        }
    }
    prs->payload_got += take;
    if (prs->payload_got == prs->payload_len) {
        frame_done(prs, ev);
    }
    return take;
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
            feed_header_byte(p, in[used++], ev);
        } else {
            used += feed_payload(p, in + used, n - used, ev);
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
