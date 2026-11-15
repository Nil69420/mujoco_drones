#ifndef MUJOCO_DRONES_FOXGLOVE_WS_PROTO_H
#define MUJOCO_DRONES_FOXGLOVE_WS_PROTO_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

enum {
    WS_HANDSHAKE_MAX = 8192,
    WS_MAX_MESSAGE = 65536,
    WS_ACCEPT_LEN = 28
};

typedef enum {
    WS_HS_NEED_MORE = 0,
    WS_HS_OK = 1,
    WS_HS_BAD = 2
} ws_handshake_status_t;

typedef struct {
    char accept[WS_ACCEPT_LEN + 1];
    size_t request_len;
    bool wants_foxglove_subprotocol;
} ws_handshake_t;

ws_handshake_status_t ws_handshake_check(const char *buf, size_t len, ws_handshake_t *out);
int ws_handshake_response(const ws_handshake_t *hs, char *dst, size_t cap);

typedef enum {
    WS_EV_NONE = 0,
    WS_EV_TEXT,
    WS_EV_BINARY,
    WS_EV_PING,
    WS_EV_PONG,
    WS_EV_CLOSE,
    WS_EV_ERROR
} ws_event_kind_t;

typedef struct {
    ws_event_kind_t kind;
    const uint8_t *data;
    size_t len;
    uint16_t close_code;
} ws_event_t;

typedef enum {
    WS_STATE_HEADER = 0,
    WS_STATE_PAYLOAD
} ws_state_t;

typedef struct {
    ws_state_t state;
    uint8_t head[14];
    size_t head_len;
    size_t head_need;
    bool fin;
    uint8_t opcode;
    bool masked;
    uint8_t mask[4];
    uint64_t payload_len;
    uint64_t payload_got;
    bool in_fragment;
    bool failed;
    uint8_t fragment_opcode;
    size_t msg_len;
    uint8_t msg[WS_MAX_MESSAGE];
    uint8_t control[125];
    size_t control_len;
} ws_parser_t;

void ws_parser_init(ws_parser_t *p);
size_t ws_parser_feed(ws_parser_t *p, const uint8_t *in, size_t n, ws_event_t *ev);
size_t ws_frame_header(uint8_t dst[10], uint8_t opcode, size_t len);
size_t ws_encode_frame(uint8_t *dst, size_t cap, uint8_t opcode, const void *data, size_t len);
bool ws_utf8_valid(const uint8_t *s, size_t n);

#endif
