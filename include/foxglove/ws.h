#ifndef MUJOCO_DRONES_FOXGLOVE_WS_H
#define MUJOCO_DRONES_FOXGLOVE_WS_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "foxglove/ws_proto.h"

enum {
    WS_OP_TEXT = 0x1,
    WS_OP_BIN = 0x2,
    WS_OP_CLOSE = 0x8,
    WS_OP_PING = 0x9,
    WS_OP_PONG = 0xA,
};

typedef struct {
    int fd;
    ws_parser_t *parser;
    uint8_t *tx;
    size_t tx_cap;
    size_t tx_head;
    size_t tx_len;
    uint64_t dropped;
} ws_conn_t;

int ws_conn_open(ws_conn_t *conn, int fd, size_t tx_cap);
void ws_conn_close(ws_conn_t *conn);
bool ws_conn_enqueue(ws_conn_t *conn, uint8_t opcode, const void *head, size_t head_len,
                     const void *body, size_t body_len);
int ws_conn_flush(ws_conn_t *conn);
bool ws_conn_pending(const ws_conn_t *conn);

#endif
