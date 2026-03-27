#ifndef MUJOCO_DRONES_FOXGLOVE_WS_H
#define MUJOCO_DRONES_FOXGLOVE_WS_H

#include <stddef.h>
#include <stdint.h>
#include <sys/types.h>

#define WS_OP_TEXT   0x1
#define WS_OP_BIN    0x2
#define WS_OP_CLOSE  0x8
#define WS_OP_PING   0x9
#define WS_OP_PONG   0xA

int ws_send_frame(int fd, uint8_t opcode, const void *data, size_t len);
ssize_t ws_recv_frame(int fd, uint8_t *buf, size_t buf_sz, uint8_t *opcode);
int ws_accept_handshake(int fd);

#endif
