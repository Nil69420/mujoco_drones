#include "foxglove/ws.h"

#include <errno.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>

int ws_conn_open(ws_conn_t *conn, int fd, size_t tx_cap) {
    memset(conn, 0, sizeof(*conn));
    conn->fd = -1;
    conn->parser = malloc(sizeof(*conn->parser));
    conn->tx = malloc(tx_cap);
    if (conn->parser == NULL || conn->tx == NULL) {
        free(conn->parser);
        free(conn->tx);
        conn->parser = NULL;
        conn->tx = NULL;
        return -1;
    }
    ws_parser_init(conn->parser);
    conn->tx_cap = tx_cap;
    conn->fd = fd;
    return 0;
}

void ws_conn_close(ws_conn_t *conn) {
    if (conn->fd >= 0) {
        close(conn->fd);
    }
    free(conn->parser);
    free(conn->tx);
    memset(conn, 0, sizeof(*conn));
    conn->fd = -1;
}

bool ws_conn_enqueue(ws_conn_t *conn, uint8_t opcode, const void *head, size_t head_len,
                     const void *body, size_t body_len) {
    uint8_t hdr[10];
    size_t payload = head_len + body_len;
    size_t hdr_len = ws_frame_header(hdr, opcode, payload);
    size_t total = hdr_len + payload;
    if (conn->tx_head > 0 && conn->tx_cap - conn->tx_len - conn->tx_head < total) {
        memmove(conn->tx, conn->tx + conn->tx_head, conn->tx_len);
        conn->tx_head = 0;
    }
    if (conn->tx_cap - conn->tx_head - conn->tx_len < total) {
        conn->dropped++;
        return false;
    }
    uint8_t *dst = conn->tx + conn->tx_head + conn->tx_len;
    memcpy(dst, hdr, hdr_len);
    if (head_len > 0) {
        memcpy(dst + hdr_len, head, head_len);
    }
    if (body_len > 0) {
        memcpy(dst + hdr_len + head_len, body, body_len);
    }
    conn->tx_len += total;
    return true;
}

int ws_conn_flush(ws_conn_t *conn) {
    while (conn->tx_len > 0) {
        ssize_t n = send(conn->fd, conn->tx + conn->tx_head, conn->tx_len,
                         MSG_NOSIGNAL | MSG_DONTWAIT);
        if (n < 0) {
            if (errno == EINTR) {
                continue;
            }
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                return 0;
            }
            return -1;
        }
        conn->tx_head += (size_t)n;
        conn->tx_len -= (size_t)n;
    }
    conn->tx_head = 0;
    return 0;
}

bool ws_conn_pending(const ws_conn_t *conn) {
    return conn->tx_len > 0;
}
