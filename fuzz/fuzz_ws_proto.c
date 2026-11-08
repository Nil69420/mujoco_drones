#include "foxglove/ops.h"
#include "foxglove/ws_proto.h"

#include <stdint.h>
#include <stdlib.h>
#include <string.h>

int LLVMFuzzerTestOneInput(const uint8_t *data, size_t size);

int LLVMFuzzerTestOneInput(const uint8_t *data, size_t size) {
    if (size < 2) {
        return 0;
    }
    uint8_t mode = data[0];
    const uint8_t *body = data + 1;
    size_t n = size - 1;
    if ((mode & 2U) != 0) {
        fg_op_t op;
        (void)fg_parse_client_op((const char *)body, n, &op);
        return 0;
    }
    if ((mode & 1U) == 0) {
        ws_handshake_t hs;
        char *copy = malloc(n + 1);
        if (copy == NULL) {
            return 0;
        }
        memcpy(copy, body, n);
        copy[n] = '\0';
        ws_handshake_status_t st = ws_handshake_check(copy, n, &hs);
        if (st == WS_HS_OK) {
            char resp[512];
            (void)ws_handshake_response(&hs, resp, sizeof(resp));
        }
        free(copy);
        return 0;
    }
    ws_parser_t *p = malloc(sizeof(*p));
    if (p == NULL) {
        return 0;
    }
    ws_parser_init(p);
    size_t chunk = 1 + (size_t)(mode >> 1);
    size_t off = 0;
    while (off < n) {
        size_t take = n - off < chunk ? n - off : chunk;
        ws_event_t ev;
        size_t used = ws_parser_feed(p, body + off, take, &ev);
        if (used > take) {
            abort();
        }
        if (ev.kind == WS_EV_ERROR) {
            break;
        }
        if (ev.data != NULL && ev.len > 0) {
            volatile uint8_t sink = ev.data[ev.len - 1];
            (void)sink;
        }
        off += used == 0 ? take : used;
    }
    free(p);
    return 0;
}
