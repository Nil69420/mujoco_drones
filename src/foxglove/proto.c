#include "foxglove/proto.h"
#include "foxglove/ws.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

int fg_send_server_info(fg_client_t *c) {
    const char *msg =
        "{\"op\":\"serverInfo\","
        "\"name\":\"mujoco_drones\","
        "\"capabilities\":[],"
        "\"metadata\":{}}";
    return ws_conn_enqueue(&c->conn, WS_OP_TEXT, msg, strlen(msg), NULL, 0) ? 0 : -1;
}

int fg_send_status(fg_client_t *c, int level, const char *message) {
    char buf[256];
    int n = snprintf(buf, sizeof(buf), "{\"op\":\"status\",\"level\":%d,\"message\":\"%s\"}",
                     level, message);
    if (n <= 0 || (size_t)n >= sizeof(buf)) {
        return -1;
    }
    return ws_conn_enqueue(&c->conn, WS_OP_TEXT, buf, (size_t)n, NULL, 0) ? 0 : -1;
}

int fg_send_advertise(fg_client_t *c, const fg_channel_t *channels, int n) {
    size_t need = 128;
    for (int i = 0; i < n; i++) {
        need += 512;
        if (channels[i].schema) {
            need += strlen(channels[i].schema) * 2;
        }
    }

    char *buf = malloc(need);
    if (!buf) return -1;

    char *esc = malloc(need);
    if (!esc) { free(buf); return -1; }

    int off = 0;
    int rem = (int)need;
    int w = 0;

    w = snprintf(buf + off, (size_t)rem, "{\"op\":\"advertise\",\"channels\":[");
    if (w < 0 || w >= rem) { free(buf); free(esc); return -1; }
    off += w; rem -= w;

    for (int i = 0; i < n; i++) {
        if (rem < 300) { free(buf); free(esc); return -1; }
        if (i > 0) { buf[off++] = ','; rem--; }

        if (channels[i].schema && channels[i].schema[0]) {
            size_t ej = 0;
            for (const char *schema_ptr = channels[i].schema;
                 *schema_ptr && ej + 2 < need;
                 schema_ptr++) {
                if (*schema_ptr == '"' || *schema_ptr == '\\') esc[ej++] = '\\';
                esc[ej++] = *schema_ptr;
            }
            esc[ej] = '\0';

            w = snprintf(buf + off, (size_t)rem,
                    "{\"id\":%u,"
                    "\"topic\":\"%s\","
                    "\"encoding\":\"json\","
                    "\"schemaName\":\"%s\","
                    "\"schemaEncoding\":\"jsonschema\","
                    "\"schema\":\"%s\"}",
                    channels[i].id,
                    channels[i].topic,
                    channels[i].schema_name,
                    esc);
        } else {
            w = snprintf(buf + off, (size_t)rem,
                    "{\"id\":%u,"
                    "\"topic\":\"%s\","
                    "\"encoding\":\"json\","
                    "\"schemaName\":\"%s\","
                    "\"schema\":\"\"}",
                    channels[i].id,
                    channels[i].topic,
                    channels[i].schema_name);
        }
        if (w < 0 || w >= rem) { free(buf); free(esc); return -1; }
        off += w; rem -= w;
    }

    if (rem < 3) { free(buf); free(esc); return -1; }
    buf[off++] = ']';
    buf[off++] = '}';
    buf[off]   = '\0';

    int rc = ws_conn_enqueue(&c->conn, WS_OP_TEXT, buf, (size_t)off, NULL, 0) ? 0 : -1;
    free(buf);
    free(esc);
    return rc;
}

int fg_send_message(fg_client_t *c, uint32_t sub_id, uint64_t timestamp_ns,
                    const void *json, size_t json_len) {
    uint8_t head[13];
    head[0] = 0x01;
    head[1] = (uint8_t)(sub_id);
    head[2] = (uint8_t)(sub_id >> 8);
    head[3] = (uint8_t)(sub_id >> 16);
    head[4] = (uint8_t)(sub_id >> 24);
    for (int i = 0; i < 8; i++) {
        head[5 + i] = (uint8_t)(timestamp_ns >> (8 * i));
    }
    return ws_conn_enqueue(&c->conn, WS_OP_BIN, head, sizeof(head), json, json_len) ? 0 : 1;
}

static int fg_find_sub_id(const fg_client_t *client, uint32_t sub_id) {
    for (int i = 0; i < client->num_subs; i++) {
        if (client->subs[i].sub_id == sub_id) {
            return i;
        }
    }
    return -1;
}

int fg_handle_client_message(fg_client_t *client, const char *msg, size_t len,
                             uint32_t num_channels) {
    fg_op_t op;
    if (fg_parse_client_op(msg, len, &op) != 0) {
        (void)fg_send_status(client, 2, "invalid message");
        return -1;
    }
    int rc = 0;
    if (op.kind == FG_OP_SUBSCRIBE) {
        for (int i = 0; i < op.n_subs; i++) {
            const fg_op_sub_t *s = &op.subs[i];
            if (s->channel_id == 0 || s->channel_id > num_channels) {
                (void)fg_send_status(client, 2, "unknown channel");
                rc = -1;
            } else if (fg_find_sub_id(client, s->id) >= 0) {
                (void)fg_send_status(client, 2, "duplicate subscription id");
                rc = -1;
            } else if (client->num_subs >= FG_MAX_SUBS) {
                (void)fg_send_status(client, 2, "too many subscriptions");
                rc = -1;
            } else {
                client->subs[client->num_subs].sub_id = s->id;
                client->subs[client->num_subs].channel_id = s->channel_id;
                client->num_subs++;
            }
        }
        if (op.overflow) {
            (void)fg_send_status(client, 2, "too many entries");
            rc = -1;
        }
    } else if (op.kind == FG_OP_UNSUBSCRIBE) {
        for (int i = 0; i < op.n_unsub; i++) {
            int at = fg_find_sub_id(client, op.unsub_ids[i]);
            if (at >= 0) {
                client->subs[at] = client->subs[--client->num_subs];
            }
        }
    }
    return rc;
}

const fg_subscription_t *fg_find_sub(const fg_client_t *c, uint32_t ch_id) {
    for (int i = 0; i < c->num_subs; i++) {
        if (c->subs[i].channel_id == ch_id) {
            return &c->subs[i];
        }
    }
    return NULL;
}

void fg_close_client(fg_client_t *c) {
    ws_conn_close(&c->conn);
    c->alive    = false;
    c->num_subs = 0;
}
