#include "foxglove/proto.h"
#include "foxglove/ws.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

int fg_send_server_info(int fd) {
    const char *msg =
        "{\"op\":\"serverInfo\","
        "\"name\":\"mujoco_drones\","
        "\"capabilities\":[],"
        "\"metadata\":{}}";
    return ws_send_frame(fd, WS_OP_TEXT, msg, strlen(msg));
}

int fg_send_advertise(int fd, const fg_channel_t *channels, int n) {
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
            for (const char *p = channels[i].schema; *p && ej + 2 < need; p++) {
                if (*p == '"' || *p == '\\') esc[ej++] = '\\';
                esc[ej++] = *p;
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

    int rc = ws_send_frame(fd, WS_OP_TEXT, buf, (size_t)off);
    free(buf);
    free(esc);
    return rc;
}

int fg_send_message(int fd, uint32_t sub_id, uint64_t timestamp_ns,
                    const void *json, size_t json_len) {
    size_t total = 1 + 4 + 8 + json_len;
    uint8_t *frame_buf = malloc(total);
    if (!frame_buf) return -1;

    frame_buf[0] = 0x01;

    frame_buf[1] = (uint8_t)(sub_id);
    frame_buf[2] = (uint8_t)(sub_id >> 8);
    frame_buf[3] = (uint8_t)(sub_id >> 16);
    frame_buf[4] = (uint8_t)(sub_id >> 24);

    for (int i = 0; i < 8; i++) {
        frame_buf[5 + i] = (uint8_t)(timestamp_ns >> (8 * i));
    }

    memcpy(frame_buf + 13, json, json_len);

    int rc = ws_send_frame(fd, WS_OP_BIN, frame_buf, total);
    free(frame_buf);
    return rc;
}

int fg_parse_uint(const char *s, const char *key, uint32_t *out) {
    const char *pos = strstr(s, key);
    if (!pos) return -1;
    pos += strlen(key);
    while (*pos && (*pos == ':' || *pos == ' ' || *pos == '"')) pos++;
    if (*pos < '0' || *pos > '9') return -1;
    *out = (uint32_t)strtoul(pos, NULL, 10);
    return 0;
}

void fg_handle_client_message(fg_client_t *client, const char *msg,
                              size_t len) {
    char buf[4096];
    size_t copy = len < sizeof(buf) - 1 ? len : sizeof(buf) - 1;
    memcpy(buf, msg, copy);
    buf[copy] = '\0';

    if (strstr(buf, "\"subscribe\"")) {
        /* Foxglove Studio may send all subscriptions in one message:
         * {"op":"subscribe","subscriptions":[{"id":1,"channelId":1},...]}
         * Walk every {…} object in the array so all channels are registered. */
        const char *arr = strchr(buf, '[');
        const char *p   = arr ? arr + 1 : buf;
        while (*p && *p != ']') {
            const char *obj = strchr(p, '{');
            if (!obj) break;
            const char *end = strchr(obj, '}');
            if (!end) break;

            uint32_t sub_id = 0, ch_id = 0;
            int got_sub = fg_parse_uint(obj, "\"id\"", &sub_id);
            int got_ch  = fg_parse_uint(obj, "\"channelId\"", &ch_id);
            if (got_sub == 0 && got_ch == 0 && ch_id > 0 &&
                client->num_subs < FG_MAX_SUBS) {
                client->subs[client->num_subs].sub_id     = sub_id;
                client->subs[client->num_subs].channel_id = ch_id;
                client->num_subs++;
                fprintf(stderr,
                        "[fg_proto] subscribed: sub_id=%u channelId=%u "
                        "(total=%d)\n",
                        sub_id, ch_id, client->num_subs);
            }
            p = end + 1;
        }
    } else if (strstr(buf, "\"unsubscribe\"")) {
        const char *pos = strstr(buf, "\"subscriptionIds\"");
        if (pos) {
            pos = strchr(pos, '[');
            if (pos) {
                pos++;
                while (*pos && *pos != ']') {
                    char *end = NULL;
                    uint32_t sub_id = (uint32_t)strtoul(pos, &end, 10);
                    if (end == pos) break;
                    for (int i = 0; i < client->num_subs; i++) {
                        if (client->subs[i].sub_id == sub_id) {
                            client->subs[i] = client->subs[--client->num_subs];
                            break;
                        }
                    }
                    pos = end;
                    while (*pos == ',' || *pos == ' ') pos++;
                }
            }
        }
    }
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
    if (c->fd >= 0) {
        close(c->fd);
        c->fd = -1;
    }
    c->alive    = false;
    c->num_subs = 0;
}
