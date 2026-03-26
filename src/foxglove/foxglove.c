#include "foxglove.h"
#include "../sensors/sensor_types.h"

#include <arpa/inet.h>
#include <errno.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <poll.h>
#include <pthread.h>
#include <stdatomic.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/uio.h>
#include <unistd.h>

static void sha1_transform(uint32_t state[5], const uint8_t block[64]) {
    uint32_t w[80];
    for (int i = 0; i < 16; i++) {
        w[i] = ((uint32_t)block[4*i] << 24)
             | ((uint32_t)block[4*i+1] << 16)
             | ((uint32_t)block[4*i+2] << 8)
             | ((uint32_t)block[4*i+3]);
    }
    for (int i = 16; i < 80; i++) {
        uint32_t t = w[i-3] ^ w[i-8] ^ w[i-14] ^ w[i-16];
        w[i] = (t << 1) | (t >> 31);
    }

    uint32_t a = state[0], b = state[1], c = state[2];
    uint32_t d = state[3], e = state[4];

    for (int i = 0; i < 80; i++) {
        uint32_t f = 0;
        uint32_t k = 0;
        if      (i < 20) { f = (b & c) | (~b & d);           k = 0x5A827999U; }
        else if (i < 40) { f = b ^ c ^ d;                    k = 0x6ED9EBA1U; }
        else if (i < 60) { f = (b & c) | (b & d) | (c & d);  k = 0x8F1BBCDCU; }
        else              { f = b ^ c ^ d;                    k = 0xCA62C1D6U; }

        uint32_t tmp = ((a << 5) | (a >> 27)) + f + e + k + w[i];
        e = d; d = c; c = (b << 30) | (b >> 2); b = a; a = tmp;
    }
    state[0] += a; state[1] += b; state[2] += c;
    state[3] += d; state[4] += e;
}

static void sha1(const uint8_t *data, size_t len, uint8_t out[20]) {
    uint32_t state[5] = {
        0x67452301U, 0xEFCDAB89U, 0x98BADCFEU, 0x10325476U, 0xC3D2E1F0U
    };

    size_t i = 0;
    for (; i + 64 <= len; i += 64) {
        sha1_transform(state, data + i);
    }

    uint8_t block[64];
    size_t rem = len - i;
    memcpy(block, data + i, rem);
    block[rem++] = 0x80;

    if (rem > 56) {
        memset(block + rem, 0, 64 - rem);
        sha1_transform(state, block);
        rem = 0;
    }
    memset(block + rem, 0, 56 - rem);

    uint64_t bits = (uint64_t)len * 8;
    for (int j = 0; j < 8; j++) {
        block[56 + j] = (uint8_t)(bits >> (56 - 8*j));
    }

    sha1_transform(state, block);

    for (int j = 0; j < 5; j++) {
        out[4*j+0] = (uint8_t)(state[j] >> 24);
        out[4*j+1] = (uint8_t)(state[j] >> 16);
        out[4*j+2] = (uint8_t)(state[j] >> 8);
        out[4*j+3] = (uint8_t)(state[j]);
    }
}

static const char b64_table[] =
    "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

static size_t base64_encode(const uint8_t *in, size_t len, char *out) {
    size_t o = 0;
    for (size_t i = 0; i < len; i += 3) {
        uint32_t v = (uint32_t)in[i] << 16;
        if (i + 1 < len) v |= (uint32_t)in[i+1] << 8;
        if (i + 2 < len) v |= (uint32_t)in[i+2];

        out[o++] = b64_table[(v >> 18) & 0x3FU];
        out[o++] = b64_table[(v >> 12) & 0x3FU];
        out[o++] = (char)((i + 1 < len) ? b64_table[(v >> 6) & 0x3FU] : '=');
        out[o++] = (char)((i + 2 < len) ? b64_table[v & 0x3FU] : '=');
    }
    out[o] = '\0';
    return o;
}

#define WS_OP_TEXT   0x1
#define WS_OP_BIN    0x2
#define WS_OP_CLOSE  0x8
#define WS_OP_PING   0x9
#define WS_OP_PONG   0xA

static const char WS_MAGIC[] = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11";

static int ws_send_frame(int fd, uint8_t opcode, const void *data, size_t len) {
    uint8_t hdr[10];
    size_t hdr_len = 0;

    hdr[0] = (uint8_t)(0x80U | (opcode & 0x0FU));

    if (len < 126) {
        hdr[1] = (uint8_t)len;
        hdr_len = 2;
    } else if (len <= 0xFFFF) {
        hdr[1] = 126;
        hdr[2] = (uint8_t)(len >> 8);
        hdr[3] = (uint8_t)(len);
        hdr_len = 4;
    } else {
        hdr[1] = 127;
        for (int i = 0; i < 8; i++) {
            hdr[2+i] = (uint8_t)(len >> (56 - 8*i));
        }
        hdr_len = 10;
    }

    struct iovec iov[2] = {
        { .iov_base = hdr,          .iov_len = hdr_len },
        { .iov_base = (void*)data,  .iov_len = len },
    };
    ssize_t total = (ssize_t)(hdr_len + len);
    ssize_t sent = writev(fd, iov, data ? 2 : 1);
    return (sent == total) ? 0 : -1;
}

static ssize_t ws_recv_frame(int fd, uint8_t *buf, size_t buf_sz,
                             uint8_t *opcode) {
    uint8_t hdr[2];
    if (recv(fd, hdr, 2, MSG_WAITALL) != 2) return -1;

    *opcode = hdr[0] & 0x0FU;
    bool masked = (hdr[1] & 0x80U) != 0;
    uint64_t plen = hdr[1] & 0x7FU;

    if (plen == 126) {
        uint8_t ext[2];
        if (recv(fd, ext, 2, MSG_WAITALL) != 2) return -1;
        plen = ((uint64_t)ext[0] << 8) | ext[1];
    } else if (plen == 127) {
        uint8_t ext[8];
        if (recv(fd, ext, 8, MSG_WAITALL) != 8) return -1;
        plen = 0;
        for (int i = 0; i < 8; i++) {
            plen = (plen << 8) | ext[i];
        }
    }

    uint8_t mask[4] = {0};
    if (masked) {
        if (recv(fd, mask, 4, MSG_WAITALL) != 4) return -1;
    }

    if (plen > buf_sz) {
        size_t remaining = (size_t)plen;
        while (remaining > 0) {
            uint8_t tmp[4096];
            size_t chunk = remaining < sizeof(tmp) ? remaining : sizeof(tmp);
            ssize_t n = recv(fd, tmp, chunk, 0);
            if (n <= 0) return -1;
            remaining -= (size_t)n;
        }
        return -1;
    }

    size_t payload_len = (size_t)plen;
    if (payload_len > 0) {
        size_t got = 0;
        while (got < payload_len) {
            ssize_t n = recv(fd, buf + got, payload_len - got, 0);
            if (n <= 0) return -1;
            got += (size_t)n;
        }
    }

    if (masked) {
        for (size_t i = 0; i < payload_len; i++) {
            buf[i] ^= mask[i & 3];
        }
    }

    return (ssize_t)payload_len;
}

typedef struct {
    uint32_t    id;
    const char *topic;
    const char *schema_name;
    size_t      msg_size;
} fg_channel_t;

typedef struct {
    uint32_t sub_id;
    uint32_t channel_id;
} fg_subscription_t;

#define FG_MAX_SUBS  32

typedef struct {
    int                fd;
    bool               alive;
    fg_subscription_t  subs[FG_MAX_SUBS];
    int                num_subs;
} fg_client_t;

struct foxglove_bridge {
    transport_t   *tp;
    uint16_t       port;
    int            listen_fd;
    atomic_bool    running;
    pthread_t      thread;

    fg_channel_t   channels[FG_MAX_CHANNELS];
    int            num_channels;

    transport_sub_t subs[FG_MAX_CHANNELS];

    fg_client_t    clients[FG_MAX_CLIENTS];
};

static const struct {
    const char *topic;
    const char *schema;
    size_t      size;
} DRONE_TOPICS[] = {
    { "/drone/imu",          "sensor_imu_t",         sizeof(sensor_imu_t)         },
    { "/drone/gnss",         "sensor_gnss_t",        sizeof(sensor_gnss_t)        },
    { "/drone/baro",         "sensor_baro_t",        sizeof(sensor_baro_t)        },
    { "/drone/lidar",        "sensor_lidar_t",       sizeof(sensor_lidar_t)       },
    { "/drone/infrared",     "sensor_infrared_t",    sizeof(sensor_infrared_t)    },
    { "/drone/camera/meta",  "sensor_camera_meta_t", sizeof(sensor_camera_meta_t) },
};
#define NUM_DRONE_TOPICS  (int)(sizeof(DRONE_TOPICS) / sizeof(DRONE_TOPICS[0]))

static int fg_send_server_info(int fd) {
    const char *msg =
        "{\"op\":\"serverInfo\","
        "\"name\":\"mujoco_drones\","
        "\"capabilities\":[],"
        "\"supportedEncodings\":[\"json\"],"
        "\"metadata\":{}}";
    return ws_send_frame(fd, WS_OP_TEXT, msg, strlen(msg));
}

static int fg_send_advertise(int fd, const fg_channel_t *channels, int n) {
    char buf[4096];
    int off = 0;
    off += snprintf(buf + off, sizeof(buf) - (size_t)off,
                    "{\"op\":\"advertise\",\"channels\":[");

    for (int i = 0; i < n; i++) {
        if (i > 0) buf[off++] = ',';
        off += snprintf(buf + off, sizeof(buf) - (size_t)off,
                        "{\"id\":%u,"
                        "\"topic\":\"%s\","
                        "\"encoding\":\"json\","
                        "\"schemaName\":\"%s\","
                        "\"schema\":\"\"}",
                        channels[i].id,
                        channels[i].topic,
                        channels[i].schema_name);
    }
    off += snprintf(buf + off, sizeof(buf) - (size_t)off, "]}");

    return ws_send_frame(fd, WS_OP_TEXT, buf, (size_t)off);
}

static int serialize_imu(const void *data, char *buf, size_t sz) {
    const sensor_imu_t *m = data;
    return snprintf(buf, sz,
        "{\"timestamp_ns\":%lu,\"seq\":%u,"
        "\"accel\":[%.6f,%.6f,%.6f],"
        "\"gyro\":[%.6f,%.6f,%.6f],"
        "\"mag\":[%.6f,%.6f,%.6f],"
        "\"orientation\":[%.6f,%.6f,%.6f,%.6f]}",
        (unsigned long)m->header.timestamp_ns, m->header.sequence,
        m->accel[0], m->accel[1], m->accel[2],
        m->gyro[0],  m->gyro[1],  m->gyro[2],
        m->mag[0],   m->mag[1],   m->mag[2],
        m->orientation[0], m->orientation[1],
        m->orientation[2], m->orientation[3]);
}

static int serialize_gnss(const void *data, char *buf, size_t sz) {
    const sensor_gnss_t *m = data;
    return snprintf(buf, sz,
        "{\"timestamp_ns\":%lu,\"seq\":%u,"
        "\"latitude\":%.8f,\"longitude\":%.8f,\"altitude\":%.4f,"
        "\"velocity\":[%.6f,%.6f,%.6f],"
        "\"fix_type\":%u,\"num_satellites\":%u,\"hdop\":%.2f}",
        (unsigned long)m->header.timestamp_ns, m->header.sequence,
        m->latitude, m->longitude, m->altitude,
        m->velocity[0], m->velocity[1], m->velocity[2],
        m->fix_type, m->num_satellites, m->hdop);
}

static int serialize_baro(const void *data, char *buf, size_t sz) {
    const sensor_baro_t *m = data;
    return snprintf(buf, sz,
        "{\"timestamp_ns\":%lu,\"seq\":%u,"
        "\"pressure_pa\":%.2f,\"temperature_c\":%.2f,\"altitude_m\":%.4f}",
        (unsigned long)m->header.timestamp_ns, m->header.sequence,
        m->pressure_pa, m->temperature_c, m->altitude_m);
}

static int serialize_lidar(const void *data, char *buf, size_t sz) {
    const sensor_lidar_t *m = data;
    int off = snprintf(buf, sz,
        "{\"timestamp_ns\":%lu,\"seq\":%u,"
        "\"num_rays\":%u,\"angle_min\":%.4f,\"angle_max\":%.4f,"
        "\"range_min\":%.4f,\"range_max\":%.4f,\"ranges\":[",
        (unsigned long)m->header.timestamp_ns, m->header.sequence,
        m->num_rays, (double)m->angle_min, (double)m->angle_max,
        (double)m->range_min, (double)m->range_max);

    for (int i = 0; i < m->num_rays && (size_t)off < sz - 20; i++) {
        if (i > 0) buf[off++] = ',';
        off += snprintf(buf + off, sz - (size_t)off, "%.4f",
                        (double)m->ranges[i]);
    }
    off += snprintf(buf + off, sz - (size_t)off, "]}");
    return off;
}

static int serialize_infrared(const void *data, char *buf, size_t sz) {
    const sensor_infrared_t *m = data;
    return snprintf(buf, sz,
        "{\"timestamp_ns\":%lu,\"seq\":%u,"
        "\"range_m\":%.4f,\"range_min\":%.4f,"
        "\"range_max\":%.4f,\"beam_angle\":%.4f}",
        (unsigned long)m->header.timestamp_ns, m->header.sequence,
        (double)m->range_m, (double)m->range_min,
        (double)m->range_max, (double)m->beam_angle);
}

static int serialize_camera_meta(const void *data, char *buf, size_t sz) {
    const sensor_camera_meta_t *m = data;
    return snprintf(buf, sz,
        "{\"timestamp_ns\":%lu,\"seq\":%u,"
        "\"width\":%u,\"height\":%u,\"channels\":%u,"
        "\"fov_y\":%.4f,\"rgb_size\":%u,\"depth_size\":%u}",
        (unsigned long)m->header.timestamp_ns, m->header.sequence,
        m->width, m->height, m->channels,
        (double)m->fov_y, m->rgb_size, m->depth_size);
}

typedef int (*serialize_fn)(const void *data, char *buf, size_t sz);

static const serialize_fn serializers[] = {
    serialize_imu,
    serialize_gnss,
    serialize_baro,
    serialize_lidar,
    serialize_infrared,
    serialize_camera_meta,
};

static int fg_send_message(int fd, uint32_t sub_id, uint64_t timestamp_ns,
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

static int ws_accept_handshake(int fd) {
    char req[4096];
    ssize_t n = recv(fd, req, sizeof(req) - 1, 0);
    if (n <= 0) return -1;
    req[n] = '\0';

    const char *key_hdr = strstr(req, "Sec-WebSocket-Key: ");
    if (!key_hdr) return -1;
    key_hdr += 19;

    char ws_key[64];
    int ki = 0;
    while (key_hdr[ki] && key_hdr[ki] != '\r' && ki < 60) {
        ws_key[ki] = key_hdr[ki];
        ki++;
    }
    ws_key[ki] = '\0';

    char concat[128];
    snprintf(concat, sizeof(concat), "%s%s", ws_key, WS_MAGIC);

    uint8_t hash[20];
    sha1((const uint8_t *)concat, strlen(concat), hash);

    char accept_b64[32];
    base64_encode(hash, 20, accept_b64);

    char resp[512];
    int rlen = snprintf(resp, sizeof(resp),
        "HTTP/1.1 101 Switching Protocols\r\n"
        "Upgrade: websocket\r\n"
        "Connection: Upgrade\r\n"
        "Sec-WebSocket-Accept: %s\r\n"
        "Sec-WebSocket-Protocol: foxglove.websocket.v1\r\n"
        "\r\n",
        accept_b64);

    return (send(fd, resp, (size_t)rlen, 0) == rlen) ? 0 : -1;
}

static uint32_t parse_uint(const char *s, const char *key) {
    const char *p = strstr(s, key);
    if (!p) return 0;
    p += strlen(key);
    while (*p && (*p == ':' || *p == ' ' || *p == '"')) p++;
    return (uint32_t)strtoul(p, NULL, 10);
}

static void handle_client_message(fg_client_t *client, const char *msg,
                                  size_t len) {
    char buf[4096];
    size_t copy = len < sizeof(buf) - 1 ? len : sizeof(buf) - 1;
    memcpy(buf, msg, copy);
    buf[copy] = '\0';

    if (strstr(buf, "\"subscribe\"")) {
        uint32_t sub_id = parse_uint(buf, "\"id\"");
        uint32_t ch_id  = parse_uint(buf, "\"channelId\"");
        if (sub_id > 0 && ch_id > 0 && client->num_subs < FG_MAX_SUBS) {
            client->subs[client->num_subs].sub_id     = sub_id;
            client->subs[client->num_subs].channel_id = ch_id;
            client->num_subs++;
        }
    } else if (strstr(buf, "\"unsubscribe\"")) {
        uint32_t sub_id = parse_uint(buf, "\"subscriptionIds\"");
        for (int i = 0; i < client->num_subs; i++) {
            if (client->subs[i].sub_id == sub_id) {
                client->subs[i] = client->subs[--client->num_subs];
                break;
            }
        }
    }
}

static const fg_subscription_t *find_sub(const fg_client_t *c, uint32_t ch_id) {
    for (int i = 0; i < c->num_subs; i++) {
        if (c->subs[i].channel_id == ch_id) {
            return &c->subs[i];
        }
    }
    return NULL;
}

static void close_client(fg_client_t *c) {
    if (c->fd >= 0) {
        close(c->fd);
        c->fd = -1;
    }
    c->alive    = false;
    c->num_subs = 0;
}

static void *bridge_thread(void *arg) {
    foxglove_bridge_t *br = arg;

    uint8_t recv_buf[4096];
    char    json_buf[8192];
    uint8_t msg_buf[sizeof(sensor_lidar_t)];

    while (atomic_load(&br->running)) {
        struct pollfd pfds[1 + FG_MAX_CLIENTS];
        int nfds = 0;

        pfds[nfds].fd     = br->listen_fd;
        pfds[nfds].events = POLLIN;
        nfds++;

        for (int i = 0; i < FG_MAX_CLIENTS; i++) {
            if (br->clients[i].alive) {
                pfds[nfds].fd     = br->clients[i].fd;
                pfds[nfds].events = POLLIN;
                nfds++;
            }
        }

        int ready = poll(pfds, (nfds_t)nfds, 10);

        if (ready > 0 && (pfds[0].revents & POLLIN)) {
            int cfd = accept(br->listen_fd, NULL, NULL);
            if (cfd >= 0) {
                int slot = -1;
                for (int i = 0; i < FG_MAX_CLIENTS; i++) {
                    if (!br->clients[i].alive) { slot = i; break; }
                }

                if (slot < 0 || ws_accept_handshake(cfd) != 0) {
                    close(cfd);
                } else {
                    br->clients[slot].fd       = cfd;
                    br->clients[slot].alive     = true;
                    br->clients[slot].num_subs  = 0;

                    fg_send_server_info(cfd);
                    fg_send_advertise(cfd, br->channels, br->num_channels);

                    fprintf(stderr, "[foxglove] client connected (slot %d)\n",
                            slot);
                }
            }
        }

        int pfd_idx = 1;
        for (int i = 0; i < FG_MAX_CLIENTS; i++) {
            if (!br->clients[i].alive) continue;

            if (pfd_idx < nfds && (pfds[pfd_idx].revents & POLLIN)) {
                uint8_t opcode = 0;
                ssize_t plen = ws_recv_frame(br->clients[i].fd,
                                             recv_buf, sizeof(recv_buf),
                                             &opcode);
                if (plen < 0 || opcode == WS_OP_CLOSE) {
                    fprintf(stderr, "[foxglove] client disconnected (slot %d)\n",
                            i);
                    close_client(&br->clients[i]);
                } else if (opcode == WS_OP_TEXT && plen > 0) {
                    handle_client_message(&br->clients[i],
                                          (const char *)recv_buf, (size_t)plen);
                } else if (opcode == WS_OP_PING) {
                    ws_send_frame(br->clients[i].fd, WS_OP_PONG,
                                  recv_buf, (size_t)plen);
                }
            }
            pfd_idx++;
        }

        for (int ch = 0; ch < br->num_channels; ch++) {
            size_t out_len = 0;
            int rc = transport_read_next(&br->subs[ch],
                                         msg_buf, sizeof(msg_buf),
                                         &out_len);
            if (rc != 0 || out_len == 0) continue;

            int jlen = serializers[ch](msg_buf, json_buf, sizeof(json_buf));
            if (jlen <= 0) continue;

            uint64_t ts = 0;
            if (out_len >= sizeof(sensor_header_t)) {
                const sensor_header_t *hdr = (const sensor_header_t *)msg_buf;
                ts = hdr->timestamp_ns;
            }

            uint32_t ch_id = br->channels[ch].id;
            for (int ci = 0; ci < FG_MAX_CLIENTS; ci++) {
                if (!br->clients[ci].alive) continue;

                const fg_subscription_t *sub = find_sub(&br->clients[ci], ch_id);
                if (!sub) continue;

                if (fg_send_message(br->clients[ci].fd, sub->sub_id,
                                    ts, json_buf, (size_t)jlen) != 0) {
                    fprintf(stderr, "[foxglove] send failed, dropping "
                                    "client %d\n", ci);
                    close_client(&br->clients[ci]);
                }
            }
        }
    }

    for (int i = 0; i < FG_MAX_CLIENTS; i++) {
        if (br->clients[i].alive) close_client(&br->clients[i]);
    }

    return NULL;
}

foxglove_bridge_t *foxglove_create(transport_t *tp, uint16_t port) {
    if (!tp) return NULL;

    foxglove_bridge_t *br = calloc(1, sizeof(foxglove_bridge_t));
    if (!br) return NULL;

    br->tp        = tp;
    br->port      = port;
    br->listen_fd = -1;
    atomic_store(&br->running, false);

    for (int i = 0; i < FG_MAX_CLIENTS; i++) {
        br->clients[i].fd    = -1;
        br->clients[i].alive = false;
    }

    for (int i = 0; i < NUM_DRONE_TOPICS && i < FG_MAX_CHANNELS; i++) {
        br->channels[i].id          = (uint32_t)(i + 1);
        br->channels[i].topic       = DRONE_TOPICS[i].topic;
        br->channels[i].schema_name = DRONE_TOPICS[i].schema;
        br->channels[i].msg_size    = DRONE_TOPICS[i].size;

        if (transport_subscribe(tp, DRONE_TOPICS[i].topic,
                                DRONE_TOPICS[i].size,
                                &br->subs[i]) != 0) {
            fprintf(stderr, "[foxglove] WARNING: failed to subscribe to %s\n",
                    DRONE_TOPICS[i].topic);
        }
    }
    br->num_channels = NUM_DRONE_TOPICS;

    return br;
}

int foxglove_start(foxglove_bridge_t *br) {
    if (!br) return -1;

    br->listen_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (br->listen_fd < 0) {
        perror("[foxglove] socket");
        return -1;
    }

    int opt = 1;
    setsockopt(br->listen_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    struct sockaddr_in addr = {0};
    addr.sin_family      = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port        = htons(br->port);

    if (bind(br->listen_fd, (struct sockaddr *)&addr, sizeof(addr)) != 0) {
        perror("[foxglove] bind");
        close(br->listen_fd);
        br->listen_fd = -1;
        return -1;
    }

    if (listen(br->listen_fd, 4) != 0) {
        perror("[foxglove] listen");
        close(br->listen_fd);
        br->listen_fd = -1;
        return -1;
    }

    atomic_store(&br->running, true);

    if (pthread_create(&br->thread, NULL, bridge_thread, br) != 0) {
        perror("[foxglove] pthread_create");
        atomic_store(&br->running, false);
        close(br->listen_fd);
        br->listen_fd = -1;
        return -1;
    }

    fprintf(stderr, "[foxglove] WebSocket server on ws://0.0.0.0:%u\n",
            br->port);
    return 0;
}

void foxglove_destroy(foxglove_bridge_t *br) {
    if (!br) return;

    atomic_store(&br->running, false);

    if (br->listen_fd >= 0) {
        close(br->listen_fd);
        br->listen_fd = -1;
    }

    pthread_join(br->thread, NULL);

    for (int i = 0; i < br->num_channels; i++) {
        transport_close_sub(&br->subs[i]);
    }

    free(br);
}
