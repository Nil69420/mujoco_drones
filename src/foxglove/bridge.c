#define _GNU_SOURCE
#include "foxglove/foxglove.h"
#include "foxglove/proto.h"
#include "foxglove/serialize.h"
#include "foxglove/ws.h"
#include "sensors/sensor_types.h"

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
#include <sys/time.h>
#include <time.h>
#include <unistd.h>

typedef struct {
    int       fd;
    uint64_t  deadline_ms;
    char     *buf;
    size_t    len;
} fg_pending_t;

enum {
    FG_MAX_PENDING = 8,
    FG_HANDSHAKE_TIMEOUT_MS = 3000,
    FG_STALL_TIMEOUT_MS = 10000,
    FG_RX_CHUNK = 4096,
    FG_LISTEN_BACKLOG = 16
};

#define FG_TX_CAP ((size_t)4 * 1024 * 1024)

struct foxglove_bridge {
    transport_t   *tp;
    uint16_t       port;
    char           bind_addr[64];
    fg_pending_t   pending[FG_MAX_PENDING];
    int            listen_fd;
    atomic_bool    running;
    pthread_t      thread;
    bool           thread_started;

    fg_channel_t   channels[FG_MAX_CHANNELS];
    int            num_channels;

    transport_sub_t subs[FG_MAX_CHANNELS];

    fg_client_t    clients[FG_MAX_CLIENTS];

    uint8_t *cam_rgb_buf;
    char    *cam_json_buf;
    size_t   cam_rgb_buf_sz;
    size_t   cam_json_buf_sz;
    int      cam_rgb_ch;
};

static const char SCHEMA_IMU[] =
    "{\"type\":\"object\",\"properties\":{"
    "\"timestamp_ns\":{\"type\":\"integer\"},"
    "\"seq\":{\"type\":\"integer\"},"
    "\"accel\":{\"type\":\"array\",\"items\":{\"type\":\"number\"}},"
    "\"gyro\":{\"type\":\"array\",\"items\":{\"type\":\"number\"}},"
    "\"mag\":{\"type\":\"array\",\"items\":{\"type\":\"number\"}},"
    "\"orientation\":{\"type\":\"array\",\"items\":{\"type\":\"number\"}}"
    "}}";

static const char SCHEMA_GNSS[] =
    "{\"type\":\"object\",\"properties\":{"
    "\"timestamp_ns\":{\"type\":\"integer\"},"
    "\"seq\":{\"type\":\"integer\"},"
    "\"latitude\":{\"type\":\"number\"},"
    "\"longitude\":{\"type\":\"number\"},"
    "\"altitude\":{\"type\":\"number\"},"
    "\"velocity\":{\"type\":\"array\",\"items\":{\"type\":\"number\"}},"
    "\"fix_type\":{\"type\":\"integer\"},"
    "\"num_satellites\":{\"type\":\"integer\"},"
    "\"hdop\":{\"type\":\"number\"}"
    "}}";

static const char SCHEMA_BARO[] =
    "{\"type\":\"object\",\"properties\":{"
    "\"timestamp_ns\":{\"type\":\"integer\"},"
    "\"seq\":{\"type\":\"integer\"},"
    "\"pressure_pa\":{\"type\":\"number\"},"
    "\"temperature_c\":{\"type\":\"number\"},"
    "\"altitude_m\":{\"type\":\"number\"}"
    "}}";

static const char SCHEMA_LIDAR[] =
    "{\"type\":\"object\",\"properties\":{"
    "\"timestamp_ns\":{\"type\":\"integer\"},"
    "\"seq\":{\"type\":\"integer\"},"
    "\"num_rays\":{\"type\":\"integer\"},"
    "\"angle_min\":{\"type\":\"number\"},"
    "\"angle_max\":{\"type\":\"number\"},"
    "\"range_min\":{\"type\":\"number\"},"
    "\"range_max\":{\"type\":\"number\"},"
    "\"ranges\":{\"type\":\"array\",\"items\":{\"type\":\"number\"}}"
    "}}";

static const char SCHEMA_INFRARED[] =
    "{\"type\":\"object\",\"properties\":{"
    "\"timestamp_ns\":{\"type\":\"integer\"},"
    "\"seq\":{\"type\":\"integer\"},"
    "\"range_m\":{\"type\":\"number\"},"
    "\"range_min\":{\"type\":\"number\"},"
    "\"range_max\":{\"type\":\"number\"},"
    "\"beam_angle\":{\"type\":\"number\"}"
    "}}";

static const char SCHEMA_CAMERA_META[] =
    "{\"type\":\"object\",\"properties\":{"
    "\"timestamp_ns\":{\"type\":\"integer\"},"
    "\"seq\":{\"type\":\"integer\"},"
    "\"width\":{\"type\":\"integer\"},"
    "\"height\":{\"type\":\"integer\"},"
    "\"channels\":{\"type\":\"integer\"},"
    "\"fov_y\":{\"type\":\"number\"},"
    "\"rgb_size\":{\"type\":\"integer\"},"
    "\"depth_size\":{\"type\":\"integer\"}"
    "}}";

static const char SCHEMA_RAW_IMAGE[] =
    "{\"type\":\"object\",\"properties\":{"
    "\"timestamp\":{\"type\":\"object\",\"properties\":{"
    "\"sec\":{\"type\":\"integer\"},\"nsec\":{\"type\":\"integer\"}}},"
    "\"frame_id\":{\"type\":\"string\"},"
    "\"width\":{\"type\":\"integer\"},"
    "\"height\":{\"type\":\"integer\"},"
    "\"encoding\":{\"type\":\"string\"},"
    "\"step\":{\"type\":\"integer\"},"
    "\"data\":{\"type\":\"string\",\"contentEncoding\":\"base64\"}"
    "}}";

static const struct {
    const char *topic;
    const char *schema_name;
    const char *schema;
    size_t      size;
} DRONE_TOPICS[] = {
    { "/drone/imu",          "sensor_imu_t",         SCHEMA_IMU,         sizeof(sensor_imu_t)         },
    { "/drone/gnss",         "sensor_gnss_t",        SCHEMA_GNSS,        sizeof(sensor_gnss_t)        },
    { "/drone/baro",         "sensor_baro_t",        SCHEMA_BARO,        sizeof(sensor_baro_t)        },
    { "/drone/lidar",        "sensor_lidar_t",       SCHEMA_LIDAR,       sizeof(sensor_lidar_t)       },
    { "/drone/infrared",     "sensor_infrared_t",    SCHEMA_INFRARED,    sizeof(sensor_infrared_t)    },
    { "/drone/camera/meta",  "sensor_camera_meta_t", SCHEMA_CAMERA_META, sizeof(sensor_camera_meta_t) },
    { "/drone/camera/rgb",   "foxglove.RawImage",    SCHEMA_RAW_IMAGE,
      sizeof(sensor_camera_rgb_hdr_t) +
      (size_t)CAM_BUF_MAX_W * (size_t)CAM_BUF_MAX_H * (size_t)3 },
};
enum {
    NUM_DRONE_TOPICS = (int)(sizeof(DRONE_TOPICS) / sizeof(DRONE_TOPICS[0])),
    DRONE_TOPIC_CAM_RGB_IDX = 6,
};

static bool bridge_validate_camera_rgb(const uint8_t *msg_buf,
                                       size_t msg_len,
                                       size_t max_len) {
    if (msg_len < sizeof(sensor_camera_rgb_hdr_t)) {
        return false;
    }

    const sensor_camera_rgb_hdr_t *hdr = (const sensor_camera_rgb_hdr_t *)msg_buf;
    if (hdr->width == 0 || hdr->height == 0 || hdr->channels != 3) {
        return false;
    }

    size_t pixel_count = (size_t)hdr->width * (size_t)hdr->height;
    if (pixel_count / (size_t)hdr->height != (size_t)hdr->width) {
        return false;
    }

    size_t rgb_size = pixel_count * (size_t)hdr->channels;
    if (rgb_size / (size_t)hdr->channels != pixel_count) {
        return false;
    }

    size_t total = sizeof(sensor_camera_rgb_hdr_t) + rgb_size;
    if (total < sizeof(sensor_camera_rgb_hdr_t)) {
        return false;
    }

    if (total > msg_len || total > max_len) {
        return false;
    }

    return true;
}

static uint64_t now_ms(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000U + (uint64_t)ts.tv_nsec / 1000000U;
}

static void reply_and_close(int fd, const char *response) {
    (void)send(fd, response, strlen(response), MSG_NOSIGNAL | MSG_DONTWAIT);
    close(fd);
}

static void pending_free(fg_pending_t *pd) {
    free(pd->buf);
    pd->buf = NULL;
    pd->fd = -1;
    pd->len = 0;
}

static void bridge_accept_connections(foxglove_bridge_t *br) {
    for (;;) {
        int cfd = accept4(br->listen_fd, NULL, NULL, SOCK_NONBLOCK | SOCK_CLOEXEC);
        if (cfd < 0) {
            return;
        }
        int slot = -1;
        for (int i = 0; i < FG_MAX_PENDING; i++) {
            if (br->pending[i].fd < 0) { slot = i; break; }
        }
        char *buf = slot >= 0 ? malloc(WS_HANDSHAKE_MAX + 1) : NULL;
        if (slot < 0 || buf == NULL) {
            free(buf);
            reply_and_close(cfd, "HTTP/1.1 503 Service Unavailable\r\nConnection: close\r\n"
                                 "Content-Length: 0\r\n\r\n");
            continue;
        }
        int flag = 1;
        setsockopt(cfd, IPPROTO_TCP, TCP_NODELAY, &flag, sizeof(flag));
        setsockopt(cfd, SOL_SOCKET, SO_KEEPALIVE, &flag, sizeof(flag));
        int idle = 30;
        int intvl = 10;
        int cnt = 3;
        setsockopt(cfd, IPPROTO_TCP, TCP_KEEPIDLE, &idle, sizeof(idle));
        setsockopt(cfd, IPPROTO_TCP, TCP_KEEPINTVL, &intvl, sizeof(intvl));
        setsockopt(cfd, IPPROTO_TCP, TCP_KEEPCNT, &cnt, sizeof(cnt));
        br->pending[slot].fd = cfd;
        br->pending[slot].buf = buf;
        br->pending[slot].len = 0;
        br->pending[slot].deadline_ms = now_ms() + FG_HANDSHAKE_TIMEOUT_MS;
    }
}

static void bridge_close_client(foxglove_bridge_t *br, int ci, const char *why) {
    fprintf(stderr, "[foxglove] client disconnected (slot %d): %s\n", ci, why);
    fg_close_client(&br->clients[ci]);
}

static void bridge_send_close(fg_client_t *c, uint16_t code) {
    uint8_t body[2] = {(uint8_t)(code >> 8), (uint8_t)code};
    (void)ws_conn_enqueue(&c->conn, WS_OP_CLOSE, body, sizeof(body), NULL, 0);
    (void)ws_conn_flush(&c->conn);
}

static void bridge_feed_client(foxglove_bridge_t *br, int ci, const uint8_t *data, size_t n) {
    fg_client_t *c = &br->clients[ci];
    size_t off = 0;
    while (off < n && c->alive) {
        ws_event_t ev;
        size_t used = ws_parser_feed(c->conn.parser, data + off, n - off, &ev);
        off += used;
        switch (ev.kind) {
        case WS_EV_TEXT:
            (void)fg_handle_client_message(c, (const char *)ev.data, ev.len,
                                           (uint32_t)br->num_channels);
            break;
        case WS_EV_PING:
            (void)ws_conn_enqueue(&c->conn, WS_OP_PONG, ev.data, ev.len, NULL, 0);
            break;
        case WS_EV_CLOSE:
            bridge_send_close(c, ev.close_code == 1005 ? 1000 : ev.close_code);
            bridge_close_client(br, ci, "client closed");
            return;
        case WS_EV_ERROR:
            bridge_send_close(c, ev.close_code);
            bridge_close_client(br, ci, "protocol error");
            return;
        case WS_EV_BINARY:
        case WS_EV_PONG:
        case WS_EV_NONE:
            break;
        }
        if (used == 0) {
            break;
        }
    }
}

static void bridge_promote(foxglove_bridge_t *br, int pi, const ws_handshake_t *hs) {
    fg_pending_t *pd = &br->pending[pi];
    int slot = -1;
    for (int i = 0; i < FG_MAX_CLIENTS; i++) {
        if (!br->clients[i].alive) { slot = i; break; }
    }
    if (slot < 0) {
        reply_and_close(pd->fd, "HTTP/1.1 503 Service Unavailable\r\nConnection: close\r\n"
                                "Content-Length: 0\r\n\r\n");
        pending_free(pd);
        return;
    }
    char resp[512];
    int rlen = ws_handshake_response(hs, resp, sizeof(resp));
    if (rlen < 0 || send(pd->fd, resp, (size_t)rlen, MSG_NOSIGNAL | MSG_DONTWAIT) != rlen) {
        close(pd->fd);
        pending_free(pd);
        return;
    }
    fg_client_t *c = &br->clients[slot];
    if (ws_conn_open(&c->conn, pd->fd, FG_TX_CAP) != 0) {
        close(pd->fd);
        pending_free(pd);
        return;
    }
    c->alive = true;
    c->num_subs = 0;
    c->stall_since_ms = 0;
    size_t extra = pd->len > hs->request_len ? pd->len - hs->request_len : 0;
    uint8_t *leftover = NULL;
    if (extra > 0) {
        leftover = malloc(extra);
        if (leftover != NULL) {
            memcpy(leftover, pd->buf + hs->request_len, extra);
        }
    }
    pending_free(pd);
    (void)fg_send_server_info(c);
    (void)fg_send_advertise(c, br->channels, br->num_channels);
    (void)ws_conn_flush(&c->conn);
    fprintf(stderr, "[foxglove] client connected (slot %d)\n", slot);
    if (leftover != NULL) {
        bridge_feed_client(br, slot, leftover, extra);
        free(leftover);
    }
}

static void bridge_handle_pending(foxglove_bridge_t *br, int pi) {
    fg_pending_t *pd = &br->pending[pi];
    ssize_t n = recv(pd->fd, pd->buf + pd->len, WS_HANDSHAKE_MAX - pd->len, MSG_DONTWAIT);
    if (n == 0 || (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR)) {
        close(pd->fd);
        pending_free(pd);
        return;
    }
    if (n < 0) {
        return;
    }
    pd->len += (size_t)n;
    ws_handshake_t hs;
    ws_handshake_status_t st = ws_handshake_check(pd->buf, pd->len, &hs);
    if (st == WS_HS_NEED_MORE) {
        return;
    }
    if (st == WS_HS_BAD) {
        reply_and_close(pd->fd, "HTTP/1.1 400 Bad Request\r\nConnection: close\r\n"
                                "Content-Length: 0\r\n\r\n");
        pending_free(pd);
        return;
    }
    bridge_promote(br, pi, &hs);
}

static void bridge_handle_client_data(foxglove_bridge_t *br, int ci) {
    uint8_t rx[FG_RX_CHUNK];
    ssize_t n = recv(br->clients[ci].conn.fd, rx, sizeof(rx), MSG_DONTWAIT);
    if (n == 0) {
        bridge_close_client(br, ci, "peer closed");
        return;
    }
    if (n < 0) {
        if (errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) {
            bridge_close_client(br, ci, "receive error");
        }
        return;
    }
    bridge_feed_client(br, ci, rx, (size_t)n);
}

static void bridge_flush_clients(foxglove_bridge_t *br) {
    uint64_t now = now_ms();
    for (int ci = 0; ci < FG_MAX_CLIENTS; ci++) {
        fg_client_t *c = &br->clients[ci];
        if (!c->alive || !ws_conn_pending(&c->conn)) {
            if (c->alive) {
                c->stall_since_ms = 0;
            }
            continue;
        }
        size_t before = c->conn.tx_len;
        if (ws_conn_flush(&c->conn) != 0) {
            bridge_close_client(br, ci, "send error");
            continue;
        }
        if (!ws_conn_pending(&c->conn) || c->conn.tx_len < before) {
            c->stall_since_ms = 0;
        } else if (c->stall_since_ms == 0) {
            c->stall_since_ms = now;
        } else if (now - c->stall_since_ms > FG_STALL_TIMEOUT_MS) {
            bridge_close_client(br, ci, "client stopped reading");
        }
    }
}

static void bridge_broadcast_channel(foxglove_bridge_t *br, int ch,
                                     uint8_t *msg_buf, size_t msg_buf_sz,
                                     char *json_buf, size_t json_buf_sz) {
    enum { MAX_DRAIN_SUBSCRIBED = 64 };
    enum { MAX_DRAIN_UNSUBSCRIBED = 1 };
    enum { MAX_ZERO_LEN_READS = 4 };

    uint32_t ch_id = br->channels[ch].id;
    bool has_sub = false;
    for (int ci = 0; ci < FG_MAX_CLIENTS; ci++) {
        if (!br->clients[ci].alive) continue;
        if (fg_find_sub(&br->clients[ci], ch_id)) {
            has_sub = true;
            break;
        }
    }
    int max_drain = has_sub ? MAX_DRAIN_SUBSCRIBED : MAX_DRAIN_UNSUBSCRIBED;

    size_t out_len = 0;
    size_t last_valid_len = 0;
    int drained = 0;
    int zero_len_reads = 0;

    for (int i = 0; i < max_drain; i++) {
        if (transport_read_next(&br->subs[ch], msg_buf, msg_buf_sz,
                                &out_len) != 0) {
            break;
        }
        if (out_len > 0) {
            drained++;
            last_valid_len = out_len;
            zero_len_reads = 0;
        } else {
            zero_len_reads++;
            if (zero_len_reads >= MAX_ZERO_LEN_READS) {
                break;
            }
        }
    }
    if (drained == 0) return;
    if (!has_sub) return;

    if (ch == br->cam_rgb_ch &&
        !bridge_validate_camera_rgb(msg_buf, last_valid_len, br->cam_rgb_buf_sz)) {
        return;
    }

    int jlen = fg_serializers[ch](msg_buf, json_buf, json_buf_sz);
    if (jlen <= 0 || jlen >= (int)json_buf_sz) return;

    uint64_t ts = 0;
    if (last_valid_len >= sizeof(sensor_header_t)) {
        const sensor_header_t *hdr = (const sensor_header_t *)msg_buf;
        ts = hdr->timestamp_ns;
    }

    for (int ci = 0; ci < FG_MAX_CLIENTS; ci++) {
        if (!br->clients[ci].alive) continue;

        const fg_subscription_t *sub = fg_find_sub(&br->clients[ci], ch_id);
        if (!sub) continue;

        (void)fg_send_message(&br->clients[ci], sub->sub_id, ts, json_buf, (size_t)jlen);
    }
}

static void *bridge_thread(void *arg) {
    foxglove_bridge_t *br = arg;

    char    json_buf[8192];
    uint8_t msg_buf[sizeof(sensor_lidar_t)];

    while (atomic_load(&br->running)) {
        struct pollfd pfds[1 + FG_MAX_PENDING + FG_MAX_CLIENTS];
        int owner[1 + FG_MAX_PENDING + FG_MAX_CLIENTS];
        int nfds = 0;

        pfds[nfds].fd     = br->listen_fd;
        pfds[nfds].events = POLLIN;
        owner[nfds]       = -1;
        nfds++;

        for (int i = 0; i < FG_MAX_PENDING; i++) {
            if (br->pending[i].fd >= 0) {
                pfds[nfds].fd     = br->pending[i].fd;
                pfds[nfds].events = POLLIN;
                owner[nfds]       = 1000 + i;
                nfds++;
            }
        }
        for (int i = 0; i < FG_MAX_CLIENTS; i++) {
            if (br->clients[i].alive) {
                pfds[nfds].fd     = br->clients[i].conn.fd;
                pfds[nfds].events = (short)(POLLIN | (ws_conn_pending(&br->clients[i].conn) ? POLLOUT : 0));
                owner[nfds]       = i;
                nfds++;
            }
        }

        int ready = poll(pfds, (nfds_t)nfds, 5);

        if (ready > 0) {
            if (pfds[0].revents & POLLIN) {
                bridge_accept_connections(br);
            }
            for (int k = 1; k < nfds; k++) {
                if (owner[k] >= 1000) {
                    if (pfds[k].revents & (POLLIN | POLLHUP | POLLERR)) {
                        bridge_handle_pending(br, owner[k] - 1000);
                    }
                } else if (br->clients[owner[k]].alive &&
                           (pfds[k].revents & (POLLIN | POLLHUP | POLLERR))) {
                    bridge_handle_client_data(br, owner[k]);
                }
            }
        }

        uint64_t tick = now_ms();
        for (int i = 0; i < FG_MAX_PENDING; i++) {
            if (br->pending[i].fd >= 0 && tick > br->pending[i].deadline_ms) {
                close(br->pending[i].fd);
                pending_free(&br->pending[i]);
            }
        }

        for (int ch = 0; ch < br->num_channels; ch++) {
            if (ch == br->cam_rgb_ch && !br->cam_rgb_buf) {
                continue;
            }

            if (br->cam_rgb_buf && ch == br->cam_rgb_ch) {
                bridge_broadcast_channel(br, ch,
                                         br->cam_rgb_buf, br->cam_rgb_buf_sz,
                                         br->cam_json_buf, br->cam_json_buf_sz);
            } else {
                bridge_broadcast_channel(br, ch, msg_buf, sizeof(msg_buf),
                                         json_buf, sizeof(json_buf));
            }
        }

        bridge_flush_clients(br);
    }

    for (int i = 0; i < FG_MAX_CLIENTS; i++) {
        if (br->clients[i].alive) fg_close_client(&br->clients[i]);
    }
    for (int i = 0; i < FG_MAX_PENDING; i++) {
        if (br->pending[i].fd >= 0) {
            close(br->pending[i].fd);
            pending_free(&br->pending[i]);
        }
    }

    return NULL;
}

foxglove_bridge_t *foxglove_create(transport_t *tp, uint16_t port) {
    if (!tp) return NULL;

    foxglove_bridge_t *br = calloc(1, sizeof(foxglove_bridge_t));
    if (!br) return NULL;

    br->tp             = tp;
    br->port           = port;
    br->listen_fd      = -1;
    br->thread_started = false;
    atomic_store(&br->running, false);

    const char *env_bind = getenv("MUJOCO_DRONES_FOXGLOVE_BIND");
    snprintf(br->bind_addr, sizeof(br->bind_addr), "%s",
             env_bind != NULL && env_bind[0] != '\0' ? env_bind : "127.0.0.1");
    for (int i = 0; i < FG_MAX_CLIENTS; i++) {
        br->clients[i].conn.fd = -1;
        br->clients[i].alive   = false;
    }
    for (int i = 0; i < FG_MAX_PENDING; i++) {
        br->pending[i].fd = -1;
    }

    int num_drone_topics = NUM_DRONE_TOPICS;
    int max_channels = FG_MAX_CHANNELS;
    int topic_count = num_drone_topics < max_channels ?
                      num_drone_topics : max_channels;
    for (int i = 0; i < topic_count; i++) {
        br->channels[i].id          = (uint32_t)(i + 1);
        br->channels[i].topic       = DRONE_TOPICS[i].topic;
        br->channels[i].schema_name = DRONE_TOPICS[i].schema_name;
        br->channels[i].schema      = DRONE_TOPICS[i].schema;
        br->channels[i].msg_size    = DRONE_TOPICS[i].size;

        if (transport_subscribe(tp, DRONE_TOPICS[i].topic,
                                DRONE_TOPICS[i].size,
                                &br->subs[i]) != 0) {
            fprintf(stderr, "[foxglove] WARNING: failed to subscribe to %s\n",
                    DRONE_TOPICS[i].topic);
        }
    }
    br->num_channels = topic_count;

    /* Allocate dedicated heap buffers for the large camera RGB channel */
    br->cam_rgb_ch      = DRONE_TOPIC_CAM_RGB_IDX;
    br->cam_rgb_buf_sz  = sizeof(sensor_camera_rgb_hdr_t) +
                          (size_t)CAM_BUF_MAX_W * CAM_BUF_MAX_H * 3;
    br->cam_json_buf_sz = (size_t)CAM_BUF_MAX_W * CAM_BUF_MAX_H * 4 + 512;
    br->cam_rgb_buf  = malloc(br->cam_rgb_buf_sz);
    br->cam_json_buf = malloc(br->cam_json_buf_sz);
    if (!br->cam_rgb_buf || !br->cam_json_buf) {
        fprintf(stderr, "[foxglove] WARNING: failed to alloc camera buffers, "
                        "camera stream disabled\n");
        free(br->cam_rgb_buf);
        free(br->cam_json_buf);
        br->cam_rgb_buf  = NULL;
        br->cam_json_buf = NULL;
        br->cam_rgb_ch   = -1;
    }

    return br;
}

int foxglove_start(foxglove_bridge_t *br) {
    if (!br) return -1;

    br->listen_fd = socket(AF_INET, SOCK_STREAM | SOCK_NONBLOCK | SOCK_CLOEXEC, 0);
    if (br->listen_fd < 0) {
        perror("[foxglove] socket");
        return -1;
    }

    int opt = 1;
    setsockopt(br->listen_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    struct sockaddr_in addr = {0};
    addr.sin_family      = AF_INET;
    if (inet_pton(AF_INET, br->bind_addr, &addr.sin_addr) != 1) {
        fprintf(stderr, "[foxglove] invalid bind address '%s'\n", br->bind_addr);
        close(br->listen_fd);
        br->listen_fd = -1;
        return -1;
    }
    addr.sin_port        = htons(br->port);

    if (bind(br->listen_fd, (struct sockaddr *)&addr, sizeof(addr)) != 0) {
        perror("[foxglove] bind");
        close(br->listen_fd);
        br->listen_fd = -1;
        return -1;
    }

    if (listen(br->listen_fd, FG_LISTEN_BACKLOG) != 0) {
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
    br->thread_started = true;

    fprintf(stderr, "[foxglove] WebSocket server on ws://%s:%u\n", br->bind_addr,
            br->port);
    return 0;
}

void foxglove_destroy(foxglove_bridge_t *br) {
    if (!br) return;

    atomic_store(&br->running, false);

    if (br->listen_fd >= 0) {
        close(br->listen_fd);
    }

    if (br->thread_started) {
        pthread_join(br->thread, NULL);
        br->thread_started = false;
    }

    br->listen_fd = -1;

    for (int i = 0; i < br->num_channels; i++) {
        transport_close_sub(&br->subs[i]);
    }

    free(br->cam_rgb_buf);
    free(br->cam_json_buf);
    free(br);
}
