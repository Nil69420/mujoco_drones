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

struct foxglove_bridge {
    transport_t   *tp;
    uint16_t       port;
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

/* Max camera resolution used to size the heap buffers for the RGB channel.
 * Actual runtime dimensions come from sensor_camera_rgb_hdr_t. */
enum { CAM_BUF_MAX_W = 1280, CAM_BUF_MAX_H = 720 };

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

static void bridge_accept_connection(foxglove_bridge_t *br) {
    int cfd = accept(br->listen_fd, NULL, NULL);
    if (cfd < 0) return;

    int slot = -1;
    for (int i = 0; i < FG_MAX_CLIENTS; i++) {
        if (!br->clients[i].alive) { slot = i; break; }
    }

    if (slot < 0 || ws_accept_handshake(cfd) != 0) {
        close(cfd);
        return;
    }

    int flag = 1;
    setsockopt(cfd, IPPROTO_TCP, TCP_NODELAY, &flag, sizeof(flag));

    struct timeval rcv_tv = { .tv_sec = 0, .tv_usec = 50000 };
    setsockopt(cfd, SOL_SOCKET, SO_RCVTIMEO, &rcv_tv, sizeof(rcv_tv));

    struct timeval snd_tv = { .tv_sec = 0, .tv_usec = 5000 };
    setsockopt(cfd, SOL_SOCKET, SO_SNDTIMEO, &snd_tv, sizeof(snd_tv));

    br->clients[slot].fd       = cfd;
    br->clients[slot].alive    = true;
    br->clients[slot].num_subs = 0;

    fg_send_server_info(cfd);
    fg_send_advertise(cfd, br->channels, br->num_channels);
    fprintf(stderr, "[foxglove] client connected (slot %d)\n", slot);
}

static void bridge_handle_client_data(foxglove_bridge_t *br, int ci,
                                      uint8_t *recv_buf, size_t buf_sz) {
    uint8_t opcode = 0;
    ssize_t plen = ws_recv_frame(br->clients[ci].fd,
                                 recv_buf, buf_sz, &opcode);

    if (plen < 0 || opcode == WS_OP_CLOSE) {
        fprintf(stderr, "[foxglove] client disconnected (slot %d)\n", ci);
        fg_close_client(&br->clients[ci]);
    } else if (opcode == WS_OP_TEXT && plen > 0) {
        fg_handle_client_message(&br->clients[ci],
                                (const char *)recv_buf, (size_t)plen);
    } else if (opcode == WS_OP_PING) {
        ws_send_frame(br->clients[ci].fd, WS_OP_PONG,
                      recv_buf, (size_t)plen);
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

        if (fg_send_message(br->clients[ci].fd, sub->sub_id,
                            ts, json_buf, (size_t)jlen) != 0) {
            int send_err = errno;
            if (ch == br->cam_rgb_ch &&
                (send_err == EAGAIN || send_err == EWOULDBLOCK ||
                 send_err == ETIMEDOUT || send_err == ENOBUFS)) {
                continue;
            }
            fprintf(stderr, "[foxglove] send failed, dropping client %d\n",
                    ci);
            fg_close_client(&br->clients[ci]);
        }
    }
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
            bridge_accept_connection(br);
        }

        int pfd_idx = 1;
        for (int i = 0; i < FG_MAX_CLIENTS; i++) {
            if (!br->clients[i].alive) continue;
            if (pfd_idx < nfds && (pfds[pfd_idx].revents & POLLIN)) {
                bridge_handle_client_data(br, i, recv_buf, sizeof(recv_buf));
            }
            pfd_idx++;
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
    }

    for (int i = 0; i < FG_MAX_CLIENTS; i++) {
        if (br->clients[i].alive) fg_close_client(&br->clients[i]);
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

    for (int i = 0; i < FG_MAX_CLIENTS; i++) {
        br->clients[i].fd    = -1;
        br->clients[i].alive = false;
    }

    int topic_count = NUM_DRONE_TOPICS < FG_MAX_CHANNELS ?
                      NUM_DRONE_TOPICS : FG_MAX_CHANNELS;
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
    br->thread_started = true;

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

    if (br->thread_started) {
        pthread_join(br->thread, NULL);
        br->thread_started = false;
    }

    for (int i = 0; i < br->num_channels; i++) {
        transport_close_sub(&br->subs[i]);
    }

    free(br->cam_rgb_buf);
    free(br->cam_json_buf);
    free(br);
}
