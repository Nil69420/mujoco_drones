#include "foxglove/foxglove.h"
#include "foxglove/proto.h"
#include "foxglove/serialize.h"
#include "foxglove/ws.h"
#include "sensors/sensor_types.h"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <poll.h>
#include <pthread.h>
#include <stdatomic.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
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
                    fg_close_client(&br->clients[i]);
                } else if (opcode == WS_OP_TEXT && plen > 0) {
                    fg_handle_client_message(&br->clients[i],
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

            int jlen = fg_serializers[ch](msg_buf, json_buf, sizeof(json_buf));
            if (jlen <= 0 || jlen >= (int)sizeof(json_buf)) continue;

            uint64_t ts = 0;
            if (out_len >= sizeof(sensor_header_t)) {
                const sensor_header_t *hdr = (const sensor_header_t *)msg_buf;
                ts = hdr->timestamp_ns;
            }

            uint32_t ch_id = br->channels[ch].id;
            for (int ci = 0; ci < FG_MAX_CLIENTS; ci++) {
                if (!br->clients[ci].alive) continue;

                const fg_subscription_t *sub = fg_find_sub(&br->clients[ci], ch_id);
                if (!sub) continue;

                if (fg_send_message(br->clients[ci].fd, sub->sub_id,
                                    ts, json_buf, (size_t)jlen) != 0) {
                    fprintf(stderr, "[foxglove] send failed, dropping "
                                    "client %d\n", ci);
                    fg_close_client(&br->clients[ci]);
                }
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

    free(br);
}
