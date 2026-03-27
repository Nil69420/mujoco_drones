#ifndef MUJOCO_DRONES_FOXGLOVE_PROTO_H
#define MUJOCO_DRONES_FOXGLOVE_PROTO_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

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

int fg_send_server_info(int fd);
int fg_send_advertise(int fd, const fg_channel_t *channels, int n);
int fg_send_message(int fd, uint32_t sub_id, uint64_t timestamp_ns,
                    const void *json, size_t json_len);

uint32_t fg_parse_uint(const char *s, const char *key);
void fg_handle_client_message(fg_client_t *client, const char *msg, size_t len);
const fg_subscription_t *fg_find_sub(const fg_client_t *c, uint32_t ch_id);
void fg_close_client(fg_client_t *c);

#endif
