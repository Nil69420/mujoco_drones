#ifndef MUJOCO_DRONES_FOXGLOVE_OPS_H
#define MUJOCO_DRONES_FOXGLOVE_OPS_H

#include <stddef.h>
#include <stdint.h>

enum {
    FG_OPS_MAX_ENTRIES = 32,
    FG_OPS_MAX_DEPTH = 16,
    FG_OPS_MAX_MESSAGE = 65536
};

typedef enum {
    FG_OP_UNKNOWN = 0,
    FG_OP_SUBSCRIBE = 1,
    FG_OP_UNSUBSCRIBE = 2
} fg_op_kind_t;

typedef struct {
    uint32_t id;
    uint32_t channel_id;
} fg_op_sub_t;

typedef struct {
    fg_op_kind_t kind;
    fg_op_sub_t subs[FG_OPS_MAX_ENTRIES];
    int n_subs;
    uint32_t unsub_ids[FG_OPS_MAX_ENTRIES];
    int n_unsub;
    int overflow;
} fg_op_t;

int fg_parse_client_op(const char *msg, size_t len, fg_op_t *out);

#endif
