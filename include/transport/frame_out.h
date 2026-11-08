#ifndef MUJOCO_DRONES_FRAME_OUT_H
#define MUJOCO_DRONES_FRAME_OUT_H

#include <stddef.h>
#include <stdint.h>

typedef struct {
    void *handle;
    size_t max_frame;
    uint64_t committed;
    uint64_t failed;
} frame_out_t;

int frame_out_open(frame_out_t *out, const char *name, size_t max_frame, uint32_t slots);
uint8_t *frame_out_loan(frame_out_t *out, size_t len);
int frame_out_commit(frame_out_t *out);
void frame_out_abort(frame_out_t *out);
void frame_out_close(frame_out_t *out);
void frame_out_name(char *buf, size_t cap, const char *prefix, const char *suffix);

#endif
