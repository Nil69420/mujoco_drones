#include "transport/frame_out.h"

#include <stdio.h>
#include <string.h>

#include "renoir.h"

int frame_out_open(frame_out_t *out, const char *name, size_t max_frame, uint32_t slots) {
    memset(out, 0, sizeof(*out));
    RenoirFrameHandle handle = NULL;
    enum RenoirErrorCode rc = renoir_frame_open(name, max_frame, slots, 0, &handle);
    if (rc != RenoirErrorCode_Success) {
        fprintf(stderr, "[frame] cannot open ring '%s' (%zu bytes x %u): error %d\n", name,
                max_frame, (unsigned)slots, (int)rc);
        return -1;
    }
    out->handle = handle;
    out->max_frame = max_frame;
    return 0;
}

uint8_t *frame_out_loan(frame_out_t *out, size_t len) {
    uint8_t *data = NULL;
    if (out->handle == NULL || len > out->max_frame ||
        renoir_frame_loan(out->handle, len, &data) != RenoirErrorCode_Success) {
        out->failed++;
        return NULL;
    }
    return data;
}

int frame_out_commit(frame_out_t *out) {
    if (renoir_frame_commit_notify(out->handle) != RenoirErrorCode_Success) {
        out->failed++;
        return -1;
    }
    out->committed++;
    return 0;
}

void frame_out_abort(frame_out_t *out) {
    if (out->handle != NULL) {
        (void)renoir_frame_abort(out->handle);
    }
}

void frame_out_close(frame_out_t *out) {
    if (out->handle != NULL) {
        renoir_frame_close(out->handle);
    }
    memset(out, 0, sizeof(*out));
}

void frame_out_name(char *buf, size_t cap, const char *prefix, const char *suffix) {
    const char *p = (prefix != NULL && prefix[0] != '\0') ? prefix : "/drone";
    while (*p == '/') {
        p++;
    }
    size_t n = 0;
    for (; *p != '\0' && n + 1 < cap; p++) {
        buf[n++] = *p == '/' ? '_' : *p;
    }
    for (const char *s = suffix; *s != '\0' && n + 1 < cap; s++) {
        buf[n++] = *s;
    }
    buf[n] = '\0';
}
