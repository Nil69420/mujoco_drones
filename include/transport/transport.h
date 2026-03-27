#ifndef MUJOCO_DRONES_TRANSPORT_H
#define MUJOCO_DRONES_TRANSPORT_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

typedef struct transport_t     transport_t;
typedef struct transport_pub_t transport_pub_t;
typedef struct transport_sub_t transport_sub_t;

typedef struct {
    int  (*init)(transport_t *tp);
    void (*shutdown)(transport_t *tp);
    int  (*advertise)(transport_t *tp, const char *topic,
                      size_t max_msg_size, transport_pub_t *pub);
    int  (*subscribe)(transport_t *tp, const char *topic,
                      size_t max_msg_size, transport_sub_t *sub);
    int  (*publish)(transport_pub_t *pub, const void *data, size_t len);
    int  (*read_next)(transport_sub_t *sub, void *buf, size_t buf_size,
                      size_t *out_len);
    void (*close_pub)(transport_pub_t *pub);
    void (*close_sub)(transport_sub_t *sub);
} transport_ops_t;

struct transport_t {
    const transport_ops_t *ops;
    void                  *ctx;
};

struct transport_pub_t {
    transport_t *tp;
    void        *handle;
};

struct transport_sub_t {
    transport_t *tp;
    void        *handle;
};

static inline int transport_init(transport_t *tp) {
    return tp->ops->init(tp);
}

static inline void transport_shutdown(transport_t *tp) {
    tp->ops->shutdown(tp);
}

static inline int transport_advertise(transport_t *tp, const char *topic,
                                      size_t max_sz, transport_pub_t *pub) {
    pub->tp = tp;
    return tp->ops->advertise(tp, topic, max_sz, pub);
}

static inline int transport_subscribe(transport_t *tp, const char *topic,
                                      size_t max_sz, transport_sub_t *sub) {
    sub->tp = tp;
    return tp->ops->subscribe(tp, topic, max_sz, sub);
}

static inline int transport_publish(transport_pub_t *pub,
                                    const void *data, size_t len) {
    return pub->tp->ops->publish(pub, data, len);
}

static inline int transport_read_next(transport_sub_t *sub,
                                      void *buf, size_t buf_size,
                                      size_t *out_len) {
    return sub->tp->ops->read_next(sub, buf, buf_size, out_len);
}

static inline void transport_close_pub(transport_pub_t *pub) {
    if (pub->tp) {
        pub->tp->ops->close_pub(pub);
    }
}

static inline void transport_close_sub(transport_sub_t *sub) {
    if (sub->tp) {
        sub->tp->ops->close_sub(sub);
    }
}

#endif
