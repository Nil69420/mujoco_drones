#include "transport_renoir.h"

#include <renoir.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

typedef struct {
    RenoirPublisherHandle publisher;
} renoir_pub_ctx_t;

typedef struct {
    RenoirSubscriberHandle subscriber;
} renoir_sub_ctx_t;

typedef struct {
    RenoirTopicManagerHandle topic_mgr;
} renoir_ctx_t;

#define LARGE_PAYLOAD_THRESHOLD  4096

static int renoir_tp_init(transport_t *tp) {
    renoir_ctx_t *ctx = calloc(1, sizeof(renoir_ctx_t));
    if (!ctx) return -1;

    ctx->topic_mgr = renoir_topic_manager_create();
    if (!ctx->topic_mgr) {
        fprintf(stderr, "[transport/renoir] failed to create topic manager\n");
        free(ctx);
        return -1;
    }

    tp->ctx = ctx;
    return 0;
}

static void renoir_tp_shutdown(transport_t *tp) {
    renoir_ctx_t *ctx = tp->ctx;
    if (!ctx) return;

    if (ctx->topic_mgr) {
        renoir_topic_manager_destroy(ctx->topic_mgr);
        ctx->topic_mgr = NULL;
    }
    free(ctx);
    tp->ctx = NULL;
}

static int renoir_tp_advertise(transport_t *tp, const char *topic,
                            size_t max_msg_size, transport_pub_t *pub) {
    renoir_ctx_t *ctx = tp->ctx;
    enum RenoirErrorCode rc = Success;

    bool large = (max_msg_size > LARGE_PAYLOAD_THRESHOLD);
    struct RenoirTopicOptions opts = {
        .pattern             = 0,
        .ring_capacity       = large ? 4 : 64,
        .max_payload_size    = large ? LARGE_PAYLOAD_THRESHOLD : max_msg_size + 256,
        .use_shared_pool     = large,
        .shared_pool_threshold = large ? LARGE_PAYLOAD_THRESHOLD : 0,
        .enable_notifications = true,
    };

    RenoirTopicId tid = {0};
    rc = renoir_topic_register(ctx->topic_mgr, topic, &opts, &tid);
    if (rc != Success) {
        fprintf(stderr, "[transport/renoir] topic_register '%s' failed: %d\n",
                topic, rc);
        return -1;
    }

    renoir_pub_ctx_t *pctx = calloc(1, sizeof(renoir_pub_ctx_t));
    if (!pctx) return -1;

    struct RenoirPublisherOptions pub_opts = {0};
    rc = renoir_publisher_create(ctx->topic_mgr, topic, &pub_opts,
                                 &pctx->publisher);
    if (rc != Success) {
        fprintf(stderr, "[transport/renoir] publisher_create '%s' failed: %d\n",
                topic, rc);
        free(pctx);
        return -1;
    }

    pub->handle = pctx;
    return 0;
}

static int renoir_tp_subscribe(transport_t *tp, const char *topic,
                            size_t max_msg_size, transport_sub_t *sub) {
    renoir_ctx_t *ctx = tp->ctx;
    enum RenoirErrorCode rc = Success;

    bool large = (max_msg_size > LARGE_PAYLOAD_THRESHOLD);
    struct RenoirTopicOptions opts = {
        .pattern             = 0,
        .ring_capacity       = large ? 4 : 64,
        .max_payload_size    = large ? LARGE_PAYLOAD_THRESHOLD : max_msg_size + 256,
        .use_shared_pool     = large,
        .shared_pool_threshold = large ? LARGE_PAYLOAD_THRESHOLD : 0,
        .enable_notifications = true,
    };

    RenoirTopicId tid = {0};
    rc = renoir_topic_register(ctx->topic_mgr, topic, &opts, &tid);
    if (rc != Success) {
        fprintf(stderr, "[transport/renoir] topic_register '%s' failed: %d\n",
                topic, rc);
        return -1;
    }

    renoir_sub_ctx_t *sctx = calloc(1, sizeof(renoir_sub_ctx_t));
    if (!sctx) return -1;

    struct RenoirSubscriberOptions sub_opts = {
        .mode = 1,
    };
    rc = renoir_subscriber_create(ctx->topic_mgr, topic, &sub_opts,
                                  &sctx->subscriber);
    if (rc != Success) {
        fprintf(stderr, "[transport/renoir] subscriber_create '%s' failed: %d\n",
                topic, rc);
        free(sctx);
        return -1;
    }

    sub->handle = sctx;
    return 0;
}

static int renoir_tp_publish(transport_pub_t *pub, const void *data, size_t len) {
    renoir_pub_ctx_t *pctx = pub->handle;
    if (!pctx) return -1;

    enum RenoirErrorCode rc = renoir_publish_try(
        pctx->publisher,
        (const uint8_t *)data, len,
        NULL, NULL);

    if (rc == BufferFull) return 0;
    return (rc == Success) ? 0 : -1;
}

static int renoir_tp_read_next(transport_sub_t *sub, void *buf,
                            size_t buf_size, size_t *out_len) {
    renoir_sub_ctx_t *sctx = sub->handle;
    if (!sctx) return -1;

    struct RenoirReceivedMessage msg = {0};
    enum RenoirErrorCode rc = renoir_subscribe_read_next(
        sctx->subscriber, &msg, 0);

    if (rc == BufferEmpty) return 1;
    if (rc != Success)     return -1;

    size_t copy_len = msg.payload_len < buf_size ? msg.payload_len : buf_size;
    if (msg.payload_ptr && copy_len > 0) {
        memcpy(buf, msg.payload_ptr, copy_len);
    }

    if (out_len) {
        *out_len = copy_len;
    }

    renoir_message_release(msg.handle);
    return 0;
}

static void renoir_tp_close_pub(transport_pub_t *pub) {
    renoir_pub_ctx_t *pctx = pub->handle;
    if (!pctx) return;

    if (pctx->publisher) {
        renoir_publisher_destroy(pctx->publisher);
    }

    free(pctx);
    pub->handle = NULL;
}

static void renoir_tp_close_sub(transport_sub_t *sub) {
    renoir_sub_ctx_t *sctx = sub->handle;
    if (!sctx) return;

    if (sctx->subscriber) {
        renoir_subscriber_destroy(sctx->subscriber);
    }

    free(sctx);
    sub->handle = NULL;
}

static const transport_ops_t renoir_ops = {
    .init      = renoir_tp_init,
    .shutdown  = renoir_tp_shutdown,
    .advertise = renoir_tp_advertise,
    .subscribe = renoir_tp_subscribe,
    .publish   = renoir_tp_publish,
    .read_next = renoir_tp_read_next,
    .close_pub = renoir_tp_close_pub,
    .close_sub = renoir_tp_close_sub,
};

int transport_renoir_create(transport_t *tp) {
    if (!tp) return -1;
    tp->ops = &renoir_ops;
    tp->ctx = NULL;
    return 0;
}
