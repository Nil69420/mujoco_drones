#include "foxglove/ops.h"
#include "test_support.h"

#include <string.h>

static int parse(const char *s, fg_op_t *op) {
    return fg_parse_client_op(s, strlen(s), op);
}

static void test_valid(void) {
    fg_op_t op;
    CHECK(parse("{\"op\":\"subscribe\",\"subscriptions\":[{\"id\":7,\"channelId\":3},"
                "{\"channelId\":4,\"id\":8}]}", &op) == 0);
    CHECK(op.kind == FG_OP_SUBSCRIBE && op.n_subs == 2);
    CHECK(op.subs[0].id == 7 && op.subs[0].channel_id == 3);
    CHECK(op.subs[1].id == 8 && op.subs[1].channel_id == 4);

    CHECK(parse(" { \"subscriptions\" : [ { \"id\" : 1 , \"channelId\" : 2 } ] , \"op\" : \"subscribe\" } ", &op) == 0);
    CHECK(op.kind == FG_OP_SUBSCRIBE && op.n_subs == 1);

    CHECK(parse("{\"op\":\"unsubscribe\",\"subscriptionIds\":[1,2,3]}", &op) == 0);
    CHECK(op.kind == FG_OP_UNSUBSCRIBE && op.n_unsub == 3 && op.unsub_ids[2] == 3);

    CHECK(parse("{\"op\":\"somethingElse\",\"x\":[1,2,{\"y\":null}],\"z\":true}", &op) == 0);
    CHECK(op.kind == FG_OP_UNKNOWN && op.n_subs == 0);
    CHECK(parse("{}", &op) == 0 && op.kind == FG_OP_UNKNOWN);
    CHECK(parse("{\"op\":\"sub\\u0073cribe\"}", &op) == 0);

    CHECK(parse("{\"op\":\"subscribe\",\"note\":\"subscribe\",\"subscriptions\":[]}", &op) == 0);
    CHECK(op.kind == FG_OP_SUBSCRIBE && op.n_subs == 0);
    CHECK(parse("{\"note\":\"op subscribe\",\"op\":\"x\"}", &op) == 0 && op.kind == FG_OP_UNKNOWN);

    CHECK(parse("{\"op\":\"subscribe\",\"subscriptions\":[{\"id\":-1,\"channelId\":1},"
                "{\"id\":1.5,\"channelId\":1},{\"id\":4294967296,\"channelId\":1},"
                "{\"id\":4294967295,\"channelId\":1}]}", &op) == 0);
    CHECK(op.n_subs == 1 && op.subs[0].id == 4294967295U);
}

static void test_invalid(void) {
    fg_op_t op;
    const char *bad[] = {
        "", "{", "}", "[]", "null", "123", "\"x\"", "{\"op\"}", "{\"op\":}", "{\"op\":\"a\",}",
        "{\"op\":\"a\"} x", "{\"op\":\"a\"}{}", "{'op':'a'}", "{\"op\":\"a\\q\"}",
        "{\"op\":\"a\\u12G4\"}", "{\"op\":\"a\nb\"}", "{\"op\":01}", "{\"op\":1.}",
        "{\"op\":-}", "{\"op\":+1}", "{\"op\":.5}", "{\"op\":1e}", "{\"op\":tru}",
        "{\"op\":\"subscribe\",\"subscriptions\":[{\"id\":1,\"channelId\":2}}",
        "{\"op\":\"subscribe\",\"subscriptions\":[{\"id\":1,\"channelId\":2},]}",
        "{\"op\":\"unsubscribe\",\"subscriptionIds\":[1,]}",
        "{\"op\":\"unsubscribe\",\"subscriptionIds\":[1 2]}",
    };
    for (size_t i = 0; i < sizeof(bad) / sizeof(bad[0]); i++) {
        if (parse(bad[i], &op) == 0) {
            fprintf(stderr, "FAIL accepted invalid json: %s\n", bad[i]);
            g_failures++;
        }
    }
    char deep[200];
    memset(deep, '[', 100);
    memset(deep + 100, ']', 100);
    char wrapped[400];
    int n = snprintf(wrapped, sizeof(wrapped), "{\"a\":%.*s}", 200, deep);
    CHECK(n > 0 && fg_parse_client_op(wrapped, (size_t)n, &op) != 0);
    char shallow[64];
    n = snprintf(shallow, sizeof(shallow), "{\"a\":[[[[1]]]]}");
    CHECK(fg_parse_client_op(shallow, (size_t)n, &op) == 0);
    CHECK(fg_parse_client_op("{}", FG_OPS_MAX_MESSAGE + 1, &op) != 0);
}

static void test_limits(void) {
    fg_op_t op;
    char big[4096];
    size_t at = (size_t)snprintf(big, sizeof(big), "{\"op\":\"subscribe\",\"subscriptions\":[");
    for (int i = 0; i < 60; i++) {
        at += (size_t)snprintf(big + at, sizeof(big) - at, "%s{\"id\":%d,\"channelId\":1}", i ? "," : "", i);
    }
    at += (size_t)snprintf(big + at, sizeof(big) - at, "]}");
    CHECK(fg_parse_client_op(big, at, &op) == 0);
    CHECK(op.n_subs == FG_OPS_MAX_ENTRIES && op.overflow == 1);
}

int main(void) {
    test_valid();
    test_invalid();
    test_limits();
    if (g_failures != 0) {
        fprintf(stderr, "%d check(s) failed\n", g_failures);
        return 1;
    }
    puts("test_fg_ops OK");
    return 0;
}
