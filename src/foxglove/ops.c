#include "foxglove/ops.h"

#include <string.h>

typedef struct {
    const char *p;
    const char *end;
} cursor_t;

static void skip_ws(cursor_t *c) {
    while (c->p < c->end && (*c->p == ' ' || *c->p == '\t' || *c->p == '\n' || *c->p == '\r')) {
        c->p++;
    }
}

static int expect(cursor_t *c, char ch) {
    skip_ws(c);
    if (c->p < c->end && *c->p == ch) {
        c->p++;
        return 0;
    }
    return -1;
}

static int hex4(const char *s) {
    for (int i = 0; i < 4; i++) {
        char ch = s[i];
        int ok = (ch >= '0' && ch <= '9') || (ch >= 'a' && ch <= 'f') || (ch >= 'A' && ch <= 'F');
        if (!ok) {
            return -1;
        }
    }
    return 0;
}

static int parse_string(cursor_t *c, char *out, size_t cap, size_t *out_len) {
    skip_ws(c);
    if (c->p >= c->end || *c->p != '"') {
        return -1;
    }
    c->p++;
    size_t n = 0;
    while (c->p < c->end) {
        unsigned char ch = (unsigned char)*c->p++;
        if (ch == '"') {
            if (out != NULL && cap > 0) {
                out[n < cap ? n : cap - 1] = '\0';
            }
            if (out_len != NULL) {
                *out_len = n;
            }
            return 0;
        }
        if (ch < 0x20) {
            return -1;
        }
        char decoded = (char)ch;
        if (ch == '\\') {
            if (c->p >= c->end) {
                return -1;
            }
            char esc = *c->p++;
            switch (esc) {
            case '"':
            case '\\':
            case '/':
                decoded = esc;
                break;
            case 'b':
            case 'f':
            case 'n':
            case 'r':
            case 't':
                decoded = '?';
                break;
            case 'u':
                if (c->end - c->p < 4 || hex4(c->p) != 0) {
                    return -1;
                }
                c->p += 4;
                decoded = '?';
                break;
            default:
                return -1;
            }
        }
        if (out != NULL && n + 1 < cap) {
            out[n] = decoded;
        }
        n++;
    }
    return -1;
}

static int parse_number(cursor_t *c, int *is_uint, uint32_t *value) {
    skip_ws(c);
    const char *start = c->p;
    if (c->p < c->end && *c->p == '-') {
        c->p++;
    }
    if (c->p >= c->end) {
        return -1;
    }
    if (*c->p == '0') {
        c->p++;
    } else if (*c->p >= '1' && *c->p <= '9') {
        while (c->p < c->end && *c->p >= '0' && *c->p <= '9') {
            c->p++;
        }
    } else {
        return -1;
    }
    int integral = 1;
    if (c->p < c->end && *c->p == '.') {
        integral = 0;
        c->p++;
        if (c->p >= c->end || *c->p < '0' || *c->p > '9') {
            return -1;
        }
        while (c->p < c->end && *c->p >= '0' && *c->p <= '9') {
            c->p++;
        }
    }
    if (c->p < c->end && (*c->p == 'e' || *c->p == 'E')) {
        integral = 0;
        c->p++;
        if (c->p < c->end && (*c->p == '+' || *c->p == '-')) {
            c->p++;
        }
        if (c->p >= c->end || *c->p < '0' || *c->p > '9') {
            return -1;
        }
        while (c->p < c->end && *c->p >= '0' && *c->p <= '9') {
            c->p++;
        }
    }
    *is_uint = 0;
    *value = 0;
    if (integral && *start != '-') {
        uint64_t v = 0;
        int ok = 1;
        for (const char *q = start; q < c->p; q++) {
            v = v * 10U + (uint64_t)(*q - '0');
            if (v > 0xFFFFFFFFULL) {
                ok = 0;
                break;
            }
        }
        if (ok) {
            *is_uint = 1;
            *value = (uint32_t)v;
        }
    }
    return 0;
}

static int skip_value(cursor_t *c, int depth);

static int skip_literal(cursor_t *c, const char *word) {
    size_t n = strlen(word);
    if ((size_t)(c->end - c->p) < n || memcmp(c->p, word, n) != 0) {
        return -1;
    }
    c->p += n;
    return 0;
}

static int skip_container(cursor_t *c, int depth, char close, int is_object) {
    if (depth >= FG_OPS_MAX_DEPTH) {
        return -1;
    }
    skip_ws(c);
    if (c->p < c->end && *c->p == close) {
        c->p++;
        return 0;
    }
    for (;;) {
        if (is_object) {
            if (parse_string(c, NULL, 0, NULL) != 0 || expect(c, ':') != 0) {
                return -1;
            }
        }
        if (skip_value(c, depth + 1) != 0) {
            return -1;
        }
        skip_ws(c);
        if (c->p >= c->end) {
            return -1;
        }
        if (*c->p == ',') {
            c->p++;
            continue;
        }
        if (*c->p == close) {
            c->p++;
            return 0;
        }
        return -1;
    }
}

static int skip_value(cursor_t *c, int depth) {
    skip_ws(c);
    if (c->p >= c->end) {
        return -1;
    }
    switch (*c->p) {
    case '{':
        c->p++;
        return skip_container(c, depth, '}', 1);
    case '[':
        c->p++;
        return skip_container(c, depth, ']', 0);
    case '"':
        return parse_string(c, NULL, 0, NULL);
    case 't':
        return skip_literal(c, "true");
    case 'f':
        return skip_literal(c, "false");
    case 'n':
        return skip_literal(c, "null");
    default: {
        int is_uint = 0;
        uint32_t v = 0;
        return parse_number(c, &is_uint, &v);
    }
    }
}

static int parse_subscription_object(cursor_t *c, fg_op_t *out) {
    if (expect(c, '{') != 0) {
        return -1;
    }
    fg_op_sub_t sub = {0, 0};
    int have_id = 0;
    int have_ch = 0;
    skip_ws(c);
    if (c->p < c->end && *c->p == '}') {
        c->p++;
        return 0;
    }
    for (;;) {
        char key[32];
        size_t klen = 0;
        if (parse_string(c, key, sizeof(key), &klen) != 0 || expect(c, ':') != 0) {
            return -1;
        }
        skip_ws(c);
        if (c->p < c->end && ((*c->p >= '0' && *c->p <= '9') || *c->p == '-') &&
            (strcmp(key, "id") == 0 || strcmp(key, "channelId") == 0)) {
            int is_uint = 0;
            uint32_t v = 0;
            if (parse_number(c, &is_uint, &v) != 0) {
                return -1;
            }
            if (is_uint && strcmp(key, "id") == 0) {
                sub.id = v;
                have_id = 1;
            } else if (is_uint) {
                sub.channel_id = v;
                have_ch = 1;
            }
        } else if (skip_value(c, 2) != 0) {
            return -1;
        }
        skip_ws(c);
        if (c->p >= c->end) {
            return -1;
        }
        if (*c->p == ',') {
            c->p++;
            continue;
        }
        if (*c->p == '}') {
            c->p++;
            break;
        }
        return -1;
    }
    if (have_id && have_ch) {
        if (out->n_subs < FG_OPS_MAX_ENTRIES) {
            out->subs[out->n_subs++] = sub;
        } else {
            out->overflow = 1;
        }
    }
    return 0;
}

static int parse_subscriptions(cursor_t *c, fg_op_t *out) {
    if (expect(c, '[') != 0) {
        return -1;
    }
    skip_ws(c);
    if (c->p < c->end && *c->p == ']') {
        c->p++;
        return 0;
    }
    for (;;) {
        if (parse_subscription_object(c, out) != 0) {
            return -1;
        }
        skip_ws(c);
        if (c->p >= c->end) {
            return -1;
        }
        if (*c->p == ',') {
            c->p++;
            continue;
        }
        if (*c->p == ']') {
            c->p++;
            return 0;
        }
        return -1;
    }
}

static int parse_id_list(cursor_t *c, fg_op_t *out) {
    if (expect(c, '[') != 0) {
        return -1;
    }
    skip_ws(c);
    if (c->p < c->end && *c->p == ']') {
        c->p++;
        return 0;
    }
    for (;;) {
        int is_uint = 0;
        uint32_t v = 0;
        if (parse_number(c, &is_uint, &v) != 0) {
            return -1;
        }
        if (is_uint) {
            if (out->n_unsub < FG_OPS_MAX_ENTRIES) {
                out->unsub_ids[out->n_unsub++] = v;
            } else {
                out->overflow = 1;
            }
        }
        skip_ws(c);
        if (c->p >= c->end) {
            return -1;
        }
        if (*c->p == ',') {
            c->p++;
            continue;
        }
        if (*c->p == ']') {
            c->p++;
            return 0;
        }
        return -1;
    }
}

int fg_parse_client_op(const char *msg, size_t len, fg_op_t *out) {
    memset(out, 0, sizeof(*out));
    if (len == 0 || len > FG_OPS_MAX_MESSAGE) {
        return -1;
    }
    cursor_t c = {msg, msg + len};
    if (expect(&c, '{') != 0) {
        return -1;
    }
    char op[32] = "";
    skip_ws(&c);
    if (c.p < c.end && *c.p == '}') {
        c.p++;
    } else {
        for (;;) {
            char key[32];
            size_t klen = 0;
            if (parse_string(&c, key, sizeof(key), &klen) != 0 || expect(&c, ':') != 0) {
                return -1;
            }
            skip_ws(&c);
            int rc;
            if (klen == 2 && strcmp(key, "op") == 0 && c.p < c.end && *c.p == '"') {
                rc = parse_string(&c, op, sizeof(op), NULL);
            } else if (klen == 13 && strcmp(key, "subscriptions") == 0 && c.p < c.end &&
                       *c.p == '[') {
                rc = parse_subscriptions(&c, out);
            } else if (klen == 15 && strcmp(key, "subscriptionIds") == 0 && c.p < c.end &&
                       *c.p == '[') {
                rc = parse_id_list(&c, out);
            } else {
                rc = skip_value(&c, 1);
            }
            if (rc != 0) {
                return -1;
            }
            skip_ws(&c);
            if (c.p >= c.end) {
                return -1;
            }
            if (*c.p == ',') {
                c.p++;
                continue;
            }
            if (*c.p == '}') {
                c.p++;
                break;
            }
            return -1;
        }
    }
    skip_ws(&c);
    if (c.p != c.end) {
        return -1;
    }
    if (strcmp(op, "subscribe") == 0) {
        out->kind = FG_OP_SUBSCRIBE;
    } else if (strcmp(op, "unsubscribe") == 0) {
        out->kind = FG_OP_UNSUBSCRIBE;
    } else {
        out->kind = FG_OP_UNKNOWN;
    }
    if (out->kind != FG_OP_SUBSCRIBE) {
        out->n_subs = 0;
    }
    if (out->kind != FG_OP_UNSUBSCRIBE) {
        out->n_unsub = 0;
    }
    return 0;
}
