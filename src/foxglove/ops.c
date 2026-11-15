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
        uint64_t acc = 0;
        int ok = 1;
        for (const char *digit = start; digit < c->p; digit++) {
            acc = acc * 10U + (uint64_t)(*digit - '0');
            if (acc > 0xFFFFFFFFULL) {
                ok = 0;
                break;
            }
        }
        if (ok) {
            *is_uint = 1;
            *value = (uint32_t)acc;
        }
    }
    return 0;
}

static int skip_literal(cursor_t *c, const char *word) {
    size_t n = strlen(word);
    if ((size_t)(c->end - c->p) < n || memcmp(c->p, word, n) != 0) {
        return -1;
    }
    c->p += n;
    return 0;
}

static int skip_scalar(cursor_t *cur) {
    switch (*cur->p) {
    case '"':
        return parse_string(cur, NULL, 0, NULL);
    case 't':
        return skip_literal(cur, "true");
    case 'f':
        return skip_literal(cur, "false");
    case 'n':
        return skip_literal(cur, "null");
    default: {
        int is_uint = 0;
        uint32_t value = 0;
        return parse_number(cur, &is_uint, &value);
    }
    }
}

typedef struct {
    char closers[FG_OPS_MAX_DEPTH];
    int size;
} nest_t;

static int enter_entry(cursor_t *cur, char closer) {
    if (closer == '}') {
        return (parse_string(cur, NULL, 0, NULL) != 0 || expect(cur, ':') != 0) ? -1 : 0;
    }
    return 0;
}

static int finish_value(cursor_t *cur, nest_t *nest) {
    while (nest->size > 0) {
        skip_ws(cur);
        if (cur->p >= cur->end) {
            return -1;
        }
        char closer = nest->closers[nest->size - 1];
        if (*cur->p == ',') {
            cur->p++;
            return enter_entry(cur, closer) == 0 ? 0 : -1;
        }
        if (*cur->p != closer) {
            return -1;
        }
        cur->p++;
        nest->size--;
    }
    return 1;
}

static int skip_value(cursor_t *cur, int base_depth) {
    nest_t nest;
    nest.size = 0;
    for (;;) {
        skip_ws(cur);
        if (cur->p >= cur->end) {
            return -1;
        }
        char ch = *cur->p;
        if (ch == '{' || ch == '[') {
            if (base_depth + nest.size >= FG_OPS_MAX_DEPTH) {
                return -1;
            }
            char closer = ch == '{' ? '}' : ']';
            cur->p++;
            skip_ws(cur);
            if (cur->p < cur->end && *cur->p == closer) {
                cur->p++;
            } else {
                nest.closers[nest.size++] = closer;
                if (enter_entry(cur, closer) != 0) {
                    return -1;
                }
                continue;
            }
        } else if (skip_scalar(cur) != 0) {
            return -1;
        }
        int state = finish_value(cur, &nest);
        if (state != 0) {
            return state > 0 ? 0 : -1;
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
            uint32_t value = 0;
            if (parse_number(c, &is_uint, &value) != 0) {
                return -1;
            }
            if (is_uint && strcmp(key, "id") == 0) {
                sub.id = value;
                have_id = 1;
            } else if (is_uint) {
                sub.channel_id = value;
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
        uint32_t value = 0;
        if (parse_number(c, &is_uint, &value) != 0) {
            return -1;
        }
        if (is_uint) {
            if (out->n_unsub < FG_OPS_MAX_ENTRIES) {
                out->unsub_ids[out->n_unsub++] = value;
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
    cursor_t cur = {msg, msg + len};
    if (expect(&cur, '{') != 0) {
        return -1;
    }
    char op[32] = "";
    skip_ws(&cur);
    if (cur.p < cur.end && *cur.p == '}') {
        cur.p++;
    } else {
        for (;;) {
            char key[32];
            size_t klen = 0;
            if (parse_string(&cur, key, sizeof(key), &klen) != 0 || expect(&cur, ':') != 0) {
                return -1;
            }
            skip_ws(&cur);
            int rc = 0;
            if (klen == 2 && strcmp(key, "op") == 0 && cur.p < cur.end && *cur.p == '"') {
                rc = parse_string(&cur, op, sizeof(op), NULL);
            } else if (klen == 13 && strcmp(key, "subscriptions") == 0 && cur.p < cur.end &&
                       *cur.p == '[') {
                rc = parse_subscriptions(&cur, out);
            } else if (klen == 15 && strcmp(key, "subscriptionIds") == 0 && cur.p < cur.end &&
                       *cur.p == '[') {
                rc = parse_id_list(&cur, out);
            } else {
                rc = skip_value(&cur, 1);
            }
            if (rc != 0) {
                return -1;
            }
            skip_ws(&cur);
            if (cur.p >= cur.end) {
                return -1;
            }
            if (*cur.p == ',') {
                cur.p++;
                continue;
            }
            if (*cur.p == '}') {
                cur.p++;
                break;
            }
            return -1;
        }
    }
    skip_ws(&cur);
    if (cur.p != cur.end) {
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
