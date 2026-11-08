#include "urdf_import_config.h"

#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static const char *trim(const char *s) {
    while (*s && isspace((unsigned char)*s)) {
        s++;
    }
    return s;
}

static void trim_end(char *s) {
    size_t len = strlen(s);
    while (len > 0 && isspace((unsigned char)s[len - 1])) {
        s[--len] = '\0';
    }
}

static int parse_double(const char *text, double *out) {
    char *end = NULL;
    *out = strtod(text, &end);
    if (end == text) {
        return -1;
    }
    while (*end && isspace((unsigned char)*end)) {
        end++;
    }
    return *end == '\0' ? 0 : -1;
}

static int parse_range(const char *text, double out[2]) {
    const char *cursor = text;
    for (int i = 0; i < 2; i++) {
        char *end = NULL;
        out[i] = strtod(cursor, &end);
        if (end == cursor) {
            return -1;
        }
        cursor = end;
    }
    while (*cursor && isspace((unsigned char)*cursor)) {
        cursor++;
    }
    return *cursor == '\0' ? 0 : -1;
}

static int add_entry(actuator_config_t *cfg, const char *name, int is_motor, double kp, double kv,
                     int has_torque, double torque, int has_ctrlrange, const double ctrlrange[2],
                     int is_spin, char *err, size_t err_sz) {
    if (cfg->count >= CONFIG_MAX_ACTUATORS) {
        snprintf(err, err_sz, "too many joint sections (max %d)", CONFIG_MAX_ACTUATORS);
        return -1;
    }
    for (int i = 0; i < cfg->count; i++) {
        if (strcmp(cfg->entries[i].joint_name, name) == 0 && cfg->entries[i].is_spin == is_spin) {
            snprintf(err, err_sz, "duplicate section for joint '%s'", name);
            return -1;
        }
    }
    config_actuator_t *entry = &cfg->entries[cfg->count++];
    snprintf(entry->joint_name, sizeof(entry->joint_name), "%.*s",
             (int)(sizeof(entry->joint_name) - 1), name);
    entry->is_motor = is_motor;
    entry->kp = kp;
    entry->kv = kv;
    entry->has_torque = has_torque;
    entry->torque = torque;
    entry->has_ctrlrange = has_ctrlrange;
    entry->ctrlrange[0] = ctrlrange[0];
    entry->ctrlrange[1] = ctrlrange[1];
    entry->is_spin = is_spin;
    entry->matched = 0;
    return 0;
}

static int apply_key_value(const char *key, const char *value, int line_no, int *have_type,
                           int *is_motor, double *kp, double *kv, int *has_torque, double *torque,
                           int *has_ctrlrange, double ctrlrange[2], char *err, size_t err_sz) {
    if (strcmp(key, "type") == 0) {
        if (strcmp(value, "motor") == 0 || strcmp(value, "velocity") == 0) {
            *is_motor = 1;
        } else if (strcmp(value, "position") == 0) {
            *is_motor = 0;
        } else {
            snprintf(err, err_sz,
                     "line %d: unknown type '%s' "
                     "(expected 'motor' or 'position')",
                     line_no, value);
            return -1;
        }
        *have_type = 1;
        return 0;
    }
    if (strcmp(key, "kp") == 0) {
        if (parse_double(value, kp) != 0) {
            snprintf(err, err_sz, "line %d: invalid kp '%s'", line_no, value);
            return -1;
        }
        return 0;
    }
    if (strcmp(key, "kv") == 0) {
        if (parse_double(value, kv) != 0) {
            snprintf(err, err_sz, "line %d: invalid kv '%s'", line_no, value);
            return -1;
        }
        return 0;
    }
    if (strcmp(key, "torque") == 0) {
        if (parse_double(value, torque) != 0) {
            snprintf(err, err_sz, "line %d: invalid torque '%s'", line_no, value);
            return -1;
        }
        *has_torque = 1;
        return 0;
    }
    if (strcmp(key, "ctrlrange") == 0) {
        if (parse_range(value, ctrlrange) != 0) {
            snprintf(err, err_sz, "line %d: invalid ctrlrange '%s'", line_no, value);
            return -1;
        }
        *has_ctrlrange = 1;
        return 0;
    }
    snprintf(err, err_sz, "line %d: unknown key '%s'", line_no, key);
    return -1;
}

typedef struct {
    actuator_config_t *cfg;
    char *err;
    size_t err_sz;
    int in_section;
    int in_channels;
    int is_spin;
    int have_type;
    int is_motor;
    int has_torque;
    int has_ctrlrange;
    double kp;
    double kv;
    double torque;
    double ctrlrange[2];
    char name[CONFIG_NAME_LEN];
} parser_t;

static void reset_section(parser_t *prs) {
    prs->have_type = 0;
    prs->is_motor = 0;
    prs->kp = 40.0;
    prs->kv = 0.0;
    prs->has_torque = 0;
    prs->torque = 0.0;
    prs->has_ctrlrange = 0;
}

static int flush_section(parser_t *prs, int line_no) {
    if (!prs->in_section || prs->in_channels) {
        return 0;
    }
    if (!prs->have_type) {
        if (line_no > 0) {
            snprintf(prs->err, prs->err_sz, "line %d: section for joint '%s' has no type", line_no,
                     prs->name);
        } else {
            snprintf(prs->err, prs->err_sz, "section for joint '%s' has no type", prs->name);
        }
        return -1;
    }
    return add_entry(prs->cfg, prs->name, prs->is_motor, prs->kp, prs->kv, prs->has_torque,
                     prs->torque, prs->has_ctrlrange, prs->ctrlrange, prs->is_spin, prs->err,
                     prs->err_sz);
}

static int begin_section(parser_t *prs, char *text, int line_no) {
    if (flush_section(prs, line_no) != 0) {
        return -1;
    }
    char *close = strchr(text, ']');
    if (!close) {
        snprintf(prs->err, prs->err_sz, "line %d: malformed section header", line_no);
        return -1;
    }
    *close = '\0';
    const char *name = trim(text + 1);
    if (strcmp(name, "channels") == 0) {
        prs->in_channels = 1;
        prs->is_spin = 0;
        prs->in_section = 1;
        return 0;
    }
    if (strncmp(name, "joint ", 6) != 0 && strncmp(name, "spin ", 5) != 0) {
        snprintf(prs->err, prs->err_sz,
                 "line %d: expected '[joint <name>]', '[spin <name>]' or '[channels]'", line_no);
        return -1;
    }
    prs->is_spin = strncmp(name, "spin ", 5) == 0;
    snprintf(prs->name, sizeof(prs->name), "%.*s", (int)(sizeof(prs->name) - 1),
             trim(name + (prs->is_spin ? 5 : 6)));
    prs->in_channels = 0;
    prs->in_section = 1;
    reset_section(prs);
    return 0;
}

static void parse_channel_order(actuator_config_t *cfg, const char *value) {
    const char *cursor = value;
    while (*cursor != '\0' && cfg->channel_order_count < CONFIG_MAX_ACTUATORS) {
        const char *comma = strchr(cursor, ',');
        const char *end = comma ? comma : cursor + strlen(cursor);
        const char *start = cursor;
        while (start < end && isspace((unsigned char)*start)) {
            start++;
        }
        const char *stop = end;
        while (stop > start && isspace((unsigned char)stop[-1])) {
            stop--;
        }
        if (stop > start) {
            char *slot = cfg->channel_order[cfg->channel_order_count];
            size_t room = sizeof(cfg->channel_order[0]) - 1;
            size_t len = (size_t)(stop - start) < room ? (size_t)(stop - start) : room;
            memcpy(slot, start, len);
            slot[len] = '\0';
            cfg->channel_order_count++;
        }
        cursor = comma ? comma + 1 : end;
    }
}

static int handle_pair(parser_t *prs, char *text, int line_no) {
    if (!prs->in_section) {
        snprintf(prs->err, prs->err_sz, "line %d: key/value outside of a '[joint <name>]' section",
                 line_no);
        return -1;
    }
    char *eq = strchr(text, '=');
    if (!eq) {
        snprintf(prs->err, prs->err_sz, "line %d: expected 'key = value'", line_no);
        return -1;
    }
    *eq = '\0';
    char key[64];
    snprintf(key, sizeof(key), "%.*s", (int)(sizeof(key) - 1), trim(text));
    char value[256];
    snprintf(value, sizeof(value), "%.*s", (int)(sizeof(value) - 1), trim(eq + 1));
    trim_end(key);
    trim_end(value);
    if (!prs->in_channels) {
        return apply_key_value(key, value, line_no, &prs->have_type, &prs->is_motor, &prs->kp,
                               &prs->kv, &prs->has_torque, &prs->torque, &prs->has_ctrlrange,
                               prs->ctrlrange, prs->err, prs->err_sz);
    }
    if (strcmp(key, "order") != 0) {
        snprintf(prs->err, prs->err_sz, "line %d: unknown key '%s' in [channels]", line_no, key);
        return -1;
    }
    parse_channel_order(prs->cfg, value);
    return 0;
}

static int parse_line(parser_t *prs, char *line, int line_no) {
    char *text = line;
    while (*text && isspace((unsigned char)*text)) {
        text++;
    }
    char *eol = strchr(text, '\n');
    if (eol) {
        *eol = '\0';
    }
    if (*text == '\0' || *text == '#') {
        return 0;
    }
    return *text == '[' ? begin_section(prs, text, line_no) : handle_pair(prs, text, line_no);
}

int config_parse(const char *path, actuator_config_t *cfg, char *err, size_t err_sz) {
    memset(cfg, 0, sizeof(*cfg));
    FILE *file = fopen(path, "r");
    if (!file) {
        snprintf(err, err_sz, "cannot open config file '%s'", path);
        return -1;
    }
    parser_t prs;
    memset(&prs, 0, sizeof(prs));
    prs.cfg = cfg;
    prs.err = err;
    prs.err_sz = err_sz;
    reset_section(&prs);
    char line[1024];
    int line_no = 0;
    int status = 0;
    while (status == 0 && fgets(line, sizeof(line), file) != NULL) {
        status = parse_line(&prs, line, ++line_no);
    }
    fclose(file);
    return status != 0 ? -1 : flush_section(&prs, 0);
}

const config_actuator_t *config_find(const actuator_config_t *cfg, const char *joint) {
    for (int i = 0; i < cfg->count; i++) {
        if (!cfg->entries[i].is_spin && strcmp(cfg->entries[i].joint_name, joint) == 0) {
            return &cfg->entries[i];
        }
    }
    return NULL;
}

const config_actuator_t *config_find_spin(const actuator_config_t *cfg, const char *joint) {
    for (int i = 0; i < cfg->count; i++) {
        if (cfg->entries[i].is_spin && strcmp(cfg->entries[i].joint_name, joint) == 0) {
            return &cfg->entries[i];
        }
    }
    return NULL;
}
