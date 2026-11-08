#define _GNU_SOURCE
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
        if (strcmp(value, "motor") == 0) {
            *is_motor = 1;
        } else if (strcmp(value, "position") == 0) {
            *is_motor = 0;
        } else if (strcmp(value, "velocity") == 0) {
            *is_motor = 1;
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

int config_parse(const char *path, actuator_config_t *cfg, char *err, size_t err_sz) {
    memset(cfg, 0, sizeof(*cfg));
    FILE *f = fopen(path, "r");
    if (!f) {
        snprintf(err, err_sz, "cannot open config file '%s'", path);
        return -1;
    }

    char line[1024];
    int line_no = 0;
    int in_section = 0;
    char section_name[CONFIG_NAME_LEN] = "";
    int have_type = 0;
    int is_motor = 0;
    double kp = 40.0;
    double kv = 0.0;
    int has_torque = 0;
    double torque = 0.0;
    int has_ctrlrange = 0;
    int section_is_spin = 0;
    int in_channels_section = 0;
    double ctrlrange[2] = {0.0, 0.0};

    while (fgets(line, sizeof(line), f) != NULL) {
        line_no++;
        char *text = line;
        while (*text && isspace((unsigned char)*text)) {
            text++;
        }
        char *eol = strchr(text, '\n');
        if (eol) {
            *eol = '\0';
        }
        if (*text == '\0' || *text == '#') {
            continue;
        }
        if (*text == '[') {
            if (in_section) {
                if (!have_type) {
                    snprintf(err, err_sz, "line %d: section for joint '%s' has no type", line_no,
                             section_name);
                    fclose(f);
                    return -1;
                }
                if (add_entry(cfg, section_name, is_motor, kp, kv, has_torque, torque,
                              has_ctrlrange, ctrlrange, section_is_spin, err, err_sz) != 0) {
                    fclose(f);
                    return -1;
                }
            }
            char *close = strchr(text, ']');
            if (!close) {
                snprintf(err, err_sz, "line %d: malformed section header", line_no);
                fclose(f);
                return -1;
            }
            *close = '\0';
            const char *name = trim(text + 1);
            if (strcmp(name, "channels") == 0) {
                in_channels_section = 1;
                section_is_spin = 0;
                snprintf(section_name, sizeof(section_name), "%s", "channels");
                in_section = 1;
                have_type = 1;
                continue;
            }
            if (strncmp(name, "joint ", 6) != 0 && strncmp(name, "spin ", 5) != 0) {
                snprintf(err, err_sz,
                         "line %d: expected '[joint <name>]', '[spin <name>]' or "
                         "'[channels]'",
                         line_no);
                fclose(f);
                return -1;
            }
            section_is_spin = strncmp(name, "spin ", 5) == 0;
            snprintf(section_name, sizeof(section_name), "%.*s", (int)(sizeof(section_name) - 1),
                     trim(name + (section_is_spin ? 5 : 6)));
            in_channels_section = 0;
            in_section = 1;
            have_type = 0;
            is_motor = 0;
            kp = 40.0;
            kv = 0.0;
            has_torque = 0;
            torque = 0.0;
            has_ctrlrange = 0;
            continue;
        }
        if (!in_section) {
            snprintf(err, err_sz, "line %d: key/value outside of a '[joint <name>]' section",
                     line_no);
            fclose(f);
            return -1;
        }

        char *eq = strchr(text, '=');
        if (!eq) {
            snprintf(err, err_sz, "line %d: expected 'key = value'", line_no);
            fclose(f);
            return -1;
        }
        *eq = '\0';
        char key[64];
        snprintf(key, sizeof(key), "%.*s", (int)(sizeof(key) - 1), trim(text));
        char value[256];
        snprintf(value, sizeof(value), "%.*s", (int)(sizeof(value) - 1), trim(eq + 1));
        trim_end(key);
        trim_end(value);

        if (in_channels_section) {
            if (strcmp(key, "order") == 0) {
                char *copy = strdup(value);
                if (!copy) {
                    fclose(f);
                    return -1;
                }
                char *tok = strtok(copy, ",");
                while (tok && cfg->channel_order_count < CONFIG_MAX_ACTUATORS) {
                    char *t = tok;
                    while (*t && isspace((unsigned char)*t)) {
                        t++;
                    }
                    char *tend = t + strlen(t);
                    while (tend > t && isspace((unsigned char)tend[-1])) {
                        tend--;
                    }
                    if (tend > t) {
                        snprintf(cfg->channel_order[cfg->channel_order_count],
                                 sizeof(cfg->channel_order[cfg->channel_order_count]),
                                 "%.*s", (int)(sizeof(cfg->channel_order[0]) - 1), t);
                        cfg->channel_order_count++;
                    }
                    tok = strtok(NULL, ",");
                }
                free(copy);
            } else {
                snprintf(err, err_sz, "line %d: unknown key '%s' in [channels]", line_no,
                         key);
                fclose(f);
                return -1;
            }
        } else if (apply_key_value(key, value, line_no, &have_type, &is_motor, &kp, &kv,
                                   &has_torque, &torque, &has_ctrlrange, ctrlrange, err,
                                   err_sz) != 0) {
            fclose(f);
            return -1;
        }
    }

    if (in_section) {
        if (!have_type) {
            snprintf(err, err_sz, "section for joint '%s' has no type", section_name);
            fclose(f);
            return -1;
        }
        if (add_entry(cfg, section_name, is_motor, kp, kv, has_torque, torque, has_ctrlrange,
                      ctrlrange, section_is_spin, err, err_sz) != 0) {
            fclose(f);
            return -1;
        }
    }
    fclose(f);
    return 0;
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
