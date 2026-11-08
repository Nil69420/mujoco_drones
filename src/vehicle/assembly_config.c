#include "scenario.h"
#include "vehicle/assembly_config.h"

#include <ctype.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

typedef enum {
    SECTION_NONE,
    SECTION_SCENARIO,
    SECTION_VEHICLE,
} section_t;

typedef struct {
    const char *path;
    int line;
} parse_ctx_t;

static int valid_id(const char *id) {
    if (!id || id[0] == '\0') return 0;
    for (const char *ch = id; *ch; ch++) {
        if (!isalnum((unsigned char)*ch) && *ch != '_') return 0;
    }
    return 1;
}

static char *trim(char *s) {
    while (*s && isspace((unsigned char)*s)) s++;
    char *end = s + strlen(s);
    while (end > s && isspace((unsigned char)end[-1])) end--;
    *end = '\0';
    return s;
}

typedef struct {
    int have_env;
    int have_seed;
    int have_duration;
    int have_scenario;
    section_t section;
    vehicle_assembly_spec_t *cur_vehicle;
} parse_state_t;

static int parse_triple(const parse_ctx_t *ctx, const char *key, const char *value,
                        double out[3]) {
    const char *cursor = value;
    char *end = NULL;
    for (int axis = 0; axis < 3; axis++) {
        out[axis] = strtod(cursor, &end);
        if (end == cursor) {
            fprintf(stderr, "%s:%d: %s needs 3 numbers\n", ctx->path, ctx->line, key);
            return -1;
        }
        cursor = end;
    }
    if (*trim((char *)cursor) != '\0') {
        fprintf(stderr, "%s:%d: %s needs 3 numbers\n", ctx->path, ctx->line, key);
        return -1;
    }
    return 0;
}

static int handle_scenario_key(const parse_ctx_t *ctx, parse_state_t *state, scenario_t *out,
                               const char *key, const char *value) {
    if (strcmp(key, "environment") == 0) {
        snprintf(out->environment, sizeof(out->environment), "%s", value);
        state->have_env = 1;
    } else if (strcmp(key, "environment_seed") == 0) {
        out->environment_seed = (unsigned)strtoul(value, NULL, 10);
        state->have_seed = 1;
    } else if (strcmp(key, "weather") == 0) {
        snprintf(out->weather, sizeof(out->weather), "%s", value);
    } else if (strcmp(key, "duration") == 0) {
        out->duration_s = strtod(value, NULL);
        state->have_duration = 1;
    } else {
        fprintf(stderr, "%s:%d: warning: unknown key '%s'\n", ctx->path, ctx->line, key);
    }
    return 0;
}

typedef struct {
    const char *key;
    size_t offset;
} string_field_t;

typedef struct {
    const char *key;
    size_t offset;
} number_field_t;

static const string_field_t STRING_FIELDS[] = {
    {"airframe", offsetof(vehicle_assembly_spec_t, airframe)},
    {"controller", offsetof(vehicle_assembly_spec_t, controller)},
    {"estimator", offsetof(vehicle_assembly_spec_t, estimator)},
    {"localization", offsetof(vehicle_assembly_spec_t, localization)},
    {"imu", offsetof(vehicle_assembly_spec_t, imu)},
    {"gnss", offsetof(vehicle_assembly_spec_t, gnss)},
    {"baro", offsetof(vehicle_assembly_spec_t, baro)},
    {"mag", offsetof(vehicle_assembly_spec_t, mag)},
    {"rangefinder", offsetof(vehicle_assembly_spec_t, rangefinder)},
};

static const number_field_t NUMBER_FIELDS[] = {
    {"yaw", offsetof(vehicle_assembly_spec_t, spawn_yaw)},
    {"speed", offsetof(vehicle_assembly_spec_t, spawn_speed)},
    {"vibration", offsetof(vehicle_assembly_spec_t, vibration)},
    {"drag_area", offsetof(vehicle_assembly_spec_t, drag_area)},
};

static int handle_vehicle_flag(const parse_ctx_t *ctx, vehicle_assembly_spec_t *veh, const char *key,
                               const char *value, int *handled) {
    *handled = 1;
    if (strcmp(key, "gnss_rtk") == 0) {
        veh->gnss_rtk = strcmp(value, "true") == 0 || strcmp(value, "1") == 0;
    } else if (strcmp(key, "gnss_start") == 0) {
        if (strcmp(value, "cold") != 0 && strcmp(value, "warm") != 0) {
            fprintf(stderr, "%s:%d: gnss_start must be warm or cold\n", ctx->path, ctx->line);
            return -1;
        }
        veh->gnss_cold_start = strcmp(value, "cold") == 0;
    } else if (strcmp(key, "spawn") == 0) {
        return parse_triple(ctx, key, value, veh->spawn_pos);
    } else if (strcmp(key, "target") == 0) {
        veh->has_target = 1;
        return parse_triple(ctx, key, value, veh->target_pos);
    } else {
        *handled = 0;
    }
    return 0;
}

static int handle_vehicle_key(const parse_ctx_t *ctx, vehicle_assembly_spec_t *veh, const char *key,
                              const char *value) {
    int handled = 0;
    int status = handle_vehicle_flag(ctx, veh, key, value, &handled);
    if (handled) {
        return status;
    }
    for (size_t idx = 0; idx < sizeof(STRING_FIELDS) / sizeof(STRING_FIELDS[0]); idx++) {
        if (strcmp(key, STRING_FIELDS[idx].key) == 0) {
            snprintf((char *)veh + STRING_FIELDS[idx].offset, ASSEMBLY_MAX_NAME_LEN, "%s", value);
            return 0;
        }
    }
    for (size_t idx = 0; idx < sizeof(NUMBER_FIELDS) / sizeof(NUMBER_FIELDS[0]); idx++) {
        if (strcmp(key, NUMBER_FIELDS[idx].key) == 0) {
            double *dest = (double *)(void *)((char *)veh + NUMBER_FIELDS[idx].offset);
            *dest = strtod(value, NULL);
            return 0;
        }
    }
    fprintf(stderr, "%s:%d: warning: unknown key '%s'\n", ctx->path, ctx->line, key);
    return 0;
}

static int handle_key_value(const parse_ctx_t *ctx, parse_state_t *state, scenario_t *out,
                            const char *key, const char *value) {
    if (state->section == SECTION_SCENARIO) {
        return handle_scenario_key(ctx, state, out, key, value);
    }
    if (state->section == SECTION_VEHICLE && state->cur_vehicle) {
        return handle_vehicle_key(ctx, state->cur_vehicle, key, value);
    }
    fprintf(stderr, "%s:%d: key '%s' outside any section\n", ctx->path, ctx->line, key);
    return -1;
}

static int handle_section_header(const parse_ctx_t *ctx, parse_state_t *state,
                                 scenario_t *out, const char *header) {
    if (strcmp(header, "scenario") == 0) {
        if (state->have_scenario) {
            fprintf(stderr, "%s:%d: duplicate [scenario] section\n",
                    ctx->path, ctx->line);
            return -1;
        }
        state->have_scenario = 1;
        state->section = SECTION_SCENARIO;
        return 0;
    }

    if (strncmp(header, "vehicle ", 8) == 0) {
        const char *id = trim((char *)header + 8);
        if (!valid_id(id)) {
            fprintf(stderr,
                    "%s:%d: invalid vehicle id '%s' (alphanumeric "
                    "and underscore only)\n",
                    ctx->path, ctx->line, id);
            return -1;
        }
        if (out->vehicle_count >= SCENARIO_MAX_VEHICLES) {
            fprintf(stderr, "%s:%d: too many vehicles (max %d)\n",
                    ctx->path, ctx->line, SCENARIO_MAX_VEHICLES);
            return -1;
        }
        state->cur_vehicle = &out->vehicles[out->vehicle_count++];
        snprintf(state->cur_vehicle->id,
                 sizeof(state->cur_vehicle->id), "%s", id);
        state->section = SECTION_VEHICLE;
        return 0;
    }

    fprintf(stderr, "%s:%d: unknown section [%s]\n",
            ctx->path, ctx->line, header);
    return -1;
}

int scenario_load(const char *path, scenario_t *out) {
    if (!path || !out) return -1;
    memset(out, 0, sizeof(*out));
    out->duration_s = 0.0;

    FILE *f = fopen(path, "r");
    if (!f) {
        fprintf(stderr, "scenario: cannot open '%s'\n", path);
        return -1;
    }

    parse_ctx_t ctx = { .path = path, .line = 0 };
    parse_state_t state;
    memset(&state, 0, sizeof(state));

    char line[512];
    while (fgets(line, sizeof(line), f)) {
        ctx.line++;

        char *hash = strchr(line, '#');
        if (hash) *hash = '\0';
        char *t = trim(line);
        if (*t == '\0') continue;

        int rc = 0;
        if (*t == '[') {
            char *close = strchr(t, ']');
            if (!close) {
                fprintf(stderr, "%s:%d: malformed section header\n",
                        ctx.path, ctx.line);
                fclose(f);
                return -1;
            }
            *close = '\0';
            rc = handle_section_header(&ctx, &state, out, trim(t + 1));
        } else {
            char *eq = strchr(t, '=');
            if (!eq) {
                fprintf(stderr, "%s:%d: expected key = value\n",
                        ctx.path, ctx.line);
                fclose(f);
                return -1;
            }
            *eq = '\0';
            rc = handle_key_value(&ctx, &state, out, trim(t),
                                  trim(eq + 1));
        }
        if (rc != 0) {
            fclose(f);
            return -1;
        }
    }
    fclose(f);

    if (!state.have_scenario) {
        fprintf(stderr, "scenario: missing required [scenario] section\n");
        return -1;
    }
    if (!state.have_env || !state.have_seed || !state.have_duration) {
        fprintf(stderr,
                "scenario: [scenario] requires environment, "
                "environment_seed and duration\n");
        return -1;
    }
    return 0;
}
