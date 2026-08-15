#include "scenario.h"
#include "vehicle/assembly_config.h"

#include <ctype.h>
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

static int handle_key_value(const parse_ctx_t *ctx, parse_state_t *state,
                            scenario_t *out, const char *key,
                            const char *value) {
    if (state->section == SECTION_SCENARIO) {
        if (strcmp(key, "environment") == 0) {
            snprintf(out->environment, sizeof(out->environment), "%s", value);
            state->have_env = 1;
        } else if (strcmp(key, "environment_seed") == 0) {
            out->environment_seed = (unsigned)strtoul(value, NULL, 10);
            state->have_seed = 1;
        } else if (strcmp(key, "duration") == 0) {
            out->duration_s = strtod(value, NULL);
            state->have_duration = 1;
        } else {
            fprintf(stderr, "%s:%d: warning: unknown key '%s'\n",
                    ctx->path, ctx->line, key);
        }
        return 0;
    }

    if (state->section == SECTION_VEHICLE && state->cur_vehicle) {
        vehicle_assembly_spec_t *veh = state->cur_vehicle;
        if (strcmp(key, "airframe") == 0) {
            snprintf(veh->airframe, sizeof(veh->airframe), "%s", value);
        } else if (strcmp(key, "controller") == 0) {
            snprintf(veh->controller, sizeof(veh->controller), "%s", value);
        } else if (strcmp(key, "estimator") == 0) {
            snprintf(veh->estimator, sizeof(veh->estimator), "%s", value);
        } else if (strcmp(key, "localization") == 0) {
            snprintf(veh->localization, sizeof(veh->localization), "%s",
                     value);
        } else if (strcmp(key, "spawn") == 0) {
            char work[64];
            snprintf(work, sizeof(work), "%s", value);
            char *cursor = work;
            char *end = NULL;
            veh->spawn_pos[0] = strtod(cursor, &end);
            if (end == cursor) goto bad_spawn;
            cursor = end;
            veh->spawn_pos[1] = strtod(cursor, &end);
            if (end == cursor) goto bad_spawn;
            cursor = end;
            veh->spawn_pos[2] = strtod(cursor, &end);
            if (end == cursor || *trim(end) != '\0') goto bad_spawn;
            goto spawn_ok;
        bad_spawn:
            fprintf(stderr, "%s:%d: spawn needs 3 numbers\n",
                    ctx->path, ctx->line);
            return -1;
        spawn_ok:
            ;
        } else if (strcmp(key, "yaw") == 0) {
            veh->spawn_yaw = strtod(value, NULL);
        } else if (strcmp(key, "speed") == 0) {
            veh->spawn_speed = strtod(value, NULL);
        } else {
            fprintf(stderr, "%s:%d: warning: unknown key '%s'\n",
                    ctx->path, ctx->line, key);
        }
        return 0;
    }

    fprintf(stderr, "%s:%d: key '%s' outside any section\n",
            ctx->path, ctx->line, key);
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
