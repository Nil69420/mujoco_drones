#include "conditions/conditions.h"
#include "environments/biome.h"
#include "environments/environment_ops.h"
#include "test_support.h"

#include <mujoco/mujoco.h>
#include <stdint.h>
#include <string.h>

enum { MAX_GEOMS = 4000, STEPS = 60 };

static mjSpec *load_base(void) {
    char error[300];
    mjSpec *spec = mj_parseXML("model/environments/base_scene.xml", NULL, error, sizeof(error));
    if (spec == NULL) {
        fprintf(stderr, "base scene: %s\n", error);
    }
    return spec;
}

static mjModel *build(const biome_profile_t *p, unsigned seed) {
    mjSpec *spec = load_base();
    if (spec == NULL) {
        return NULL;
    }
    if (biome_generate(spec, p, seed) != 0) {
        mj_deleteSpec(spec);
        return NULL;
    }
    mjModel *model = mj_compile(spec, NULL);
    if (model == NULL) {
        fprintf(stderr, "%s: compile failed: %s\n", p->name, mjs_getError(spec));
    }
    mj_deleteSpec(spec);
    return model;
}

static uint64_t fingerprint(const mjModel *m) {
    uint64_t h = 1469598103934665603ULL;
    for (int i = 0; i < 3 * m->ngeom; i++) {
        int64_t v = (int64_t)(m->geom_pos[i] * 1000.0);
        h = (h ^ (uint64_t)v) * 1099511628211ULL;
    }
    for (int i = 0; i < m->nhfielddata; i++) {
        h = (h ^ (uint64_t)(m->hfield_data[i] * 65535.0F)) * 1099511628211ULL;
    }
    return h;
}

static int is_solid_object(const char *name) {
    static const char *const tags[] = {"_body", "_trunk", "_rock", "_cactus", "_hull", "_ice", "_pole"};
    for (size_t i = 0; i < sizeof(tags) / sizeof(tags[0]); i++) {
        if (strstr(name, tags[i]) != NULL) {
            return 1;
        }
    }
    return 0;
}

static void check_profile(const biome_profile_t *p) {
    mjModel *model = build(p, 17);
    if (model == NULL) {
        g_failures++;
        return;
    }
    mjData *data = mj_makeData(model);
    CHECK(model->ngeom > 1 && model->ngeom <= MAX_GEOMS);
    for (int i = 0; i < STEPS; i++) {
        mj_step(model, data);
    }
    CHECK(isfinite(data->time));

    int ground = mj_name2id(model, mjOBJ_GEOM, "env_ground");
    CHECK(ground >= 0);
    CHECK(fabs(biome_terrain_height(&p->terrain, 17, 0.0, 0.0)) < 1e-9 || p->terrain.pad_radius_m == 0.0);

    double ext = p->terrain.half_extent_m * 0.8;
    double worst = 0.0;
    for (int k = 0; k < 40; k++) {
        double x = ext * (2.0 * (double)((k * 37) % 41) / 40.0 - 1.0);
        double y = ext * (2.0 * (double)((k * 53) % 43) / 42.0 - 1.0);
        double want = biome_terrain_height(&p->terrain, 17, x, y);
        double from[3] = {x, y, want + 500.0};
        double dir[3] = {0.0, 0.0, -1.0};
        int gid = -1;
        mjtByte group[mjNGROUP];
        memset(group, 1, sizeof(group));
        double dist = mj_ray(model, data, from, dir, group, 1, -1, &gid, NULL);
        if (gid == ground && dist > 0.0) {
            double got = from[2] - dist;
            double err = fabs(got - want);
            worst = err > worst ? err : worst;
        }
    }
    double tolerance = 0.05 + 0.02 * (p->terrain.amplitude_m + p->terrain.swell_m);
    if (worst > tolerance) {
        fprintf(stderr, "%s: terrain mismatch %.3f m (tolerance %.3f)\n", p->name, worst, tolerance);
        g_failures++;
    }

    for (int g = 0; g < model->ngeom; g++) {
        const char *name = mj_id2name(model, mjOBJ_GEOM, g);
        if (name == NULL || !is_solid_object(name)) {
            continue;
        }
        double dx = model->geom_pos[3 * g];
        double dy = model->geom_pos[3 * g + 1];
        double reach = model->geom_size[3 * g] > model->geom_size[3 * g + 1]
                           ? model->geom_size[3 * g] : model->geom_size[3 * g + 1];
        if (strstr(name, "_pole") != NULL || strstr(name, "_trunk") != NULL) {
            reach = model->geom_size[3 * g];
        }
        double clearance = sqrt(dx * dx + dy * dy) - 1.5 * reach;
        if (clearance < 8.0 && strstr(name, "_hull") == NULL) {
            fprintf(stderr, "%s: %s only %.1f m from the spawn point\n", p->name, name, clearance);
            g_failures++;
            break;
        }
        if (p->water_rgba[3] > 0.0F && model->geom_pos[3 * g + 2] < p->water_level_m && strstr(name, "_hull") == NULL) {
            fprintf(stderr, "%s: %s is under water\n", p->name, name);
            g_failures++;
            break;
        }
    }
    mj_deleteData(data);
    mj_deleteModel(model);
}

static void check_determinism(const biome_profile_t *p) {
    mjModel *a = build(p, 5);
    mjModel *b = build(p, 5);
    mjModel *c = build(p, 6);
    CHECK(a != NULL && b != NULL && c != NULL);
    if (a != NULL && b != NULL && c != NULL) {
        CHECK(fingerprint(a) == fingerprint(b));
        if (p->terrain.amplitude_m > 0.0 || p->scatter[0].count > 0) {
            CHECK(fingerprint(a) != fingerprint(c));
        }
    }
    mj_deleteModel(a);
    mj_deleteModel(b);
    mj_deleteModel(c);
}

static void check_registry(void) {
    CHECK(environment_registry_count() == (int)BIOME_PROFILE_COUNT);
    CHECK(environment_registry_count() >= 15);
    CHECK(environment_registry_find("does_not_exist") == NULL);
    static const char *const required[] = {"desert", "forest", "agricultural", "mountains", "snow",
                                          "tundra", "ocean", "urban", "rural", "city"};
    for (size_t i = 0; i < sizeof(required) / sizeof(required[0]); i++) {
        const environment_ops_t *ops = environment_registry_find(required[i]);
        CHECK(ops != NULL);
        if (ops != NULL) {
            mjSpec *spec = load_base();
            CHECK(spec != NULL && ops->generate(ops, spec, 1) == 0);
            if (spec != NULL) {
                mj_deleteSpec(spec);
            }
        }
    }
    for (size_t i = 0; i < BIOME_PROFILE_COUNT; i++) {
        const biome_climate_t *cl = &BIOME_PROFILES[i].climate;
        CHECK(conditions_find(cl->default_weather) != NULL);
        CHECK(cl->sky_visibility > 0.0 && cl->sky_visibility <= 1.0);
        CHECK(cl->multipath_scale >= 1.0);
        CHECK(cl->ground_reflectivity > 0.0 && cl->ground_reflectivity <= 1.0);
        CHECK(cl->visibility_range_m >= 1000.0);
    }
}

static void check_invalid(void) {
    biome_profile_t bad = BIOME_PROFILES[0];
    mjSpec *spec = load_base();
    CHECK(spec != NULL);
    if (spec == NULL) {
        return;
    }
    bad.terrain.grid = BIOME_MAX_GRID + 2;
    CHECK(biome_generate(spec, &bad, 1) != 0);
    bad = BIOME_PROFILES[0];
    bad.terrain.grid = 4;
    CHECK(biome_generate(spec, &bad, 1) != 0);
    bad = BIOME_PROFILES[0];
    bad.scatter[0].min_size = 5.0;
    bad.scatter[0].max_size = 1.0;
    CHECK(biome_generate(spec, &bad, 1) != 0);
    CHECK(biome_generate(NULL, &BIOME_PROFILES[0], 1) != 0);
    CHECK(biome_generate(spec, NULL, 1) != 0);
    mj_deleteSpec(spec);
}

int main(void) {
    for (size_t i = 0; i < BIOME_PROFILE_COUNT; i++) {
        check_profile(&BIOME_PROFILES[i]);
        check_determinism(&BIOME_PROFILES[i]);
    }
    check_registry();
    check_invalid();
    if (g_failures != 0) {
        fprintf(stderr, "%d check(s) failed\n", g_failures);
        return 1;
    }
    printf("test_biomes OK (%zu biomes)\n", BIOME_PROFILE_COUNT);
    return 0;
}
