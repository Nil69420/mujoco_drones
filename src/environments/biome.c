#include "environments/biome.h"

#include "environments/biome_scatter.h"
#include "sensors/noise.h"

#include <math.h>
#include <mujoco/mujoco.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

enum { MIN_GRID = 9, MIN_RANGE_M = 1 };

static int add_ground(mjSpec *spec, mjsBody *world, const biome_profile_t *p, unsigned seed) {
    const biome_terrain_t *t = &p->terrain;
    int n = t->grid;
    float *data = malloc((size_t)n * (size_t)n * sizeof(float));
    double *heights = malloc((size_t)n * (size_t)n * sizeof(double));
    if (data == NULL || heights == NULL) {
        free(data);
        free(heights);
        return -1;
    }
    double lo = 1e30;
    double hi = -1e30;
    for (int row = 0; row < n; row++) {
        for (int col = 0; col < n; col++) {
            double x = -t->half_extent_m + 2.0 * t->half_extent_m * (double)col / (double)(n - 1);
            double y = -t->half_extent_m + 2.0 * t->half_extent_m * (double)row / (double)(n - 1);
            double h = biome_terrain_height(t, seed, x, y);
            heights[(size_t)row * (size_t)n + (size_t)col] = h;
            lo = h < lo ? h : lo;
            hi = h > hi ? h : hi;
        }
    }
    double range = hi - lo;
    if (range < MIN_RANGE_M * 0.05) {
        range = MIN_RANGE_M * 0.05;
    }
    for (size_t i = 0; i < (size_t)n * (size_t)n; i++) {
        data[i] = (float)((heights[i] - lo) / range);
    }
    mjsHField *hf = mjs_addHField(spec);
    mjs_setName(hf->element, "env_terrain");
    hf->nrow = n;
    hf->ncol = n;
    hf->size[0] = t->half_extent_m;
    hf->size[1] = t->half_extent_m;
    hf->size[2] = range;
    hf->size[3] = 2.0;
    mjs_setFloat(hf->userdata, data, n * n);
    free(data);
    free(heights);

    mjsGeom *geom = mjs_addGeom(world, NULL);
    mjs_setName(geom->element, "env_ground");
    geom->type = mjGEOM_HFIELD;
    mjs_setString(geom->hfieldname, "env_terrain");
    geom->pos[2] = lo;
    geom->friction[0] = p->friction;
    geom->friction[1] = p->friction;
    geom->friction[2] = 0.02;
    for (int i = 0; i < 4; i++) {
        geom->rgba[i] = p->ground_rgba[i];
    }
    return 0;
}

static void add_sky(mjSpec *spec, const biome_climate_t *c) {
    mjsTexture *tex = mjs_addTexture(spec);
    mjs_setName(tex->element, "env_sky");
    tex->type = mjTEXTURE_SKYBOX;
    tex->builtin = mjBUILTIN_GRADIENT;
    tex->width = 512;
    tex->height = 3072;
    for (int i = 0; i < 3; i++) {
        tex->rgb1[i] = (double)c->sky_zenith[i];
        tex->rgb2[i] = (double)c->sky_horizon[i];
    }
}

static void add_water(mjsBody *world, const biome_profile_t *p) {
    if (p->water_rgba[3] <= 0.0F) {
        return;
    }
    mjsGeom *geom = mjs_addGeom(world, NULL);
    mjs_setName(geom->element, "env_water");
    geom->type = mjGEOM_BOX;
    geom->size[0] = p->terrain.half_extent_m;
    geom->size[1] = p->terrain.half_extent_m;
    geom->size[2] = 0.05;
    geom->pos[2] = p->water_level_m - 0.05;
    geom->contype = 0;
    geom->conaffinity = 0;
    for (int i = 0; i < 4; i++) {
        geom->rgba[i] = p->water_rgba[i];
    }
}

int biome_generate(mjSpec *spec, const biome_profile_t *profile, unsigned seed) {
    if (spec == NULL || profile == NULL) {
        return -1;
    }
    const biome_terrain_t *t = &profile->terrain;
    if (t->grid < MIN_GRID || t->grid > BIOME_MAX_GRID || t->half_extent_m <= 0.0) {
        return -1;
    }
    mjsBody *world = mjs_findBody(spec, "world");
    if (world == NULL) {
        return -1;
    }
    if (add_ground(spec, world, profile, seed) != 0) {
        return -1;
    }
    add_water(world, profile);
    add_sky(spec, &profile->climate);
    double vis = profile->climate.visibility_range_m;
    spec->visual.map.fogend = (float)vis;
    spec->visual.map.fogstart = (float)(0.2 * vis);
    spec->visual.map.zfar = (float)(vis > 2000.0 ? 2000.0 : (vis < 500.0 ? 500.0 : vis));
    noise_rng_t rng;
    noise_seed(&rng, seed);
    for (int i = 0; i < BIOME_MAX_SCATTER; i++) {
        const biome_scatter_t *rule = &profile->scatter[i];
        if (rule->count > 0 && biome_scatter(world, profile, seed, i, &rng) != 0) {
            return -1;
        }
    }
    return 0;
}

const biome_profile_t *biome_find(const char *name) {
    if (name == NULL) {
        return NULL;
    }
    for (size_t i = 0; i < BIOME_PROFILE_COUNT; i++) {
        if (strcmp(BIOME_PROFILES[i].name, name) == 0) {
            return &BIOME_PROFILES[i];
        }
    }
    return NULL;
}
