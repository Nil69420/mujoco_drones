#include "environments/environment_ops.h"
#include "sensors/noise.h"

#include <mujoco/mujoco.h>
#include <stdio.h>
#include <stdlib.h>

#define DESERT_HALF_EXTENT 100.0
#define DESERT_ROCK_COUNT  40

static void desert_add_ground(mjsBody *world, noise_rng_t *rng) {
    (void)rng;
    mjsGeom *ground_geom = mjs_addGeom(world, NULL);
    mjs_setName(ground_geom->element, "env_ground");
    ground_geom->type = mjGEOM_BOX;
    ground_geom->size[0] = DESERT_HALF_EXTENT;
    ground_geom->size[1] = DESERT_HALF_EXTENT;
    ground_geom->size[2] = 0.5;
    ground_geom->pos[2] = -0.5;
    ground_geom->friction[0] = 0.4;
    ground_geom->friction[1] = 0.4;
    ground_geom->friction[2] = 0.02;
    ground_geom->rgba[0] = 0.76F;
    ground_geom->rgba[1] = 0.70F;
    ground_geom->rgba[2] = 0.50F;
    ground_geom->rgba[3] = 1.0F;
}

static void desert_add_rocks(mjsBody *world, noise_rng_t *rng) {
    for (int i = 0; i < DESERT_ROCK_COUNT; i++) {
        mjsGeom *rock_geom = mjs_addGeom(world, NULL);
        char name[32];
        snprintf(name, sizeof(name), "desert_rock_%d", i);
        mjs_setName(rock_geom->element, name);
        rock_geom->type = mjGEOM_ELLIPSOID;
        double radius = noise_uniform(rng);
        double ang = noise_uniform(rng) * 2.0 * 3.14159265358979323846;
        rock_geom->pos[0] = radius * cos(ang) * (DESERT_HALF_EXTENT - 5.0);
        rock_geom->pos[1] = radius * sin(ang) * (DESERT_HALF_EXTENT - 5.0);
        double rock_r = 0.2 + 1.2 * noise_uniform(rng);
        rock_geom->pos[2] = 0.25 * rock_r;
        rock_geom->size[0] = rock_r;
        rock_geom->size[1] = 0.8 * rock_r;
        rock_geom->size[2] = 0.25 * rock_r;
        rock_geom->friction[0] = 0.6;
        rock_geom->friction[1] = 0.6;
        rock_geom->friction[2] = 0.02;
        rock_geom->rgba[0] = 0.55F;
        rock_geom->rgba[1] = 0.45F;
        rock_geom->rgba[2] = 0.30F;
        rock_geom->rgba[3] = 1.0F;
    }
}

static int desert_generate(mjSpec *spec, unsigned seed) {
    if (!spec) return -1;
    mjsBody *world = mjs_findBody(spec, "world");
    if (!world) return -1;

    noise_rng_t rng;
    noise_seed(&rng, seed);

    desert_add_ground(world, &rng);
    desert_add_rocks(world, &rng);
    return 0;
}

const environment_ops_t desert_environment_ops = {
    .name     = "desert",
    .generate = desert_generate,
};
