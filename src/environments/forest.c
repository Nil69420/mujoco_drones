#include "environments/environment_ops.h"
#include "sensors/noise.h"

#include <mujoco/mujoco.h>
#include <stdio.h>
#include <stdlib.h>

#define FOREST_HALF_EXTENT 100.0
#define FOREST_TREE_COUNT  120

static void forest_add_ground(mjsBody *world, noise_rng_t *rng) {
    (void)rng;
    mjsGeom *ground_geom = mjs_addGeom(world, NULL);
    mjs_setName(ground_geom->element, "env_ground");
    ground_geom->type = mjGEOM_BOX;
    ground_geom->size[0] = FOREST_HALF_EXTENT;
    ground_geom->size[1] = FOREST_HALF_EXTENT;
    ground_geom->size[2] = 0.5;
    ground_geom->pos[2] = -0.5;
    ground_geom->friction[0] = 0.9;
    ground_geom->friction[1] = 0.9;
    ground_geom->friction[2] = 0.05;
    ground_geom->rgba[0] = 0.25F;
    ground_geom->rgba[1] = 0.45F;
    ground_geom->rgba[2] = 0.20F;
    ground_geom->rgba[3] = 1.0F;
}

static void forest_add_trees(mjsBody *world, noise_rng_t *rng) {
    for (int i = 0; i < FOREST_TREE_COUNT; i++) {
        mjsGeom *tree_geom = mjs_addGeom(world, NULL);
        char name[32];
        snprintf(name, sizeof(name), "forest_tree_%d", i);
        mjs_setName(tree_geom->element, name);
        tree_geom->type = mjGEOM_CYLINDER;
        double radius = noise_uniform(rng);
        double ang = noise_uniform(rng) * 2.0 * 3.14159265358979323846;
        tree_geom->pos[0] = radius * cos(ang) * (FOREST_HALF_EXTENT - 4.0);
        tree_geom->pos[1] = radius * sin(ang) * (FOREST_HALF_EXTENT - 4.0);
        double trunk_r = 0.15 + 0.35 * noise_uniform(rng);
        double trunk_h = 4.0 + 6.0 * noise_uniform(rng);
        tree_geom->pos[2] = 0.5 * trunk_h;
        tree_geom->size[0] = trunk_r;
        tree_geom->size[1] = 0.5 * trunk_h;
        tree_geom->friction[0] = 0.9;
        tree_geom->friction[1] = 0.9;
        tree_geom->friction[2] = 0.05;
        tree_geom->rgba[0] = 0.35F;
        tree_geom->rgba[1] = 0.25F;
        tree_geom->rgba[2] = 0.12F;
        tree_geom->rgba[3] = 1.0F;
    }
}

static int forest_generate(mjSpec *spec, unsigned seed) {
    if (!spec) return -1;
    mjsBody *world = mjs_findBody(spec, "world");
    if (!world) return -1;

    noise_rng_t rng;
    noise_seed(&rng, seed);

    forest_add_ground(world, &rng);
    forest_add_trees(world, &rng);
    return 0;
}

const environment_ops_t forest_environment_ops = {
    .name     = "forest",
    .generate = forest_generate,
};
