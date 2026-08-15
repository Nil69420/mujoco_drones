#ifndef MUJOCO_DRONES_ENVIRONMENTS_ENVIRONMENT_OPS_H
#define MUJOCO_DRONES_ENVIRONMENTS_ENVIRONMENT_OPS_H

#include <mujoco/mjspec.h>

typedef struct {
    const char *name;   /* e.g. "desert" */
    /* Procedurally populates `spec`'s worldbody with terrain/vegetation/
     * obstacles and sets any physics defaults (friction, wind — see Step
     * 10) specific to this environment, driven by `seed`. Uses the mjSpec
     * C API (mjs_addBody/mjs_addGeom/etc.) directly rather than generating
     * and re-parsing MJCF text. Returns 0 on success. */
    int (*generate)(mjSpec *spec, unsigned seed);
} environment_ops_t;

const environment_ops_t *environment_registry_find(const char *name);
int environment_registry_count(void);
const environment_ops_t *environment_registry_at(int index);

#endif
