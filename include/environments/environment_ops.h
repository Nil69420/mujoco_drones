#ifndef MUJOCO_DRONES_ENVIRONMENTS_ENVIRONMENT_OPS_H
#define MUJOCO_DRONES_ENVIRONMENTS_ENVIRONMENT_OPS_H

#include <mujoco/mjspec.h>

struct biome_profile_s;

typedef struct environment_ops_s {
    const char *name;
    const char *description;
    const struct biome_profile_s *biome;
    int (*generate)(const struct environment_ops_s *self, mjSpec *spec, unsigned seed);
} environment_ops_t;

const environment_ops_t *environment_registry_find(const char *name);
int environment_registry_count(void);
const environment_ops_t *environment_registry_at(int index);

#endif
