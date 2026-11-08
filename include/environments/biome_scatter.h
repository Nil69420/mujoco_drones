#ifndef MUJOCO_DRONES_BIOME_SCATTER_H
#define MUJOCO_DRONES_BIOME_SCATTER_H

#include "environments/biome.h"
#include "sensors/noise.h"

int biome_scatter(mjsBody *world, const biome_profile_t *profile, unsigned seed, int rule_index,
                  noise_rng_t *rng);

#endif
