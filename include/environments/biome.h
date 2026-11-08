#ifndef MUJOCO_DRONES_BIOME_H
#define MUJOCO_DRONES_BIOME_H

#include <stddef.h>
#include <stdint.h>

#include <mujoco/mjspec.h>

#include "environments/environment_ops.h"

typedef enum {
    OBJ_CONIFER = 0,
    OBJ_BROADLEAF,
    OBJ_PALM,
    OBJ_SHRUB,
    OBJ_ROCK,
    OBJ_BOULDER,
    OBJ_CACTUS,
    OBJ_CROP_FIELD,
    OBJ_HOUSE,
    OBJ_BARN,
    OBJ_MIDRISE,
    OBJ_TOWER,
    OBJ_ROAD_GRID,
    OBJ_POLE,
    OBJ_FENCE,
    OBJ_ICE_BLOCK,
    OBJ_SHIP,
    OBJ_KIND_COUNT
} biome_object_kind_t;

typedef struct {
    biome_object_kind_t kind;
    int count;
    double min_size;
    double max_size;
    float rgba[4];
    int needs_dry_land;
    int on_terrain_only;
} biome_scatter_t;

enum { BIOME_MAX_SCATTER = 8 };

typedef struct {
    double half_extent_m;
    int grid;
    double base_height_m;
    double amplitude_m;
    double wavelength_m;
    int octaves;
    double ridged;
    double ramp_m;
    double swell_m;
    double swell_wavelength_m;
    double pad_radius_m;
} biome_terrain_t;

typedef struct {
    const char *default_weather;
    double ground_reflectivity;
    double sky_visibility;
    double multipath_scale;
    double visibility_range_m;
    float sky_zenith[3];
    float sky_horizon[3];
} biome_climate_t;

struct biome_profile_s {
    const char *name;
    const char *description;
    biome_terrain_t terrain;
    float ground_rgba[4];
    double friction;
    double water_level_m;
    float water_rgba[4];
    biome_scatter_t scatter[BIOME_MAX_SCATTER];
    biome_climate_t climate;
};

typedef struct biome_profile_s biome_profile_t;

enum { BIOME_MAX_GRID = 257 };

double biome_terrain_height(const biome_terrain_t *t, unsigned seed, double x, double y);
int biome_generate(mjSpec *spec, const biome_profile_t *profile, unsigned seed);

extern const biome_profile_t BIOME_PROFILES[];
extern const size_t BIOME_PROFILE_COUNT;
const biome_profile_t *biome_find(const char *name);

#endif
