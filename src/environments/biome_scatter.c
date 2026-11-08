#include "environments/biome_scatter.h"

#include <math.h>
#include <mujoco/mujoco.h>
#include <stdio.h>
#include <string.h>

#define CLEAR_RADIUS_M 20.0
#define EDGE_MARGIN_M 6.0
#define PLACE_ATTEMPTS 40
#define MAX_LATTICE 96

typedef struct {
    mjsBody *world;
    const biome_profile_t *profile;
    unsigned seed;
    int rule_index;
    int serial;
    noise_rng_t *rng;
} ctx_t;

static double uniform(ctx_t *c, double lo, double hi) {
    return lo + (hi - lo) * noise_uniform(c->rng);
}

static void jitter_colour(ctx_t *c, const float base[4], float out[4]) {
    double k = uniform(c, 0.88, 1.12);
    for (int i = 0; i < 3; i++) {
        double val = (double)base[i] * k;
        out[i] = (float)(val > 1.0 ? 1.0 : val);
    }
    out[3] = base[3];
}

static mjsGeom *add_geom(ctx_t *c, const char *part, mjtGeom type, const double pos[3],
                         const double size[3], const float rgba[4], int collide, double yaw) {
    mjsGeom *geom = mjs_addGeom(c->world, NULL);
    char name[64];
    snprintf(name, sizeof(name), "env_%d_%d_%s", c->rule_index, c->serial, part);
    mjs_setName(geom->element, name);
    geom->type = type;
    for (int i = 0; i < 3; i++) {
        geom->pos[i] = pos[i];
        geom->size[i] = size[i];
    }
    for (int i = 0; i < 4; i++) {
        geom->rgba[i] = rgba[i];
    }
    if (!collide) {
        geom->contype = 0;
        geom->conaffinity = 0;
    }
    geom->quat[0] = cos(0.5 * yaw);
    geom->quat[1] = 0.0;
    geom->quat[2] = 0.0;
    geom->quat[3] = sin(0.5 * yaw);
    return geom;
}

static void tree(ctx_t *c, double x, double y, double h, double s, const float rgba[4], int kind) {
    static const float trunk[4] = {0.35F, 0.24F, 0.14F, 1.0F};
    double tpos[3] = {x, y, h + 0.2 * s};
    double tsize[3] = {0.03 * s + 0.05, 0.2 * s, 0.0};
    if (kind == OBJ_PALM) {
        tpos[2] = h + 0.45 * s;
        tsize[0] = 0.02 * s + 0.05;
        tsize[1] = 0.45 * s;
    }
    add_geom(c, "trunk", mjGEOM_CYLINDER, tpos, tsize, trunk, 1, 0.0);
    double cpos[3] = {x, y, h + 0.6 * s};
    double csize[3] = {0.3 * s, 0.3 * s, 0.25 * s};
    if (kind == OBJ_CONIFER) {
        cpos[2] = h + 0.55 * s;
        csize[0] = 0.18 * s;
        csize[1] = 0.18 * s;
        csize[2] = 0.45 * s;
    } else if (kind == OBJ_PALM) {
        cpos[2] = h + 0.93 * s;
        csize[2] = 0.05 * s;
    }
    add_geom(c, "crown", mjGEOM_ELLIPSOID, cpos, csize, rgba, 0, 0.0);
}

static void rock(ctx_t *c, double x, double y, double h, double s, const float rgba[4]) {
    double pos[3] = {x, y, h + 0.12 * s};
    double size[3] = {s, 0.7 * s, 0.4 * s};
    add_geom(c, "rock", mjGEOM_ELLIPSOID, pos, size, rgba, 1, uniform(c, 0.0, 6.2831853));
}

static void shrub(ctx_t *c, double x, double y, double h, double s, const float rgba[4]) {
    double pos[3] = {x, y, h + 0.3 * s};
    double size[3] = {0.5 * s, 0.5 * s, 0.35 * s};
    add_geom(c, "shrub", mjGEOM_ELLIPSOID, pos, size, rgba, 0, 0.0);
}

static void cactus(ctx_t *c, double x, double y, double h, double s, const float rgba[4]) {
    double pos[3] = {x, y, h + 0.5 * s};
    double size[3] = {0.08 * s + 0.05, 0.4 * s, 0.0};
    add_geom(c, "cactus", mjGEOM_CAPSULE, pos, size, rgba, 1, 0.0);
}

static void building(ctx_t *c, double x, double y, double h, double fw, double fd, double height,
                     const float rgba[4]) {
    double pos[3] = {x, y, h + 0.5 * height};
    double size[3] = {0.5 * fw, 0.5 * fd, 0.5 * height};
    add_geom(c, "body", mjGEOM_BOX, pos, size, rgba, 1, 0.0);
    static const float roof[4] = {0.22F, 0.22F, 0.24F, 1.0F};
    double rpos[3] = {x, y, h + height + 0.15};
    double rsize[3] = {0.52 * fw, 0.52 * fd, 0.15};
    add_geom(c, "roof", mjGEOM_BOX, rpos, rsize, roof, 1, 0.0);
}

static void crop_field(ctx_t *c, double x, double y, double h, double w, const float rgba[4]) {
    double length = 1.5 * w;
    double yaw = uniform(c, 0.0, 3.14159265);
    double dx = cos(yaw);
    double dy = sin(yaw);
    const int rows = 8;
    for (int i = 0; i < rows; i++) {
        double off = ((double)i - 0.5 * (rows - 1)) * (w / rows);
        double pos[3] = {x + dx * off, y + dy * off, h + 0.25};
        double size[3] = {0.45 * w / rows, 0.5 * length, 0.25};
        float tone[4];
        jitter_colour(c, rgba, tone);
        add_geom(c, "row", mjGEOM_BOX, pos, size, tone, 0, yaw);
        c->serial++;
    }
}

static void road_grid(ctx_t *c, const biome_scatter_t *r) {
    static const float asphalt[4] = {0.12F, 0.12F, 0.13F, 1.0F};
    double pitch = r->max_size;
    double ext = c->profile->terrain.half_extent_m - EDGE_MARGIN_M;
    for (int i = 0; i < r->count; i++) {
        double off = ((double)i - 0.5 * (r->count - 1)) * pitch;
        double along[3] = {0.0, off, 0.02};
        double sizea[3] = {ext, 0.5 * r->min_size, 0.02};
        add_geom(c, "roadx", mjGEOM_BOX, along, sizea, asphalt, 0, 0.0);
        c->serial++;
        double across[3] = {off, 0.0, 0.02};
        double sizeb[3] = {0.5 * r->min_size, ext, 0.02};
        add_geom(c, "roady", mjGEOM_BOX, across, sizeb, asphalt, 0, 0.0);
        c->serial++;
    }
}

static double road_pitch(const biome_profile_t *p) {
    for (int i = 0; i < BIOME_MAX_SCATTER; i++) {
        if (p->scatter[i].kind == OBJ_ROAD_GRID && p->scatter[i].count > 0) {
            return p->scatter[i].max_size;
        }
    }
    return 0.0;
}

static int place_free(ctx_t *c, const biome_scatter_t *r, double *x, double *y, double *h) {
    const biome_terrain_t *t = &c->profile->terrain;
    double ext = t->half_extent_m - EDGE_MARGIN_M;
    for (int attempt = 0; attempt < PLACE_ATTEMPTS; attempt++) {
        double px = uniform(c, -ext, ext);
        double py = uniform(c, -ext, ext);
        if (sqrt(px * px + py * py) < CLEAR_RADIUS_M) {
            continue;
        }
        double ph = biome_terrain_height(t, c->seed, px, py);
        if (r->needs_dry_land && ph < c->profile->water_level_m + 0.15) {
            continue;
        }
        *x = px;
        *y = py;
        *h = ph;
        return 1;
    }
    return 0;
}

static int place_lattice(ctx_t *c, double pitch, unsigned char *used, double *x, double *y,
                         double *h) {
    const biome_terrain_t *t = &c->profile->terrain;
    int cells = (int)floor(2.0 * (t->half_extent_m - EDGE_MARGIN_M) / pitch);
    if (cells > MAX_LATTICE) {
        cells = MAX_LATTICE;
    }
    for (int attempt = 0; attempt < PLACE_ATTEMPTS * 4; attempt++) {
        int i = (int)uniform(c, 0.0, (double)cells);
        int j = (int)uniform(c, 0.0, (double)cells);
        if (i >= cells || j >= cells || used[i * MAX_LATTICE + j]) {
            continue;
        }
        double px = ((double)i - 0.5 * (double)cells + 0.5) * pitch;
        double py = ((double)j - 0.5 * (double)cells + 0.5) * pitch;
        if (fabs(px) < 0.6 * pitch + CLEAR_RADIUS_M * 0.5 && fabs(py) < 0.6 * pitch + CLEAR_RADIUS_M * 0.5) {
            continue;
        }
        used[i * MAX_LATTICE + j] = 1;
        *x = px;
        *y = py;
        *h = biome_terrain_height(t, c->seed, px, py);
        return 1;
    }
    return 0;
}

static void place_one(ctx_t *c, const biome_scatter_t *r, double x, double y, double h, double s,
                      double pitch) {
    float rgba[4];
    jitter_colour(c, r->rgba, rgba);
    switch (r->kind) {
    case OBJ_CONIFER:
    case OBJ_BROADLEAF:
    case OBJ_PALM:
        tree(c, x, y, h, s, rgba, (int)r->kind);
        break;
    case OBJ_SHRUB:
        shrub(c, x, y, h, s, rgba);
        break;
    case OBJ_ROCK:
    case OBJ_BOULDER:
        rock(c, x, y, h, s, rgba);
        break;
    case OBJ_CACTUS:
        cactus(c, x, y, h, s, rgba);
        break;
    case OBJ_CROP_FIELD:
        crop_field(c, x, y, h, s, rgba);
        break;
    case OBJ_HOUSE:
        building(c, x, y, h, s, 0.8 * s, uniform(c, 2.8, 5.5), rgba);
        break;
    case OBJ_BARN:
        building(c, x, y, h, s, 0.6 * s, uniform(c, 5.0, 8.0), rgba);
        break;
    case OBJ_MIDRISE: {
        double fp = pitch > 0.0 ? pitch * uniform(c, 0.45, 0.7) : s;
        building(c, x, y, h, fp, fp, uniform(c, 10.0, 32.0), rgba);
        break;
    }
    case OBJ_TOWER: {
        double fp = pitch > 0.0 ? pitch * uniform(c, 0.4, 0.6) : s;
        building(c, x, y, h, fp, fp, uniform(c, 60.0, 160.0), rgba);
        break;
    }
    case OBJ_POLE: {
        double pos[3] = {x, y, h + 4.0};
        double size[3] = {0.12, 4.0, 0.0};
        static const float wood[4] = {0.3F, 0.22F, 0.14F, 1.0F};
        add_geom(c, "pole", mjGEOM_CYLINDER, pos, size, wood, 1, 0.0);
        break;
    }
    case OBJ_FENCE: {
        double pos[3] = {x, y, h + 0.5};
        double size[3] = {0.5 * s, 0.04, 0.5};
        add_geom(c, "fence", mjGEOM_BOX, pos, size, rgba, 1, uniform(c, 0.0, 3.14159265));
        break;
    }
    case OBJ_ICE_BLOCK: {
        double pos[3] = {x, y, h + 0.3 * s};
        double size[3] = {s, 0.8 * s, 0.35 * s};
        add_geom(c, "ice", mjGEOM_BOX, pos, size, rgba, 1, uniform(c, 0.0, 3.14159265));
        break;
    }
    case OBJ_SHIP: {
        double wl = c->profile->water_level_m;
        double hull[3] = {x, y, wl + 0.6};
        double hsize[3] = {s, 0.18 * s, 0.9};
        add_geom(c, "hull", mjGEOM_BOX, hull, hsize, rgba, 1, 0.0);
        double deck[3] = {x - 0.3 * s, y, wl + 2.4};
        double dsize[3] = {0.18 * s, 0.14 * s, 0.9};
        static const float white[4] = {0.9F, 0.9F, 0.92F, 1.0F};
        add_geom(c, "bridge", mjGEOM_BOX, deck, dsize, white, 1, 0.0);
        break;
    }
    case OBJ_ROAD_GRID:
    case OBJ_KIND_COUNT:
        break;
    }
    c->serial++;
}

int biome_scatter(mjsBody *world, const biome_profile_t *profile, unsigned seed, int rule_index,
                  noise_rng_t *rng) {
    const biome_scatter_t *rule = &profile->scatter[rule_index];
    if (rule->count <= 0 || rule->min_size <= 0.0 || rule->max_size < rule->min_size) {
        return -1;
    }
    ctx_t ctx = {world, profile, seed, rule_index, 0, rng};
    if (rule->kind == OBJ_ROAD_GRID) {
        road_grid(&ctx, rule);
        return 0;
    }
    double pitch = road_pitch(profile);
    int lattice = pitch > 0.0 && (rule->kind == OBJ_MIDRISE || rule->kind == OBJ_TOWER);
    unsigned char used[MAX_LATTICE * MAX_LATTICE];
    memset(used, 0, sizeof(used));
    for (int i = 0; i < rule->count; i++) {
        double x = 0.0;
        double y = 0.0;
        double h = 0.0;
        int ok = lattice ? place_lattice(&ctx, pitch, used, &x, &y, &h) : place_free(&ctx, rule, &x, &y, &h);
        if (rule->kind == OBJ_SHIP) {
            ok = place_free(&ctx, rule, &x, &y, &h);
        }
        if (!ok) {
            continue;
        }
        double size = rule->min_size + (rule->max_size - rule->min_size) * noise_uniform(rng);
        place_one(&ctx, rule, x, y, h, size, lattice ? pitch : 0.0);
    }
    return 0;
}
