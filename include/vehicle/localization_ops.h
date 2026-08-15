#ifndef MUJOCO_DRONES_VEHICLE_LOCALIZATION_OPS_H
#define MUJOCO_DRONES_VEHICLE_LOCALIZATION_OPS_H

#include "vehicle/sensor_snapshot.h"
#include <stdint.h>
#include <stdbool.h>

typedef struct {
    double   pos[3];
    uint64_t timestamp_ns;
    bool     valid;
} localization_fix_t;

typedef struct localization_t localization_t;

typedef struct {
    const char *name;
    int  (*init)(localization_t *loc);
    void (*ingest)(localization_t *loc, const sensor_snapshot_t *snap);
    void (*get_fix)(const localization_t *loc, localization_fix_t *out);
    void (*reset)(localization_t *loc);
    void (*destroy)(localization_t *loc);
} localization_ops_t;

struct localization_t {
    const localization_ops_t *ops;
    void *ctx;
};

const localization_ops_t *localization_registry_find(const char *name);
int localization_registry_count(void);
const localization_ops_t *localization_registry_at(int index);

#endif
