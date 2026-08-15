#ifndef MUJOCO_DRONES_VEHICLE_ESTIMATOR_OPS_H
#define MUJOCO_DRONES_VEHICLE_ESTIMATOR_OPS_H

#include "vehicle/sensor_snapshot.h"
#include <stdint.h>
#include <stdbool.h>

typedef struct {
    double   pos[3];
    double   vel[3];
    double   quat[4];      /* w, x, y, z */
    double   angvel[3];
    uint64_t timestamp_ns;
    bool     valid;
} estimated_state_t;

typedef struct estimator_t estimator_t;

typedef struct {
    const char *name;
    int  (*init)(estimator_t *est);
    void (*ingest)(estimator_t *est, const sensor_snapshot_t *snap, double dt);
    void (*get_state)(const estimator_t *est, estimated_state_t *out);
    void (*reset)(estimator_t *est);
    void (*destroy)(estimator_t *est);
} estimator_ops_t;

struct estimator_t {
    const estimator_ops_t *ops;
    void *ctx;
};

const estimator_ops_t *estimator_registry_find(const char *name);
int estimator_registry_count(void);
const estimator_ops_t *estimator_registry_at(int index);

#endif
