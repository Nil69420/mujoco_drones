#ifndef MUJOCO_DRONES_VEHICLE_CONTROLLER_OPS_H
#define MUJOCO_DRONES_VEHICLE_CONTROLLER_OPS_H

#include "vehicle/actuator.h"
#include "vehicle/estimator_ops.h"
#include "setpoint.h"

typedef struct controller_t controller_t;

typedef struct {
    const char *name;
    const char *airframe_class;   /* must equal an airframe_spec_t's airframe_class */
    int  (*init)(controller_t *ctrl, int num_channels);
    void (*update)(controller_t *ctrl, const estimated_state_t *state,
                   const setpoint_t *target, double dt, actuator_command_t *out);
    void (*reset)(controller_t *ctrl);
    void (*destroy)(controller_t *ctrl);
} controller_ops_t;

struct controller_t {
    const controller_ops_t *ops;
    void *ctx;
};

const controller_ops_t *controller_registry_find(const char *name);
int controller_registry_count(void);
const controller_ops_t *controller_registry_at(int index);

#endif
