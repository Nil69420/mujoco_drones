/*
 * controller.h - Flight controller interface
 */
#ifndef MUJOCO_DRONES_CONTROLLER_H
#define MUJOCO_DRONES_CONTROLLER_H

#include "types.h"

/* Default gains (tuned for 1000 Hz) */
ctrl_gains_t ctrl_default_gains(void);

/* Resolve actuator indices from model.  Returns 0 on success, -1 on error. */
int ctrl_resolve_actuators(sim_t *sim);

/* Reset integrators / internal state */
void ctrl_reset(sim_t *sim);

/* Controller update — called from MuJoCo callback */
void ctrl_update(const mjModel *model, mjData *data);

/* Utility: quaternion → euler (ZYX) */
void quat_to_euler(const double q[4], double *roll, double *pitch, double *yaw);

/* Utility: clamp */
static inline double clampd(double v, double lo, double hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

#endif /* MUJOCO_DRONES_CONTROLLER_H */
