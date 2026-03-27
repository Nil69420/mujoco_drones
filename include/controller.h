#ifndef MUJOCO_DRONES_CONTROLLER_H
#define MUJOCO_DRONES_CONTROLLER_H

#include "types.h"

ctrl_gains_t ctrl_default_gains(void);
int ctrl_resolve_actuators(sim_t *sim);
void ctrl_reset(sim_t *sim);
void ctrl_update(sim_t *sim);
void quat_to_euler(const double q[4], double *roll, double *pitch, double *yaw);

static inline double clampd(double v, double lo, double hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

#endif
