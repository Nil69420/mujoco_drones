#ifndef MUJOCO_DRONES_VIEWER_H
#define MUJOCO_DRONES_VIEWER_H

#include "types.h"

#ifndef NO_GLFW

int viewer_init(sim_t *sim);
void viewer_loop(sim_t *sim);
void viewer_close(void);

#endif

void viewer_print_telemetry(const sim_t *sim);

#endif
