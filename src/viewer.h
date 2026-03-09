/*
 * viewer.h - GLFW/MuJoCo interactive viewer interface
 */
#ifndef MUJOCO_DRONES_VIEWER_H
#define MUJOCO_DRONES_VIEWER_H

#include "types.h"

#ifndef NO_GLFW

/* Initialize GLFW window and MuJoCo rendering context.
   Returns 0 on success, -1 on failure. */
int viewer_init(sim_t *sim);

/* Run the render loop (blocks until window closed).
   Calls mj_step internally to keep real-time pace. */
void viewer_loop(sim_t *sim);

/* Cleanup rendering resources */
void viewer_close(void);

#endif /* NO_GLFW */

/* Print one line of telemetry (works in all modes) */
void viewer_print_telemetry(const sim_t *sim);

#endif /* MUJOCO_DRONES_VIEWER_H */
