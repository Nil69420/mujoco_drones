#ifndef MUJOCO_DRONES_SCENARIO_H
#define MUJOCO_DRONES_SCENARIO_H

#include "vehicle/assembly_config.h"
#include <stdbool.h>

enum { SCENARIO_MAX_VEHICLES = 16, SCENARIO_MAX_NAME_LEN = 64 };

typedef struct {
    char environment[SCENARIO_MAX_NAME_LEN];
    unsigned environment_seed;
    double duration_s;
    vehicle_assembly_spec_t vehicles[SCENARIO_MAX_VEHICLES];
    int vehicle_count;
} scenario_t;

/* Parses the format in this plan's §5. Returns 0 on success, -1 on parse
 * error with a message on stderr naming the file/line. */
int scenario_load(const char *path, scenario_t *out);

int scenario_main_run(const char *path, bool headless,
                      double duration_override);

#endif
