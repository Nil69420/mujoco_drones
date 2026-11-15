#ifndef MUJOCO_DRONES_BRIDGE_AP_SCENARIO_H
#define MUJOCO_DRONES_BRIDGE_AP_SCENARIO_H

#include "scenario.h"
#include "vehicle/vehicle.h"

#include <mujoco/mujoco.h>

int ap_scenario_find(const scenario_t *scenario);

int ap_scenario_run(scenario_t *scenario, vehicle_t *vehicles, const mjModel *model, mjData *data,
                    int bridged);

#endif
