#ifndef MUJOCO_DRONES_BRIDGE_TRUTH_H
#define MUJOCO_DRONES_BRIDGE_TRUTH_H

#include "bridge/ap_frames.h"
#include "vehicle/actuator.h"
#include "vehicle/vehicle.h"

#include <mujoco/mujoco.h>

void bridge_read_truth(const vehicle_t *vehicle, const mjModel *model, mjData *data,
                       const double origin_enu[3], ap_truth_t *truth);

void bridge_apply_command(const vehicle_t *vehicle, const mjModel *model, mjData *data,
                          const actuator_command_t *command);

#endif
