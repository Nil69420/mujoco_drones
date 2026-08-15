#ifndef MUJOCO_DRONES_VEHICLE_VEHICLE_H
#define MUJOCO_DRONES_VEHICLE_VEHICLE_H

#include "vehicle/actuator.h"
#include "vehicle/airframe.h"
#include "vehicle/controller_ops.h"
#include "vehicle/estimator_ops.h"
#include "vehicle/localization_ops.h"
#include "vehicle/sensor_snapshot.h"
#include "sensors/sensors.h"
#include "setpoint.h"
#include <mujoco/mujoco.h>
#include <stdint.h>

enum { VEHICLE_MAX_PREFIX_LEN = 32 };

typedef struct {
    char prefix[VEHICLE_MAX_PREFIX_LEN];   /* e.g. "drone0_", includes trailing underscore */
    const airframe_spec_t *airframe;
    controller_t   controller;
    estimator_t    estimator;
    localization_t localization;
    sensor_mgr_t   sensors;
    int actuator_index[VEHICLE_MAX_ACTUATOR_CHANNELS]; /* mj_name2id result per channel */
    setpoint_t target;
    uint32_t last_imu_seq;
    uint32_t last_gnss_seq;
    uint32_t last_baro_seq;
} vehicle_t;

/* Resolves airframe_spec_t's UNPREFIXED names against `model` by prepending
 * `vehicle->prefix` (matching whatever prefix mjs_attach used when this
 * vehicle's airframe spec was attached into the scene, Step 11). Must be
 * called after mj_compile(), since it does mj_name2id lookups. */
int vehicle_resolve(vehicle_t *vehicle, const mjModel *model,
                    transport_t *tp);

/* One simulation tick for this vehicle: sensor sampling -> estimator/
 * localization ingest -> controller update -> apply actuator_command_t to
 * data->ctrl via vehicle->actuator_index. Called once per vehicle per
 * mj_step, from the scenario loop (Step 11), after mj_step itself. */
void vehicle_step(vehicle_t *vehicle, const mjModel *model, mjData *data,
                  double dt);

void vehicle_cleanup(vehicle_t *vehicle);

#endif
