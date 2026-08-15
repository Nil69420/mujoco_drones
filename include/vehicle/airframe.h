#ifndef MUJOCO_DRONES_VEHICLE_AIRFRAME_H
#define MUJOCO_DRONES_VEHICLE_AIRFRAME_H

#include "vehicle/actuator.h"

typedef struct {
    const char *name;              /* e.g. "quad_x" */
    const char *airframe_class;    /* e.g. "rotor_multicopter", "fixed_wing_conventional" */
    const char *mjcf_path;         /* relative to model/airframes/ */
    const char *channel_names[VEHICLE_MAX_ACTUATOR_CHANNELS]; /* UNPREFIXED MJCF actuator names */
    int channel_count;
    const char *imu_site;          /* UNPREFIXED site name, required */
    const char *lidar_site;        /* UNPREFIXED, nullable (NULL = no lidar on this airframe) */
    const char *infrared_site;     /* UNPREFIXED, nullable */
    const char *camera_name;       /* UNPREFIXED, nullable */
    const char *base_body;         /* UNPREFIXED body name for pose/attitude reads */
} airframe_spec_t;

const airframe_spec_t *airframe_registry_find(const char *name);
int airframe_registry_count(void);
const airframe_spec_t *airframe_registry_at(int index);

#endif
