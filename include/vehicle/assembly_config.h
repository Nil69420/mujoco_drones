#ifndef MUJOCO_DRONES_VEHICLE_ASSEMBLY_CONFIG_H
#define MUJOCO_DRONES_VEHICLE_ASSEMBLY_CONFIG_H

#include "sensors/models/airframe_effects.h"

enum { ASSEMBLY_MAX_NAME_LEN = 64, ASSEMBLY_FX_ALL = 0xF };

typedef struct {
    char id[ASSEMBLY_MAX_NAME_LEN];              /* e.g. "drone0" -> prefix "drone0_" */
    char airframe[ASSEMBLY_MAX_NAME_LEN];
    char controller[ASSEMBLY_MAX_NAME_LEN];
    char estimator[ASSEMBLY_MAX_NAME_LEN];
    char localization[ASSEMBLY_MAX_NAME_LEN];
    double spawn_pos[3];
    double spawn_yaw;
    double spawn_speed;
    char imu[ASSEMBLY_MAX_NAME_LEN];
    char gnss[ASSEMBLY_MAX_NAME_LEN];
    char baro[ASSEMBLY_MAX_NAME_LEN];
    char mag[ASSEMBLY_MAX_NAME_LEN];
    char rangefinder[ASSEMBLY_MAX_NAME_LEN];
    char bridge[ASSEMBLY_MAX_NAME_LEN];
    double bridge_port;
    double bridge_speed;
    double vibration;
    double target_pos[3];
    int has_target;
    int gnss_cold_start;
    int gnss_rtk;
    double drag_area;
    airframe_effects_params_t effects;
    unsigned effects_mask;
} vehicle_assembly_spec_t;

#endif
