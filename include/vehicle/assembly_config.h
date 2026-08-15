#ifndef MUJOCO_DRONES_VEHICLE_ASSEMBLY_CONFIG_H
#define MUJOCO_DRONES_VEHICLE_ASSEMBLY_CONFIG_H

enum { ASSEMBLY_MAX_NAME_LEN = 64 };

typedef struct {
    char id[ASSEMBLY_MAX_NAME_LEN];              /* e.g. "drone0" -> prefix "drone0_" */
    char airframe[ASSEMBLY_MAX_NAME_LEN];
    char controller[ASSEMBLY_MAX_NAME_LEN];
    char estimator[ASSEMBLY_MAX_NAME_LEN];
    char localization[ASSEMBLY_MAX_NAME_LEN];
    double spawn_pos[3];
    double spawn_yaw;
    double spawn_speed;
} vehicle_assembly_spec_t;

#endif
