#ifndef MUJOCO_DRONES_VEHICLE_ACTUATOR_H
#define MUJOCO_DRONES_VEHICLE_ACTUATOR_H

enum { VEHICLE_MAX_ACTUATOR_CHANNELS = 16 };

typedef struct {
    int    count;
    double value[VEHICLE_MAX_ACTUATOR_CHANNELS];
} actuator_command_t;

#endif
