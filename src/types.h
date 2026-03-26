#ifndef MUJOCO_DRONES_TYPES_H
#define MUJOCO_DRONES_TYPES_H

#include <mujoco/mujoco.h>
#include <stdbool.h>

#include "setpoint.h"

#define GRAVITY          9.81
#define TOTAL_MASS       0.716
#define ARM_LENGTH       0.17
#define MOMENT_CONSTANT  0.016
#define MOTOR_KF         8.54858e-6
#define MAX_OMEGA        838.0
#define MAX_THRUST       (MOTOR_KF * MAX_OMEGA * MAX_OMEGA)
#define NUM_ROTORS       4

typedef struct {
    double kp_z,     kd_z,     ki_z;
    double kp_roll,  kd_roll;
    double kp_pitch, kd_pitch;
    double kp_yaw,   kd_yaw;
    double kp_xy,    kd_xy;
} ctrl_gains_t;

typedef struct {
    int       act_thrust[NUM_ROTORS];
    int       act_spin[NUM_ROTORS];
    double    z_integral;
    double    prev_roll;
    double    prev_pitch;
    bool      initialized;
} ctrl_state_t;

#ifdef ENABLE_IPC
#include "transport/transport.h"
#include "sensors/sensors.h"
#endif

typedef struct {
    mjModel*     model;
    mjData*      data;
    ctrl_gains_t gains;
    setpoint_t   target;
    ctrl_state_t ctrl;
#ifdef ENABLE_IPC
    transport_t   transport;
    sensor_mgr_t  sensors;
    bool          ipc_enabled;
#endif
} sim_t;

#endif
