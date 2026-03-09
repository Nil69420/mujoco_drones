/*
 * types.h - Shared types and constants for mujoco_drones
 */
#ifndef MUJOCO_DRONES_TYPES_H
#define MUJOCO_DRONES_TYPES_H

#include <mujoco/mujoco.h>
#include <stdbool.h>

/* ================================================================
 * Physical constants (from hummingbird.xacro / hummingbird.yaml)
 * ================================================================ */
#define GRAVITY          9.81
#define TOTAL_MASS       0.716       /* kg (body + 4 rotors)              */
#define ARM_LENGTH       0.17        /* m                                 */
#define MOMENT_CONSTANT  0.016       /* m   torque = km * thrust          */
#define MOTOR_KF         8.54858e-6  /* kg*m/s^2  thrust = kf * omega^2  */
#define MAX_OMEGA        838.0       /* rad/s                             */
#define MAX_THRUST       (MOTOR_KF * MAX_OMEGA * MAX_OMEGA)  /* ~6.0 N   */
#define NUM_ROTORS       4

/* ================================================================
 * Controller gains
 * ================================================================ */
typedef struct {
    double kp_z,     kd_z,     ki_z;
    double kp_roll,  kd_roll;
    double kp_pitch, kd_pitch;
    double kp_yaw,   kd_yaw;
    double kp_xy,    kd_xy;
} ctrl_gains_t;

/* ================================================================
 * Setpoint
 * ================================================================ */
typedef struct {
    double x, y, z;
    double yaw;
} setpoint_t;

/* ================================================================
 * Controller state (integrators, previous errors, etc.)
 * ================================================================ */
typedef struct {
    int       act_thrust[NUM_ROTORS];
    int       act_spin[NUM_ROTORS];
    double    z_integral;
    double    prev_roll;
    double    prev_pitch;
    bool      initialized;
} ctrl_state_t;

/* ================================================================
 * Simulation context (bundles everything non-viewer)
 * ================================================================ */
typedef struct {
    mjModel*     model;
    mjData*      data;
    ctrl_gains_t gains;
    setpoint_t   target;
    ctrl_state_t ctrl;
} sim_t;

#endif /* MUJOCO_DRONES_TYPES_H */
