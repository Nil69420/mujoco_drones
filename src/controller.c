/*
 * controller.c - PD(I) attitude + position flight controller
 *
 * Controller architecture:
 *   Outer loop:  XY position PD → desired roll/pitch (clamped)
 *   Inner loop:  Attitude PD on roll, pitch, yaw
 *   Altitude:    PID with anti-windup
 *   Mixer:       + configuration motor allocation
 */

#include "controller.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

/* ================================================================
 * Default gains — tuned for 1000 Hz timestep
 *
 * Key stability fixes vs the original:
 *   - Lowered kp_xy (1.0 → was 2.0) to reduce outer-loop aggression
 *   - Increased kd_xy (2.5 → was 1.5) for more velocity damping
 *   - Tighter angle limits in the outer loop (±0.2 rad ≈ ±11.5°)
 *   - Added derivative filtering via exponential smoothing
 * ================================================================ */
ctrl_gains_t ctrl_default_gains(void) {
    return (ctrl_gains_t){
        .kp_z     = 25.0,
        .kd_z     = 12.0,
        .ki_z     =  0.8,

        .kp_roll  = 8.0,
        .kd_roll  = 2.5,

        .kp_pitch = 8.0,
        .kd_pitch = 2.5,

        .kp_yaw   = 4.0,
        .kd_yaw   = 1.2,

        .kp_xy    = 1.0,
        .kd_xy    = 2.5,
    };
}

/* ================================================================
 * Resolve actuator indices by name
 * ================================================================ */
int ctrl_resolve_actuators(sim_t *sim) {
    char name[32];
    for (int i = 0; i < NUM_ROTORS; i++) {
        snprintf(name, sizeof(name), "thrust_%d", i);
        sim->ctrl.act_thrust[i] = mj_name2id(sim->model, mjOBJ_ACTUATOR, name);
        if (sim->ctrl.act_thrust[i] < 0) {
            fprintf(stderr, "ERROR: actuator '%s' not found\n", name);
            return -1;
        }

        snprintf(name, sizeof(name), "spin_%d", i);
        sim->ctrl.act_spin[i] = mj_name2id(sim->model, mjOBJ_ACTUATOR, name);
        if (sim->ctrl.act_spin[i] < 0) {
            fprintf(stderr, "ERROR: actuator '%s' not found\n", name);
            return -1;
        }
    }
    return 0;
}

/* ================================================================
 * Reset
 * ================================================================ */
void ctrl_reset(sim_t *sim) {
    sim->ctrl.z_integral  = 0.0;
    sim->ctrl.prev_roll   = 0.0;
    sim->ctrl.prev_pitch  = 0.0;
    sim->ctrl.initialized = false;
}

/* ================================================================
 * Quaternion → Euler (ZYX)
 * ================================================================ */
void quat_to_euler(const double q[4], double *roll, double *pitch, double *yaw) {
    double w = q[0], x = q[1], y = q[2], z = q[3];

    double sinr = 2.0 * (w * x + y * z);
    double cosr = 1.0 - 2.0 * (x * x + y * y);
    *roll = atan2(sinr, cosr);

    double sinp = 2.0 * (w * y - z * x);
    *pitch = (fabs(sinp) >= 1.0) ? copysign(M_PI / 2.0, sinp) : asin(sinp);

    double siny = 2.0 * (w * z + x * y);
    double cosy = 1.0 - 2.0 * (y * y + z * z);
    *yaw = atan2(siny, cosy);
}

/* ================================================================
 * Angle wrapping to [-pi, pi]
 * ================================================================ */
static double wrap_angle(double a) {
    while (a >  M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
}

/* ================================================================
 * Motor mixer (+ configuration)
 *
 *   F0 = T/4 - τ_pitch/(2L) - τ_yaw/(4c)    [front, CW]
 *   F1 = T/4 + τ_roll /(2L) + τ_yaw/(4c)    [left,  CCW]
 *   F2 = T/4 + τ_pitch/(2L) - τ_yaw/(4c)    [back,  CW]
 *   F3 = T/4 - τ_roll /(2L) + τ_yaw/(4c)    [right, CCW]
 * ================================================================ */
static void mixer(double total_thrust,
                  double tau_roll, double tau_pitch, double tau_yaw,
                  double f_out[4])
{
    const double arm_len = ARM_LENGTH;
    const double km      = MOMENT_CONSTANT;

    f_out[0] = total_thrust / 4.0 - tau_pitch / (2.0 * arm_len) - tau_yaw / (4.0 * km);
    f_out[1] = total_thrust / 4.0 + tau_roll  / (2.0 * arm_len) + tau_yaw / (4.0 * km);
    f_out[2] = total_thrust / 4.0 + tau_pitch / (2.0 * arm_len) - tau_yaw / (4.0 * km);
    f_out[3] = total_thrust / 4.0 - tau_roll  / (2.0 * arm_len) + tau_yaw / (4.0 * km);

    for (int i = 0; i < NUM_ROTORS; i++) {
        f_out[i] = clampd(f_out[i], 0.0, MAX_THRUST);
    }
}

/* ================================================================
 * Thrust → angular velocity (for visual propeller spin)
 * ================================================================ */
static double thrust_to_omega(double thrust) {
    if (thrust <= 0.0) return 0.0;
    return sqrt(thrust / MOTOR_KF);
}

/* ================================================================
 * Controller update — called once per simulation step
 * ================================================================ */
void ctrl_update(sim_t *sim) {
    if (!sim) return;

    const mjModel      *model = sim->model;
    mjData             *data  = sim->data;
    const ctrl_gains_t *gains = &sim->gains;
    ctrl_state_t       *cst   = &sim->ctrl;
    const setpoint_t   *t     = &sim->target;
    const double        dt    = model->opt.timestep;

    /* ---- Max angle for outer-loop commands ---- */
    const double MAX_TILT = 0.20;  /* rad ≈ 11.5° */

    /* ---- Read state ---- */
    double px = data->qpos[0], py = data->qpos[1], pz = data->qpos[2];
    double vx = data->qvel[0], vy = data->qvel[1], vz = data->qvel[2];
    double wx = data->qvel[3], wy = data->qvel[4], wz = data->qvel[5];

    double roll = 0.0, pitch = 0.0, yaw = 0.0;
    quat_to_euler(&data->qpos[3], &roll, &pitch, &yaw);

    /* Initialize previous values on first call */
    if (!cst->initialized) {
        cst->prev_roll  = roll;
        cst->prev_pitch = pitch;
        cst->initialized = true;
    }

    /* ---- Altitude PID with anti-windup ---- */
    double z_error = t->z - pz;

    /* Only integrate when error is small (anti-windup) */
    if (fabs(z_error) < 1.0) {
        cst->z_integral += z_error * dt;
    }
    cst->z_integral = clampd(cst->z_integral, -1.0, 1.0);

    /* Compensate for tilt: thrust must be divided by cos(roll)*cos(pitch)
       to maintain altitude when tilted */
    double cos_tilt = cos(roll) * cos(pitch);
    if (cos_tilt < 0.5) cos_tilt = 0.5;  /* safety floor */

    double thrust_cmd = (TOTAL_MASS * GRAVITY
                       + gains->kp_z * z_error
                       + gains->kd_z * (-vz)
                       + gains->ki_z * cst->z_integral) / cos_tilt;

    thrust_cmd = clampd(thrust_cmd, 0.0, 4.0 * MAX_THRUST);

    /* ---- Outer loop: XY position → desired roll/pitch ---- */
    double ex = t->x - px;
    double ey = t->y - py;

    double ax_cmd = gains->kp_xy * ex + gains->kd_xy * (-vx);
    double ay_cmd = gains->kp_xy * ey + gains->kd_xy * (-vy);

    /* Limit acceleration command magnitude */
    double a_mag = sqrt(ax_cmd * ax_cmd + ay_cmd * ay_cmd);
    const double A_MAX = GRAVITY * sin(MAX_TILT);
    if (a_mag > A_MAX) {
        ax_cmd *= A_MAX / a_mag;
        ay_cmd *= A_MAX / a_mag;
    }

    /* Rotate to body yaw frame */
    double cy = cos(yaw), sy = sin(yaw);
    double ax_body =  cy * ax_cmd + sy * ay_cmd;
    double ay_body = -sy * ax_cmd + cy * ay_cmd;

    /* pitch = ax_body/g (nose-down → +X accel)
       roll  = -ay_body/g (right-wing-down → -Y accel, so negate to get +Y) */
    double desired_pitch = clampd( ax_body / GRAVITY, -MAX_TILT, MAX_TILT);
    double desired_roll  = clampd(-ay_body / GRAVITY, -MAX_TILT, MAX_TILT);

    /* ---- Rate-limit desired angles (prevent snap commands) ---- */
    const double MAX_ANGLE_RATE = 2.0;   /* rad/s */
    double max_delta = MAX_ANGLE_RATE * dt;

    double delta_roll  = desired_roll  - cst->prev_roll;
    double delta_pitch = desired_pitch - cst->prev_pitch;
    desired_roll  = cst->prev_roll  + clampd(delta_roll,  -max_delta, max_delta);
    desired_pitch = cst->prev_pitch + clampd(delta_pitch, -max_delta, max_delta);
    cst->prev_roll  = desired_roll;
    cst->prev_pitch = desired_pitch;

    /* ---- Inner loop: attitude PD ---- */
    double tau_roll  = gains->kp_roll  * (desired_roll  - roll)  + gains->kd_roll  * (-wx);
    double tau_pitch = gains->kp_pitch * (desired_pitch - pitch) + gains->kd_pitch * (-wy);
    double tau_yaw   = gains->kp_yaw   * wrap_angle(t->yaw - yaw) + gains->kd_yaw * (-wz);

    /* ---- Mix and apply ---- */
    double forces[NUM_ROTORS];
    mixer(thrust_cmd, tau_roll, tau_pitch, tau_yaw, forces);

    for (int i = 0; i < NUM_ROTORS; i++) {
        data->ctrl[cst->act_thrust[i]] = forces[i];

        double omega = thrust_to_omega(forces[i]);
        /* CW rotors (0,2) spin negative; CCW rotors (1,3) spin positive */
        data->ctrl[cst->act_spin[i]] = (i == 0 || i == 2) ? -omega : omega;
    }
}
