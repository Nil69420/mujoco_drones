#include "vehicle/controller_ops.h"
#include "controller.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

#define PD_GRAVITY          9.81
#define PD_TOTAL_MASS       0.716
#define PD_ARM_LENGTH       0.17
#define PD_MOMENT_CONSTANT  0.016
#define PD_MOTOR_KF         8.54858e-6
#define PD_MAX_OMEGA        838.0
#define PD_MAX_THRUST       (PD_MOTOR_KF * PD_MAX_OMEGA * PD_MAX_OMEGA)

#define PD_MAX_TILT         0.20
#define PD_MAX_ANGLE_RATE   2.0
#define PD_MAX_ROTORS       8

extern const controller_ops_t pd_multicopter_ops;

typedef struct {
    ctrl_gains_t gains;
    double       z_integral;
    double       prev_roll;
    double       prev_pitch;
    bool         initialized;
    int          rotor_count;
    double       rotor_angle[PD_MAX_ROTORS];
    double       rotor_spin[PD_MAX_ROTORS];
} pd_multicopter_state_t;

static double pd_thrust_to_omega(double thrust) {
    if (thrust <= 0.0) return 0.0;
    return sqrt(thrust / PD_MOTOR_KF);
}

static int pd_init(controller_t *ctrl, int num_channels) {
    if (!ctrl || num_channels < 4 || num_channels > 2 * PD_MAX_ROTORS ||
        (num_channels % 2) != 0) {
        return -1;
    }

    pd_multicopter_state_t *st = calloc(1, sizeof(*st));
    if (!st) return -1;

    st->gains = ctrl_default_gains();
    st->rotor_count = num_channels / 2;

    for (int i = 0; i < st->rotor_count; i++) {
        st->rotor_angle[i] = 2.0 * M_PI * (double)i / (double)st->rotor_count;
        st->rotor_spin[i]  = (i % 2 == 0) ? -1.0 : 1.0;
    }

    ctrl->ops = &pd_multicopter_ops;
    ctrl->ctx = st;
    return 0;
}

static void pd_update(controller_t *ctrl, const estimated_state_t *state,
                      const setpoint_t *target, double dt,
                      actuator_command_t *out) {
    pd_multicopter_state_t *st = ctrl->ctx;
    if (!st || !state || !target || !out) return;

    const double *pxyz = state->pos;
    const double *vxyz = state->vel;
    const double *wxyz = state->angvel;

    double roll = 0.0, pitch = 0.0, yaw = 0.0;
    quat_to_euler(state->quat, &roll, &pitch, &yaw);

    if (!st->initialized) {
        st->prev_roll  = roll;
        st->prev_pitch = pitch;
        st->initialized = true;
    }

    double z_error = target->z - pxyz[2];
    if (fabs(z_error) < 1.0) {
        st->z_integral += z_error * dt;
    }
    st->z_integral = clampd(st->z_integral, -1.0, 1.0);

    double cos_tilt = cos(roll) * cos(pitch);
    if (cos_tilt < 0.5) cos_tilt = 0.5;

    double thrust_cmd = (PD_TOTAL_MASS * PD_GRAVITY
                       + st->gains.kp_z * z_error
                       + st->gains.kd_z * (-vxyz[2])
                       + st->gains.ki_z * st->z_integral) / cos_tilt;
    thrust_cmd = clampd(thrust_cmd, 0.0, 4.0 * PD_MAX_THRUST);

    double ex = target->x - pxyz[0];
    double ey = target->y - pxyz[1];

    double ax_cmd = st->gains.kp_xy * ex + st->gains.kd_xy * (-vxyz[0]);
    double ay_cmd = st->gains.kp_xy * ey + st->gains.kd_xy * (-vxyz[1]);

    double a_mag = sqrt(ax_cmd * ax_cmd + ay_cmd * ay_cmd);
    const double A_MAX = PD_GRAVITY * sin(PD_MAX_TILT);
    if (a_mag > A_MAX) {
        ax_cmd *= A_MAX / a_mag;
        ay_cmd *= A_MAX / a_mag;
    }

    double cy = cos(yaw), sy = sin(yaw);
    double ax_body =  cy * ax_cmd + sy * ay_cmd;
    double ay_body = -sy * ax_cmd + cy * ay_cmd;

    double desired_pitch = clampd( ax_body / PD_GRAVITY, -PD_MAX_TILT, PD_MAX_TILT);
    double desired_roll  = clampd(-ay_body / PD_GRAVITY, -PD_MAX_TILT, PD_MAX_TILT);

    double max_delta = PD_MAX_ANGLE_RATE * dt;
    double delta_roll  = desired_roll  - st->prev_roll;
    double delta_pitch = desired_pitch - st->prev_pitch;
    desired_roll  = st->prev_roll  + clampd(delta_roll,  -max_delta, max_delta);
    desired_pitch = st->prev_pitch + clampd(delta_pitch, -max_delta, max_delta);
    st->prev_roll  = desired_roll;
    st->prev_pitch = desired_pitch;

    double tau_roll  = st->gains.kp_roll  * (desired_roll  - roll)  + st->gains.kd_roll  * (-wxyz[0]);
    double tau_pitch = st->gains.kp_pitch * (desired_pitch - pitch) + st->gains.kd_pitch * (-wxyz[1]);

    double yaw_err = target->yaw - yaw;
    while (yaw_err >  M_PI) yaw_err -= 2.0 * M_PI;
    while (yaw_err < -M_PI) yaw_err += 2.0 * M_PI;
    double tau_yaw = st->gains.kp_yaw * yaw_err + st->gains.kd_yaw * (-wxyz[2]);

    int n = st->rotor_count;
    for (int i = 0; i < n; i++) {
        double f = thrust_cmd / (double)n
                 + (tau_roll * sin(st->rotor_angle[i])
                    - tau_pitch * cos(st->rotor_angle[i])) / (2.0 * PD_ARM_LENGTH)
                 + tau_yaw * st->rotor_spin[i] / (4.0 * PD_MOMENT_CONSTANT);
        out->value[i] = clampd(f, 0.0, PD_MAX_THRUST);

        double omega = pd_thrust_to_omega(out->value[i]);
        out->value[n + i] = st->rotor_spin[i] * omega;
    }
    out->count = 2 * n;
}

static void pd_reset(controller_t *ctrl) {
    pd_multicopter_state_t *st = ctrl->ctx;
    if (!st) return;
    st->z_integral  = 0.0;
    st->prev_roll   = 0.0;
    st->prev_pitch  = 0.0;
    st->initialized = false;
}

static void pd_destroy(controller_t *ctrl) {
    if (!ctrl) return;
    free(ctrl->ctx);
    ctrl->ctx = NULL;
}

const controller_ops_t pd_multicopter_ops = {
    .name           = "pd_multicopter",
    .airframe_class = "rotor_multicopter",
    .init           = pd_init,
    .update         = pd_update,
    .reset          = pd_reset,
    .destroy        = pd_destroy,
};
