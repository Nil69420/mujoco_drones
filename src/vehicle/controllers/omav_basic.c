#include "vehicle/controller_ops.h"
#include "../../../include/controller.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// Minimal thrust-vectoring hover controller for the imported OMAV.
// Channels (from the importer's [channels] order):
//   [0..5]   thrust per coax arm (site lift on the top rotor)
//   [6..11]  rotor spin (velocity servos, yaw damping)
//   [12..17] tilt arm position servos (held at 0, i.e. arms level)
// Reuses pd_multicopter's hover/attitude math with 6 rotors at 60 deg.

#define OM_GRAVITY 9.81
#define OM_ARM_LENGTH 0.3
#define OM_MOMENT_CONSTANT 0.016
#define OM_MAX_THRUST 20.0
#define OM_MAX_TILT 0.20
#define OM_MAX_ANGLE_RATE 2.0
#define OM_MAX_ROTORS 8

extern const controller_ops_t omav_basic_ops;

typedef struct {
    ctrl_gains_t gains;
    double z_integral;
    double prev_roll;
    double prev_pitch;
    bool initialized;
    int rotor_count;
    int total_channels;
    double rotor_angle[OM_MAX_ROTORS];
    double rotor_spin[OM_MAX_ROTORS];
} omav_basic_state_t;

static int om_init(controller_t *ctrl, int num_channels) {
    if (!ctrl || num_channels < 12 || num_channels > 18 || (num_channels % 3) != 0) {
        return -1;
    }
    omav_basic_state_t *st = calloc(1, sizeof(*st));
    if (!st) {
        return -1;
    }
    st->gains = ctrl_default_gains();
    // The OMAV is heavier and larger than the quad_x the default gains
    // were tuned for; scale the attitude gains up accordingly.
    st->gains.kp_roll *= 3.0;
    st->gains.kd_roll *= 2.0;
    st->gains.kp_pitch *= 3.0;
    st->gains.kd_pitch *= 2.0;
    st->gains.kp_yaw *= 3.0;
    st->gains.kd_yaw *= 2.0;
    st->rotor_count = num_channels / 3;
    st->total_channels = num_channels;
    for (int i = 0; i < st->rotor_count; i++) {
        st->rotor_angle[i] = 2.0 * M_PI * (double)i / (double)st->rotor_count;
        st->rotor_spin[i] = (i % 2 == 0) ? -1.0 : 1.0;
    }
    ctrl->ops = &omav_basic_ops;
    ctrl->ctx = st;
    return 0;
}

static void om_update(controller_t *ctrl, const estimated_state_t *state,
                      const setpoint_t *target, double dt, actuator_command_t *out) {
    omav_basic_state_t *st = ctrl->ctx;
    if (!st || !state || !target || !out) {
        return;
    }

    const double *pxyz = state->pos;
    const double *vxyz = state->vel;
    const double *wxyz = state->angvel;

    double roll = 0.0, pitch = 0.0, yaw = 0.0;
    quat_to_euler(state->quat, &roll, &pitch, &yaw);

    if (!st->initialized) {
        st->prev_roll = roll;
        st->prev_pitch = pitch;
        st->initialized = true;
    }

    double z_error = target->z - pxyz[2];
    if (fabs(z_error) < 1.0) {
        st->z_integral += z_error * dt;
    }
    st->z_integral = clampd(st->z_integral, -1.0, 1.0);

    double cos_tilt = cos(roll) * cos(pitch);
    if (cos_tilt < 0.5) {
        cos_tilt = 0.5;
    }

    double total_mass = 1.6;  // OMAV authored mass (base 1.55 + arms + rotors)
    double thrust_cmd =
        (total_mass * OM_GRAVITY + st->gains.kp_z * z_error + st->gains.kd_z * (-vxyz[2]) +
         st->gains.ki_z * st->z_integral) /
        cos_tilt;
    thrust_cmd = clampd(thrust_cmd, 0.0, 6.0 * OM_MAX_THRUST);

    double ex = target->x - pxyz[0];
    double ey = target->y - pxyz[1];
    double ax_cmd = st->gains.kp_xy * ex + st->gains.kd_xy * (-vxyz[0]);
    double ay_cmd = st->gains.kp_xy * ey + st->gains.kd_xy * (-vxyz[1]);
    double a_mag = sqrt(ax_cmd * ax_cmd + ay_cmd * ay_cmd);
    const double A_MAX = OM_GRAVITY * sin(OM_MAX_TILT);
    if (a_mag > A_MAX) {
        ax_cmd *= A_MAX / a_mag;
        ay_cmd *= A_MAX / a_mag;
    }

    double cy = cos(yaw), sy = sin(yaw);
    double desired_pitch = clampd((cy * ax_cmd + sy * ay_cmd) / OM_GRAVITY, -OM_MAX_TILT,
                                  OM_MAX_TILT);
    double desired_roll =
        clampd(-(-sy * ax_cmd + cy * ay_cmd) / OM_GRAVITY, -OM_MAX_TILT, OM_MAX_TILT);

    double max_delta = OM_MAX_ANGLE_RATE * dt;
    desired_roll = st->prev_roll + clampd(desired_roll - st->prev_roll, -max_delta, max_delta);
    desired_pitch =
        st->prev_pitch + clampd(desired_pitch - st->prev_pitch, -max_delta, max_delta);
    st->prev_roll = desired_roll;
    st->prev_pitch = desired_pitch;

    double tau_roll = st->gains.kp_roll * (desired_roll - roll) + st->gains.kd_roll * (-wxyz[0]);
    double tau_pitch =
        st->gains.kp_pitch * (desired_pitch - pitch) + st->gains.kd_pitch * (-wxyz[1]);
    double yaw_err = target->yaw - yaw;
    while (yaw_err > M_PI) {
        yaw_err -= 2.0 * M_PI;
    }
    while (yaw_err < -M_PI) {
        yaw_err += 2.0 * M_PI;
    }
    double tau_yaw = st->gains.kp_yaw * yaw_err + st->gains.kd_yaw * (-wxyz[2]);

    int n = st->rotor_count;
    for (int i = 0; i < n; i++) {
        double f = thrust_cmd / (double)n +
                   (tau_roll * sin(st->rotor_angle[i]) -
                    tau_pitch * cos(st->rotor_angle[i])) /
                       (2.0 * OM_ARM_LENGTH) +
                   tau_yaw * st->rotor_spin[i] / (4.0 * OM_MOMENT_CONSTANT);
        out->value[i] = clampd(f, 0.0, OM_MAX_THRUST);
        out->value[n + i] = st->rotor_spin[i] * sqrt(out->value[i] / 8.54858e-6);
    }
    for (int i = 0; i < n; i++) {
        out->value[2 * n + i] = 0.0;  // tilt arms held level
    }
    out->count = st->total_channels;
    for (int i = 0; i < out->count; i++) {
        if (!isfinite(out->value[i])) {
            fprintf(stderr, "OMAV-DBG: NaN at channel %d state pos=%g %g %g quat=%g %g %g %g "
                            "thrust_cmd=%g f0=%g omega0=%g\n",
                    i, pxyz[0], pxyz[1], pxyz[2], state->quat[0], state->quat[1],
                    state->quat[2], state->quat[3], thrust_cmd, out->value[0],
                    out->value[n + 0]);
            break;
        }
    }
}

static void om_reset(controller_t *ctrl) {
    omav_basic_state_t *st = ctrl->ctx;
    if (!st) {
        return;
    }
    st->z_integral = 0.0;
    st->prev_roll = 0.0;
    st->prev_pitch = 0.0;
    st->initialized = false;
}

static void om_destroy(controller_t *ctrl) {
    if (!ctrl) {
        return;
    }
    free(ctrl->ctx);
    ctrl->ctx = NULL;
}

const controller_ops_t omav_basic_ops = {
    .name = "omav_basic",
    .airframe_class = "rotor_multicopter",
    .init = om_init,
    .update = om_update,
    .reset = om_reset,
    .destroy = om_destroy,
};
