#include "vehicle/controller_ops.h"
#include "controller.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

#define FW_GRAVITY       9.81
#define FW_MAX_TILT      0.45
#define FW_CRUISE_THROTTLE 0.60
#define FW_CRUISE_SPEED  12.0
#define FW_THROTTLE_KP   0.02

typedef struct {
    double kp_alt;
    double kd_alt;
    double kp_pitch;
    double kd_pitch_rate;
    double kp_yaw;
    double kd_yaw;
    double kp_roll;
    double kd_roll;
    double kt_alt;
} fixedwing_gains_t;

static fixedwing_gains_t fw_default_gains(void) {
    return (fixedwing_gains_t){
        .kp_alt = 0.40,
        .kd_alt = 0.15,
        .kp_pitch = 3.0,
        .kd_pitch_rate = 1.2,
        .kp_yaw = 0.6,
        .kd_yaw = 0.3,
        .kp_roll = 0.8,
        .kd_roll = 0.3,
        .kt_alt = 0.06,
    };
}

extern const controller_ops_t fixedwing_basic_ops;

static int fw_init(controller_t *ctrl, int num_channels) {
    if (!ctrl || num_channels != 5) return -1;

    fixedwing_gains_t *gains = malloc(sizeof(*gains));
    if (!gains) return -1;
    *gains = fw_default_gains();

    ctrl->ops = &fixedwing_basic_ops;
    ctrl->ctx = gains;
    return 0;
}

static void fw_update(controller_t *ctrl, const estimated_state_t *state,
                      const setpoint_t *target, double dt,
                      actuator_command_t *out) {
    fixedwing_gains_t *gains = ctrl->ctx;
    if (!gains || !state || !target || !out) return;
    (void)dt;

    const double *pos = state->pos;
    const double *vel = state->vel;
    const double *angvel = state->angvel;

    double roll = 0.0, pitch = 0.0, yaw = 0.0;
    quat_to_euler(state->quat, &roll, &pitch, &yaw);

    double alt_err = target->z - pos[2];

    double yaw_err = target->yaw - yaw;
    while (yaw_err >  M_PI) yaw_err -= 2.0 * M_PI;
    while (yaw_err < -M_PI) yaw_err += 2.0 * M_PI;

    double pitch_cmd = gains->kp_alt * alt_err + gains->kd_alt * (-vel[2]);
    pitch_cmd = clampd(pitch_cmd, -0.15, 0.20);
    double elevator = gains->kp_pitch * (pitch_cmd - pitch)
                    - gains->kd_pitch_rate * angvel[1];
    elevator = clampd(elevator, -0.4, 0.4);

    double rudder = -(gains->kp_yaw * yaw_err + gains->kd_yaw * (-angvel[2]));
    rudder = clampd(rudder, -0.5, 0.5);

    double aileron = -(gains->kp_roll * roll + gains->kd_roll * (-angvel[0]));
    aileron = clampd(aileron, -0.5, 0.5);

    double fwd_speed = vel[0] * cos(yaw) + vel[1] * sin(yaw);
    double throttle = FW_CRUISE_THROTTLE
                    + FW_THROTTLE_KP * (FW_CRUISE_SPEED - fwd_speed)
                    + gains->kt_alt * alt_err;
    throttle = clampd(throttle, 0.2, 1.0);

    out->value[0] =  aileron;
    out->value[1] = -aileron;
    out->value[2] = elevator;
    out->value[3] = rudder;
    out->value[4] = throttle;
    out->count = 5;
}

static void fw_reset(controller_t *ctrl) {
    (void)ctrl;
}

static void fw_destroy(controller_t *ctrl) {
    if (!ctrl) return;
    free(ctrl->ctx);
    ctrl->ctx = NULL;
}

const controller_ops_t fixedwing_basic_ops = {
    .name           = "fixedwing_basic",
    .airframe_class = "fixed_wing_conventional",
    .init           = fw_init,
    .update         = fw_update,
    .reset          = fw_reset,
    .destroy        = fw_destroy,
};
