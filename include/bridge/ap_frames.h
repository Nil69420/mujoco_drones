#ifndef MUJOCO_DRONES_BRIDGE_AP_FRAMES_H
#define MUJOCO_DRONES_BRIDGE_AP_FRAMES_H

#include "bridge/ap_json.h"

typedef struct {
    double time_s;
    double position_enu_m[3];
    double origin_enu_m[3];
    double rotation_body_to_world[9];
    double velocity_enu_mps[3];
    double gyro_flu_rps[3];
    double accel_flu_mps2[3];
} ap_truth_t;

void ap_truth_to_state(const ap_truth_t *truth, ap_state_t *out);

void ap_state_overlay_imu(ap_state_t *state, const double gyro_flu_rps[3],
                          const double accel_flu_mps2[3]);

void ap_rotation_to_quaternion(const double rotation[9], double quat_wxyz[4]);

#endif
