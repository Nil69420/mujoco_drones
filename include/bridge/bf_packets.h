#ifndef MUJOCO_DRONES_BRIDGE_BF_PACKETS_H
#define MUJOCO_DRONES_BRIDGE_BF_PACKETS_H

#include "bridge/ap_frames.h"
#include "bridge/px4_sensors.h"

#include <stddef.h>
#include <stdint.h>

enum {
    BF_FDM_BYTES = 144,
    BF_RC_BYTES = 40,
    BF_MOTOR_BYTES = 16,
    BF_RC_CHANNELS = 16,
    BF_MOTOR_COUNT = 4,
    BF_PORT_MOTORS = 9002,
    BF_PORT_FDM = 9003,
    BF_PORT_RC = 9004
};

typedef struct {
    double timestamp_s;
    double gyro_frd_rps[3];
    double accel_frd_mps2[3];
    double quat_wxyz[4];
    double velocity_enu_mps[3];
    double lon_deg;
    double lat_deg;
    double alt_m;
    double pressure_pa;
} bf_fdm_t;

void bf_fdm_from_truth(const ap_truth_t *truth, const px4_home_t *home, bf_fdm_t *out);

size_t bf_fdm_encode(const bf_fdm_t *fdm, uint8_t *out, size_t cap);

size_t bf_rc_encode(double timestamp_s, const uint16_t channels[BF_RC_CHANNELS], uint8_t *out,
                    size_t cap);

int bf_motors_decode(const uint8_t *data, size_t len, float motors[BF_MOTOR_COUNT]);

#endif
