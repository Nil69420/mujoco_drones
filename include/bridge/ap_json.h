#ifndef MUJOCO_DRONES_BRIDGE_AP_JSON_H
#define MUJOCO_DRONES_BRIDGE_AP_JSON_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

enum {
    AP_JSON_MAGIC_16 = 18458,
    AP_JSON_MAGIC_32 = 29569,
    AP_JSON_PACKET_16_BYTES = 40,
    AP_JSON_PACKET_32_BYTES = 72,
    AP_JSON_MAX_CHANNELS = 32,
    AP_JSON_MAX_RANGEFINDERS = 6,
    AP_JSON_DEFAULT_PORT = 9002
};

typedef struct {
    uint16_t frame_rate;
    uint32_t frame_count;
    int channels;
    uint16_t pwm[AP_JSON_MAX_CHANNELS];
} ap_servo_frame_t;

typedef struct {
    double timestamp_s;
    double gyro_rps[3];
    double accel_body_mps2[3];
    double position_ned_m[3];
    double velocity_ned_mps[3];
    double quaternion_wxyz[4];
    bool has_rng[AP_JSON_MAX_RANGEFINDERS];
    double rng_m[AP_JSON_MAX_RANGEFINDERS];
    bool has_wind;
    double wind_ned_mps[3];
    bool has_airspeed;
    double airspeed_mps;
    bool has_battery;
    double battery_v;
    double battery_a;
} ap_state_t;

int ap_servo_decode(const uint8_t *buf, size_t len, ap_servo_frame_t *out);

size_t ap_state_encode(const ap_state_t *state, char *out, size_t cap);

#endif
