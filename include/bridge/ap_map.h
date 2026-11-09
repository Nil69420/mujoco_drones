#ifndef MUJOCO_DRONES_BRIDGE_AP_MAP_H
#define MUJOCO_DRONES_BRIDGE_AP_MAP_H

#include "bridge/ap_json.h"
#include "vehicle/actuator.h"

enum { AP_MAP_MAX_MOTORS = 12 };

typedef struct {
    int servo_channel;
    int thrust_channel;
    int spin_channel;
    double spin_sign;
} ap_motor_t;

typedef struct {
    int motor_count;
    ap_motor_t motor[AP_MAP_MAX_MOTORS];
    int channel_count;
    double max_thrust_n;
    double motor_constant;
    double thrust_expo;
    int pwm_min;
    int pwm_max;
} ap_map_t;

ap_map_t ap_map_quad_plus(void);

ap_map_t ap_map_quad_direct(void);

int ap_map_validate(const ap_map_t *map);

double ap_map_thrust(const ap_map_t *map, int pwm);

double ap_map_thrust_from_fraction(const ap_map_t *map, double fraction);

void ap_map_apply(const ap_map_t *map, const ap_servo_frame_t *frame, actuator_command_t *out);

void ap_map_apply_fractions(const ap_map_t *map, const float *fractions, int count,
                            actuator_command_t *out);

#endif
