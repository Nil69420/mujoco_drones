#include "bridge/ap_map.h"

#include <math.h>
#include <string.h>

enum { QUAD_MOTORS = 4, QUAD_CHANNELS = 8 };

ap_map_t ap_map_quad_plus(void) {
    ap_map_t map;
    memset(&map, 0, sizeof(map));
    map.motor_count = QUAD_MOTORS;
    map.channel_count = QUAD_CHANNELS;
    map.max_thrust_n = 6.0;
    map.motor_constant = 8.54858e-06;
    map.thrust_expo = 0.65;
    map.pwm_min = 1000;
    map.pwm_max = 2000;
    static const int rotor_for_servo[QUAD_MOTORS] = {3, 1, 0, 2};
    static const double spin_sign[QUAD_MOTORS] = {1.0, 1.0, -1.0, -1.0};
    for (int i = 0; i < QUAD_MOTORS; i++) {
        map.motor[i].servo_channel = i;
        map.motor[i].thrust_channel = rotor_for_servo[i];
        map.motor[i].spin_channel = QUAD_MOTORS + rotor_for_servo[i];
        map.motor[i].spin_sign = spin_sign[i];
    }
    return map;
}

ap_map_t ap_map_quad_direct(void) {
    ap_map_t map = ap_map_quad_plus();
    static const double spin_sign[QUAD_MOTORS] = {-1.0, 1.0, -1.0, 1.0};
    map.thrust_expo = 0.0;
    for (int i = 0; i < QUAD_MOTORS; i++) {
        map.motor[i].servo_channel = i;
        map.motor[i].thrust_channel = i;
        map.motor[i].spin_channel = QUAD_MOTORS + i;
        map.motor[i].spin_sign = spin_sign[i];
    }
    return map;
}

int ap_map_validate(const ap_map_t *map) {
    if (map == NULL || map->motor_count < 1 || map->motor_count > AP_MAP_MAX_MOTORS ||
        map->channel_count < 1 || map->channel_count > VEHICLE_MAX_ACTUATOR_CHANNELS ||
        !(map->max_thrust_n > 0.0) || !(map->motor_constant > 0.0) || map->thrust_expo < 0.0 ||
        map->thrust_expo > 1.0 || map->pwm_max <= map->pwm_min) {
        return -1;
    }
    for (int i = 0; i < map->motor_count; i++) {
        const ap_motor_t *motor = &map->motor[i];
        if (motor->servo_channel < 0 || motor->servo_channel >= AP_JSON_MAX_CHANNELS ||
            motor->thrust_channel < 0 || motor->thrust_channel >= map->channel_count ||
            motor->spin_channel >= map->channel_count) {
            return -1;
        }
    }
    return 0;
}

double ap_map_thrust_from_fraction(const ap_map_t *map, double fraction) {
    double command = isfinite(fraction) ? fmin(1.0, fmax(0.0, fraction)) : 0.0;
    double frac = (1.0 - map->thrust_expo) * command + map->thrust_expo * command * command;
    return map->max_thrust_n * frac;
}

double ap_map_thrust(const ap_map_t *map, int pwm) {
    double span = (double)(map->pwm_max - map->pwm_min);
    return ap_map_thrust_from_fraction(map, ((double)pwm - (double)map->pwm_min) / span);
}

void ap_map_apply(const ap_map_t *map, const ap_servo_frame_t *frame, actuator_command_t *out) {
    memset(out, 0, sizeof(*out));
    out->count = map->channel_count;
    for (int i = 0; i < map->motor_count; i++) {
        const ap_motor_t *motor = &map->motor[i];
        double thrust = 0.0;
        if (motor->servo_channel < frame->channels) {
            thrust = ap_map_thrust(map, frame->pwm[motor->servo_channel]);
        }
        out->value[motor->thrust_channel] = thrust;
        if (motor->spin_channel >= 0) {
            out->value[motor->spin_channel] = motor->spin_sign * sqrt(thrust / map->motor_constant);
        }
    }
}

void ap_map_apply_fractions(const ap_map_t *map, const float *fractions, int count,
                            actuator_command_t *out) {
    memset(out, 0, sizeof(*out));
    out->count = map->channel_count;
    for (int i = 0; i < map->motor_count; i++) {
        const ap_motor_t *motor = &map->motor[i];
        double thrust = 0.0;
        if (motor->servo_channel < count) {
            thrust = ap_map_thrust_from_fraction(map, (double)fractions[motor->servo_channel]);
        }
        out->value[motor->thrust_channel] = thrust;
        if (motor->spin_channel >= 0) {
            out->value[motor->spin_channel] = motor->spin_sign * sqrt(thrust / map->motor_constant);
        }
    }
}
