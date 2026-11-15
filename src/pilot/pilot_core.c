#include "pilot/pilot.h"

#include <linux/input-event-codes.h>
#include <math.h>
#include <string.h>

static const double ARM_HOLD_S = 0.5;
static const double ARM_THROTTLE_MAX = 0.05;
static const double SWITCH_THRESHOLD = 0.5;

static double clamp_unit(double value) {
    return fmin(fmax(value, -1.0), 1.0);
}

void pilot_init(pilot_t *pilot, const pilot_profile_t *profile) {
    memset(pilot, 0, sizeof(*pilot));
    pilot->profile = *profile;
    for (int i = 0; i < PILOT_AXIS_SLOTS; i++) {
        pilot->minimum[i] = -32768;
        pilot->maximum[i] = 32767;
    }
    pilot->connected = true;
}

void pilot_set_axis_range(pilot_t *pilot, uint16_t code, int32_t minimum, int32_t maximum) {
    if (code < PILOT_AXIS_SLOTS && maximum > minimum) {
        pilot->minimum[code] = minimum;
        pilot->maximum[code] = maximum;
        pilot->axis_present[code] = 1U;
        pilot->value[code] = minimum + (maximum - minimum) / 2;
    }
}

void pilot_feed(pilot_t *pilot, uint16_t type, uint16_t code, int32_t value) {
    if (type == EV_ABS && code < PILOT_AXIS_SLOTS) {
        pilot->value[code] = value;
    } else if (type == EV_KEY && code < PILOT_BUTTON_SLOTS) {
        pilot->button[code] = value != 0 ? 1U : 0U;
    }
}

void pilot_set_connected(pilot_t *pilot, bool connected) {
    pilot->connected = connected;
}

static double axis_unit(const pilot_t *pilot, uint16_t code) {
    if (code >= PILOT_AXIS_SLOTS) {
        return 0.0;
    }
    double span = (double)pilot->maximum[code] - (double)pilot->minimum[code];
    if (span <= 0.0) {
        return 0.0;
    }
    return clamp_unit(2.0 * ((double)pilot->value[code] - (double)pilot->minimum[code]) / span - 1.0);
}

static double shape(double value, double deadzone, double expo) {
    double magnitude = fabs(value);
    if (magnitude <= deadzone) {
        return 0.0;
    }
    double scaled = (magnitude - deadzone) / (1.0 - deadzone);
    double curved = scaled * (1.0 - expo) + scaled * scaled * scaled * expo;
    return value < 0.0 ? -curved : curved;
}

static double stick(const pilot_t *pilot, uint16_t code, bool invert) {
    double value = axis_unit(pilot, code);
    return shape(invert ? -value : value, pilot->profile.deadzone, pilot->profile.expo);
}

static double throttle_input(pilot_t *pilot, double dt_s) {
    const pilot_profile_t *profile = &pilot->profile;
    if (profile->throttle_mode == PILOT_THROTTLE_TRIGGER) {
        return (axis_unit(pilot, profile->axis_throttle) + 1.0) * 0.5;
    }
    if (profile->throttle_mode == PILOT_THROTTLE_DIRECT) {
        double value = axis_unit(pilot, profile->axis_throttle);
        return (clamp_unit(profile->invert_throttle ? -value : value) + 1.0) * 0.5;
    }
    double rate = stick(pilot, profile->axis_throttle, profile->invert_throttle);
    pilot->throttle += rate * profile->throttle_rate * dt_s;
    pilot->throttle = fmin(fmax(pilot->throttle, 0.0), 1.0);
    return pilot->throttle;
}

static bool switch_on(const pilot_t *pilot, uint16_t axis) {
    return axis != PILOT_NONE && axis_unit(pilot, axis) > SWITCH_THRESHOLD;
}

static void update_arming(pilot_t *pilot, double dt_s, double throttle) {
    const pilot_profile_t *profile = &pilot->profile;
    if (profile->axis_arm != PILOT_NONE) {
        bool wanted = switch_on(pilot, profile->axis_arm);
        pilot->armed = wanted && (pilot->armed || throttle <= ARM_THROTTLE_MAX);
    } else if (profile->button_arm < PILOT_BUTTON_SLOTS) {
        bool down = pilot->button[profile->button_arm] != 0;
        bool edge = down && !pilot->arm_button_down;
        if (edge && pilot->armed) {
            pilot->armed = false;
            pilot->arm_hold_s = 0.0;
        } else if (edge || !down) {
            pilot->arm_hold_s = 0.0;
        } else if (!pilot->armed && throttle <= ARM_THROTTLE_MAX) {
            pilot->arm_hold_s += dt_s;
            if (pilot->arm_hold_s >= ARM_HOLD_S) {
                pilot->armed = true;
            }
        }
        pilot->arm_button_down = down;
    }
    if (profile->axis_mode != PILOT_NONE) {
        pilot->mode = switch_on(pilot, profile->axis_mode);
    } else if (profile->button_mode < PILOT_BUTTON_SLOTS) {
        bool down = pilot->button[profile->button_mode] != 0;
        if (down && !pilot->mode_button_down) {
            pilot->mode = !pilot->mode;
        }
        pilot->mode_button_down = down;
    }
}

void pilot_update(pilot_t *pilot, double dt_s, pilot_state_t *out) {
    memset(out, 0, sizeof(*out));
    if (!pilot->connected) {
        pilot->armed = false;
        pilot->throttle = 0.0;
        out->failsafe = true;
        return;
    }
    const pilot_profile_t *profile = &pilot->profile;
    out->roll = stick(pilot, profile->axis_roll, profile->invert_roll);
    out->pitch = stick(pilot, profile->axis_pitch, profile->invert_pitch);
    out->yaw = stick(pilot, profile->axis_yaw, profile->invert_yaw);
    out->throttle = throttle_input(pilot, dt_s);
    update_arming(pilot, dt_s, out->throttle);
    out->arm = pilot->armed;
    out->mode = pilot->mode;
    for (int i = 0; i < PILOT_AUX_CHANNELS; i++) {
        out->aux[i] = profile->axis_aux[i] != PILOT_NONE ? axis_unit(pilot, profile->axis_aux[i]) : 0.0;
    }
}

static uint16_t microseconds(double unit) {
    return (uint16_t)lround(1500.0 + 500.0 * clamp_unit(unit));
}

void pilot_to_rc(const pilot_state_t *state, uint16_t channels[PILOT_RC_CHANNELS]) {
    for (int i = 0; i < PILOT_RC_CHANNELS; i++) {
        channels[i] = 1500;
    }
    if (state->failsafe) {
        channels[2] = 1000;
        channels[4] = 1000;
        channels[5] = 1000;
        return;
    }
    channels[0] = microseconds(state->roll);
    channels[1] = microseconds(state->pitch);
    channels[2] = (uint16_t)lround(1000.0 + 1000.0 * fmin(fmax(state->throttle, 0.0), 1.0));
    channels[3] = microseconds(state->yaw);
    channels[4] = state->arm ? 2000 : 1000;
    channels[5] = state->mode ? 2000 : 1000;
    for (int i = 0; i < PILOT_AUX_CHANNELS; i++) {
        channels[6 + i] = microseconds(state->aux[i]);
    }
}

static const double MIN_TRAVEL = 0.4;

int pilot_calibrate_pick(const pilot_t *pilot, const int32_t start[PILOT_AXIS_SLOTS],
                         const int32_t end[PILOT_AXIS_SLOTS], const uint16_t *used, int used_count,
                         uint16_t *code, bool *invert) {
    double best = MIN_TRAVEL;
    int best_code = -1;
    for (int axis = 0; axis < PILOT_AXIS_SLOTS; axis++) {
        double span = (double)pilot->maximum[axis] - (double)pilot->minimum[axis];
        bool taken = false;
        for (int i = 0; i < used_count; i++) {
            taken = taken || used[i] == axis;
        }
        if (taken || span <= 0.0 || pilot->axis_present[axis] == 0U) {
            continue;
        }
        double travel = fabs((double)end[axis] - (double)start[axis]) / span;
        if (travel > best) {
            best = travel;
            best_code = axis;
        }
    }
    if (best_code < 0) {
        return -1;
    }
    *code = (uint16_t)best_code;
    *invert = end[best_code] < start[best_code];
    return 0;
}
