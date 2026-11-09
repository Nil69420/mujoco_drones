#ifndef MUJOCO_DRONES_PILOT_PILOT_H
#define MUJOCO_DRONES_PILOT_PILOT_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

enum {
    PILOT_AXIS_SLOTS = 64,
    PILOT_BUTTON_SLOTS = 768,
    PILOT_BUTTON_BASE = 0,
    PILOT_AUX_CHANNELS = 4,
    PILOT_RC_CHANNELS = 16,
    PILOT_NONE = 0xFFFF,
    PILOT_NAME_BYTES = 64
};

typedef enum {
    PILOT_THROTTLE_DIRECT = 0,
    PILOT_THROTTLE_INTEGRATE = 1,
    PILOT_THROTTLE_TRIGGER = 2
} pilot_throttle_mode_t;

typedef struct {
    char name[PILOT_NAME_BYTES];
    char name_hint[PILOT_NAME_BYTES];
    uint16_t vendor;
    uint16_t product;
    bool verified;
    uint16_t axis_roll;
    uint16_t axis_pitch;
    uint16_t axis_yaw;
    uint16_t axis_throttle;
    bool invert_roll;
    bool invert_pitch;
    bool invert_yaw;
    bool invert_throttle;
    pilot_throttle_mode_t throttle_mode;
    double deadzone;
    double expo;
    double throttle_rate;
    uint16_t button_arm;
    uint16_t button_mode;
    uint16_t axis_arm;
    uint16_t axis_mode;
    uint16_t axis_aux[PILOT_AUX_CHANNELS];
} pilot_profile_t;

typedef struct {
    double roll;
    double pitch;
    double yaw;
    double throttle;
    double aux[PILOT_AUX_CHANNELS];
    bool arm;
    bool mode;
    bool failsafe;
} pilot_state_t;

typedef struct {
    pilot_profile_t profile;
    int32_t value[PILOT_AXIS_SLOTS];
    int32_t minimum[PILOT_AXIS_SLOTS];
    int32_t maximum[PILOT_AXIS_SLOTS];
    uint8_t button[PILOT_BUTTON_SLOTS];
    uint8_t axis_present[PILOT_AXIS_SLOTS];
    bool connected;
    bool armed;
    bool mode;
    bool arm_button_down;
    bool mode_button_down;
    double arm_hold_s;
    double throttle;
} pilot_t;

int pilot_calibrate_pick(const pilot_t *pilot, const int32_t start[PILOT_AXIS_SLOTS],
                         const int32_t end[PILOT_AXIS_SLOTS], const uint16_t *used, int used_count,
                         uint16_t *code, bool *invert);

bool pilot_profile_for_device(uint16_t vendor, uint16_t product, const char *name, pilot_profile_t *out);

void pilot_profile_generic(pilot_profile_t *out);

int pilot_profile_count(void);

bool pilot_profile_at(int index, pilot_profile_t *out);

int pilot_profile_parse(const char *text, pilot_profile_t *out, char *error, size_t error_len);

int pilot_profile_format(const pilot_profile_t *profile, char *out, size_t cap);

void pilot_init(pilot_t *pilot, const pilot_profile_t *profile);

void pilot_set_axis_range(pilot_t *pilot, uint16_t code, int32_t minimum, int32_t maximum);

void pilot_feed(pilot_t *pilot, uint16_t type, uint16_t code, int32_t value);

void pilot_set_connected(pilot_t *pilot, bool connected);

void pilot_update(pilot_t *pilot, double dt_s, pilot_state_t *out);

void pilot_to_rc(const pilot_state_t *state, uint16_t channels[PILOT_RC_CHANNELS]);

int pilot_axis_code(const char *name);

int pilot_button_code(const char *name);

#endif
