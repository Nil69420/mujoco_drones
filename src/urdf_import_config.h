#ifndef URDF_IMPORT_CONFIG_H
#define URDF_IMPORT_CONFIG_H

#include <stddef.h>

#define CONFIG_MAX_ACTUATORS 64
#define CONFIG_NAME_LEN 256

typedef struct {
    char joint_name[CONFIG_NAME_LEN];
    int is_motor;   // 1 = motor, 0 = position
    double kp;      // position only
    double kv;      // position only
    int has_torque; // motor only: yaw gear (gear[5])
    double torque;
    int has_ctrlrange; // ctrlrange override
    double ctrlrange[2];
    int is_spin; // spin (velocity) actuator companion to the joint (uses kv)
    int matched; // set when the joint exists in the model
} config_actuator_t;

typedef struct {
    config_actuator_t entries[CONFIG_MAX_ACTUATORS];
    int count;
    char channel_order[CONFIG_MAX_ACTUATORS][CONFIG_NAME_LEN];
    int channel_order_count;
} actuator_config_t;

// Parse an INI-style actuator override file (see stage2_plan.md §4a).
// Returns 0 on success, -1 on error (message in err).
int config_parse(const char *path, actuator_config_t *cfg, char *err, size_t err_sz);

// Look up a joint override by name; NULL if none.
const config_actuator_t *config_find(const actuator_config_t *cfg, const char *joint);
const config_actuator_t *config_find_spin(const actuator_config_t *cfg, const char *joint);

#endif
