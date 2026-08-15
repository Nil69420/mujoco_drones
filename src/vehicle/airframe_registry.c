#include "vehicle/airframe.h"

#include <stddef.h>
#include <string.h>

static const airframe_spec_t QUAD_X = {
    .name           = "quad_x",
    .airframe_class = "rotor_multicopter",
    .mjcf_path      = "quad_x.xml",
    .channel_names  = {
        "thrust_0", "thrust_1", "thrust_2", "thrust_3",
        "spin_0",   "spin_1",   "spin_2",   "spin_3",
    },
    .channel_count  = 8,
    .imu_site       = "imu_site",
    .lidar_site     = "lidar_site",
    .infrared_site  = "infrared_site",
    .camera_name    = "drone_camera",
    .base_body      = "base_link",
};

static const airframe_spec_t FIXEDWING_BASIC = {
    .name           = "fixedwing_basic",
    .airframe_class = "fixed_wing_conventional",
    .mjcf_path      = "fixedwing_basic.xml",
    .channel_names  = {
        "aileron_left", "aileron_right", "elevator", "rudder", "throttle",
    },
    .channel_count  = 5,
    .imu_site       = "imu_site",
    .lidar_site     = NULL,
    .infrared_site  = NULL,
    .camera_name    = NULL,
    .base_body      = "base_link",
};

static const airframe_spec_t *const *airframe_table(int *count) {
    static const airframe_spec_t *const table[] = {
        &QUAD_X,
        &FIXEDWING_BASIC,
    };
    *count = (int)(sizeof(table) / sizeof(table[0]));
    return table;
}

const airframe_spec_t *airframe_registry_find(const char *name) {
    if (!name) return NULL;
    int count = 0;
    const airframe_spec_t *const *table = airframe_table(&count);
    for (int i = 0; i < count; i++) {
        if (table[i]->name && strcmp(table[i]->name, name) == 0) {
            return table[i];
        }
    }
    return NULL;
}

int airframe_registry_count(void) {
    int count = 0;
    airframe_table(&count);
    return count;
}

const airframe_spec_t *airframe_registry_at(int index) {
    int count = 0;
    const airframe_spec_t *const *table = airframe_table(&count);
    if (index < 0 || index >= count) return NULL;
    return table[index];
}
