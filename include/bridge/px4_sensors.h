#ifndef MUJOCO_DRONES_BRIDGE_PX4_SENSORS_H
#define MUJOCO_DRONES_BRIDGE_PX4_SENSORS_H

#include "bridge/ap_json.h"
#include "bridge/px4_msgs.h"
#include "sensors/sensor_types.h"

#include <stdbool.h>

typedef struct {
    double lat_deg;
    double lon_deg;
    double alt_m;
    double field_ned_gauss[3];
} px4_home_t;

px4_home_t px4_home_default(void);

void px4_sensor_from_state(const ap_state_t *state, const px4_home_t *home, px4_hil_sensor_t *out);

void px4_gps_from_state(const ap_state_t *state, const px4_home_t *home, px4_hil_gps_t *out);

double px4_isa_pressure_hpa(double altitude_m);

typedef struct {
    const sensor_imu_t *imu;
    bool imu_fresh;
    bool mag_modelled;
    const sensor_baro_t *baro;
    bool baro_fresh;
} px4_model_samples_t;

void px4_sensor_apply_models(px4_hil_sensor_t *sensor, const px4_model_samples_t *samples);

#endif
