#ifndef MUJOCO_DRONES_VEHICLE_SENSOR_SNAPSHOT_H
#define MUJOCO_DRONES_VEHICLE_SENSOR_SNAPSHOT_H

#include "sensors/sensor_types.h"
#include <stdbool.h>

typedef struct {
    sensor_imu_t  imu;
    sensor_gnss_t gnss;
    sensor_baro_t baro;
    bool has_imu;
    bool has_gnss;
    bool has_baro;
    bool has_origin;
    double origin_lat_deg;
    double origin_lon_deg;
    double origin_alt_m;
} sensor_snapshot_t;

#endif
