#ifndef MUJOCO_DRONES_SENSORS_INTERNAL_H
#define MUJOCO_DRONES_SENSORS_INTERNAL_H

#include "sensors/sensors.h"

enum {
    CAMERA_RING_SLOTS = 4,
};
#define MAG_DEFAULT_RATE_HZ 100.0
#define SENSOR_ROTOR_RATED_RPM 8000.0
#define SENSOR_VIBRATION_BASE_MS2 0.6
#define SENSOR_MOUNT_HZ 45.0
#define SENSOR_MOUNT_ZETA 0.15
#define SENSOR_THERMAL_TAU_S 150.0
#define SENSOR_SELF_HEATING_K 10.0

void sensor_update_conditions(sensor_mgr_t *mgr, const mjData *data);
void sensor_read_imu(sensor_mgr_t *mgr, const mjData *data, uint64_t t_ns);
void sensor_step_gnss_model(sensor_mgr_t *mgr, const mjData *data, uint64_t t_ns);
void sensor_read_gnss(sensor_mgr_t *mgr, const mjData *data, uint64_t t_ns);
void sensor_read_baro(sensor_mgr_t *mgr, const mjData *data, uint64_t t_ns);
void sensor_read_lidar(sensor_mgr_t *mgr, const mjModel *model, mjData *data, uint64_t t_ns);
void sensor_read_infrared(sensor_mgr_t *mgr, const mjModel *model, mjData *data, uint64_t t_ns);
int sensor_camera_setup(sensor_mgr_t *mgr, const mjModel *model, transport_t *tp,
                        const sensor_config_t *config, const char *topic_prefix);

#endif
