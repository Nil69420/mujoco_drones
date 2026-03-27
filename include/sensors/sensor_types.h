#ifndef MUJOCO_DRONES_SENSOR_TYPES_H
#define MUJOCO_DRONES_SENSOR_TYPES_H

#include <stdint.h>

typedef struct {
    uint64_t timestamp_ns;
    uint32_t sequence;
    uint32_t sensor_id;
} sensor_header_t;

enum {
    SENSOR_ID_IMU      = 1,
    SENSOR_ID_GNSS     = 2,
    SENSOR_ID_BARO     = 3,
    SENSOR_ID_LIDAR    = 4,
    SENSOR_ID_CAMERA   = 5,
    SENSOR_ID_INFRARED = 6,
};

typedef struct {
    sensor_header_t header;
    double accel[3];
    double gyro[3];
    double mag[3];
    double orientation[4];
} sensor_imu_t;

typedef struct {
    sensor_header_t header;
    double latitude;
    double longitude;
    double altitude;
    double velocity[3];
    double hdop;
    uint8_t num_satellites;
    uint8_t fix_type;
    uint8_t _pad[6];
} sensor_gnss_t;

typedef struct {
    sensor_header_t header;
    double pressure_pa;
    double temperature_c;
    double altitude_m;
} sensor_baro_t;

#define LIDAR_MAX_RAYS 360

typedef struct {
    sensor_header_t header;
    uint16_t num_rays;
    uint16_t _pad;
    float    angle_min;
    float    angle_max;
    float    range_min;
    float    range_max;
    float    ranges[LIDAR_MAX_RAYS];
} sensor_lidar_t;

typedef struct {
    sensor_header_t header;
    uint16_t width;
    uint16_t height;
    uint8_t  channels;
    uint8_t  _pad[3];
    float    fov_y;
    uint32_t rgb_size;
    uint32_t depth_size;
    float    depth_scale;
} sensor_camera_meta_t;

typedef struct {
    sensor_header_t header;
    float range_m;
    float range_min;
    float range_max;
    float beam_angle;
} sensor_infrared_t;

typedef struct {
    sensor_header_t header;
    double x, y, z;
    double yaw;
    uint8_t mode;
    uint8_t _pad[7];
} command_setpoint_t;

#endif
