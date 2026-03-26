#ifndef MUJOCO_DRONES_SENSORS_H
#define MUJOCO_DRONES_SENSORS_H

#include <mujoco/mujoco.h>
#include <stdbool.h>
#include <stdint.h>

#include "sensor_types.h"
#include "../transport/transport.h"
#include "../setpoint.h"

typedef struct {
    bool imu;
    bool gnss;
    bool baro;
    bool lidar;
    bool camera;
    bool infrared;
} sensor_enable_t;

typedef struct {
    sensor_enable_t enable;

    double imu_rate;
    double gnss_rate;
    double baro_rate;
    double lidar_rate;
    double camera_rate;
    double infrared_rate;

    uint16_t lidar_num_rays;
    float    lidar_range_max;

    uint16_t camera_width;
    uint16_t camera_height;
    float    camera_fov;

    double gnss_origin_lat;
    double gnss_origin_lon;
    double gnss_origin_alt;

    double imu_accel_noise;
    double imu_gyro_noise;
    double imu_gyro_bias_diffusion;
    double gnss_pos_noise;
    double baro_alt_noise;
    double lidar_range_noise;
    double ir_range_noise;
} sensor_config_t;

typedef struct {
    sensor_config_t config;

    int adr_accel;
    int adr_gyro;
    int adr_quat;
    int adr_pos;
    int adr_heading;
    int adr_linvel;
    int adr_mag_y;
    int adr_mag_z;

    int site_imu;
    int site_lidar;
    int site_infrared;
    int body_base;
    int camera_id;

    int dec_imu,    dec_gnss,   dec_baro;
    int dec_lidar,  dec_camera, dec_infrared;

    int inv_imu,    inv_gnss,   inv_baro;
    int inv_lidar,  inv_camera, inv_infrared;

    uint32_t seq_imu, seq_gnss, seq_baro;
    uint32_t seq_lidar, seq_camera, seq_infrared;

    transport_pub_t pub_imu;
    transport_pub_t pub_gnss;
    transport_pub_t pub_baro;
    transport_pub_t pub_lidar;
    transport_pub_t pub_infrared;
    transport_pub_t pub_camera_meta;
    transport_pub_t pub_camera_rgb;
    transport_pub_t pub_camera_depth;

    transport_sub_t sub_command;

    mjvScene   cam_scene;
    mjrContext cam_context;
    mjvCamera  cam_view;
    mjvOption  cam_opt;
    bool       cam_initialized;
    bool       cam_due;

    unsigned char *rgb_buf;
    float         *depth_buf;

    double gyro_bias[3];
    double accel_bias[3];
} sensor_mgr_t;

sensor_config_t sensor_default_config(void);

int sensor_init(sensor_mgr_t *mgr, const mjModel *model,
                transport_t *tp, const sensor_config_t *config);

void sensor_update(sensor_mgr_t *mgr, const mjModel *model, mjData *data,
                   setpoint_t *target);

void sensor_render_camera(sensor_mgr_t *mgr, const mjModel *model,
                          mjData *data);

void sensor_cleanup(sensor_mgr_t *mgr);

#endif
