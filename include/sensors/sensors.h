#ifndef MUJOCO_DRONES_SENSORS_H
#define MUJOCO_DRONES_SENSORS_H

#include <mujoco/mujoco.h>
#include <stdbool.h>
#include <stdint.h>

#include "sensors/sensor_types.h"
#include "sensors/noise.h"
#include "sensors/models/baro_model.h"
#include "sensors/models/gnss_model.h"
#include "sensors/models/imu_model.h"
#include "sensors/models/mag_model.h"
#include "conditions/conditions.h"
#include "sensors/camera_egl.h"
#include "sensors/models/camera_model.h"
#include "transport/transport.h"
#include "transport/frame_out.h"
#include "setpoint.h"

typedef struct {
    bool imu;
    bool gnss;
    bool baro;
    bool lidar;
    bool camera;
    bool infrared;
    bool command;
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

    const char *imu_profile;
    const char *gnss_profile;
    bool gnss_cold_start;
    const char *baro_profile;
    const char *mag_profile;
    double mag_rate;
    const char *weather;
    const char *camera_profile;
    bool camera_mono;
    bool camera_headless;
    bool camera_legacy_topic;
    double camera_exposure_s;
    double vibration_scale;
    double mag_field_enu_ut[3];
    double sky_visibility;
    double multipath_scale;
    int rtk_corrections;
    double ambient_temp_c;
    double sea_level_pressure_pa;
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

    double acc_imu, acc_gnss, acc_baro;
    double acc_lidar, acc_camera, acc_infrared;


    uint32_t seq_imu, seq_gnss, seq_baro;
    uint32_t seq_lidar, seq_camera, seq_infrared;

    transport_pub_t pub_imu;
    transport_pub_t pub_gnss;
    transport_pub_t pub_baro;
    transport_pub_t pub_lidar;
    transport_pub_t pub_infrared;
    transport_pub_t pub_camera_meta;
    transport_pub_t pub_camera_rgb;

    transport_sub_t sub_command;

    mjvScene   cam_scene;
    mjrContext cam_context;
    mjvCamera  cam_view;
    mjvOption  cam_opt;
    bool       cam_initialized;
    bool       cam_due;

    camera_egl_t   egl;
    camera_model_t cam_model;
    bool           use_cam_model;
    frame_out_t    cam_out;
    bool           cam_out_open;
    unsigned char *rgb_buf;
    float         *depth_buf;

    double gyro_bias[3];
    double accel_bias[3];

    sensor_imu_t  imu_latest;
    sensor_gnss_t gnss_latest;
    sensor_baro_t baro_latest;

    noise_rng_t rng;

    double sim_dt;
    double sensor_temp_c;
    imu_model_t  imu_model;
    gnss_model_t gnss_model;
    baro_model_t baro_model;
    bool use_imu_model, use_gnss_model, use_baro_model, use_mag_model;
    mag_model_t  mag_model;
    int          mag_every;
    int          mag_count;
    double       mag_hold[3];
    thermal_t    thermal;
    vibration_t  vibration;
    bool         use_vibration;
    int          act_thrust[4];
    double       thrust_max;
    double       vib_accel[3];
    double       vib_gyro[3];
    double       thrust_fraction;
} sensor_mgr_t;

sensor_config_t sensor_default_config(void);

int sensor_init(sensor_mgr_t *mgr, const mjModel *model,
                transport_t *tp, const sensor_config_t *config,
                const char *topic_prefix);

void sensor_update(sensor_mgr_t *mgr, const mjModel *model, mjData *data,
                   setpoint_t *target);

void sensor_render_camera(sensor_mgr_t *mgr, const mjModel *model,
                          mjData *data);

void sensor_cleanup(sensor_mgr_t *mgr);

#endif
