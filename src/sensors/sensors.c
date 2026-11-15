#include "sensors/sensors_internal.h"
#include "conditions/conditions.h"
#include "sensors/noise.h"

#include <math.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

sensor_config_t sensor_default_config(void) {
    sensor_config_t cfg = {0};

    cfg.enable.imu      = true;
    cfg.enable.gnss     = true;
    cfg.enable.baro     = true;
    cfg.enable.lidar    = true;
    cfg.enable.camera   = true;
    cfg.enable.infrared = true;
    cfg.enable.command  = true;

    cfg.imu_rate      = 1000.0;
    cfg.gnss_rate     = 10.0;
    cfg.baro_rate     = 50.0;
    cfg.lidar_rate    = 30.0;
    cfg.camera_rate   = 8.0;
    cfg.infrared_rate = 100.0;

    cfg.lidar_num_rays  = 36;
    cfg.lidar_range_max = 30.0F;

    cfg.camera_width  = 640;
    cfg.camera_height = 360;
    cfg.camera_fov    = 60.0F * (float)M_PI / 180.0F;

    cfg.gnss_origin_lat = 47.3667;
    cfg.gnss_origin_lon = 8.5500;
    cfg.gnss_origin_alt = 500.0;

    cfg.imu_accel_noise        = 0.02;
    cfg.imu_gyro_noise         = 0.001;
    cfg.imu_gyro_bias_diffusion = 0.0001;
    cfg.gnss_pos_noise         = 1.5;
    cfg.baro_alt_noise         = 0.5;
    cfg.lidar_range_noise      = 0.01;
    cfg.ir_range_noise         = 0.005;

    cfg.mag_field_enu_ut[0]    = 0.9;
    cfg.mag_field_enu_ut[1]    = 21.4;
    cfg.mag_field_enu_ut[2]    = -42.3;
    cfg.camera_exposure_s      = 0.004;
    cfg.mag_rate               = 0.0;
    cfg.ground_reflectivity    = 0.3;
    cfg.sky_visibility         = 1.0;
    cfg.multipath_scale        = 1.0;
    cfg.ambient_temp_c         = 15.0;
    cfg.sea_level_pressure_pa  = 101325.0;

    return cfg;
}

static int tick_due(double *acc, double rate_hz, double timestep) {
    if (rate_hz <= 0.0) {
        return 0;
    }
    *acc += rate_hz * timestep;
    if (*acc >= 1.0 - 1e-9) {
        *acc -= 1.0;
        if (*acc > 1.0) {
            *acc = 0.0;
        }
        return 1;
    }
    return 0;
}

void sensor_update(sensor_mgr_t *mgr, const mjModel *model, mjData *data,
                   setpoint_t *target) {
    uint64_t t_ns = (uint64_t)(data->time * 1e9);
    sensor_update_conditions(mgr, data);

    if (mgr->config.enable.imu && tick_due(&mgr->acc_imu, mgr->config.imu_rate, mgr->sim_dt)) {
        sensor_read_imu(mgr, data, t_ns);
    }

    if (mgr->config.enable.gnss && mgr->use_gnss_model) {
        sensor_step_gnss_model(mgr, data, t_ns);
    } else if (mgr->config.enable.gnss &&
               tick_due(&mgr->acc_gnss, mgr->config.gnss_rate, mgr->sim_dt)) {
        sensor_read_gnss(mgr, data, t_ns);
    }

    if (mgr->config.enable.baro && tick_due(&mgr->acc_baro, mgr->config.baro_rate, mgr->sim_dt)) {
        sensor_read_baro(mgr, data, t_ns);
    }

    if (mgr->config.enable.lidar &&
        tick_due(&mgr->acc_lidar, mgr->config.lidar_rate, mgr->sim_dt)) {
        sensor_read_lidar(mgr, model, data, t_ns);
    }

    if (mgr->config.enable.infrared &&
        tick_due(&mgr->acc_infrared, mgr->config.infrared_rate, mgr->sim_dt)) {
        sensor_read_infrared(mgr, model, data, t_ns);
    }

    if (mgr->config.enable.camera &&
        tick_due(&mgr->acc_camera, mgr->config.camera_rate, mgr->sim_dt)) {
        mgr->cam_due = true;
    }

    if (mgr->config.enable.command) {
        command_setpoint_t cmd;
        size_t cmd_len = 0;
        int rc = transport_read_next(&mgr->sub_command, &cmd, sizeof(cmd),
                                     &cmd_len);
        if (rc == 0 && cmd_len == sizeof(cmd)) {
            target->x   = cmd.x;
            target->y   = cmd.y;
            target->z   = cmd.z;
            target->yaw = cmd.yaw;
        }
    }
}

void sensor_cleanup(sensor_mgr_t *mgr) {
    transport_close_pub(&mgr->pub_imu);
    transport_close_pub(&mgr->pub_gnss);
    transport_close_pub(&mgr->pub_baro);
    transport_close_pub(&mgr->pub_lidar);
    transport_close_pub(&mgr->pub_infrared);
    transport_close_pub(&mgr->pub_camera_meta);
    transport_close_pub(&mgr->pub_camera_rgb);

    transport_close_sub(&mgr->sub_command);

    if (mgr->cam_initialized) {
        mjv_freeScene(&mgr->cam_scene);
        mjr_freeContext(&mgr->cam_context);
        mgr->cam_initialized = false;
    }
    camera_egl_close(&mgr->egl);
    camera_model_free(&mgr->cam_model);
    frame_out_close(&mgr->cam_out);
    free(mgr->rgb_buf);
    free(mgr->depth_buf);
    mgr->rgb_buf   = NULL;
    mgr->depth_buf = NULL;
}
