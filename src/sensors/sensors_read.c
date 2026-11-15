#include "sensors/sensors_internal.h"
#include "conditions/conditions.h"
#include "sensors/noise.h"

#include <math.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

void sensor_update_conditions(sensor_mgr_t *mgr, const mjData *data) {
    double fraction = 0.0;
    if ((mgr->use_vibration || mgr->use_effects) && mgr->thrust_max > 0.0) {
        double rpm[4];
        for (int i = 0; i < 4; i++) {
            double f = data->ctrl[mgr->act_thrust[i]] / mgr->thrust_max;
            f = fmin(fmax(f, 0.0), 1.0);
            rpm[i] = SENSOR_ROTOR_RATED_RPM * sqrt(f);
            fraction += f * 0.25;
        }
        if (mgr->use_vibration) {
            vibration_step(&mgr->vibration, rpm, mgr->sim_dt, mgr->vib_accel, mgr->vib_gyro);
        }
    }
    mgr->airframe_thrust_fraction = fraction;
    mgr->thrust_fraction = mgr->use_vibration ? fraction : 0.0;
    double heat = mgr->use_vibration ? 1.0 : 0.0;
    mgr->thermal.self_heating_k = SENSOR_SELF_HEATING_K * heat;
    mgr->sensor_temp_c = thermal_step(&mgr->thermal, mgr->config.ambient_temp_c, mgr->sim_dt);
}

void sensor_read_imu(sensor_mgr_t *mgr, const mjData *data, uint64_t t_ns) {
    sensor_imu_t msg = {0};
    msg.header.timestamp_ns = t_ns;
    msg.header.sequence     = mgr->seq_imu++;
    msg.header.sensor_id    = SENSOR_ID_IMU;

    double dt = 1.0 / mgr->config.imu_rate;

    if (mgr->use_imu_model) {
        const double zero[3] = {0.0, 0.0, 0.0};
        imu_model_step(&mgr->imu_model, &data->sensordata[mgr->adr_gyro],
                       &data->sensordata[mgr->adr_accel], mgr->sensor_temp_c,
                       mgr->use_vibration ? mgr->vib_accel : zero,
                       mgr->use_vibration ? mgr->vib_gyro : zero, msg.gyro, msg.accel);
        if (mgr->use_effects) {
            msg.accel[2] -= airframe_effects_accel_z_offset(&mgr->effects, mgr->airframe_thrust_fraction);
        }
    }

    for (int i = 0; i < 3 && !mgr->use_imu_model; i++) {
        double bias_g = noise_random_walk(&mgr->rng, &mgr->gyro_bias[i],
                                          mgr->config.imu_gyro_bias_diffusion,
                                          dt);
        msg.accel[i] = data->sensordata[mgr->adr_accel + i]
                     + noise_gaussian(&mgr->rng, mgr->config.imu_accel_noise);
        msg.gyro[i]  = data->sensordata[mgr->adr_gyro + i]
                     + noise_gaussian(&mgr->rng, mgr->config.imu_gyro_noise) + bias_g;
    }

    if (mgr->use_mag_model && mgr->adr_quat >= 0) {
        const double *quat = &data->sensordata[mgr->adr_quat];
        double rot[9];
        mju_quat2Mat(rot, quat);
        double field_nt[3];
        for (int i = 0; i < 3; i++) {
            field_nt[i] = 1000.0 * (rot[i] * mgr->config.mag_field_enu_ut[0] +
                                    rot[3 + i] * mgr->config.mag_field_enu_ut[1] +
                                    rot[6 + i] * mgr->config.mag_field_enu_ut[2]);
        }
        if (mgr->mag_count == 0) {
            double out_nt[3];
            mag_model_step(&mgr->mag_model, field_nt, 8.0 * mgr->thrust_fraction,
                           mgr->sensor_temp_c, out_nt);
            for (int i = 0; i < 3; i++) {
                mgr->mag_hold[i] = out_nt[i] * 1e-3;
            }
        }
        mgr->mag_count = (mgr->mag_count + 1) % mgr->mag_every;
        memcpy(msg.mag, mgr->mag_hold, sizeof(msg.mag));
    } else if (mgr->adr_heading >= 0) {
        msg.mag[0] = data->sensordata[mgr->adr_heading];
        msg.mag[1] = data->sensordata[mgr->adr_heading + 1];
        msg.mag[2] = data->sensordata[mgr->adr_heading + 2];
    }

    if (mgr->adr_quat >= 0) {
        memcpy(msg.orientation, &data->sensordata[mgr->adr_quat],
               4 * sizeof(double));
    }

    mgr->imu_latest = msg;
    transport_publish(&mgr->pub_imu, &msg, sizeof(msg));
}

void sensor_step_gnss_model(sensor_mgr_t *mgr, const mjData *data, uint64_t t_ns) {
    double pos[3];
    double vel[3] = {0.0, 0.0, 0.0};
    memcpy(pos, &data->sensordata[mgr->adr_pos], sizeof(pos));
    if (mgr->adr_linvel >= 0) {
        memcpy(vel, &data->sensordata[mgr->adr_linvel], sizeof(vel));
    }
    gnss_env_t env = {mgr->config.sky_visibility, mgr->config.multipath_scale,
                      mgr->config.rtk_corrections};
    gnss_sample_t smp;
    if (gnss_model_step(&mgr->gnss_model, data->time, pos, vel, &env, &smp) == 0) {
        return;
    }
    static const double R_EARTH = 6371000.0;
    double lat0_rad = mgr->config.gnss_origin_lat * M_PI / 180.0;
    sensor_gnss_t msg = {0};
    msg.header.timestamp_ns = smp.stamp_s > 0.0 ? (uint64_t)(smp.stamp_s * 1e9) : 1U;
    (void)t_ns;
    msg.header.sequence     = mgr->seq_gnss++;
    msg.header.sensor_id    = SENSOR_ID_GNSS;
    msg.latitude  = mgr->config.gnss_origin_lat + (smp.pos_enu[1] / R_EARTH) * (180.0 / M_PI);
    msg.longitude = mgr->config.gnss_origin_lon +
                    (smp.pos_enu[0] / (R_EARTH * cos(lat0_rad))) * (180.0 / M_PI);
    msg.altitude  = mgr->config.gnss_origin_alt + smp.pos_enu[2];
    memcpy(msg.velocity, smp.vel_enu, sizeof(msg.velocity));
    msg.fix_type       = (uint8_t)(smp.fix >= GNSS_FIX_3D ? 3 : 0);
    msg.num_satellites = (uint8_t)smp.satellites;
    msg.hdop           = smp.hdop;
    mgr->gnss_latest = msg;
    transport_publish(&mgr->pub_gnss, &msg, sizeof(msg));
}

void sensor_read_gnss(sensor_mgr_t *mgr, const mjData *data, uint64_t t_ns) {
    sensor_gnss_t msg = {0};
    msg.header.timestamp_ns = t_ns;
    msg.header.sequence     = mgr->seq_gnss++;
    msg.header.sensor_id    = SENSOR_ID_GNSS;

    double pos[3];
    memcpy(pos, &data->sensordata[mgr->adr_pos], 3 * sizeof(double));

    static const double R_EARTH = 6371000.0;
    double lat0_rad = mgr->config.gnss_origin_lat * M_PI / 180.0;

    msg.latitude  = mgr->config.gnss_origin_lat +
                    (pos[1] / R_EARTH) * (180.0 / M_PI);
    msg.longitude = mgr->config.gnss_origin_lon +
                    (pos[0] / (R_EARTH * cos(lat0_rad))) * (180.0 / M_PI);
    msg.altitude  = mgr->config.gnss_origin_alt + pos[2];

    if (mgr->adr_linvel >= 0) {
        memcpy(msg.velocity, &data->sensordata[mgr->adr_linvel],
               3 * sizeof(double));
    }

    msg.fix_type       = 3;
    msg.num_satellites = 12;
    msg.hdop           = 1.2;

    double noise_m = mgr->config.gnss_pos_noise;
    msg.latitude  += noise_gaussian(&mgr->rng, noise_m / R_EARTH) * (180.0 / M_PI);
    msg.longitude += noise_gaussian(&mgr->rng, noise_m / R_EARTH) * (180.0 / M_PI);
    msg.altitude  += noise_gaussian(&mgr->rng, noise_m);

    mgr->gnss_latest = msg;
    transport_publish(&mgr->pub_gnss, &msg, sizeof(msg));
}

void sensor_read_baro(sensor_mgr_t *mgr, const mjData *data, uint64_t t_ns) {
    sensor_baro_t msg = {0};
    msg.header.timestamp_ns = t_ns;
    msg.header.sequence     = mgr->seq_baro++;
    msg.header.sensor_id    = SENSOR_ID_BARO;

    double true_alt = data->sensordata[mgr->adr_pos + 2] + mgr->config.gnss_origin_alt;
    if (mgr->use_baro_model) {
        atmosphere_sample_t atm = atmosphere_at(mgr->config.ambient_temp_c,
                                                mgr->config.sea_level_pressure_pa, true_alt);
        double pressure_pa = mgr->use_effects
                                 ? airframe_effects_baro(&mgr->effects, atm.pressure_pa,
                                                         1.0 / mgr->config.baro_rate)
                                 : atm.pressure_pa;
        msg.pressure_pa = baro_model_step(&mgr->baro_model, pressure_pa,
                                          mgr->sensor_temp_c, mgr->thrust_fraction);
        msg.altitude_m = baro_pressure_to_altitude(msg.pressure_pa,
                                                   mgr->config.sea_level_pressure_pa);
        msg.temperature_c = atm.temp_c;
    } else {
        double alt = true_alt + noise_gaussian(&mgr->rng, mgr->config.baro_alt_noise);
        msg.altitude_m    = alt;
        msg.temperature_c = 15.0;
        msg.pressure_pa = 101325.0 * pow(1.0 - 0.0065 * alt / 288.15, 5.2561);
    }

    mgr->baro_latest = msg;
    transport_publish(&mgr->pub_baro, &msg, sizeof(msg));
}

void sensor_read_lidar(sensor_mgr_t *mgr, const mjModel *model,
                       mjData *data, uint64_t t_ns) {
    sensor_lidar_t msg = {0};
    msg.header.timestamp_ns = t_ns;
    msg.header.sequence     = mgr->seq_lidar++;
    msg.header.sensor_id    = SENSOR_ID_LIDAR;

    int nrays = mgr->config.lidar_num_rays;
    if (nrays > LIDAR_MAX_RAYS) nrays = LIDAR_MAX_RAYS;

    msg.num_rays  = (uint16_t)nrays;
    msg.angle_min = 0.0F;
    msg.angle_max = 2.0F * (float)M_PI;
    msg.range_min = 0.1F;
    msg.range_max = mgr->config.lidar_range_max;

    const double *site_pos = &data->site_xpos[(ptrdiff_t)mgr->site_lidar * 3];
    const double *site_mat = &data->site_xmat[(ptrdiff_t)mgr->site_lidar * 9];

    float angle_step = (msg.angle_max - msg.angle_min) / (float)nrays;

    for (int i = 0; i < nrays; i++) {
        float angle = msg.angle_min + (float)i * angle_step;
        const double dir_local[3] = { (double)cosf(angle), (double)sinf(angle), 0.0 };

        double dir_world[3];
        for (int row = 0; row < 3; row++) {
            dir_world[row] = site_mat[3*row + 0] * dir_local[0]
                           + site_mat[3*row + 1] * dir_local[1]
                           + site_mat[3*row + 2] * dir_local[2];
        }

        int geomid = -1;
        mjtNum dist = mj_ray(model, data, site_pos, dir_world,
                             NULL, 1, mgr->body_base, &geomid, NULL);

        if (dist >= 0 && dist <= (double)msg.range_max) {
            msg.ranges[i] = (float)dist
                          + (float)noise_gaussian(&mgr->rng, mgr->config.lidar_range_noise);
        } else {
            msg.ranges[i] = -1.0F;
        }
    }

    size_t payload = offsetof(sensor_lidar_t, ranges)
                   + (size_t)nrays * sizeof(float);
    transport_publish(&mgr->pub_lidar, &msg, payload);
}

void sensor_read_infrared(sensor_mgr_t *mgr, const mjModel *model,
                          mjData *data, uint64_t t_ns) {
    sensor_infrared_t msg = {0};
    msg.header.timestamp_ns = t_ns;
    msg.header.sequence     = mgr->seq_infrared++;
    msg.header.sensor_id    = SENSOR_ID_INFRARED;

    msg.range_min  = 0.02F;
    msg.range_max  = 5.0F;
    msg.beam_angle = 0.05F;

    const double *site_pos = &data->site_xpos[(ptrdiff_t)mgr->site_infrared * 3];
    const double *site_mat = &data->site_xmat[(ptrdiff_t)mgr->site_infrared * 9];

    double dir_world[3] = {
        site_mat[3*0 + 2],
        site_mat[3*1 + 2],
        site_mat[3*2 + 2],
    };

    int geomid = -1;
    mjtNum dist = mj_ray(model, data, site_pos, dir_world,
                         NULL, 1, mgr->body_base, &geomid, NULL);

    if (mgr->use_range_model) {
        const range_profile_t *rp = mgr->range_model.profile;
        msg.range_min = (float)rp->min_range_m;
        msg.range_max = (float)rp->max_range_90_m;
        double reading = 0.0;
        if (dist >= 0 && range_model_step(&mgr->range_model, dist, mgr->config.ground_reflectivity,
                                          &reading)) {
            msg.range_m = (float)reading;
        } else {
            msg.range_m = -1.0F;
        }
    } else if (dist >= 0 && dist <= (double)msg.range_max) {
        msg.range_m = (float)dist
                    + (float)noise_gaussian(&mgr->rng, mgr->config.ir_range_noise);
    } else {
        msg.range_m = -1.0F;
    }

    transport_publish(&mgr->pub_infrared, &msg, sizeof(msg));
}
