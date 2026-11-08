#include "sensors/sensors.h"
#include "conditions/conditions.h"
#include "sensors/noise.h"

#include <math.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static void build_topic(char *buf, size_t bufsz, const char *prefix,
                        const char *suffix) {
    snprintf(buf, bufsz, "/%s%s", prefix ? prefix : "drone", suffix);
}

static void build_name(char *buf, size_t bufsz, const char *prefix,
                       const char *name) {
    if (prefix) {
        snprintf(buf, bufsz, "%s_%s", prefix, name);
    } else {
        snprintf(buf, bufsz, "%s", name);
    }
}

static int resolve_sensor_adr(const mjModel *m, const char *prefix,
                              const char *name) {
    char full[64];
    build_name(full, sizeof(full), prefix, name);
    int id = mj_name2id(m, mjOBJ_SENSOR, full);
    if (id < 0) {
        fprintf(stderr, "[sensors] WARNING: sensor '%s' not found in model\n",
                full);
        return -1;
    }
    return m->sensor_adr[id];
}

static int resolve_site_id(const mjModel *m, const char *prefix,
                           const char *name) {
    char full[64];
    build_name(full, sizeof(full), prefix, name);
    return mj_name2id(m, mjOBJ_SITE, full);
}

#define CAMERA_RING_SLOTS 4U
#define SENSOR_ROTOR_RATED_RPM 8000.0
#define SENSOR_VIBRATION_BASE_MS2 0.6
#define SENSOR_MOUNT_HZ 45.0
#define SENSOR_MOUNT_ZETA 0.15
#define SENSOR_THERMAL_TAU_S 150.0
#define SENSOR_SELF_HEATING_K 10.0

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
    cfg.mag_rate               = 100.0;
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

int sensor_init(sensor_mgr_t *mgr, const mjModel *model,
                transport_t *tp, const sensor_config_t *config,
                const char *topic_prefix) {
    memset(mgr, 0, sizeof(*mgr));
    mgr->config = *config;

    char topic_imu[64], topic_gnss[64], topic_baro[64], topic_lidar[64];
    char topic_infrared[64], topic_camera_meta[64], topic_camera_rgb[64];
    char topic_command[64];
    build_topic(topic_imu,        sizeof(topic_imu),        topic_prefix, "/imu");
    build_topic(topic_gnss,       sizeof(topic_gnss),       topic_prefix, "/gnss");
    build_topic(topic_baro,       sizeof(topic_baro),       topic_prefix, "/baro");
    build_topic(topic_lidar,      sizeof(topic_lidar),      topic_prefix, "/lidar");
    build_topic(topic_infrared,   sizeof(topic_infrared),   topic_prefix, "/infrared");
    build_topic(topic_camera_meta,sizeof(topic_camera_meta),topic_prefix, "/camera/meta");
    build_topic(topic_camera_rgb, sizeof(topic_camera_rgb), topic_prefix, "/camera/rgb");
    build_topic(topic_command,    sizeof(topic_command),    topic_prefix, "/command");


    mgr->adr_accel   = resolve_sensor_adr(model, topic_prefix, "imu_accel");
    mgr->adr_gyro    = resolve_sensor_adr(model, topic_prefix, "imu_gyro");
    mgr->adr_quat    = resolve_sensor_adr(model, topic_prefix, "orientation");
    mgr->adr_pos     = resolve_sensor_adr(model, topic_prefix, "position");
    mgr->adr_heading = resolve_sensor_adr(model, topic_prefix, "heading");
    mgr->adr_linvel  = resolve_sensor_adr(model, topic_prefix, "linvel");
    mgr->adr_mag_y   = resolve_sensor_adr(model, topic_prefix, "mag_y");
    mgr->adr_mag_z   = resolve_sensor_adr(model, topic_prefix, "mag_z");

    mgr->site_imu      = resolve_site_id(model, topic_prefix, "imu_site");
    mgr->site_lidar    = resolve_site_id(model, topic_prefix, "lidar_site");
    mgr->site_infrared = resolve_site_id(model, topic_prefix, "infrared_site");
    {
        char full[64];
        if (topic_prefix) {
            build_name(full, sizeof(full), topic_prefix, "base_link");
        } else {
            snprintf(full, sizeof(full), "%s", "base_link");
        }
        mgr->body_base = mj_name2id(model, mjOBJ_BODY, full);
        if (topic_prefix) {
            build_name(full, sizeof(full), topic_prefix, "drone_camera");
        } else {
            snprintf(full, sizeof(full), "%s", "drone_camera");
        }
        mgr->camera_id = mj_name2id(model, mjOBJ_CAMERA, full);
    }

    if (mgr->adr_accel < 0 || mgr->adr_gyro < 0 || mgr->adr_quat < 0 ||
        mgr->adr_pos < 0 || mgr->body_base < 0) {
        fprintf(stderr, "[sensors] ERROR: required sensors/bodies missing "
                        "from model\n");
        return -1;
    }

    if (config->enable.lidar && mgr->site_lidar < 0) {
        fprintf(stderr, "[sensors] WARNING: lidar_site not found, disabling\n");
        mgr->config.enable.lidar = false;
    }
    if (config->enable.infrared && mgr->site_infrared < 0) {
        fprintf(stderr, "[sensors] WARNING: infrared_site not found, disabling\n");
        mgr->config.enable.infrared = false;
    }
    if (config->enable.camera && mgr->camera_id < 0) {
        fprintf(stderr, "[sensors] WARNING: drone_camera not found, disabling\n");
        mgr->config.enable.camera = false;
    }


    mgr->acc_imu = mgr->acc_gnss = mgr->acc_baro = 1.0;
    mgr->acc_lidar = mgr->acc_camera = mgr->acc_infrared = 1.0;

    if (mgr->config.enable.imu) {
        if (transport_advertise(tp, topic_imu,
                                sizeof(sensor_imu_t), &mgr->pub_imu) != 0) {
            fprintf(stderr, "[sensors] ERROR: failed to advertise %s, "
                            "disabling IMU\n", topic_imu);
            mgr->config.enable.imu = false;
        }
    }

    if (mgr->config.enable.gnss) {
        if (transport_advertise(tp, topic_gnss,
                                sizeof(sensor_gnss_t), &mgr->pub_gnss) != 0) {
            fprintf(stderr, "[sensors] ERROR: failed to advertise %s, "
                            "disabling GNSS\n", topic_gnss);
            mgr->config.enable.gnss = false;
        }
    }

    if (mgr->config.enable.baro) {
        if (transport_advertise(tp, topic_baro,
                                sizeof(sensor_baro_t), &mgr->pub_baro) != 0) {
            fprintf(stderr, "[sensors] ERROR: failed to advertise %s, "
                            "disabling barometer\n", topic_baro);
            mgr->config.enable.baro = false;
        }
    }

    if (mgr->config.enable.lidar) {
        if (transport_advertise(tp, topic_lidar,
                                sizeof(sensor_lidar_t), &mgr->pub_lidar) != 0) {
            fprintf(stderr, "[sensors] ERROR: failed to advertise %s, "
                            "disabling LiDAR\n", topic_lidar);
            mgr->config.enable.lidar = false;
        }
    }

    if (mgr->config.enable.infrared) {
        if (transport_advertise(tp, topic_infrared,
                                sizeof(sensor_infrared_t),
                                &mgr->pub_infrared) != 0) {
            fprintf(stderr, "[sensors] ERROR: failed to advertise %s, "
                            "disabling infrared\n", topic_infrared);
            mgr->config.enable.infrared = false;
        }
    }

    if (mgr->config.enable.camera && config->camera_headless) {
        if (camera_egl_open(&mgr->egl) != 0) {
            fprintf(stderr, "[sensors] ERROR: headless camera unavailable, disabling camera\n");
            mgr->config.enable.camera = false;
        }
    }
    if (mgr->config.enable.camera && config->camera_mono) {
        const camera_profile_t *cp = camera_profile_find(config->camera_profile != NULL
                                                             ? config->camera_profile
                                                             : "ar0234");
        if (cp == NULL || camera_model_init(&mgr->cam_model, cp, config->camera_width,
                                            config->camera_height, 1006) != 0) {
            fprintf(stderr, "[sensors] ERROR: camera profile '%s' unusable at %ux%u\n",
                    config->camera_profile != NULL ? config->camera_profile : "ar0234",
                    (unsigned)config->camera_width, (unsigned)config->camera_height);
            return -1;
        }
        if (config->camera_rate > cp->max_fps + 1e-9) {
            fprintf(stderr, "[sensors] ERROR: camera rate %.0f fps exceeds the %s limit of %.0f\n",
                    config->camera_rate, cp->name, cp->max_fps);
            camera_model_free(&mgr->cam_model);
            return -1;
        }
        mgr->use_cam_model = true;
    }
    if (mgr->config.enable.camera) {
        size_t rgb_sz   = (size_t)config->camera_width *
                          config->camera_height * 3;
        size_t rgb_total = sizeof(sensor_camera_rgb_hdr_t) + rgb_sz;
        size_t rgb_max   = sizeof(sensor_camera_rgb_hdr_t) +
                           (size_t)CAM_BUF_MAX_W * CAM_BUF_MAX_H * 3;
        size_t depth_sz = (size_t)config->camera_width *
                          config->camera_height * sizeof(float);

        if (transport_advertise(tp, topic_camera_meta,
                                sizeof(sensor_camera_meta_t),
                                &mgr->pub_camera_meta) != 0) {
            fprintf(stderr, "[sensors] ERROR: failed to advertise %s, "
                            "disabling camera\n", topic_camera_meta);
            mgr->config.enable.camera = false;
        } else if (transport_advertise(tp, topic_camera_rgb, rgb_max,
                                       &mgr->pub_camera_rgb) != 0) {
            fprintf(stderr, "[sensors] ERROR: failed to advertise %s, "
                            "disabling camera\n", topic_camera_rgb);
            mgr->config.enable.camera = false;
        }

        if (mgr->config.enable.camera) {
            mgr->rgb_buf   = malloc(rgb_total);
            mgr->depth_buf = mgr->use_cam_model ? NULL : malloc(depth_sz);
            char ring_name[96];
            frame_out_name(ring_name, sizeof(ring_name), topic_prefix, "_camera_frame");
            size_t ring_frame = sizeof(sensor_camera_rgb_hdr_t) +
                                (size_t)config->camera_width * config->camera_height *
                                    (mgr->use_cam_model ? 1U : 3U);
            if (frame_out_open(&mgr->cam_out, ring_name, ring_frame, CAMERA_RING_SLOTS) == 0) {
                mgr->cam_out_open = true;
            }
            if (!mgr->rgb_buf || (!mgr->depth_buf && !mgr->use_cam_model) || !mgr->cam_out_open) {
                fprintf(stderr, "[sensors] ERROR: failed to allocate camera "
                                "buffers\n");
                mgr->config.enable.camera = false;
            }
        }
    }

    if (mgr->config.enable.command) {
        if (transport_subscribe(tp, topic_command,
                                sizeof(command_setpoint_t),
                                &mgr->sub_command) != 0) {
            fprintf(stderr, "[sensors] ERROR: failed to subscribe to %s, "
                            "command uplink disabled\n", topic_command);
            mgr->config.enable.command = false;
        }
    }

    noise_seed(&mgr->rng, 42);

    mgr->sim_dt = model->opt.timestep;
    const struct { const char *name; double rate; const char *profile; } rates[] = {
        {"IMU", config->imu_rate, config->imu_profile},
        {"GNSS", config->gnss_rate, config->gnss_profile},
        {"barometer", config->baro_rate, config->baro_profile},
    };
    for (size_t i = 0; i < sizeof(rates) / sizeof(rates[0]); i++) {
        if (rates[i].profile != NULL && rates[i].rate * mgr->sim_dt > 1.0 + 1e-9) {
            fprintf(stderr, "[sensors] %s rate %.0f Hz exceeds the simulation step rate %.0f Hz\n",
                    rates[i].name, rates[i].rate, 1.0 / mgr->sim_dt);
            return -1;
        }
    }
    if (config->weather != NULL) {
        const conditions_profile_t *wp = conditions_find(config->weather);
        if (wp == NULL) {
            fprintf(stderr, "[sensors] unknown weather preset '%s'\n", config->weather);
            return -1;
        }
        mgr->config.ambient_temp_c = wp->ambient_temp_c;
        mgr->config.sea_level_pressure_pa = wp->sea_level_pressure_pa;
    }
    config = &mgr->config;
    thermal_init(&mgr->thermal, config->ambient_temp_c, SENSOR_THERMAL_TAU_S, SENSOR_SELF_HEATING_K);
    mgr->sensor_temp_c = config->ambient_temp_c;
    if (config->mag_profile != NULL) {
        const mag_profile_t *pr = mag_profile_find(config->mag_profile);
        if (pr == NULL || mag_model_init(&mgr->mag_model, pr, config->mag_rate, 1004) != 0) {
            fprintf(stderr, "[sensors] unknown or unusable magnetometer profile '%s'\n",
                    config->mag_profile);
            return -1;
        }
        mgr->mag_every = (int)lround(config->imu_rate / config->mag_rate);
        if (mgr->mag_every < 1) mgr->mag_every = 1;
        mgr->mag_count = 0;
        mgr->use_mag_model = true;
    }
    if (config->vibration_scale > 0.0) {
        int ok = 1;
        char nm[32];
        for (int i = 0; i < 4; i++) {
            snprintf(nm, sizeof(nm), "thrust_%d", i);
            mgr->act_thrust[i] = mj_name2id(model, mjOBJ_ACTUATOR, nm);
            ok = ok && mgr->act_thrust[i] >= 0;
        }
        if (!ok || vibration_init(&mgr->vibration, 4, 2, SENSOR_ROTOR_RATED_RPM,
                                  SENSOR_VIBRATION_BASE_MS2 * config->vibration_scale,
                                  SENSOR_MOUNT_HZ, SENSOR_MOUNT_ZETA, 1005) != 0) {
            fprintf(stderr, "[sensors] vibration needs actuators thrust_0..3\n");
            return -1;
        }
        mgr->thrust_max = model->actuator_ctrlrange[2 * mgr->act_thrust[0] + 1];
        mgr->use_vibration = true;
    }
    if (config->imu_profile != NULL) {
        const imu_profile_t *pr = imu_profile_find(config->imu_profile);
        if (pr == NULL || imu_model_init(&mgr->imu_model, pr, config->imu_rate, 1001) != 0) {
            fprintf(stderr, "[sensors] unknown or unusable IMU profile '%s' at %.0f Hz\n",
                    config->imu_profile, config->imu_rate);
            return -1;
        }
        mgr->use_imu_model = true;
    }
    if (config->gnss_profile != NULL) {
        const gnss_profile_t *pr = gnss_profile_find(config->gnss_profile);
        if (pr == NULL || gnss_model_init(&mgr->gnss_model, pr, config->gnss_rate,
                                          mgr->sim_dt, 1002) != 0) {
            fprintf(stderr, "[sensors] unknown or unusable GNSS profile '%s' at %.0f Hz\n",
                    config->gnss_profile, config->gnss_rate);
            return -1;
        }
        mgr->use_gnss_model = true;
    }
    if (config->baro_profile != NULL) {
        const baro_profile_t *pr = baro_profile_find(config->baro_profile);
        if (pr == NULL || baro_model_init(&mgr->baro_model, pr, config->baro_rate, 1003) != 0) {
            fprintf(stderr, "[sensors] unknown or unusable barometer profile '%s' at %.0f Hz\n",
                    config->baro_profile, config->baro_rate);
            return -1;
        }
        mgr->use_baro_model = true;
    }

    printf("[sensors] initialized: IMU=%d GNSS=%d Baro=%d LiDAR=%d "
           "Camera=%d IR=%d Command=%d\n",
           mgr->config.enable.imu,    mgr->config.enable.gnss,
           mgr->config.enable.baro,   mgr->config.enable.lidar,
           mgr->config.enable.camera, mgr->config.enable.infrared,
           mgr->config.enable.command);
    return 0;
}

static void update_conditions(sensor_mgr_t *mgr, const mjData *data) {
    double fraction = 0.0;
    if (mgr->use_vibration && mgr->thrust_max > 0.0) {
        double rpm[4];
        for (int i = 0; i < 4; i++) {
            double f = data->ctrl[mgr->act_thrust[i]] / mgr->thrust_max;
            f = f < 0.0 ? 0.0 : (f > 1.0 ? 1.0 : f);
            rpm[i] = SENSOR_ROTOR_RATED_RPM * sqrt(f);
            fraction += f * 0.25;
        }
        vibration_step(&mgr->vibration, rpm, mgr->sim_dt, mgr->vib_accel, mgr->vib_gyro);
    }
    mgr->thrust_fraction = fraction;
    double heat = mgr->use_vibration ? 1.0 : 0.0;
    mgr->thermal.self_heating_k = SENSOR_SELF_HEATING_K * heat;
    mgr->sensor_temp_c = thermal_step(&mgr->thermal, mgr->config.ambient_temp_c, mgr->sim_dt);
}

static void read_imu(sensor_mgr_t *mgr, const mjData *data, uint64_t t_ns) {
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
        const double *q = &data->sensordata[mgr->adr_quat];
        double r[9];
        mju_quat2Mat(r, q);
        double field_nt[3];
        for (int i = 0; i < 3; i++) {
            field_nt[i] = 1000.0 * (r[i] * mgr->config.mag_field_enu_ut[0] +
                                    r[3 + i] * mgr->config.mag_field_enu_ut[1] +
                                    r[6 + i] * mgr->config.mag_field_enu_ut[2]);
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

static void step_gnss_model(sensor_mgr_t *mgr, const mjData *data, uint64_t t_ns) {
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
    msg.header.timestamp_ns = t_ns;
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

static void read_gnss(sensor_mgr_t *mgr, const mjData *data, uint64_t t_ns) {
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

    msg.fix_type       = 2;
    msg.num_satellites = 12;
    msg.hdop           = 1.2;

    double noise_m = mgr->config.gnss_pos_noise;
    msg.latitude  += noise_gaussian(&mgr->rng, noise_m / R_EARTH) * (180.0 / M_PI);
    msg.longitude += noise_gaussian(&mgr->rng, noise_m / R_EARTH) * (180.0 / M_PI);
    msg.altitude  += noise_gaussian(&mgr->rng, noise_m);

    mgr->gnss_latest = msg;
    transport_publish(&mgr->pub_gnss, &msg, sizeof(msg));
}

static void read_baro(sensor_mgr_t *mgr, const mjData *data, uint64_t t_ns) {
    sensor_baro_t msg = {0};
    msg.header.timestamp_ns = t_ns;
    msg.header.sequence     = mgr->seq_baro++;
    msg.header.sensor_id    = SENSOR_ID_BARO;

    double true_alt = data->sensordata[mgr->adr_pos + 2] + mgr->config.gnss_origin_alt;
    if (mgr->use_baro_model) {
        atmosphere_sample_t atm = atmosphere_at(mgr->config.ambient_temp_c,
                                                mgr->config.sea_level_pressure_pa, true_alt);
        msg.pressure_pa = baro_model_step(&mgr->baro_model, atm.pressure_pa,
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

static void read_lidar(sensor_mgr_t *mgr, const mjModel *model,
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
        double dir_local[3] = { (double)cosf(angle), (double)sinf(angle), 0.0 };

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

static void read_infrared(sensor_mgr_t *mgr, const mjModel *model,
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

    if (dist >= 0 && dist <= (double)msg.range_max) {
        msg.range_m = (float)dist
                    + (float)noise_gaussian(&mgr->rng, mgr->config.ir_range_noise);
    } else {
        msg.range_m = -1.0F;
    }

    transport_publish(&mgr->pub_infrared, &msg, sizeof(msg));
}

void sensor_update(sensor_mgr_t *mgr, const mjModel *model, mjData *data,
                   setpoint_t *target) {
    uint64_t t_ns = (uint64_t)(data->time * 1e9);
    update_conditions(mgr, data);

    if (mgr->config.enable.imu && tick_due(&mgr->acc_imu, mgr->config.imu_rate, mgr->sim_dt)) {
        read_imu(mgr, data, t_ns);
    }

    if (mgr->config.enable.gnss && mgr->use_gnss_model) {
        step_gnss_model(mgr, data, t_ns);
    } else if (mgr->config.enable.gnss &&
               tick_due(&mgr->acc_gnss, mgr->config.gnss_rate, mgr->sim_dt)) {
        read_gnss(mgr, data, t_ns);
    }

    if (mgr->config.enable.baro && tick_due(&mgr->acc_baro, mgr->config.baro_rate, mgr->sim_dt)) {
        read_baro(mgr, data, t_ns);
    }

    if (mgr->config.enable.lidar &&
        tick_due(&mgr->acc_lidar, mgr->config.lidar_rate, mgr->sim_dt)) {
        read_lidar(mgr, model, data, t_ns);
    }

    if (mgr->config.enable.infrared &&
        tick_due(&mgr->acc_infrared, mgr->config.infrared_rate, mgr->sim_dt)) {
        read_infrared(mgr, model, data, t_ns);
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

void sensor_render_camera(sensor_mgr_t *mgr, const mjModel *model,
                          mjData *data) {
    if (!mgr->config.enable.camera || !mgr->cam_due) {
        return;
    }
    mgr->cam_due = false;

    int w = mgr->config.camera_width;
    int h = mgr->config.camera_height;

    if (!mgr->cam_initialized) {
        mjv_defaultScene(&mgr->cam_scene);
        mjv_makeScene(model, &mgr->cam_scene, 1000);
        mjModel *writable = (mjModel *)model;
        if (writable->vis.global.offwidth < w) {
            writable->vis.global.offwidth = w;
        }
        if (writable->vis.global.offheight < h) {
            writable->vis.global.offheight = h;
        }
        mjr_defaultContext(&mgr->cam_context);
        mgr->cam_context.offWidth  = w;
        mgr->cam_context.offHeight = h;
        mjr_makeContext(model, &mgr->cam_context, mjFONTSCALE_100);

        mjv_defaultCamera(&mgr->cam_view);
        mgr->cam_view.type       = mjCAMERA_FIXED;
        mgr->cam_view.fixedcamid = mgr->camera_id;

        mjv_defaultOption(&mgr->cam_opt);

        mgr->cam_initialized = true;
    }

    mjrRect vp = { 0, 0, w, h };
    mjv_updateScene(model, data, &mgr->cam_opt, NULL,
                    &mgr->cam_view, mjCAT_ALL, &mgr->cam_scene);
    mjr_setBuffer(mjFB_OFFSCREEN, &mgr->cam_context);
    mjr_render(vp, &mgr->cam_scene, &mgr->cam_context);

    uint8_t *pixels = mgr->rgb_buf + sizeof(sensor_camera_rgb_hdr_t);
    mjr_readPixels(pixels, mgr->depth_buf, vp, &mgr->cam_context);

    if (mgr->use_cam_model) {
        uint64_t t_mono = (uint64_t)(data->time * 1e9);
        size_t mono_sz = (size_t)(unsigned)w * (unsigned)h;
        uint8_t *frame = frame_out_loan(&mgr->cam_out, sizeof(sensor_camera_rgb_hdr_t) + mono_sz);
        if (frame == NULL) {
            return;
        }
        uint8_t *mono = frame + sizeof(sensor_camera_rgb_hdr_t);
        camera_model_apply(&mgr->cam_model, pixels, mono, mgr->sensor_temp_c,
                           mgr->config.ambient_temp_c, mgr->config.camera_exposure_s);
        sensor_camera_rgb_hdr_t *mh = (sensor_camera_rgb_hdr_t *)frame;
        memset(mh, 0, sizeof(*mh));
        mh->header.timestamp_ns = t_mono;
        mh->header.sequence     = mgr->seq_camera;
        mh->header.sensor_id    = SENSOR_ID_CAMERA;
        mh->width    = (uint16_t)w;
        mh->height   = (uint16_t)h;
        mh->channels = 1;
        sensor_camera_meta_t mm = {0};
        mm.header.timestamp_ns = t_mono;
        mm.header.sequence     = mgr->seq_camera++;
        mm.header.sensor_id    = SENSOR_ID_CAMERA;
        mm.width    = (uint16_t)w;
        mm.height   = (uint16_t)h;
        mm.channels = 1;
        mm.fov_y    = mgr->config.camera_fov;
        mm.rgb_size = (uint32_t)mono_sz;
        transport_publish(&mgr->pub_camera_meta, &mm, sizeof(mm));
        if (mgr->config.camera_legacy_topic) {
            transport_publish(&mgr->pub_camera_rgb, frame, sizeof(sensor_camera_rgb_hdr_t) + mono_sz);
        }
        (void)frame_out_commit(&mgr->cam_out);
        return;
    }

    size_t row_bytes = (size_t)(unsigned)w * 3U;
    uint8_t *temp_row = malloc(row_bytes);
    if (temp_row) {
        size_t half_rows = (size_t)(unsigned)h / 2U;
        size_t h_sz = (size_t)(unsigned)h;
        for (size_t row = 0; row < half_rows; row++) {
            uint8_t *top = pixels + row * row_bytes;
            uint8_t *bot = pixels + (h_sz - 1U - row) * row_bytes;
            memcpy(temp_row, top, row_bytes);
            memcpy(top, bot, row_bytes);
            memcpy(bot, temp_row, row_bytes);
        }
        free(temp_row);
    }

    uint64_t t_ns = (uint64_t)(data->time * 1e9);
    size_t rgb_sz   = (size_t)(unsigned)w * (unsigned)h * 3;
    size_t depth_sz = (size_t)(unsigned)w * (unsigned)h * sizeof(float);

    /* Fill camera RGB header so downstream consumers (bridge) have metadata */
    sensor_camera_rgb_hdr_t *rgb_hdr = (sensor_camera_rgb_hdr_t *)mgr->rgb_buf;
    rgb_hdr->header.timestamp_ns = t_ns;
    rgb_hdr->header.sequence     = mgr->seq_camera;
    rgb_hdr->header.sensor_id    = SENSOR_ID_CAMERA;
    rgb_hdr->width    = (uint16_t)w;
    rgb_hdr->height   = (uint16_t)h;
    rgb_hdr->channels = 3;

    sensor_camera_meta_t meta = {0};
    meta.header.timestamp_ns = t_ns;
    meta.header.sequence     = mgr->seq_camera++;
    meta.header.sensor_id    = SENSOR_ID_CAMERA;
    meta.width       = (uint16_t)w;
    meta.height      = (uint16_t)h;
    meta.channels    = 3;
    meta.fov_y       = mgr->config.camera_fov;
    meta.rgb_size    = (uint32_t)rgb_sz;
    meta.depth_size  = (uint32_t)depth_sz;
    meta.depth_scale = 1.0F;

    transport_publish(&mgr->pub_camera_meta, &meta, sizeof(meta));
    if (mgr->config.camera_legacy_topic) {
        transport_publish(&mgr->pub_camera_rgb, mgr->rgb_buf,
                          sizeof(sensor_camera_rgb_hdr_t) + rgb_sz);
    }
    uint8_t *frame = frame_out_loan(&mgr->cam_out, sizeof(sensor_camera_rgb_hdr_t) + rgb_sz);
    if (frame != NULL) {
        memcpy(frame, mgr->rgb_buf, sizeof(sensor_camera_rgb_hdr_t) + rgb_sz);
        (void)frame_out_commit(&mgr->cam_out);
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
