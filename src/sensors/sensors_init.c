#include "sensors/sensors_internal.h"
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
        double mag_rate = config->mag_rate;
        if (pr != NULL && mag_rate <= 0.0) {
            mag_rate = pr->max_rate_hz < MAG_DEFAULT_RATE_HZ ? pr->max_rate_hz : MAG_DEFAULT_RATE_HZ;
            mgr->config.mag_rate = mag_rate;
        }
        if (pr == NULL || mag_model_init(&mgr->mag_model, pr, mag_rate, 1004) != 0) {
            fprintf(stderr, "[sensors] unknown or unusable magnetometer profile '%s'\n",
                    config->mag_profile);
            return -1;
        }
        mgr->mag_every = (int)lround(config->imu_rate / mgr->config.mag_rate);
        if (mgr->mag_every < 1) mgr->mag_every = 1;
        mgr->mag_count = 0;
        mgr->use_mag_model = true;
    }
    if (config->range_profile != NULL) {
        const range_profile_t *pr = range_profile_find(config->range_profile);
        if (pr == NULL || range_model_init(&mgr->range_model, pr, config->infrared_rate, 1007) != 0) {
            fprintf(stderr, "[sensors] unknown or unusable rangefinder profile '%s' at %.0f Hz\n",
                    config->range_profile, config->infrared_rate);
            return -1;
        }
        mgr->use_range_model = true;
    }
    if (config->vibration_scale > 0.0) {
        int ok = 1;
        char nm[32];
        for (int i = 0; i < 4; i++) {
            char full[64];
            snprintf(nm, sizeof(nm), "thrust_%d", i);
            build_name(full, sizeof(full), topic_prefix, nm);
            mgr->act_thrust[i] = mj_name2id(model, mjOBJ_ACTUATOR, full);
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
        mgr->gnss_model.warm_start = config->gnss_cold_start ? 0 : 1;
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
