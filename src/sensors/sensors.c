#include "sensors/sensors.h"
#include "sensors/noise.h"

#include <math.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define TOPIC_IMU           "/drone/imu"
#define TOPIC_GNSS          "/drone/gnss"
#define TOPIC_BARO          "/drone/baro"
#define TOPIC_LIDAR         "/drone/lidar"
#define TOPIC_INFRARED      "/drone/infrared"
#define TOPIC_CAMERA_META   "/drone/camera/meta"
#define TOPIC_CAMERA_RGB    "/drone/camera/rgb"
#define TOPIC_CAMERA_DEPTH  "/drone/camera/depth"
#define TOPIC_COMMAND       "/drone/command"

static int resolve_sensor_adr(const mjModel *m, const char *name) {
    int id = mj_name2id(m, mjOBJ_SENSOR, name);
    if (id < 0) {
        fprintf(stderr, "[sensors] WARNING: sensor '%s' not found in model\n",
                name);
        return -1;
    }
    return m->sensor_adr[id];
}

sensor_config_t sensor_default_config(void) {
    sensor_config_t cfg = {0};

    cfg.enable.imu      = true;
    cfg.enable.gnss     = true;
    cfg.enable.baro     = true;
    cfg.enable.lidar    = true;
    cfg.enable.camera   = true;
    cfg.enable.infrared = true;

    cfg.imu_rate      = 1000.0;
    cfg.gnss_rate     = 10.0;
    cfg.baro_rate     = 50.0;
    cfg.lidar_rate    = 30.0;
    cfg.camera_rate   = 30.0;
    cfg.infrared_rate = 100.0;

    cfg.lidar_num_rays  = 36;
    cfg.lidar_range_max = 30.0F;

    cfg.camera_width  = 320;
    cfg.camera_height = 240;
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

    return cfg;
}

static int rate_to_interval(double rate_hz, double timestep) {
    if (rate_hz <= 0.0) return 0;
    int n = (int)(1.0 / (rate_hz * timestep));
    return n < 1 ? 1 : n;
}

int sensor_init(sensor_mgr_t *mgr, const mjModel *model,
                transport_t *tp, const sensor_config_t *config) {
    memset(mgr, 0, sizeof(*mgr));
    mgr->config = *config;

    double dt = model->opt.timestep;

    mgr->adr_accel   = resolve_sensor_adr(model, "imu_accel");
    mgr->adr_gyro    = resolve_sensor_adr(model, "imu_gyro");
    mgr->adr_quat    = resolve_sensor_adr(model, "orientation");
    mgr->adr_pos     = resolve_sensor_adr(model, "position");
    mgr->adr_heading = resolve_sensor_adr(model, "heading");
    mgr->adr_linvel  = resolve_sensor_adr(model, "linvel");
    mgr->adr_mag_y   = resolve_sensor_adr(model, "mag_y");
    mgr->adr_mag_z   = resolve_sensor_adr(model, "mag_z");

    mgr->site_imu      = mj_name2id(model, mjOBJ_SITE, "imu_site");
    mgr->site_lidar    = mj_name2id(model, mjOBJ_SITE, "lidar_site");
    mgr->site_infrared = mj_name2id(model, mjOBJ_SITE, "infrared_site");
    mgr->body_base     = mj_name2id(model, mjOBJ_BODY, "base_link");
    mgr->camera_id     = mj_name2id(model, mjOBJ_CAMERA, "drone_camera");

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

    mgr->inv_imu      = rate_to_interval(config->imu_rate,      dt);
    mgr->inv_gnss     = rate_to_interval(config->gnss_rate,     dt);
    mgr->inv_baro     = rate_to_interval(config->baro_rate,     dt);
    mgr->inv_lidar    = rate_to_interval(config->lidar_rate,    dt);
    mgr->inv_camera   = rate_to_interval(config->camera_rate,   dt);
    mgr->inv_infrared = rate_to_interval(config->infrared_rate, dt);

    mgr->dec_imu = mgr->dec_gnss = mgr->dec_baro = 1;
    mgr->dec_lidar = mgr->dec_camera = mgr->dec_infrared = 1;

    if (mgr->config.enable.imu) {
        transport_advertise(tp, TOPIC_IMU,
                            sizeof(sensor_imu_t), &mgr->pub_imu);
    }

    if (mgr->config.enable.gnss) {
        transport_advertise(tp, TOPIC_GNSS,
                            sizeof(sensor_gnss_t), &mgr->pub_gnss);
    }

    if (mgr->config.enable.baro) {
        transport_advertise(tp, TOPIC_BARO,
                            sizeof(sensor_baro_t), &mgr->pub_baro);
    }

    if (mgr->config.enable.lidar) {
        transport_advertise(tp, TOPIC_LIDAR,
                            sizeof(sensor_lidar_t), &mgr->pub_lidar);
    }

    if (mgr->config.enable.infrared) {
        transport_advertise(tp, TOPIC_INFRARED,
                            sizeof(sensor_infrared_t), &mgr->pub_infrared);
    }

    if (mgr->config.enable.camera) {
        size_t rgb_sz   = (size_t)config->camera_width *
                          config->camera_height * 3;
        size_t rgb_total = sizeof(sensor_camera_rgb_hdr_t) + rgb_sz;
        size_t depth_sz = (size_t)config->camera_width *
                          config->camera_height * sizeof(float);

        transport_advertise(tp, TOPIC_CAMERA_META,
                            sizeof(sensor_camera_meta_t),
                            &mgr->pub_camera_meta);
        transport_advertise(tp, TOPIC_CAMERA_RGB,   rgb_total,
                            &mgr->pub_camera_rgb);
        transport_advertise(tp, TOPIC_CAMERA_DEPTH, depth_sz,
                            &mgr->pub_camera_depth);

        mgr->rgb_buf   = malloc(rgb_total);
        mgr->depth_buf = malloc(depth_sz);
        if (!mgr->rgb_buf || !mgr->depth_buf) {
            fprintf(stderr, "[sensors] ERROR: failed to allocate camera "
                            "buffers\n");
            mgr->config.enable.camera = false;
        }
    }

    transport_subscribe(tp, TOPIC_COMMAND,
                        sizeof(command_setpoint_t), &mgr->sub_command);

    noise_seed(&mgr->rng, 42);

    printf("[sensors] initialized: IMU=%d GNSS=%d Baro=%d LiDAR=%d "
           "Camera=%d IR=%d\n",
           mgr->config.enable.imu,    mgr->config.enable.gnss,
           mgr->config.enable.baro,   mgr->config.enable.lidar,
           mgr->config.enable.camera, mgr->config.enable.infrared);
    return 0;
}

static void read_imu(sensor_mgr_t *mgr, const mjData *data, uint64_t t_ns) {
    sensor_imu_t msg = {0};
    msg.header.timestamp_ns = t_ns;
    msg.header.sequence     = mgr->seq_imu++;
    msg.header.sensor_id    = SENSOR_ID_IMU;

    double dt = 0.001;

    for (int i = 0; i < 3; i++) {
        double bias_g = noise_random_walk(&mgr->rng, &mgr->gyro_bias[i],
                                          mgr->config.imu_gyro_bias_diffusion,
                                          dt);
        msg.accel[i] = data->sensordata[mgr->adr_accel + i]
                     + noise_gaussian(&mgr->rng, mgr->config.imu_accel_noise);
        msg.gyro[i]  = data->sensordata[mgr->adr_gyro + i]
                     + noise_gaussian(&mgr->rng, mgr->config.imu_gyro_noise) + bias_g;
    }

    if (mgr->adr_heading >= 0) {
        msg.mag[0] = data->sensordata[mgr->adr_heading];
        msg.mag[1] = data->sensordata[mgr->adr_heading + 1];
        msg.mag[2] = data->sensordata[mgr->adr_heading + 2];
    }

    if (mgr->adr_quat >= 0) {
        memcpy(msg.orientation, &data->sensordata[mgr->adr_quat],
               4 * sizeof(double));
    }

    transport_publish(&mgr->pub_imu, &msg, sizeof(msg));
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

    transport_publish(&mgr->pub_gnss, &msg, sizeof(msg));
}

static void read_baro(sensor_mgr_t *mgr, const mjData *data, uint64_t t_ns) {
    sensor_baro_t msg = {0};
    msg.header.timestamp_ns = t_ns;
    msg.header.sequence     = mgr->seq_baro++;
    msg.header.sensor_id    = SENSOR_ID_BARO;

    double alt = data->sensordata[mgr->adr_pos + 2]
               + mgr->config.gnss_origin_alt
               + noise_gaussian(&mgr->rng, mgr->config.baro_alt_noise);

    msg.altitude_m    = alt;
    msg.temperature_c = 15.0;
    msg.pressure_pa = 101325.0 * pow(1.0 - 0.0065 * alt / 288.15, 5.2561);

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

    /* Debug counter: print publish activity every 500 IMU steps (~5 s) */
    static unsigned long dbg_step;
    bool dbg_print = (++dbg_step % 500 == 1);

    if (mgr->config.enable.imu && mgr->inv_imu > 0 && --mgr->dec_imu <= 0) {
        mgr->dec_imu = mgr->inv_imu;
        read_imu(mgr, data, t_ns);
        if (dbg_print) {
            fprintf(stderr, "[sensors] step %lu: IMU published (seq=%u)\n",
                    dbg_step, mgr->seq_imu - 1);
        }
    }

    if (mgr->config.enable.gnss && mgr->inv_gnss > 0 && --mgr->dec_gnss <= 0) {
        mgr->dec_gnss = mgr->inv_gnss;
        read_gnss(mgr, data, t_ns);
        if (dbg_print) {
            fprintf(stderr, "[sensors] step %lu: GNSS published (seq=%u)\n",
                    dbg_step, mgr->seq_gnss - 1);
        }
    }

    if (mgr->config.enable.baro && mgr->inv_baro > 0 && --mgr->dec_baro <= 0) {
        mgr->dec_baro = mgr->inv_baro;
        read_baro(mgr, data, t_ns);
        if (dbg_print) {
            fprintf(stderr, "[sensors] step %lu: Baro published (seq=%u)\n",
                    dbg_step, mgr->seq_baro - 1);
        }
    }

    if (mgr->config.enable.lidar && mgr->inv_lidar > 0 &&
        --mgr->dec_lidar <= 0) {
        mgr->dec_lidar = mgr->inv_lidar;
        read_lidar(mgr, model, data, t_ns);
        if (dbg_print) {
            fprintf(stderr, "[sensors] step %lu: LiDAR published (seq=%u)\n",
                    dbg_step, mgr->seq_lidar - 1);
        }
    }

    if (mgr->config.enable.infrared && mgr->inv_infrared > 0 &&
        --mgr->dec_infrared <= 0) {
        mgr->dec_infrared = mgr->inv_infrared;
        read_infrared(mgr, model, data, t_ns);
        if (dbg_print) {
            fprintf(stderr, "[sensors] step %lu: IR published (seq=%u)\n",
                    dbg_step, mgr->seq_infrared - 1);
        }
    }

    if (mgr->config.enable.camera && mgr->inv_camera > 0 &&
        --mgr->dec_camera <= 0) {
        mgr->dec_camera = mgr->inv_camera;
        mgr->cam_due = true;
    }

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

    transport_publish(&mgr->pub_camera_meta,  &meta,        sizeof(meta));
    transport_publish(&mgr->pub_camera_rgb,   mgr->rgb_buf,
                      sizeof(sensor_camera_rgb_hdr_t) + rgb_sz);
    transport_publish(&mgr->pub_camera_depth, mgr->depth_buf, depth_sz);
}

void sensor_cleanup(sensor_mgr_t *mgr) {
    transport_close_pub(&mgr->pub_imu);
    transport_close_pub(&mgr->pub_gnss);
    transport_close_pub(&mgr->pub_baro);
    transport_close_pub(&mgr->pub_lidar);
    transport_close_pub(&mgr->pub_infrared);
    transport_close_pub(&mgr->pub_camera_meta);
    transport_close_pub(&mgr->pub_camera_rgb);
    transport_close_pub(&mgr->pub_camera_depth);

    transport_close_sub(&mgr->sub_command);

    if (mgr->cam_initialized) {
        mjv_freeScene(&mgr->cam_scene);
        mjr_freeContext(&mgr->cam_context);
        mgr->cam_initialized = false;
    }
    free(mgr->rgb_buf);
    free(mgr->depth_buf);
    mgr->rgb_buf   = NULL;
    mgr->depth_buf = NULL;
}
