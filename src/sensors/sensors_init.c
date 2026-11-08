#include "sensors/sensors_internal.h"
#include "conditions/conditions.h"
#include "sensors/noise.h"

#include <math.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

enum {
    SEED_IMU = 1001,
    SEED_GNSS = 1002,
    SEED_BARO = 1003,
    SEED_MAG = 1004,
    SEED_VIBRATION = 1005,
    SEED_RANGE = 1007,
    NAME_LEN = 160,
    ROTORS = 4
};

#define RATE_EPSILON 1e-9

static void build_topic(char *buf, size_t cap, const char *prefix, const char *suffix) {
    snprintf(buf, cap, "/%s%s", prefix ? prefix : "drone", suffix);
}

static void build_name(char *buf, size_t cap, const char *prefix, const char *name) {
    if (prefix) {
        snprintf(buf, cap, "%s_%s", prefix, name);
    } else {
        snprintf(buf, cap, "%s", name);
    }
}

static int resolve_sensor_adr(const mjModel *mdl, const char *prefix, const char *name) {
    char full[NAME_LEN];
    build_name(full, sizeof(full), prefix, name);
    int sid = mj_name2id(mdl, mjOBJ_SENSOR, full);
    if (sid < 0) {
        fprintf(stderr, "[sensors] WARNING: sensor '%s' not found in model\n", full);
        return -1;
    }
    return mdl->sensor_adr[sid];
}

static int resolve_named(const mjModel *mdl, int kind, const char *prefix, const char *name) {
    char full[NAME_LEN];
    build_name(full, sizeof(full), prefix, name);
    return mj_name2id(mdl, kind, full);
}

static int resolve_handles(sensor_mgr_t *mgr, const mjModel *mdl, const char *prefix) {
    mgr->adr_accel = resolve_sensor_adr(mdl, prefix, "imu_accel");
    mgr->adr_gyro = resolve_sensor_adr(mdl, prefix, "imu_gyro");
    mgr->adr_quat = resolve_sensor_adr(mdl, prefix, "orientation");
    mgr->adr_pos = resolve_sensor_adr(mdl, prefix, "position");
    mgr->adr_heading = resolve_sensor_adr(mdl, prefix, "heading");
    mgr->adr_linvel = resolve_sensor_adr(mdl, prefix, "linvel");
    mgr->adr_mag_y = resolve_sensor_adr(mdl, prefix, "mag_y");
    mgr->adr_mag_z = resolve_sensor_adr(mdl, prefix, "mag_z");
    mgr->site_imu = resolve_named(mdl, mjOBJ_SITE, prefix, "imu_site");
    mgr->site_lidar = resolve_named(mdl, mjOBJ_SITE, prefix, "lidar_site");
    mgr->site_infrared = resolve_named(mdl, mjOBJ_SITE, prefix, "infrared_site");
    mgr->body_base = resolve_named(mdl, mjOBJ_BODY, prefix, "base_link");
    mgr->camera_id = resolve_named(mdl, mjOBJ_CAMERA, prefix, "drone_camera");
    if (mgr->adr_accel < 0 || mgr->adr_gyro < 0 || mgr->adr_quat < 0 || mgr->adr_pos < 0 ||
        mgr->body_base < 0) {
        fprintf(stderr, "[sensors] ERROR: required sensors/bodies missing from model\n");
        return -1;
    }
    if (mgr->config.enable.lidar && mgr->site_lidar < 0) {
        fprintf(stderr, "[sensors] WARNING: lidar_site not found, disabling\n");
        mgr->config.enable.lidar = false;
    }
    if (mgr->config.enable.infrared && mgr->site_infrared < 0) {
        fprintf(stderr, "[sensors] WARNING: infrared_site not found, disabling\n");
        mgr->config.enable.infrared = false;
    }
    if (mgr->config.enable.camera && mgr->camera_id < 0) {
        fprintf(stderr, "[sensors] WARNING: drone_camera not found, disabling\n");
        mgr->config.enable.camera = false;
    }
    return 0;
}

typedef struct {
    const char *suffix;
    const char *label;
    size_t size;
    bool *enabled;
    transport_pub_t *pub;
} publisher_t;

static void advertise_all(sensor_mgr_t *mgr, transport_t *tp, const char *prefix) {
    sensor_enable_t *en = &mgr->config.enable;
    publisher_t table[] = {
        {"/imu", "IMU", sizeof(sensor_imu_t), &en->imu, &mgr->pub_imu},
        {"/gnss", "GNSS", sizeof(sensor_gnss_t), &en->gnss, &mgr->pub_gnss},
        {"/baro", "barometer", sizeof(sensor_baro_t), &en->baro, &mgr->pub_baro},
        {"/lidar", "LiDAR", sizeof(sensor_lidar_t), &en->lidar, &mgr->pub_lidar},
        {"/infrared", "infrared", sizeof(sensor_infrared_t), &en->infrared, &mgr->pub_infrared},
    };
    for (size_t idx = 0; idx < sizeof(table) / sizeof(table[0]); idx++) {
        if (!*table[idx].enabled) {
            continue;
        }
        char topic[NAME_LEN];
        build_topic(topic, sizeof(topic), prefix, table[idx].suffix);
        if (transport_advertise(tp, topic, table[idx].size, table[idx].pub) != 0) {
            fprintf(stderr, "[sensors] ERROR: failed to advertise %s, disabling %s\n", topic,
                    table[idx].label);
            *table[idx].enabled = false;
        }
    }
    if (en->command) {
        char topic[NAME_LEN];
        build_topic(topic, sizeof(topic), prefix, "/command");
        if (transport_subscribe(tp, topic, sizeof(command_setpoint_t), &mgr->sub_command) != 0) {
            fprintf(stderr, "[sensors] ERROR: failed to subscribe to %s, command uplink disabled\n",
                    topic);
            en->command = false;
        }
    }
}

static int check_rates(const sensor_mgr_t *mgr, const sensor_config_t *cfg) {
    const struct {
        const char *name;
        double rate;
        const char *profile;
    } rates[] = {
        {"IMU", cfg->imu_rate, cfg->imu_profile},
        {"GNSS", cfg->gnss_rate, cfg->gnss_profile},
        {"barometer", cfg->baro_rate, cfg->baro_profile},
    };
    for (size_t idx = 0; idx < sizeof(rates) / sizeof(rates[0]); idx++) {
        if (rates[idx].profile != NULL && rates[idx].rate * mgr->sim_dt > 1.0 + RATE_EPSILON) {
            fprintf(stderr, "[sensors] %s rate %.0f Hz exceeds the simulation step rate %.0f Hz\n",
                    rates[idx].name, rates[idx].rate, 1.0 / mgr->sim_dt);
            return -1;
        }
    }
    return 0;
}

static int init_environment(sensor_mgr_t *mgr, const sensor_config_t *cfg) {
    if (cfg->weather != NULL) {
        const conditions_profile_t *wea = conditions_find(cfg->weather);
        if (wea == NULL) {
            fprintf(stderr, "[sensors] unknown weather preset '%s'\n", cfg->weather);
            return -1;
        }
        mgr->config.ambient_temp_c = wea->ambient_temp_c;
        mgr->config.sea_level_pressure_pa = wea->sea_level_pressure_pa;
    }
    thermal_init(&mgr->thermal, mgr->config.ambient_temp_c, SENSOR_THERMAL_TAU_S,
                 SENSOR_SELF_HEATING_K);
    mgr->sensor_temp_c = mgr->config.ambient_temp_c;
    return 0;
}

static int init_imu_gnss_baro(sensor_mgr_t *mgr, const sensor_config_t *cfg) {
    if (cfg->imu_profile != NULL) {
        const imu_profile_t *prf = imu_profile_find(cfg->imu_profile);
        if (prf == NULL || imu_model_init(&mgr->imu_model, prf, cfg->imu_rate, SEED_IMU) != 0) {
            fprintf(stderr, "[sensors] unknown or unusable IMU profile '%s' at %.0f Hz\n",
                    cfg->imu_profile, cfg->imu_rate);
            return -1;
        }
        mgr->use_imu_model = true;
    }
    if (cfg->gnss_profile != NULL) {
        const gnss_profile_t *prf = gnss_profile_find(cfg->gnss_profile);
        if (prf == NULL ||
            gnss_model_init(&mgr->gnss_model, prf, cfg->gnss_rate, mgr->sim_dt, SEED_GNSS) != 0) {
            fprintf(stderr, "[sensors] unknown or unusable GNSS profile '%s' at %.0f Hz\n",
                    cfg->gnss_profile, cfg->gnss_rate);
            return -1;
        }
        mgr->gnss_model.warm_start = cfg->gnss_cold_start ? 0 : 1;
        mgr->use_gnss_model = true;
    }
    if (cfg->baro_profile != NULL) {
        const baro_profile_t *prf = baro_profile_find(cfg->baro_profile);
        if (prf == NULL || baro_model_init(&mgr->baro_model, prf, cfg->baro_rate, SEED_BARO) != 0) {
            fprintf(stderr, "[sensors] unknown or unusable barometer profile '%s' at %.0f Hz\n",
                    cfg->baro_profile, cfg->baro_rate);
            return -1;
        }
        mgr->use_baro_model = true;
    }
    return 0;
}

static int init_mag_range(sensor_mgr_t *mgr, const sensor_config_t *cfg) {
    if (cfg->mag_profile != NULL) {
        const mag_profile_t *prf = mag_profile_find(cfg->mag_profile);
        double rate = cfg->mag_rate;
        if (prf != NULL && rate <= 0.0) {
            rate = prf->max_rate_hz < MAG_DEFAULT_RATE_HZ ? prf->max_rate_hz : MAG_DEFAULT_RATE_HZ;
            mgr->config.mag_rate = rate;
        }
        if (prf == NULL || mag_model_init(&mgr->mag_model, prf, rate, SEED_MAG) != 0) {
            fprintf(stderr, "[sensors] unknown or unusable magnetometer profile '%s'\n",
                    cfg->mag_profile);
            return -1;
        }
        mgr->mag_every = (int)lround(cfg->imu_rate / mgr->config.mag_rate);
        mgr->mag_every = mgr->mag_every < 1 ? 1 : mgr->mag_every;
        mgr->mag_count = 0;
        mgr->use_mag_model = true;
    }
    if (cfg->range_profile != NULL) {
        const range_profile_t *prf = range_profile_find(cfg->range_profile);
        if (prf == NULL ||
            range_model_init(&mgr->range_model, prf, cfg->infrared_rate, SEED_RANGE) != 0) {
            fprintf(stderr, "[sensors] unknown or unusable rangefinder profile '%s' at %.0f Hz\n",
                    cfg->range_profile, cfg->infrared_rate);
            return -1;
        }
        mgr->use_range_model = true;
    }
    return 0;
}

static int init_vibration(sensor_mgr_t *mgr, const mjModel *mdl, const sensor_config_t *cfg,
                          const char *prefix) {
    if (cfg->vibration_scale <= 0.0) {
        return 0;
    }
    bool found = true;
    for (int rotor = 0; rotor < ROTORS; rotor++) {
        char base[32];
        snprintf(base, sizeof(base), "thrust_%d", rotor);
        mgr->act_thrust[rotor] = resolve_named(mdl, mjOBJ_ACTUATOR, prefix, base);
        found = found && mgr->act_thrust[rotor] >= 0;
    }
    if (!found ||
        vibration_init(&mgr->vibration, ROTORS, 2, SENSOR_ROTOR_RATED_RPM,
                       SENSOR_VIBRATION_BASE_MS2 * cfg->vibration_scale, SENSOR_MOUNT_HZ,
                       SENSOR_MOUNT_ZETA, SEED_VIBRATION) != 0) {
        fprintf(stderr, "[sensors] vibration needs actuators thrust_0..3\n");
        return -1;
    }
    mgr->thrust_max = mdl->actuator_ctrlrange[2 * mgr->act_thrust[0] + 1];
    mgr->use_vibration = true;
    return 0;
}

int sensor_init(sensor_mgr_t *mgr, const mjModel *model, transport_t *tp,
                const sensor_config_t *config, const char *topic_prefix) {
    memset(mgr, 0, sizeof(*mgr));
    mgr->config = *config;
    mgr->sim_dt = model->opt.timestep;
    mgr->acc_imu = mgr->acc_gnss = mgr->acc_baro = 1.0;
    mgr->acc_lidar = mgr->acc_camera = mgr->acc_infrared = 1.0;
    noise_seed(&mgr->rng, 42);

    int failed = resolve_handles(mgr, model, topic_prefix) != 0 || check_rates(mgr, config) != 0 ||
                 init_environment(mgr, config) != 0 || init_imu_gnss_baro(mgr, config) != 0 ||
                 init_mag_range(mgr, config) != 0 ||
                 init_vibration(mgr, model, config, topic_prefix) != 0;
    if (!failed) {
        advertise_all(mgr, tp, topic_prefix);
        failed = sensor_camera_setup(mgr, model, tp, &mgr->config, topic_prefix) != 0;
    }
    if (failed) {
        sensor_cleanup(mgr);
        return -1;
    }
    printf("[sensors] initialized: IMU=%d GNSS=%d Baro=%d LiDAR=%d Camera=%d IR=%d Command=%d\n",
           mgr->config.enable.imu, mgr->config.enable.gnss, mgr->config.enable.baro,
           mgr->config.enable.lidar, mgr->config.enable.camera, mgr->config.enable.infrared,
           mgr->config.enable.command);
    return 0;
}
