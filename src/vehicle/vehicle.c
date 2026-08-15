#include "vehicle/vehicle.h"

#include <stdio.h>
#include <string.h>

static bool sample_is_fresh(uint32_t *last_seen, uint32_t seq) {
    if (seq == *last_seen) return false;
    *last_seen = seq;
    return true;
}

int vehicle_resolve(vehicle_t *vehicle, const mjModel *model,
                    transport_t *tp) {
    if (!vehicle || !model || !vehicle->airframe) return -1;

    const airframe_spec_t *af = vehicle->airframe;

    for (int i = 0; i < af->channel_count; i++) {
        char full[64];
        snprintf(full, sizeof(full), "%s%s", vehicle->prefix,
                 af->channel_names[i]);
        vehicle->actuator_index[i] = mj_name2id(model, mjOBJ_ACTUATOR, full);
        if (vehicle->actuator_index[i] < 0) {
            fprintf(stderr, "[vehicle] ERROR: actuator '%s' not found\n",
                    full);
            return -1;
        }
    }
    for (int i = af->channel_count; i < VEHICLE_MAX_ACTUATOR_CHANNELS; i++) {
        vehicle->actuator_index[i] = -1;
    }

    if (vehicle->controller.ops && vehicle->controller.ops->init) {
        if (vehicle->controller.ops->init(&vehicle->controller,
                                          af->channel_count) != 0) {
            fprintf(stderr, "[vehicle] ERROR: controller '%s' init failed\n",
                    vehicle->controller.ops->name);
            return -1;
        }
    }
    if (vehicle->estimator.ops && vehicle->estimator.ops->init) {
        if (vehicle->estimator.ops->init(&vehicle->estimator) != 0) {
            fprintf(stderr, "[vehicle] ERROR: estimator '%s' init failed\n",
                    vehicle->estimator.ops->name);
            return -1;
        }
    }
    if (vehicle->localization.ops && vehicle->localization.ops->init) {
        if (vehicle->localization.ops->init(&vehicle->localization) != 0) {
            fprintf(stderr, "[vehicle] ERROR: localization '%s' init failed\n",
                    vehicle->localization.ops->name);
            return -1;
        }
    }

    sensor_config_t scfg = sensor_default_config();
    scfg.enable.camera = false;

    char topic_root[VEHICLE_MAX_PREFIX_LEN];
    size_t prefix_len = strlen(vehicle->prefix);
    size_t root_len = (prefix_len > 0 && vehicle->prefix[prefix_len - 1] == '_')
                      ? prefix_len - 1 : prefix_len;
    if (root_len >= sizeof(topic_root)) root_len = sizeof(topic_root) - 1;
    memcpy(topic_root, vehicle->prefix, root_len);
    topic_root[root_len] = '\0';

    if (sensor_init(&vehicle->sensors, model, tp, &scfg, topic_root) != 0) {
        fprintf(stderr, "[vehicle] ERROR: sensor_init failed for %s\n",
                vehicle->prefix);
        return -1;
    }

    return 0;
}

void vehicle_step(vehicle_t *vehicle, const mjModel *model, mjData *data,
                  double dt) {
    if (!vehicle || !model || !data) return;

    sensor_snapshot_t snap;
    memset(&snap, 0, sizeof(snap));
    if (vehicle->sensors.config.enable.imu &&
        sample_is_fresh(&vehicle->last_imu_seq,
                        vehicle->sensors.imu_latest.header.sequence)) {
        snap.has_imu = true;
        snap.imu = vehicle->sensors.imu_latest;
    }
    if (vehicle->sensors.config.enable.gnss &&
        sample_is_fresh(&vehicle->last_gnss_seq,
                        vehicle->sensors.gnss_latest.header.sequence)) {
        snap.has_gnss = true;
        snap.gnss = vehicle->sensors.gnss_latest;
    }
    if (vehicle->sensors.config.enable.baro &&
        sample_is_fresh(&vehicle->last_baro_seq,
                        vehicle->sensors.baro_latest.header.sequence)) {
        snap.has_baro = true;
        snap.baro = vehicle->sensors.baro_latest;
    }

    if (vehicle->estimator.ops && vehicle->estimator.ops->ingest) {
        vehicle->estimator.ops->ingest(&vehicle->estimator, &snap, dt);
    }
    if (vehicle->localization.ops && vehicle->localization.ops->ingest) {
        vehicle->localization.ops->ingest(&vehicle->localization, &snap);
    }

    estimated_state_t est;
    memset(&est, 0, sizeof(est));
    if (vehicle->estimator.ops && vehicle->estimator.ops->get_state) {
        vehicle->estimator.ops->get_state(&vehicle->estimator, &est);
    }

    actuator_command_t out;
    memset(&out, 0, sizeof(out));
    if (vehicle->controller.ops && vehicle->controller.ops->update) {
        vehicle->controller.ops->update(&vehicle->controller, &est,
                                        &vehicle->target, dt, &out);
    }

    for (int i = 0; i < out.count; i++) {
        int id = vehicle->actuator_index[i];
        if (id >= 0 && id < model->nu) {
            data->ctrl[id] = out.value[i];
        }
    }
}

void vehicle_cleanup(vehicle_t *vehicle) {
    if (!vehicle) return;
    if (vehicle->controller.ops && vehicle->controller.ops->destroy) {
        vehicle->controller.ops->destroy(&vehicle->controller);
    }
    if (vehicle->estimator.ops && vehicle->estimator.ops->destroy) {
        vehicle->estimator.ops->destroy(&vehicle->estimator);
    }
    if (vehicle->localization.ops && vehicle->localization.ops->destroy) {
        vehicle->localization.ops->destroy(&vehicle->localization);
    }
    sensor_cleanup(&vehicle->sensors);
}
