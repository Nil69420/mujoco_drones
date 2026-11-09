#include "bridge/px4_runner.h"

#include "bridge/ap_frames.h"
#include "bridge/px4_link.h"
#include "bridge/truth.h"

#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

enum { SYSTEM_ID = 1, COMPONENT_ID = 51, MESSAGE_BYTES = 128, STARTUP_WAIT_MS = 5 };

static const double STEP_S = 0.004;
static const double GPS_PERIOD_S = 0.1;

struct px4_runner {
    px4_link_t *link;
    ap_map_t map;
    px4_home_t home;
    uint8_t seq;
    double next_gps_s;
    bool awaiting;
    bool replied_once;
    bool have_origin;
    double origin_enu[3];
    px4_actuator_controls_t controls;
    bool announced;
    uint32_t last_imu_seq;
    uint32_t last_baro_seq;
};

px4_runner_t *px4_runner_create(const ap_map_t *map, const px4_home_t *home, uint16_t port) {
    if (map == NULL || home == NULL || ap_map_validate(map) != 0) {
        return NULL;
    }
    px4_runner_t *runner = calloc(1, sizeof(*runner));
    if (runner == NULL) {
        return NULL;
    }
    runner->link = px4_link_open("127.0.0.1", port);
    if (runner->link == NULL) {
        free(runner);
        return NULL;
    }
    runner->map = *map;
    runner->home = *home;
    return runner;
}

uint16_t px4_runner_port(const px4_runner_t *runner) {
    return runner != NULL ? px4_link_port(runner->link) : 0;
}

static void overlay_models(px4_runner_t *runner, const vehicle_t *vehicle, px4_hil_sensor_t *sensor) {
    const sensor_mgr_t *mgr = &vehicle->sensors;
    px4_model_samples_t samples;
    memset(&samples, 0, sizeof(samples));
    uint32_t imu_seq = mgr->imu_latest.header.sequence;
    uint32_t baro_seq = mgr->baro_latest.header.sequence;
    if (mgr->config.enable.imu && imu_seq != 0) {
        samples.imu = &mgr->imu_latest;
        samples.imu_fresh = imu_seq != runner->last_imu_seq;
        samples.mag_modelled = mgr->use_mag_model;
    }
    if (mgr->config.enable.baro && baro_seq != 0) {
        samples.baro = &mgr->baro_latest;
        samples.baro_fresh = baro_seq != runner->last_baro_seq;
    }
    runner->last_imu_seq = imu_seq;
    runner->last_baro_seq = baro_seq;
    px4_sensor_apply_models(sensor, &samples);
}

static int send_state(px4_runner_t *runner, const vehicle_t *vehicle, const ap_state_t *state) {
    uint8_t buf[MESSAGE_BYTES];
    px4_hil_sensor_t sensor;
    px4_sensor_from_state(state, &runner->home, &sensor);
    overlay_models(runner, vehicle, &sensor);
    size_t len = px4_encode_hil_sensor(buf, sizeof(buf), runner->seq++, SYSTEM_ID, COMPONENT_ID, &sensor);
    if (len == 0 || px4_link_send(runner->link, buf, len) != PX4_LINK_READY) {
        return -1;
    }
    if (state->timestamp_s >= runner->next_gps_s) {
        px4_hil_gps_t gps;
        px4_gps_from_state(state, &runner->home, &gps);
        len = px4_encode_hil_gps(buf, sizeof(buf), runner->seq++, SYSTEM_ID, COMPONENT_ID, &gps);
        if (len == 0 || px4_link_send(runner->link, buf, len) != PX4_LINK_READY) {
            return -1;
        }
        runner->next_gps_s = state->timestamp_s + GPS_PERIOD_S;
    }
    return 0;
}

static void advance(const mjModel *model, mjData *data, px4_tick_fn tick, void *ctx) {
    double target = data->time + STEP_S;
    double half_step = 0.5 * model->opt.timestep;
    while (data->time < target - half_step) {
        tick(ctx);
    }
}

static void apply_controls(const px4_runner_t *runner, const vehicle_t *vehicle, const mjModel *model,
                           mjData *data) {
    actuator_command_t command;
    ap_map_apply_fractions(&runner->map, runner->controls.controls, PX4_ACTUATOR_COUNT, &command);
    bridge_apply_command(vehicle, model, data, &command);
}

int px4_runner_exchange(px4_runner_t *runner, const vehicle_t *vehicle, const mjModel *model,
                        mjData *data, px4_tick_fn tick, void *ctx, int timeout_ms) {
    if (runner == NULL || vehicle == NULL || model == NULL || data == NULL || tick == NULL ||
        vehicle->base_body_id < 0) {
        return PX4_RUNNER_FAILED;
    }
    if (!px4_link_connected(runner->link)) {
        if (runner->announced) {
            fprintf(stderr, "[px4] connection to PX4 lost at t=%.3f s\n", data->time);
            return PX4_RUNNER_FAILED;
        }
        int status = px4_link_accept(runner->link, timeout_ms);
        if (status == PX4_LINK_ERROR) {
            return PX4_RUNNER_FAILED;
        }
        runner->announced = status == PX4_LINK_READY;
        return PX4_RUNNER_WAITING;
    }
    if (!runner->have_origin) {
        for (int i = 0; i < 3; i++) {
            runner->origin_enu[i] = data->xpos[3 * vehicle->base_body_id + i];
        }
        runner->have_origin = true;
    }
    if (!runner->awaiting) {
        ap_truth_t truth;
        ap_state_t state;
        bridge_read_truth(vehicle, model, data, runner->origin_enu, &truth);
        memset(&state, 0, sizeof(state));
        ap_truth_to_state(&truth, &state);
        if (send_state(runner, vehicle, &state) != 0) {
            fprintf(stderr, "[px4] sending sensors failed at t=%.3f s (link closed?)\n", data->time);
            return PX4_RUNNER_FAILED;
        }
        runner->awaiting = true;
    }
    px4_actuator_controls_t fresh;
    int wait_ms = runner->replied_once ? timeout_ms : STARTUP_WAIT_MS;
    int status = px4_link_wait_controls(runner->link, wait_ms, &fresh);
    if (status == PX4_LINK_ERROR) {
        fprintf(stderr, "[px4] link closed by PX4 at t=%.3f s\n", data->time);
        return PX4_RUNNER_FAILED;
    }
    if (status == PX4_LINK_READY) {
        runner->controls = fresh;
        runner->replied_once = true;
    } else if (runner->replied_once) {
        return PX4_RUNNER_WAITING;
    }
    runner->awaiting = false;
    apply_controls(runner, vehicle, model, data);
    advance(model, data, tick, ctx);
    return PX4_RUNNER_EXCHANGED;
}

void px4_runner_destroy(px4_runner_t *runner) {
    if (runner == NULL) {
        return;
    }
    px4_link_close(runner->link);
    free(runner);
}
