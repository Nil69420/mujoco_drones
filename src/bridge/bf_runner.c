#include "bridge/bf_runner.h"

#include "bridge/bf_link.h"
#include "bridge/truth.h"

#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

enum { STARTUP_WAIT_MS = 2, RESYNC_MS = 700 };

static const double STEP_S = 0.001;

struct bf_runner {
    bf_link_t *link;
    ap_map_t map;
    px4_home_t home;
    bool awaiting;
    bool replied_once;
    bool have_origin;
    int waited_ms;
    double origin_enu[3];
    float motors[BF_MOTOR_COUNT];
};

bf_runner_t *bf_runner_create(const ap_map_t *map, const px4_home_t *home, uint16_t motors_port,
                              uint16_t fdm_port) {
    if (map == NULL || home == NULL || ap_map_validate(map) != 0) {
        return NULL;
    }
    bf_runner_t *runner = calloc(1, sizeof(*runner));
    if (runner == NULL) {
        return NULL;
    }
    runner->link = bf_link_open("127.0.0.1", motors_port, fdm_port);
    if (runner->link == NULL) {
        free(runner);
        return NULL;
    }
    runner->map = *map;
    runner->home = *home;
    return runner;
}

uint16_t bf_runner_port(const bf_runner_t *runner) {
    return runner != NULL ? bf_link_port(runner->link) : 0;
}

static void advance(const mjModel *model, mjData *data, bf_tick_fn tick, void *ctx) {
    double target = data->time + STEP_S;
    double half_step = 0.5 * model->opt.timestep;
    while (data->time < target - half_step) {
        tick(ctx);
    }
}

int bf_runner_exchange(bf_runner_t *runner, const vehicle_t *vehicle, const mjModel *model,
                       mjData *data, bf_tick_fn tick, void *ctx, int timeout_ms) {
    if (runner == NULL || vehicle == NULL || model == NULL || data == NULL || tick == NULL ||
        vehicle->base_body_id < 0) {
        return BF_RUNNER_FAILED;
    }
    if (!runner->have_origin) {
        for (int i = 0; i < 3; i++) {
            runner->origin_enu[i] = data->xpos[3 * vehicle->base_body_id + i];
        }
        runner->have_origin = true;
    }
    if (!runner->awaiting) {
        ap_truth_t truth;
        bf_fdm_t fdm;
        bridge_read_truth(vehicle, model, data, runner->origin_enu, &truth);
        bf_fdm_from_truth(&truth, &runner->home, &fdm);
        if (bf_link_send_fdm(runner->link, &fdm) != BF_LINK_READY) {
            fprintf(stderr, "[betaflight] sending state failed at t=%.3f s\n", data->time);
            return BF_RUNNER_FAILED;
        }
        runner->awaiting = true;
    }
    float fresh[BF_MOTOR_COUNT];
    int wait_ms = runner->replied_once ? timeout_ms : STARTUP_WAIT_MS;
    int status = bf_link_wait_motors(runner->link, wait_ms, fresh);
    if (status == BF_LINK_ERROR) {
        fprintf(stderr, "[betaflight] receiving motors failed at t=%.3f s\n", data->time);
        return BF_RUNNER_FAILED;
    }
    if (status == BF_LINK_READY) {
        memcpy(runner->motors, fresh, sizeof(runner->motors));
        runner->replied_once = true;
        runner->waited_ms = 0;
    } else if (runner->replied_once) {
        runner->waited_ms += wait_ms;
        if (runner->waited_ms >= RESYNC_MS) {
            runner->awaiting = false;
            runner->waited_ms = 0;
        }
        return BF_RUNNER_WAITING;
    }
    runner->awaiting = false;
    actuator_command_t command;
    ap_map_apply_fractions(&runner->map, runner->motors, BF_MOTOR_COUNT, &command);
    bridge_apply_command(vehicle, model, data, &command);
    advance(model, data, tick, ctx);
    return BF_RUNNER_EXCHANGED;
}

void bf_runner_destroy(bf_runner_t *runner) {
    if (runner == NULL) {
        return;
    }
    bf_link_close(runner->link);
    free(runner);
}
