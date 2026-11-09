#include "bridge/ap_runner.h"

#include "bridge/ap_frames.h"
#include "bridge/ap_link.h"
#include "bridge/truth.h"

#include <math.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

enum { DEFAULT_RATE_HZ = 400 };

struct ap_runner {
    ap_link_t *link;
    ap_map_t map;
    bool have_last;
    uint32_t last_frame;
    ap_state_t last_state;
    double origin_enu[3];
    bool have_origin;
};

ap_runner_t *ap_runner_create(const ap_map_t *map, uint16_t port) {
    if (map == NULL || ap_map_validate(map) != 0) {
        return NULL;
    }
    ap_runner_t *runner = calloc(1, sizeof(*runner));
    if (runner == NULL) {
        return NULL;
    }
    runner->link = ap_link_open("127.0.0.1", port);
    if (runner->link == NULL) {
        free(runner);
        return NULL;
    }
    runner->map = *map;
    return runner;
}

uint16_t ap_runner_port(const ap_runner_t *runner) {
    return runner != NULL ? ap_link_port(runner->link) : 0;
}

static void advance(const mjModel *model, mjData *data, ap_tick_fn tick, void *ctx,
                    unsigned rate_hz) {
    double period = 1.0 / (double)(rate_hz > 0 ? rate_hz : DEFAULT_RATE_HZ);
    double target = data->time + period;
    double half_step = 0.5 * model->opt.timestep;
    while (data->time < target - half_step) {
        tick(ctx);
    }
}

int ap_runner_exchange(ap_runner_t *runner, const vehicle_t *vehicle, const mjModel *model,
                       mjData *data, ap_tick_fn tick, void *ctx, int timeout_ms) {
    if (runner == NULL || vehicle == NULL || model == NULL || data == NULL || tick == NULL ||
        vehicle->base_body_id < 0) {
        return AP_RUNNER_FAILED;
    }
    ap_servo_frame_t frame;
    int status = ap_link_wait_frame(runner->link, timeout_ms, &frame);
    if (status != AP_LINK_GOT_FRAME) {
        return status == AP_LINK_ERROR ? AP_RUNNER_FAILED : AP_RUNNER_WAITING;
    }
    if (!runner->have_origin) {
        for (int i = 0; i < 3; i++) {
            runner->origin_enu[i] = data->xpos[3 * vehicle->base_body_id + i];
        }
        runner->have_origin = true;
    }
    if (runner->have_last && frame.frame_count == runner->last_frame) {
        return ap_link_send_state(runner->link, &runner->last_state) == 0 ? AP_RUNNER_EXCHANGED
                                                                           : AP_RUNNER_FAILED;
    }
    if (runner->have_last && frame.frame_count < runner->last_frame) {
        mj_resetData(model, data);
        mj_forward(model, data);
    }
    actuator_command_t command;
    ap_map_apply(&runner->map, &frame, &command);
    bridge_apply_command(vehicle, model, data, &command);
    advance(model, data, tick, ctx, frame.frame_rate);

    ap_truth_t truth;
    bridge_read_truth(vehicle, model, data, runner->origin_enu, &truth);
    ap_truth_to_state(&truth, &runner->last_state);
    runner->last_frame = frame.frame_count;
    runner->have_last = true;
    return ap_link_send_state(runner->link, &runner->last_state) == 0 ? AP_RUNNER_EXCHANGED
                                                                       : AP_RUNNER_FAILED;
}

void ap_runner_destroy(ap_runner_t *runner) {
    if (runner == NULL) {
        return;
    }
    ap_link_close(runner->link);
    free(runner);
}
