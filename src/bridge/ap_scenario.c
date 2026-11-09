#include "bridge/ap_scenario.h"

#include "bridge/ap_runner.h"

#include <math.h>
#include <signal.h>
#include <mujoco/mujoco.h>
#include <stdio.h>
#include <string.h>

enum { BRIDGE_IDLE_TIMEOUT_MS = 120000, MAX_UDP_PORT = 65535 };

static volatile sig_atomic_t g_stop = 0;

static void on_signal(int signo) {
    (void)signo;
    g_stop = 1;
}

static void install_handlers(void) {
    struct sigaction action;
    memset(&action, 0, sizeof(action));
    action.sa_handler = on_signal;
    (void)sigemptyset(&action.sa_mask);
    (void)sigaction(SIGTERM, &action, NULL);
    (void)sigaction(SIGINT, &action, NULL);
}

typedef struct {
    scenario_t *scenario;
    vehicle_t *vehicles;
    const mjModel *model;
    mjData *data;
} tick_ctx_t;

static void bridge_tick(void *ctx) {
    tick_ctx_t *tick = ctx;
    for (int i = 0; i < tick->scenario->vehicle_count; i++) {
        sensor_update(&tick->vehicles[i].sensors, tick->model, tick->data, &tick->vehicles[i].target);
    }
    mj_step(tick->model, tick->data);
}

int ap_scenario_run(scenario_t *scenario, vehicle_t *vehicles, const mjModel *model,
                       mjData *data, int bridged) {
    double requested = scenario->vehicles[bridged].bridge_port;
    unsigned port = AP_JSON_DEFAULT_PORT;
    if (requested != 0.0) {
        if (!(requested >= 1.0 && requested <= MAX_UDP_PORT) || requested != floor(requested)) {
            fprintf(stderr, "[scenario] ERROR: bridge_port must be an integer from 1 to 65535\n");
            return 1;
        }
        port = (unsigned)requested;
    }
    ap_map_t map = ap_map_quad_plus();
    ap_runner_t *runner = ap_runner_create(&map, (uint16_t)port);
    if (runner == NULL) {
        fprintf(stderr, "[scenario] ERROR: cannot open the ArduPilot link on port %u\n",
                port);
        return 1;
    }
    printf("[scenario] ArduPilot JSON link listening on 127.0.0.1:%u\n", (unsigned)ap_runner_port(runner));
    tick_ctx_t ctx = {scenario, vehicles, model, data};
    int rc = 0;
    int idle_ms = 0;
    g_stop = 0;
    install_handlers();
    while (data->time < scenario->duration_s && g_stop == 0) {
        int status = ap_runner_exchange(runner, &vehicles[bridged], model, data, bridge_tick, &ctx, 200);
        if (status == AP_RUNNER_FAILED) {
            rc = 1;
            break;
        }
        idle_ms = status == AP_RUNNER_EXCHANGED ? 0 : idle_ms + 200;
        if (idle_ms >= BRIDGE_IDLE_TIMEOUT_MS) {
            fprintf(stderr, "[scenario] no ArduPilot frame for %d s, stopping\n",
                    BRIDGE_IDLE_TIMEOUT_MS / 1000);
            rc = 1;
            break;
        }
    }
    ap_runner_destroy(runner);
    return rc;
}

int ap_scenario_find(const scenario_t *scenario) {
    int found = -1;
    for (int i = 0; i < scenario->vehicle_count; i++) {
        if (scenario->vehicles[i].bridge[0] != '\0') {
            if (strcmp(scenario->vehicles[i].bridge, "ardupilot") != 0 || found >= 0) {
                return -2;
            }
            found = i;
        }
    }
    return found;
}
