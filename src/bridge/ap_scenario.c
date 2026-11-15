#include "bridge/ap_scenario.h"

#include "bridge/ap_runner.h"
#include "bridge/bf_runner.h"
#include "bridge/px4_runner.h"

#include <math.h>
#include <stdbool.h>
#include <mujoco/mujoco.h>
#include <signal.h>
#include <stdio.h>
#include <string.h>
#include <time.h>

enum { BRIDGE_IDLE_TIMEOUT_MS = 120000, MAX_UDP_PORT = 65535, EXCHANGE_TIMEOUT_MS = 200 };

static const double MIN_SLEEP_S = 0.0005;
static const double LOG_PERIOD_S = 0.25;
static const double MAX_SPEED = 1000.0;

typedef void (*tick_fn)(void *ctx);

typedef struct {
    const char *name;
    unsigned default_port;
    void *(*create)(uint16_t port);
    int (*exchange)(void *runner, const vehicle_t *vehicle, const mjModel *model, mjData *data,
                    tick_fn tick, void *ctx, int timeout_ms);
    uint16_t (*port)(const void *runner);
    void (*destroy)(void *runner);
    int exchanged;
    int failed;
} bridge_ops_t;

static void *ap_create(uint16_t port) {
    ap_map_t map = ap_map_quad_plus();
    return ap_runner_create(&map, port);
}

static int ap_exchange(void *runner, const vehicle_t *vehicle, const mjModel *model, mjData *data,
                       tick_fn tick, void *ctx, int timeout_ms) {
    return ap_runner_exchange(runner, vehicle, model, data, tick, ctx, timeout_ms);
}

static uint16_t ap_port(const void *runner) {
    return ap_runner_port(runner);
}

static void ap_destroy(void *runner) {
    ap_runner_destroy(runner);
}

static void *px4_create(uint16_t port) {
    ap_map_t map = ap_map_quad_direct();
    px4_home_t home = px4_home_default();
    return px4_runner_create(&map, &home, port);
}

static int px4_exchange(void *runner, const vehicle_t *vehicle, const mjModel *model, mjData *data,
                        tick_fn tick, void *ctx, int timeout_ms) {
    return px4_runner_exchange(runner, vehicle, model, data, tick, ctx, timeout_ms);
}

static uint16_t px4_port(const void *runner) {
    return px4_runner_port(runner);
}

static void px4_destroy(void *runner) {
    px4_runner_destroy(runner);
}

static void *bf_create(uint16_t port) {
    ap_map_t map = ap_map_quad_direct();
    px4_home_t home = px4_home_default();
    return bf_runner_create(&map, &home, port, (uint16_t)(port + 1U));
}

static int bf_exchange(void *runner, const vehicle_t *vehicle, const mjModel *model, mjData *data,
                       tick_fn tick, void *ctx, int timeout_ms) {
    return bf_runner_exchange(runner, vehicle, model, data, tick, ctx, timeout_ms);
}

static uint16_t bf_port(const void *runner) {
    return bf_runner_port(runner);
}

static void bf_destroy(void *runner) {
    bf_runner_destroy(runner);
}

static const bridge_ops_t BRIDGES[] = {
    {"ardupilot", AP_JSON_DEFAULT_PORT, ap_create, ap_exchange, ap_port, ap_destroy,
     AP_RUNNER_EXCHANGED, AP_RUNNER_FAILED},
    {"px4", PX4_BRIDGE_DEFAULT_PORT, px4_create, px4_exchange, px4_port, px4_destroy,
     PX4_RUNNER_EXCHANGED, PX4_RUNNER_FAILED},
    {"betaflight", BF_BRIDGE_DEFAULT_PORT, bf_create, bf_exchange, bf_port, bf_destroy,
     BF_RUNNER_EXCHANGED, BF_RUNNER_FAILED},
};

static volatile sig_atomic_t *stop_flag(void) {
    static volatile sig_atomic_t flag = 0;
    return &flag;
}

static void on_signal(int signo) {
    (void)signo;
    *stop_flag() = 1;
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

static const bridge_ops_t *find_ops(const char *name) {
    for (size_t i = 0; i < sizeof(BRIDGES) / sizeof(BRIDGES[0]); i++) {
        if (strcmp(BRIDGES[i].name, name) == 0) {
            return &BRIDGES[i];
        }
    }
    return NULL;
}

static int pick_port(const vehicle_assembly_spec_t *spec, const bridge_ops_t *ops, unsigned *port) {
    double requested = spec->bridge_port;
    *port = ops->default_port;
    if (requested == 0.0) {
        return 0;
    }
    if (!(requested >= 1.0 && requested <= MAX_UDP_PORT) || requested != floor(requested)) {
        fprintf(stderr, "[scenario] ERROR: bridge_port must be an integer from 1 to 65535\n");
        return -1;
    }
    *port = (unsigned)requested;
    return 0;
}

static int check_speed(const vehicle_assembly_spec_t *spec) {
    if (spec->bridge_speed < 0.0 || spec->bridge_speed > MAX_SPEED || spec->bridge_speed != spec->bridge_speed) {
        fprintf(stderr, "[scenario] ERROR: bridge_speed must be between 0 and %.0f\n", MAX_SPEED);
        return -1;
    }
    return 0;
}

static double monotonic_s(void) {
    struct timespec now;
    (void)clock_gettime(CLOCK_MONOTONIC, &now);
    return (double)now.tv_sec + 1.0e-9 * (double)now.tv_nsec;
}

static void pace(double sim_time_s, double start_s, double speed) {
    double ahead = sim_time_s / speed - (monotonic_s() - start_s);
    if (ahead > MIN_SLEEP_S) {
        struct timespec delay;
        delay.tv_sec = (time_t)ahead;
        delay.tv_nsec = (long)((ahead - (double)delay.tv_sec) * 1.0e9);
        (void)nanosleep(&delay, NULL);
    }
}

static void log_state(const vehicle_t *vehicle, const mjModel *model, const mjData *data,
                      const double origin[3]) {
    (void)model;
    size_t body = (size_t)vehicle->base_body_id;
    double yaw_deg = atan2(data->xmat[9 * body + 3], data->xmat[9 * body]) * 57.29577951308232;
    (void)fprintf(stderr, "[bridge] t=%.3f pos=(%.3f %.3f %.3f) yaw=%.1f\n", data->time,
                  data->xpos[3 * body] - origin[0], data->xpos[3 * body + 1] - origin[1],
                  data->xpos[3 * body + 2] - origin[2], yaw_deg);
    (void)fflush(stderr);
}

static int bridge_loop(const bridge_ops_t *ops, void *runner, scenario_t *scenario,
                       vehicle_t *vehicles, const mjModel *model, mjData *data, int bridged) {
    tick_ctx_t ctx = {scenario, vehicles, model, data};
    double speed = scenario->vehicles[bridged].bridge_speed;
    speed = speed > 0.0 ? speed : 1.0;
    int idle_ms = 0;
    bool started = false;
    double next_log_s = 0.0;
    double origin[3] = {0.0, 0.0, 0.0};
    bool have_origin = false;
    double start_wall_s = 0.0;
    double start_sim_s = 0.0;
    *stop_flag() = 0;
    install_handlers();
    while (data->time < scenario->duration_s && *stop_flag() == 0) {
        int status = ops->exchange(runner, &vehicles[bridged], model, data, bridge_tick, &ctx,
                                   EXCHANGE_TIMEOUT_MS);
        if (status == ops->failed) {
            fprintf(stderr, "[scenario] %s bridge failed at t=%.3f s\n", ops->name, data->time);
            return 1;
        }
        if (status == ops->exchanged) {
            if (!started) {
                started = true;
                start_wall_s = monotonic_s();
                start_sim_s = data->time;
            }
            pace(data->time - start_sim_s, start_wall_s, speed);
            if (!have_origin && vehicles[bridged].base_body_id >= 0) {
                for (int i = 0; i < 3; i++) {
                    origin[i] = data->xpos[3 * vehicles[bridged].base_body_id + i];
                }
                have_origin = true;
            }
            if (have_origin && data->time >= next_log_s) {
                log_state(&vehicles[bridged], model, data, origin);
                next_log_s = data->time + LOG_PERIOD_S;
            }
        }
        idle_ms = status == ops->exchanged ? 0 : idle_ms + EXCHANGE_TIMEOUT_MS;
        if (idle_ms >= BRIDGE_IDLE_TIMEOUT_MS) {
            fprintf(stderr, "[scenario] no %s traffic for %d s, stopping\n", ops->name,
                    BRIDGE_IDLE_TIMEOUT_MS / 1000);
            return 1;
        }
    }
    return 0;
}

int ap_scenario_run(scenario_t *scenario, vehicle_t *vehicles, const mjModel *model, mjData *data,
                    int bridged) {
    const bridge_ops_t *ops = find_ops(scenario->vehicles[bridged].bridge);
    unsigned port = 0;
    if (ops == NULL || pick_port(&scenario->vehicles[bridged], ops, &port) != 0 ||
        check_speed(&scenario->vehicles[bridged]) != 0) {
        return 1;
    }
    void *runner = ops->create((uint16_t)port);
    if (runner == NULL) {
        fprintf(stderr, "[scenario] ERROR: cannot open the %s link on port %u\n", ops->name, port);
        return 1;
    }
    printf("[scenario] %s link listening on 127.0.0.1:%u\n", ops->name, (unsigned)ops->port(runner));
    int rc = bridge_loop(ops, runner, scenario, vehicles, model, data, bridged);
    ops->destroy(runner);
    return rc;
}

int ap_scenario_find(const scenario_t *scenario) {
    int found = -1;
    for (int i = 0; i < scenario->vehicle_count; i++) {
        if (scenario->vehicles[i].bridge[0] != '\0') {
            if (find_ops(scenario->vehicles[i].bridge) == NULL || found >= 0) {
                return -2;
            }
            found = i;
        }
    }
    return found;
}
