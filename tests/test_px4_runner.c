#include "bridge/mav2.h"
#include "bridge/px4_msgs.h"
#include "bridge/px4_runner.h"
#include "test_support.h"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <poll.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>

static const char *MODEL_XML =
    "<mujoco><option timestep='0.001' gravity='0 0 -9.81'/><worldbody>"
    "<body name='base_link' pos='0 0 1'><freejoint/>"
    "<geom type='box' size='0.1 0.1 0.02' mass='1'/>"
    "<site name='imu_site' pos='0 0 0' size='0.01'/></body></worldbody>"
    "<actuator><general name='thrust' site='imu_site' gear='0 0 1 0 0 0' ctrlrange='0 60'/>"
    "</actuator><sensor><accelerometer name='accel' site='imu_site'/>"
    "<gyro name='gyro' site='imu_site'/></sensor></mujoco>";

typedef struct {
    const mjModel *model;
    mjData *data;
} tick_ctx_t;

static void tick(void *ctx) {
    tick_ctx_t *context = ctx;
    mj_step(context->model, context->data);
}

typedef struct {
    int sensors;
    int gps;
    uint64_t last_sensor_us;
    mav2_parser_t parser;
} seen_t;

static void drain(int fd, seen_t *seen) {
    struct pollfd pfd = {fd, POLLIN, 0};
    while (poll(&pfd, 1, 30) == 1) {
        uint8_t chunk[2048];
        ssize_t count = recv(fd, chunk, sizeof(chunk), 0);
        if (count <= 0) {
            return;
        }
        for (ssize_t i = 0; i < count; i++) {
            mav2_frame_t frame;
            if (mav2_parser_feed(&seen->parser, chunk[i], &frame) == 1) {
                if (frame.msgid == MAV2_MSG_HIL_SENSOR) {
                    seen->sensors++;
                    uint64_t usec = 0;
                    for (int b = 7; b >= 0; b--) {
                        usec = (usec << 8) | frame.payload[b];
                    }
                    seen->last_sensor_us = usec;
                } else if (frame.msgid == MAV2_MSG_HIL_GPS) {
                    seen->gps++;
                }
            }
        }
    }
}

static void send_controls(int fd, float first) {
    mav2_frame_t frame;
    memset(&frame, 0, sizeof(frame));
    frame.msgid = MAV2_MSG_HIL_ACTUATOR_CONTROLS;
    frame.len = 81;
    memcpy(frame.payload + 16, &first, sizeof(first));
    uint8_t buf[MAV2_MAX_FRAME_BYTES];
    size_t len = mav2_encode(buf, sizeof(buf), &frame);
    (void)send(fd, buf, len, 0);
}

static int connect_to(uint16_t port) {
    int fd = socket(AF_INET, SOCK_STREAM, 0);
    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);
    (void)inet_pton(AF_INET, "127.0.0.1", &addr.sin_addr);
    if (connect(fd, (struct sockaddr *)&addr, sizeof(addr)) != 0) {
        (void)close(fd);
        return -1;
    }
    return fd;
}

static ap_map_t single_motor_map(void) {
    ap_map_t map;
    memset(&map, 0, sizeof(map));
    map.motor_count = 1;
    map.channel_count = 1;
    map.motor[0].spin_channel = -1;
    map.max_thrust_n = 20.0;
    map.motor_constant = 1.0e-5;
    map.pwm_min = 1000;
    map.pwm_max = 2000;
    return map;
}

static void run_lockstep(void) {
    char error[512] = "";
    mjSpec *spec = mj_parseXMLString(MODEL_XML, NULL, error, sizeof(error));
    mjModel *model = spec != NULL ? mj_compile(spec, NULL) : NULL;
    CHECK(model != NULL);
    if (model == NULL) {
        return;
    }
    mjData *data = mj_makeData(model);
    mj_forward(model, data);
    vehicle_t vehicle;
    memset(&vehicle, 0, sizeof(vehicle));
    vehicle.base_body_id = mj_name2id(model, mjOBJ_BODY, "base_link");
    vehicle.sensors.adr_gyro = model->sensor_adr[mj_name2id(model, mjOBJ_SENSOR, "gyro")];
    vehicle.sensors.adr_accel = model->sensor_adr[mj_name2id(model, mjOBJ_SENSOR, "accel")];

    ap_map_t map = single_motor_map();
    px4_home_t home = px4_home_default();
    px4_runner_t *runner = px4_runner_create(&map, &home, 0);
    CHECK(runner != NULL);
    tick_ctx_t ctx = {model, data};
    seen_t seen;
    memset(&seen, 0, sizeof(seen));
    mav2_parser_reset(&seen.parser);

    CHECK(px4_runner_exchange(runner, &vehicle, model, data, tick, &ctx, 20) == PX4_RUNNER_WAITING);
    int fd = connect_to(px4_runner_port(runner));
    CHECK(fd >= 0);
    CHECK(px4_runner_exchange(runner, &vehicle, model, data, tick, &ctx, 1000) == PX4_RUNNER_WAITING);

    CHECK(px4_runner_exchange(runner, &vehicle, model, data, tick, &ctx, 20) == PX4_RUNNER_EXCHANGED);
    CHECK_NEAR(data->time, 0.004, 1e-9);
    drain(fd, &seen);
    CHECK(seen.sensors == 1 && seen.gps == 1 && seen.last_sensor_us == 0);

    send_controls(fd, 1.0F);
    CHECK(px4_runner_exchange(runner, &vehicle, model, data, tick, &ctx, 1000) == PX4_RUNNER_EXCHANGED);
    CHECK_NEAR(data->time, 0.008, 1e-9);
    CHECK_NEAR(data->qvel[2], 20.0 * 0.004 - 9.81 * 0.008, 2e-5);
    drain(fd, &seen);
    CHECK(seen.sensors == 2 && seen.last_sensor_us == 4000 && seen.gps == 1);

    CHECK(px4_runner_exchange(runner, &vehicle, model, data, tick, &ctx, 20) == PX4_RUNNER_WAITING);
    CHECK_NEAR(data->time, 0.008, 1e-9);
    drain(fd, &seen);
    CHECK(seen.sensors == 3 && seen.last_sensor_us == 8000);
    CHECK(px4_runner_exchange(runner, &vehicle, model, data, tick, &ctx, 20) == PX4_RUNNER_WAITING);
    drain(fd, &seen);
    CHECK(seen.sensors == 3);

    send_controls(fd, 0.0F);
    CHECK(px4_runner_exchange(runner, &vehicle, model, data, tick, &ctx, 1000) == PX4_RUNNER_EXCHANGED);
    CHECK_NEAR(data->time, 0.012, 1e-9);

    (void)close(fd);
    int status = PX4_RUNNER_EXCHANGED;
    for (int i = 0; i < 5 && status != PX4_RUNNER_FAILED; i++) {
        status = px4_runner_exchange(runner, &vehicle, model, data, tick, &ctx, 50);
    }
    CHECK(status == PX4_RUNNER_FAILED);

    px4_runner_destroy(runner);
    mj_deleteData(data);
    mj_deleteModel(model);
    mj_deleteSpec(spec);
}

static void run_bad_arguments(void) {
    ap_map_t map = single_motor_map();
    px4_home_t home = px4_home_default();
    CHECK(px4_runner_create(NULL, &home, 0) == NULL);
    CHECK(px4_runner_create(&map, NULL, 0) == NULL);
    map.pwm_max = map.pwm_min;
    CHECK(px4_runner_create(&map, &home, 0) == NULL);
    CHECK(px4_runner_port(NULL) == 0);
    CHECK(px4_runner_exchange(NULL, NULL, NULL, NULL, NULL, NULL, 1) == PX4_RUNNER_FAILED);
    px4_runner_destroy(NULL);
}

int main(void) {
    run_lockstep();
    run_bad_arguments();
    if (g_failures != 0) {
        fprintf(stderr, "%d failures\n", g_failures);
        return 1;
    }
    printf("px4_runner tests passed\n");
    return 0;
}
