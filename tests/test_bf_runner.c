#include "bridge/bf_packets.h"
#include "bridge/bf_runner.h"
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

static double read_double(const uint8_t *in) {
    uint64_t bits = 0;
    for (int i = 7; i >= 0; i--) {
        bits = (bits << 8) | in[i];
    }
    double value = 0.0;
    memcpy(&value, &bits, sizeof(value));
    return value;
}

static int receive_states(int fd, double *last_time) {
    int count = 0;
    struct pollfd pfd = {fd, POLLIN, 0};
    while (poll(&pfd, 1, 30) == 1) {
        uint8_t buf[512];
        ssize_t got = recv(fd, buf, sizeof(buf), 0);
        if (got == BF_FDM_BYTES) {
            *last_time = read_double(buf);
            count++;
        }
    }
    return count;
}

static void send_motors(int fd, uint16_t port, float first) {
    float motors[BF_MOTOR_COUNT] = {first, 0.0F, 0.0F, 0.0F};
    struct sockaddr_in to;
    memset(&to, 0, sizeof(to));
    to.sin_family = AF_INET;
    to.sin_port = htons(port);
    (void)inet_pton(AF_INET, "127.0.0.1", &to.sin_addr);
    (void)sendto(fd, motors, sizeof(motors), 0, (struct sockaddr *)&to, sizeof(to));
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

    int fake = socket(AF_INET, SOCK_DGRAM, 0);
    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    (void)inet_pton(AF_INET, "127.0.0.1", &addr.sin_addr);
    CHECK(bind(fake, (struct sockaddr *)&addr, sizeof(addr)) == 0);
    socklen_t len = sizeof(addr);
    CHECK(getsockname(fake, (struct sockaddr *)&addr, &len) == 0);

    ap_map_t map = single_motor_map();
    px4_home_t home = px4_home_default();
    bf_runner_t *runner = bf_runner_create(&map, &home, 0, ntohs(addr.sin_port));
    CHECK(runner != NULL);
    tick_ctx_t ctx = {model, data};
    double last_time = -1.0;

    CHECK(bf_runner_exchange(runner, &vehicle, model, data, tick, &ctx, 20) == BF_RUNNER_EXCHANGED);
    CHECK_NEAR(data->time, 0.001, 1e-9);
    CHECK(receive_states(fake, &last_time) == 1);
    CHECK_NEAR(last_time, 0.0, 1e-12);

    send_motors(fake, bf_runner_port(runner), 1.0F);
    CHECK(bf_runner_exchange(runner, &vehicle, model, data, tick, &ctx, 1000) == BF_RUNNER_EXCHANGED);
    CHECK_NEAR(data->time, 0.002, 1e-9);
    CHECK_NEAR(data->qvel[2], 20.0 * 0.001 - 9.81 * 0.002, 2e-5);
    CHECK(receive_states(fake, &last_time) == 1);
    CHECK_NEAR(last_time, 0.001, 1e-9);

    CHECK(bf_runner_exchange(runner, &vehicle, model, data, tick, &ctx, 400) == BF_RUNNER_WAITING);
    CHECK_NEAR(data->time, 0.002, 1e-9);
    CHECK(receive_states(fake, &last_time) == 1);
    CHECK_NEAR(last_time, 0.002, 1e-9);
    CHECK(bf_runner_exchange(runner, &vehicle, model, data, tick, &ctx, 400) == BF_RUNNER_WAITING);
    CHECK(receive_states(fake, &last_time) == 0);
    CHECK(bf_runner_exchange(runner, &vehicle, model, data, tick, &ctx, 20) == BF_RUNNER_WAITING);
    CHECK(receive_states(fake, &last_time) == 1);
    CHECK_NEAR(last_time, 0.002, 1e-9);

    send_motors(fake, bf_runner_port(runner), 0.0F);
    CHECK(bf_runner_exchange(runner, &vehicle, model, data, tick, &ctx, 1000) == BF_RUNNER_EXCHANGED);
    CHECK_NEAR(data->time, 0.003, 1e-9);

    (void)close(fake);
    bf_runner_destroy(runner);
    mj_deleteData(data);
    mj_deleteModel(model);
    mj_deleteSpec(spec);
}

static void run_bad_arguments(void) {
    ap_map_t map = single_motor_map();
    px4_home_t home = px4_home_default();
    CHECK(bf_runner_create(NULL, &home, 0, 1) == NULL);
    CHECK(bf_runner_create(&map, NULL, 0, 1) == NULL);
    map.pwm_max = map.pwm_min;
    CHECK(bf_runner_create(&map, &home, 0, 1) == NULL);
    CHECK(bf_runner_port(NULL) == 0);
    CHECK(bf_runner_exchange(NULL, NULL, NULL, NULL, NULL, NULL, 1) == BF_RUNNER_FAILED);
    bf_runner_destroy(NULL);
}

int main(void) {
    run_lockstep();
    run_bad_arguments();
    if (g_failures != 0) {
        fprintf(stderr, "%d failures\n", g_failures);
        return 1;
    }
    printf("bf_runner tests passed\n");
    return 0;
}
