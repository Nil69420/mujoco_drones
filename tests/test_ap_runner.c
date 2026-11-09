#include "bridge/ap_runner.h"
#include "test_support.h"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <poll.h>
#include <stdlib.h>
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
    int ticks;
} tick_ctx_t;

static void tick(void *ctx) {
    tick_ctx_t *context = ctx;
    mj_step(context->model, context->data);
    context->ticks++;
}

static void put16(uint8_t *out, uint16_t value) {
    out[0] = (uint8_t)(value & 0xFFU);
    out[1] = (uint8_t)(value >> 8);
}

static void send_frame(int fd, const struct sockaddr_in *to, uint32_t count, uint16_t pwm) {
    uint8_t packet[AP_JSON_PACKET_16_BYTES];
    memset(packet, 0, sizeof(packet));
    put16(packet, AP_JSON_MAGIC_16);
    put16(packet + 2, 400);
    for (int i = 0; i < 4; i++) {
        packet[4 + i] = (uint8_t)((count >> (8 * i)) & 0xFFU);
    }
    put16(packet + 8, pwm);
    (void)sendto(fd, packet, sizeof(packet), 0, (const struct sockaddr *)to, sizeof(*to));
}

static int read_reply(int fd, char *out, size_t cap) {
    struct pollfd pfd = {fd, POLLIN, 0};
    if (poll(&pfd, 1, 1000) != 1) {
        return -1;
    }
    ssize_t got = recv(fd, out, cap - 1, MSG_DONTWAIT);
    if (got <= 0) {
        return -1;
    }
    out[got] = '\0';
    return (int)got;
}

static double json_number(const char *text, const char *key) {
    const char *at = strstr(text, key);
    return at == NULL ? -1.0e9 : strtod(at + strlen(key), NULL);
}

static int make_socket(uint16_t port, struct sockaddr_in *to) {
    int fd = socket(AF_INET, SOCK_DGRAM, 0);
    memset(to, 0, sizeof(*to));
    to->sin_family = AF_INET;
    to->sin_port = htons(port);
    (void)inet_pton(AF_INET, "127.0.0.1", &to->sin_addr);
    return fd;
}

static ap_map_t single_motor_map(void) {
    ap_map_t map;
    memset(&map, 0, sizeof(map));
    map.motor_count = 1;
    map.channel_count = 1;
    map.motor[0].servo_channel = 0;
    map.motor[0].thrust_channel = 0;
    map.motor[0].spin_channel = -1;
    map.max_thrust_n = 20.0;
    map.motor_constant = 1.0e-5;
    map.thrust_expo = 0.0;
    map.pwm_min = 1000;
    map.pwm_max = 2000;
    return map;
}

static double json_element(const char *text, const char *key, int index) {
    const char *at = strstr(text, key);
    if (at == NULL) {
        return -1.0e9;
    }
    at += strlen(key);
    for (int i = 0; i < index; i++) {
        at = strchr(at, ',');
        if (at == NULL) {
            return -1.0e9;
        }
        at++;
    }
    return strtod(at, NULL);
}

static void run_exchange(void) {
    char error[512] = "";
    mjSpec *spec = mj_parseXMLString(MODEL_XML, NULL, error, sizeof(error));
    CHECK(spec != NULL);
    if (spec == NULL) {
        return;
    }
    mjModel *model = mj_compile(spec, NULL);
    CHECK(model != NULL);
    if (model == NULL) {
        mj_deleteSpec(spec);
        return;
    }
    mjData *data = mj_makeData(model);
    mj_forward(model, data);

    vehicle_t vehicle;
    memset(&vehicle, 0, sizeof(vehicle));
    vehicle.base_body_id = mj_name2id(model, mjOBJ_BODY, "base_link");
    vehicle.actuator_index[0] = 0;
    vehicle.sensors.adr_gyro = model->sensor_adr[mj_name2id(model, mjOBJ_SENSOR, "gyro")];
    vehicle.sensors.adr_accel = model->sensor_adr[mj_name2id(model, mjOBJ_SENSOR, "accel")];

    ap_map_t map = single_motor_map();
    ap_runner_t *runner = ap_runner_create(&map, 0);
    CHECK(runner != NULL);
    if (runner == NULL) {
        mj_deleteData(data);
        mj_deleteModel(model);
        mj_deleteSpec(spec);
        return;
    }
    struct sockaddr_in to;
    int fd = make_socket(ap_runner_port(runner), &to);
    tick_ctx_t ctx = {model, data, 0};
    char reply[1024];
    char again[1024];

    CHECK(ap_runner_exchange(runner, &vehicle, model, data, tick, &ctx, 20) == AP_RUNNER_WAITING);

    send_frame(fd, &to, 5, 2000);
    CHECK(ap_runner_exchange(runner, &vehicle, model, data, tick, &ctx, 1000) == AP_RUNNER_EXCHANGED);
    CHECK(read_reply(fd, reply, sizeof(reply)) > 0);
    CHECK(ctx.ticks == 2 || ctx.ticks == 3);
    CHECK_NEAR(json_number(reply, "\"timestamp\":"), data->time, 1e-6);
    double vz_ned = json_element(reply, "\"velocity\":[", 2);
    CHECK_NEAR(vz_ned, -10.19 * data->time, 1e-4);

    int ticks_before = ctx.ticks;
    send_frame(fd, &to, 5, 2000);
    CHECK(ap_runner_exchange(runner, &vehicle, model, data, tick, &ctx, 1000) == AP_RUNNER_EXCHANGED);
    CHECK(read_reply(fd, again, sizeof(again)) > 0);
    CHECK(strcmp(reply, again) == 0);
    CHECK(ctx.ticks == ticks_before);

    send_frame(fd, &to, 6, 2000);
    CHECK(ap_runner_exchange(runner, &vehicle, model, data, tick, &ctx, 1000) == AP_RUNNER_EXCHANGED);
    CHECK(read_reply(fd, again, sizeof(again)) > 0);
    CHECK(json_number(again, "\"timestamp\":") > json_number(reply, "\"timestamp\":"));

    send_frame(fd, &to, 0, 1000);
    CHECK(ap_runner_exchange(runner, &vehicle, model, data, tick, &ctx, 1000) == AP_RUNNER_EXCHANGED);
    CHECK(read_reply(fd, again, sizeof(again)) > 0);
    CHECK(json_number(again, "\"timestamp\":") < json_number(reply, "\"timestamp\":") + 0.0026);
    CHECK(data->time < 0.01);

    (void)close(fd);
    ap_runner_destroy(runner);
    mj_deleteData(data);
    mj_deleteModel(model);
    mj_deleteSpec(spec);
}

static void run_bad_arguments(void) {
    ap_map_t map = single_motor_map();
    CHECK(ap_runner_create(NULL, 0) == NULL);
    map.pwm_max = map.pwm_min;
    CHECK(ap_runner_create(&map, 0) == NULL);
    CHECK(ap_runner_port(NULL) == 0);
    CHECK(ap_runner_exchange(NULL, NULL, NULL, NULL, NULL, NULL, 1) == AP_RUNNER_FAILED);
    ap_runner_destroy(NULL);
}

int main(void) {
    run_exchange();
    run_bad_arguments();
    if (g_failures != 0) {
        fprintf(stderr, "%d failures\n", g_failures);
        return 1;
    }
    printf("ap_runner tests passed\n");
    return 0;
}
