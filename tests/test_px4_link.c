#include "bridge/px4_link.h"
#include "test_support.h"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>

static const uint8_t GOLDEN_ACTUATORS_FRAME[32] = {
    0xfd, 0x14, 0x00, 0x00, 0x05, 0x01, 0x01, 0x5d, 0x00, 0x00, 0x28, 0x23, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3f,
    0x74, 0x34,
};

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

static void test_exchange(void) {
    px4_link_t *link = px4_link_open("127.0.0.1", 0);
    CHECK(link != NULL);
    if (link == NULL) {
        return;
    }
    px4_actuator_controls_t controls;
    CHECK(px4_link_accept(link, 20) == PX4_LINK_TIMEOUT);
    CHECK(px4_link_wait_controls(link, 10, &controls) == PX4_LINK_ERROR);
    CHECK(px4_link_send(link, GOLDEN_ACTUATORS_FRAME, 4) == PX4_LINK_ERROR);

    int client = connect_to(px4_link_port(link));
    CHECK(client >= 0);
    CHECK(px4_link_accept(link, 1000) == PX4_LINK_READY && px4_link_connected(link));
    CHECK(px4_link_wait_controls(link, 20, &controls) == PX4_LINK_TIMEOUT);

    uint8_t twice[2 * sizeof(GOLDEN_ACTUATORS_FRAME)];
    memcpy(twice, GOLDEN_ACTUATORS_FRAME, sizeof(GOLDEN_ACTUATORS_FRAME));
    memcpy(twice + sizeof(GOLDEN_ACTUATORS_FRAME), GOLDEN_ACTUATORS_FRAME,
           sizeof(GOLDEN_ACTUATORS_FRAME));
    size_t half = sizeof(GOLDEN_ACTUATORS_FRAME) / 2;
    CHECK(send(client, twice, half, 0) == (ssize_t)half);
    CHECK(px4_link_wait_controls(link, 30, &controls) == PX4_LINK_TIMEOUT);
    CHECK(send(client, twice + half, sizeof(twice) - half, 0) == (ssize_t)(sizeof(twice) - half));
    CHECK(px4_link_wait_controls(link, 1000, &controls) == PX4_LINK_READY);
    CHECK(controls.time_usec == 9000);
    CHECK_NEAR(controls.controls[0], 0.5, 1e-9);

    uint8_t out[8] = {1, 2, 3, 4, 5, 6, 7, 8};
    CHECK(px4_link_send(link, out, sizeof(out)) == PX4_LINK_READY);
    uint8_t back[8];
    CHECK(recv(client, back, sizeof(back), MSG_WAITALL) == (ssize_t)sizeof(back));
    CHECK(memcmp(out, back, sizeof(out)) == 0);

    (void)close(client);
    CHECK(px4_link_wait_controls(link, 1000, &controls) == PX4_LINK_ERROR);
    CHECK(!px4_link_connected(link));

    int doomed = connect_to(px4_link_port(link));
    CHECK(doomed >= 0 && px4_link_accept(link, 1000) == PX4_LINK_READY);
    (void)close(doomed);
    int failed = 0;
    for (int i = 0; i < 200 && failed == 0; i++) {
        failed = px4_link_send(link, out, sizeof(out)) == PX4_LINK_ERROR ? 1 : 0;
        (void)usleep(1000);
    }
    CHECK(failed == 1 && !px4_link_connected(link));

    int again = connect_to(px4_link_port(link));
    CHECK(again >= 0 && px4_link_accept(link, 1000) == PX4_LINK_READY);
    (void)close(again);
    px4_link_close(link);
}

static void test_bad_arguments(void) {
    CHECK(px4_link_open(NULL, 0) == NULL);
    CHECK(px4_link_open("nope", 0) == NULL);
    CHECK(px4_link_accept(NULL, 1) == PX4_LINK_ERROR);
    CHECK(px4_link_port(NULL) == 0 && !px4_link_connected(NULL));
    px4_link_close(NULL);
}

int main(void) {
    test_exchange();
    test_bad_arguments();
    if (g_failures != 0) {
        fprintf(stderr, "%d failures\n", g_failures);
        return 1;
    }
    printf("px4_link tests passed\n");
    return 0;
}
