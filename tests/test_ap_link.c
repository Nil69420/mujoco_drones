#include "bridge/ap_link.h"
#include "test_support.h"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <poll.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>

static void put16(uint8_t *p, uint16_t v) {
    p[0] = (uint8_t)(v & 0xFFu);
    p[1] = (uint8_t)(v >> 8);
}

static int make_client(uint16_t server_port, struct sockaddr_in *to) {
    int fd = socket(AF_INET, SOCK_DGRAM, 0);
    memset(to, 0, sizeof(*to));
    to->sin_family = AF_INET;
    to->sin_port = htons(server_port);
    (void)inet_pton(AF_INET, "127.0.0.1", &to->sin_addr);
    return fd;
}

static void build_packet(uint8_t *buf, uint32_t frame) {
    memset(buf, 0, AP_JSON_PACKET_16_BYTES);
    put16(buf, AP_JSON_MAGIC_16);
    put16(buf + 2, 400);
    buf[4] = (uint8_t)(frame & 0xFFu);
    buf[5] = (uint8_t)((frame >> 8) & 0xFFu);
    put16(buf + 8, 1500);
}

static void test_exchange(void) {
    ap_link_t *link = ap_link_open("127.0.0.1", 0);
    CHECK(link != NULL);
    if (link == NULL) {
        return;
    }
    struct sockaddr_in to;
    int client = make_client(ap_link_port(link), &to);
    ap_servo_frame_t frame;
    CHECK(ap_link_wait_frame(link, 20, &frame) == AP_LINK_TIMEOUT);
    ap_state_t state;
    memset(&state, 0, sizeof(state));
    state.quaternion_wxyz[0] = 1.0;
    CHECK(ap_link_send_state(link, &state) < 0);

    uint8_t pkt[AP_JSON_PACKET_16_BYTES];
    build_packet(pkt, 7);
    CHECK(sendto(client, pkt, sizeof(pkt), 0, (struct sockaddr *)&to, sizeof(to)) ==
          (ssize_t)sizeof(pkt));
    CHECK(ap_link_wait_frame(link, 1000, &frame) == AP_LINK_GOT_FRAME);
    CHECK(frame.frame_count == 7 && frame.frame_rate == 400 && frame.pwm[0] == 1500);

    state.timestamp_s = 0.0025;
    CHECK(ap_link_send_state(link, &state) == 0);
    char reply[1024];
    struct pollfd pfd = {client, POLLIN, 0};
    CHECK(poll(&pfd, 1, 1000) == 1);
    ssize_t got = recv(client, reply, sizeof(reply) - 1, MSG_DONTWAIT);
    CHECK(got > 10);
    if (got > 0) {
        reply[got] = '\0';
        CHECK(reply[0] == '\n' && strstr(reply, "\"timestamp\":0.002500") != NULL);
    }

    uint8_t junk[16] = {1, 2, 3};
    CHECK(sendto(client, junk, sizeof(junk), 0, (struct sockaddr *)&to, sizeof(to)) > 0);
    CHECK(ap_link_wait_frame(link, 1000, &frame) == AP_LINK_TIMEOUT);

    (void)close(client);
    ap_link_close(link);
}

static void test_bad_arguments(void) {
    CHECK(ap_link_open(NULL, 0) == NULL);
    CHECK(ap_link_open("not-an-ip", 0) == NULL);
    ap_servo_frame_t frame;
    CHECK(ap_link_wait_frame(NULL, 1, &frame) == AP_LINK_ERROR);
    CHECK(ap_link_port(NULL) == 0);
    ap_link_close(NULL);
}

int main(void) {
    test_exchange();
    test_bad_arguments();
    if (g_failures != 0) {
        fprintf(stderr, "%d failures\n", g_failures);
        return 1;
    }
    printf("ap_link tests passed\n");
    return 0;
}
