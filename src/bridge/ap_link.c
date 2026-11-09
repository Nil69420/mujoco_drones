#include "bridge/ap_link.h"

#include <arpa/inet.h>
#include <errno.h>
#include <netinet/in.h>
#include <poll.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>

enum { STATE_BUFFER_BYTES = 2048, RECV_BUFFER_BYTES = 512 };

struct ap_link {
    int fd;
    uint16_t port;
    struct sockaddr_in peer;
    bool has_peer;
};

ap_link_t *ap_link_open(const char *bind_ip, uint16_t port) {
    if (bind_ip == NULL) {
        return NULL;
    }
    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);
    if (inet_pton(AF_INET, bind_ip, &addr.sin_addr) != 1) {
        return NULL;
    }
    ap_link_t *link = calloc(1, sizeof(*link));
    if (link == NULL) {
        return NULL;
    }
    link->fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (link->fd < 0) {
        free(link);
        return NULL;
    }
    int reuse = 1;
    (void)setsockopt(link->fd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));
    if (bind(link->fd, (const struct sockaddr *)&addr, sizeof(addr)) != 0) {
        (void)close(link->fd);
        free(link);
        return NULL;
    }
    socklen_t len = sizeof(addr);
    if (getsockname(link->fd, (struct sockaddr *)&addr, &len) != 0) {
        (void)close(link->fd);
        free(link);
        return NULL;
    }
    link->port = ntohs(addr.sin_port);
    return link;
}

uint16_t ap_link_port(const ap_link_t *link) {
    return link != NULL ? link->port : 0;
}

int ap_link_wait_frame(ap_link_t *link, int timeout_ms, ap_servo_frame_t *frame) {
    if (link == NULL || frame == NULL) {
        return AP_LINK_ERROR;
    }
    struct pollfd pfd = {link->fd, POLLIN, 0};
    int ready = poll(&pfd, 1, timeout_ms);
    if (ready < 0) {
        return errno == EINTR ? AP_LINK_TIMEOUT : AP_LINK_ERROR;
    }
    if (ready == 0) {
        return AP_LINK_TIMEOUT;
    }
    uint8_t buf[RECV_BUFFER_BYTES];
    struct sockaddr_in from;
    socklen_t from_len = sizeof(from);
    ssize_t got = recvfrom(link->fd, buf, sizeof(buf), 0, (struct sockaddr *)&from, &from_len);
    if (got < 0) {
        return errno == EINTR || errno == EAGAIN ? AP_LINK_TIMEOUT : AP_LINK_ERROR;
    }
    if (ap_servo_decode(buf, (size_t)got, frame) != 0) {
        return AP_LINK_TIMEOUT;
    }
    link->peer = from;
    link->has_peer = true;
    return AP_LINK_GOT_FRAME;
}

int ap_link_send_state(ap_link_t *link, const ap_state_t *state) {
    if (link == NULL || state == NULL || !link->has_peer) {
        return -1;
    }
    char text[STATE_BUFFER_BYTES];
    size_t len = ap_state_encode(state, text, sizeof(text));
    if (len == 0) {
        return -1;
    }
    ssize_t sent = sendto(link->fd, text, len, 0, (const struct sockaddr *)&link->peer,
                          sizeof(link->peer));
    return sent == (ssize_t)len ? 0 : -1;
}

void ap_link_close(ap_link_t *link) {
    if (link == NULL) {
        return;
    }
    (void)close(link->fd);
    free(link);
}
