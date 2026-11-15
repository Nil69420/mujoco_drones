#include "bridge/bf_link.h"

#include <arpa/inet.h>
#include <errno.h>
#include <netinet/in.h>
#include <poll.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>

enum { RECV_BYTES = 256 };

struct bf_link {
    int fd;
    uint16_t port;
    struct sockaddr_in dest;
};

bf_link_t *bf_link_open(const char *ip, uint16_t motors_port, uint16_t fdm_port) {
    if (ip == NULL) {
        return NULL;
    }
    struct sockaddr_in local;
    memset(&local, 0, sizeof(local));
    local.sin_family = AF_INET;
    local.sin_port = htons(motors_port);
    if (inet_pton(AF_INET, ip, &local.sin_addr) != 1) {
        return NULL;
    }
    bf_link_t *link = calloc(1, sizeof(*link));
    if (link == NULL) {
        return NULL;
    }
    link->dest = local;
    link->dest.sin_port = htons(fdm_port);
    link->fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (link->fd < 0) {
        free(link);
        return NULL;
    }
    int reuse = 1;
    (void)setsockopt(link->fd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));
    socklen_t len = sizeof(local);
    if (bind(link->fd, (const struct sockaddr *)&local, sizeof(local)) != 0 ||
        getsockname(link->fd, (struct sockaddr *)&local, &len) != 0) {
        (void)close(link->fd);
        free(link);
        return NULL;
    }
    link->port = ntohs(local.sin_port);
    return link;
}

uint16_t bf_link_port(const bf_link_t *link) {
    return link != NULL ? link->port : 0;
}

int bf_link_send_fdm(bf_link_t *link, const bf_fdm_t *fdm) {
    uint8_t buf[BF_FDM_BYTES];
    if (link == NULL || bf_fdm_encode(fdm, buf, sizeof(buf)) != BF_FDM_BYTES) {
        return BF_LINK_ERROR;
    }
    ssize_t sent = sendto(link->fd, buf, sizeof(buf), 0, (const struct sockaddr *)&link->dest,
                          sizeof(link->dest));
    return sent == (ssize_t)sizeof(buf) ? BF_LINK_READY : BF_LINK_ERROR;
}

int bf_link_wait_motors(bf_link_t *link, int timeout_ms, float motors[BF_MOTOR_COUNT]) {
    if (link == NULL || motors == NULL) {
        return BF_LINK_ERROR;
    }
    bool got = false;
    int wait_ms = timeout_ms;
    for (;;) {
        struct pollfd pfd = {link->fd, POLLIN, 0};
        int ready = poll(&pfd, 1, wait_ms);
        if (ready < 0 && errno != EINTR) {
            return BF_LINK_ERROR;
        }
        if (ready <= 0) {
            return got ? BF_LINK_READY : BF_LINK_TIMEOUT;
        }
        uint8_t buf[RECV_BYTES];
        ssize_t count = recv(link->fd, buf, sizeof(buf), 0);
        if (count < 0) {
            if (errno == EINTR || errno == EAGAIN) {
                continue;
            }
            return BF_LINK_ERROR;
        }
        if (bf_motors_decode(buf, (size_t)count, motors) == 0) {
            got = true;
        }
        wait_ms = 0;
    }
}

void bf_link_close(bf_link_t *link) {
    if (link == NULL) {
        return;
    }
    (void)close(link->fd);
    free(link);
}
