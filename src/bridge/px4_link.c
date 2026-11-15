#include "bridge/px4_link.h"

#include <arpa/inet.h>
#include <errno.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <poll.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>

enum { LISTEN_BACKLOG = 1, READ_CHUNK = 4096 };

struct px4_link {
    int listen_fd;
    int client_fd;
    uint16_t port;
    mav2_parser_t parser;
};

px4_link_t *px4_link_open(const char *bind_ip, uint16_t port) {
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
    px4_link_t *link = calloc(1, sizeof(*link));
    if (link == NULL) {
        return NULL;
    }
    link->client_fd = -1;
    link->listen_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (link->listen_fd < 0) {
        free(link);
        return NULL;
    }
    int reuse = 1;
    (void)setsockopt(link->listen_fd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));
    socklen_t len = sizeof(addr);
    if (bind(link->listen_fd, (const struct sockaddr *)&addr, sizeof(addr)) != 0 ||
        listen(link->listen_fd, LISTEN_BACKLOG) != 0 ||
        getsockname(link->listen_fd, (struct sockaddr *)&addr, &len) != 0) {
        (void)close(link->listen_fd);
        free(link);
        return NULL;
    }
    link->port = ntohs(addr.sin_port);
    mav2_parser_reset(&link->parser);
    return link;
}

uint16_t px4_link_port(const px4_link_t *link) {
    return link != NULL ? link->port : 0;
}

bool px4_link_connected(const px4_link_t *link) {
    return link != NULL && link->client_fd >= 0;
}

static void drop_client(px4_link_t *link) {
    if (link->client_fd >= 0) {
        (void)close(link->client_fd);
        link->client_fd = -1;
    }
    mav2_parser_reset(&link->parser);
}

int px4_link_accept(px4_link_t *link, int timeout_ms) {
    if (link == NULL) {
        return PX4_LINK_ERROR;
    }
    if (link->client_fd >= 0) {
        return PX4_LINK_READY;
    }
    struct pollfd pfd = {link->listen_fd, POLLIN, 0};
    int ready = poll(&pfd, 1, timeout_ms);
    if (ready < 0) {
        return errno == EINTR ? PX4_LINK_TIMEOUT : PX4_LINK_ERROR;
    }
    if (ready == 0) {
        return PX4_LINK_TIMEOUT;
    }
    int fd = accept(link->listen_fd, NULL, NULL);
    if (fd < 0) {
        return errno == EINTR || errno == EAGAIN ? PX4_LINK_TIMEOUT : PX4_LINK_ERROR;
    }
    int yes = 1;
    (void)setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &yes, sizeof(yes));
    link->client_fd = fd;
    mav2_parser_reset(&link->parser);
    return PX4_LINK_READY;
}

int px4_link_send(px4_link_t *link, const uint8_t *data, size_t len) {
    if (link == NULL || data == NULL || link->client_fd < 0) {
        return PX4_LINK_ERROR;
    }
    size_t sent = 0;
    while (sent < len) {
        ssize_t wrote = send(link->client_fd, data + sent, len - sent, MSG_NOSIGNAL);
        if (wrote < 0) {
            if (errno == EINTR) {
                continue;
            }
            drop_client(link);
            return PX4_LINK_ERROR;
        }
        sent += (size_t)wrote;
    }
    return PX4_LINK_READY;
}

int px4_link_wait_controls(px4_link_t *link, int timeout_ms, px4_actuator_controls_t *out) {
    if (link == NULL || out == NULL || link->client_fd < 0) {
        return PX4_LINK_ERROR;
    }
    bool got = false;
    int wait_ms = timeout_ms;
    for (;;) {
        struct pollfd pfd = {link->client_fd, POLLIN, 0};
        int ready = poll(&pfd, 1, wait_ms);
        if (ready < 0 && errno != EINTR) {
            return PX4_LINK_ERROR;
        }
        if (ready <= 0) {
            return got ? PX4_LINK_READY : PX4_LINK_TIMEOUT;
        }
        uint8_t chunk[READ_CHUNK];
        ssize_t count = recv(link->client_fd, chunk, sizeof(chunk), 0);
        if (count == 0 || (count < 0 && errno != EINTR && errno != EAGAIN)) {
            drop_client(link);
            return PX4_LINK_ERROR;
        }
        for (ssize_t i = 0; i < count; i++) {
            mav2_frame_t frame;
            if (mav2_parser_feed(&link->parser, chunk[i], &frame) == 1 &&
                frame.msgid == MAV2_MSG_HIL_ACTUATOR_CONTROLS &&
                px4_decode_actuator_controls(&frame, out) == 0) {
                got = true;
            }
        }
        wait_ms = 0;
    }
}

void px4_link_close(px4_link_t *link) {
    if (link == NULL) {
        return;
    }
    drop_client(link);
    (void)close(link->listen_fd);
    free(link);
}
