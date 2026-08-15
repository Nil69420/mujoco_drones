#include "sensors/sensor_types.h"
#include "transport/transport_renoir.h"

#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define TOPIC_COMMAND "/drone/command"

static void print_usage(const char *prog) {
    printf("Usage: %s [OPTIONS]\n\n", prog);
    printf("Publish a single command setpoint to %s via Renoir IPC.\n\n",
           TOPIC_COMMAND);
    printf("Options:\n");
    printf("  --x M       Target X position (default 0.0)\n");
    printf("  --y M       Target Y position (default 0.0)\n");
    printf("  --z M       Target altitude (default 1.0)\n");
    printf("  --yaw R     Target yaw in radians (default 0.0)\n");
    printf("  --mode N    Flight mode (default 0)\n");
    printf("  -h, --help  Show this help\n");
}

typedef struct {
    double  x;
    double  y;
    double  z;
    double  yaw;
    uint8_t mode;
} cmd_args_t;

static int parse_args(int argc, char **argv, cmd_args_t *args) {
    args->x    = 0.0;
    args->y    = 0.0;
    args->z    = 1.0;
    args->yaw  = 0.0;
    args->mode = 0;

    for (int i = 1; i < argc; i++) {
        if      (!strcmp(argv[i], "--x")    && i + 1 < argc) { args->x    = strtod(argv[++i], NULL); }
        else if (!strcmp(argv[i], "--y")    && i + 1 < argc) { args->y    = strtod(argv[++i], NULL); }
        else if (!strcmp(argv[i], "--z")    && i + 1 < argc) { args->z    = strtod(argv[++i], NULL); }
        else if (!strcmp(argv[i], "--yaw")  && i + 1 < argc) { args->yaw  = strtod(argv[++i], NULL); }
        else if (!strcmp(argv[i], "--mode") && i + 1 < argc) {
            char *endp = NULL;
            errno = 0;
            long mode_val = strtol(argv[++i], &endp, 10);
            if (errno != 0 || endp == argv[i] || *endp != '\0' ||
                mode_val < 0 || mode_val > 255) {
                fprintf(stderr, "ERROR: invalid --mode value: %s "
                                "(expected 0-255)\n", argv[i]);
                return -1;
            }
            args->mode = (uint8_t)mode_val;
        }
        else if (!strcmp(argv[i], "-h") || !strcmp(argv[i], "--help")) {
            print_usage(argv[0]);
            return 1;
        } else {
            fprintf(stderr, "Unknown option: %s\n", argv[i]);
            print_usage(argv[0]);
            return -1;
        }
    }
    return 0;
}

int main(int argc, char **argv) {
    cmd_args_t args = {0};
    int parse_rc = parse_args(argc, argv, &args);
    if (parse_rc > 0) return 0;
    if (parse_rc < 0) return 1;

    transport_t tp;
    if (transport_renoir_create(&tp) != 0 || transport_init(&tp) != 0) {
        fprintf(stderr, "ERROR: failed to initialize IPC transport\n");
        return 1;
    }

    transport_pub_t pub;
    if (transport_advertise(&tp, TOPIC_COMMAND,
                            sizeof(command_setpoint_t), &pub) != 0) {
        fprintf(stderr, "ERROR: failed to advertise %s\n", TOPIC_COMMAND);
        transport_shutdown(&tp);
        return 1;
    }

    command_setpoint_t cmd = {0};
    cmd.x    = args.x;
    cmd.y    = args.y;
    cmd.z    = args.z;
    cmd.yaw  = args.yaw;
    cmd.mode = args.mode;

    int rc = transport_publish(&pub, &cmd, sizeof(cmd));
    if (rc != 0) {
        fprintf(stderr, "ERROR: failed to publish %s\n", TOPIC_COMMAND);
    } else {
        printf("Published %s: x=%.2f y=%.2f z=%.2f yaw=%.2f mode=%u\n",
               TOPIC_COMMAND, cmd.x, cmd.y, cmd.z, cmd.yaw,
               (unsigned)cmd.mode);
    }

    transport_close_pub(&pub);
    transport_shutdown(&tp);
    return rc == 0 ? 0 : 1;
}
