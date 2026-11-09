#include "bridge/bf_packets.h"
#include "pilot/pilot_evdev.h"

#include <arpa/inet.h>
#include <errno.h>
#include <linux/input-event-codes.h>
#include <netinet/in.h>
#include <poll.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <time.h>
#include <unistd.h>

enum { MAX_DEVICES = 16, PROFILE_TEXT_BYTES = 4096, TICK_MS = 10, PRINT_EVERY = 10 };

static volatile sig_atomic_t g_stop = 0;

static void on_signal(int signo) {
    (void)signo;
    g_stop = 1;
}

static double now_seconds(void) {
    struct timespec now;
    (void)clock_gettime(CLOCK_MONOTONIC, &now);
    return (double)now.tv_sec + 1.0e-9 * (double)now.tv_nsec;
}

static void install_signals(void) {
    struct sigaction action;
    memset(&action, 0, sizeof(action));
    action.sa_handler = on_signal;
    (void)sigemptyset(&action.sa_mask);
    (void)sigaction(SIGINT, &action, NULL);
    (void)sigaction(SIGTERM, &action, NULL);
}

static int usage(void) {
    (void)fprintf(stderr,
                  "usage: pilot_tool profiles\n"
                  "       pilot_tool list\n"
                  "       pilot_tool monitor <event-device> [profile-file]\n"
                  "       pilot_tool calibrate <event-device> <output-profile>\n"
                  "       pilot_tool betaflight <event-device> [profile-file] [udp-port]\n");
    return 2;
}

static int cmd_profiles(void) {
    for (int i = 0; i < pilot_profile_count(); i++) {
        pilot_profile_t profile;
        if (pilot_profile_at(i, &profile)) {
            (void)printf("%-40s %04X:%04X %s\n", profile.name, (unsigned)profile.vendor,
                         (unsigned)profile.product, profile.verified ? "verified" : "unverified");
        }
    }
    return 0;
}

static int cmd_list(void) {
    pilot_device_info_t devices[MAX_DEVICES];
    int denied = 0;
    int count = pilot_evdev_list(devices, MAX_DEVICES, &denied);
    if (count < 0) {
        (void)fprintf(stderr, "cannot read /dev/input\n");
        return 1;
    }
    for (int i = 0; i < count; i++) {
        pilot_profile_t profile;
        bool known = pilot_profile_for_device(devices[i].vendor, devices[i].product, devices[i].name, &profile);
        (void)printf("%s  %04X:%04X  \"%s\"  %d axes %d buttons  profile: %s\n", devices[i].path,
                     (unsigned)devices[i].vendor, (unsigned)devices[i].product, devices[i].name,
                     devices[i].axis_count, devices[i].button_count,
                     known ? profile.name : "unknown (run calibrate)");
    }
    if (count == 0) {
        (void)printf("no game controllers found\n");
    }
    if (denied > 0) {
        (void)printf("%d input devices were not readable: log in at the console (the seat grants access to "
                     "controllers) or add yourself to the 'input' group\n",
                     denied);
    }
    return 0;
}

static int load_profile_file(const char *path, pilot_profile_t *profile) {
    FILE *file = fopen(path, "r");
    if (file == NULL) {
        (void)fprintf(stderr, "cannot open %s\n", path);
        return -1;
    }
    char text[PROFILE_TEXT_BYTES];
    size_t got = fread(text, 1, sizeof(text) - 1U, file);
    (void)fclose(file);
    text[got] = '\0';
    char error[96];
    if (pilot_profile_parse(text, profile, error, sizeof(error)) != 0) {
        (void)fprintf(stderr, "%s: %s\n", path, error);
        return -1;
    }
    return 0;
}

static int open_device(const char *path, const char *profile_file, pilot_t *pilot,
                       pilot_device_info_t *info_out) {
    pilot_device_info_t info;
    int fd = pilot_evdev_open(path, pilot, &info);
    if (fd < 0) {
        (void)fprintf(stderr, "cannot open %s (%s)\n", path, strerror(errno));
        return -1;
    }
    if (profile_file != NULL) {
        pilot_profile_t profile;
        if (load_profile_file(profile_file, &profile) != 0) {
            (void)close(fd);
            return -1;
        }
        pilot->profile = profile;
    }
    if (info_out != NULL) {
        *info_out = info;
    }
    (void)printf("%s \"%s\" profile: %s%s\n", info.path, info.name, pilot->profile.name,
                 pilot->profile.verified ? "" : " (unverified: check the directions with monitor)");
    return fd;
}

static void print_state(const pilot_state_t *state) {
    (void)printf("roll %+.2f pitch %+.2f yaw %+.2f throttle %.2f %s %s%s\n", state->roll, state->pitch,
                 state->yaw, state->throttle, state->arm ? "ARMED" : "disarmed",
                 state->mode ? "mode-on" : "mode-off", state->failsafe ? " FAILSAFE" : "");
    (void)fflush(stdout);
}

static int run_loop(int fd, pilot_t *pilot, int udp_fd, const struct sockaddr_in *target) {
    double last = now_seconds();
    double start = last;
    int tick = 0;
    while (g_stop == 0) {
        struct pollfd pfd = {fd, POLLIN, 0};
        (void)poll(&pfd, 1, TICK_MS);
        int events = 0;
        if (pilot->connected && pilot_evdev_drain(fd, pilot, &events) != 0) {
            (void)fprintf(stderr, "controller disconnected: failsafe\n");
        }
        double now = now_seconds();
        pilot_state_t state;
        pilot_update(pilot, now - last, &state);
        last = now;
        if (udp_fd >= 0) {
            uint16_t channels[PILOT_RC_CHANNELS];
            uint8_t packet[BF_RC_BYTES];
            pilot_to_rc(&state, channels);
            if (bf_rc_encode(now - start, channels, packet, sizeof(packet)) == BF_RC_BYTES) {
                (void)sendto(udp_fd, packet, sizeof(packet), 0, (const struct sockaddr *)target, sizeof(*target));
            }
        }
        if (++tick % (PRINT_EVERY * (udp_fd >= 0 ? 5 : 1)) == 0) {
            print_state(&state);
        }
    }
    return 0;
}

static int cmd_monitor(const char *path, const char *profile_file) {
    pilot_t pilot;
    int fd = open_device(path, profile_file, &pilot, NULL);
    if (fd < 0) {
        return 1;
    }
    install_signals();
    int rc = run_loop(fd, &pilot, -1, NULL);
    (void)close(fd);
    return rc;
}

static int cmd_betaflight(const char *path, const char *profile_file, unsigned port) {
    pilot_t pilot;
    int fd = open_device(path, profile_file, &pilot, NULL);
    if (fd < 0) {
        return 1;
    }
    int udp_fd = socket(AF_INET, SOCK_DGRAM, 0);
    struct sockaddr_in target;
    memset(&target, 0, sizeof(target));
    target.sin_family = AF_INET;
    target.sin_port = htons((uint16_t)port);
    (void)inet_pton(AF_INET, "127.0.0.1", &target.sin_addr);
    (void)printf("sending RC to 127.0.0.1:%u (hold the arm control for half a second with the throttle down)\n", port);
    install_signals();
    int rc = run_loop(fd, &pilot, udp_fd, &target);
    (void)close(udp_fd);
    (void)close(fd);
    return rc;
}

static void wait_for_enter(int fd, pilot_t *pilot) {
    for (;;) {
        struct pollfd pfds[2] = {{fd, POLLIN, 0}, {STDIN_FILENO, POLLIN, 0}};
        if (poll(pfds, 2, -1) < 0 && errno != EINTR) {
            return;
        }
        if ((pfds[0].revents & POLLIN) != 0) {
            (void)pilot_evdev_drain(fd, pilot, NULL);
        }
        if ((pfds[1].revents & POLLIN) != 0) {
            char line[64];
            if (fgets(line, sizeof(line), stdin) == NULL) {
                g_stop = 1;
            }
            return;
        }
    }
}

static int learn_axis(int fd, pilot_t *pilot, const char *prompt, uint16_t *used, int *used_count,
                      uint16_t *code, bool *invert) {
    int32_t start[PILOT_AXIS_SLOTS];
    int32_t end[PILOT_AXIS_SLOTS];
    (void)printf("%s, hold it there, then press Enter: ", prompt);
    (void)fflush(stdout);
    (void)pilot_evdev_drain(fd, pilot, NULL);
    memcpy(start, pilot->value, sizeof(start));
    wait_for_enter(fd, pilot);
    memcpy(end, pilot->value, sizeof(end));
    if (g_stop != 0 || pilot_calibrate_pick(pilot, start, end, used, *used_count, code, invert) != 0) {
        (void)printf("no stick movement detected\n");
        return -1;
    }
    used[(*used_count)++] = *code;
    (void)printf("  using axis %u%s\n", (unsigned)*code, *invert ? " (inverted)" : "");
    return 0;
}

static uint16_t learn_button(int fd, pilot_t *pilot, const char *prompt) {
    (void)printf("%s, hold it, then press Enter (just Enter to skip): ", prompt);
    (void)fflush(stdout);
    wait_for_enter(fd, pilot);
    uint16_t found = PILOT_NONE;
    for (int code = BTN_MISC; code < PILOT_BUTTON_SLOTS; code++) {
        if (pilot->button[code] != 0 && found == PILOT_NONE) {
            found = (uint16_t)code;
        }
    }
    (void)printf("  %s\n", found == PILOT_NONE ? "none" : "button recorded");
    return found;
}

typedef struct {
    const char *prompt;
    uint16_t *axis;
    bool *invert;
} axis_step_t;

static int write_profile(const pilot_profile_t *profile, const char *output) {
    char text[PROFILE_TEXT_BYTES];
    if (pilot_profile_format(profile, text, sizeof(text)) < 0) {
        (void)fprintf(stderr, "profile too large\n");
        return 1;
    }
    FILE *file = fopen(output, "w");
    if (file == NULL) {
        (void)fprintf(stderr, "cannot write %s\n", output);
        return 1;
    }
    int status = fputs(text, file) < 0 ? 1 : 0;
    (void)fclose(file);
    return status;
}

static int cmd_calibrate(const char *path, const char *output) {
    pilot_t pilot;
    pilot_device_info_t info;
    memset(&info, 0, sizeof(info));
    int fd = open_device(path, NULL, &pilot, &info);
    if (fd < 0) {
        return 1;
    }
    pilot_profile_t profile;
    pilot_profile_generic(&profile);
    (void)snprintf(profile.name, sizeof(profile.name), "%.40s (calibrated)", info.name);
    (void)snprintf(profile.name_hint, sizeof(profile.name_hint), "%.60s", info.name);
    profile.vendor = info.vendor;
    profile.product = info.product;
    const axis_step_t steps[] = {
        {"Push the ROLL stick fully to the right", &profile.axis_roll, &profile.invert_roll},
        {"Push the PITCH stick fully forward", &profile.axis_pitch, &profile.invert_pitch},
        {"Turn the YAW stick fully clockwise (right)", &profile.axis_yaw, &profile.invert_yaw},
        {"Push the THROTTLE stick fully up", &profile.axis_throttle, &profile.invert_throttle},
    };
    uint16_t used[8];
    int used_count = 0;
    (void)printf("Release every stick and switch before each step.\n");
    for (size_t i = 0; i < sizeof(steps) / sizeof(steps[0]); i++) {
        uint16_t code = 0;
        bool invert = false;
        if (learn_axis(fd, &pilot, steps[i].prompt, used, &used_count, &code, &invert) != 0) {
            (void)close(fd);
            return 1;
        }
        *steps[i].axis = code;
        *steps[i].invert = invert;
    }
    (void)printf("Does the throttle stay where you leave it (a radio)? [y/N]: ");
    (void)fflush(stdout);
    char answer[16] = "";
    if (fgets(answer, sizeof(answer), stdin) != NULL && (answer[0] == 'y' || answer[0] == 'Y')) {
        profile.throttle_mode = PILOT_THROTTLE_DIRECT;
        profile.deadzone = 0.02;
        profile.expo = 0.0;
    }
    profile.button_arm = learn_button(fd, &pilot, "Press and hold the button you want for ARM");
    profile.button_mode = learn_button(fd, &pilot, "Press and hold the button you want for the flight-mode toggle");
    (void)close(fd);
    if (write_profile(&profile, output) != 0) {
        return 1;
    }
    (void)printf("profile written to %s; use it with: pilot_tool monitor %s %s\n", output, path, output);
    return 0;
}

int main(int argc, char **argv) {
    if (argc < 2) {
        return usage();
    }
    if (strcmp(argv[1], "profiles") == 0) {
        return cmd_profiles();
    }
    if (strcmp(argv[1], "list") == 0) {
        return cmd_list();
    }
    if (strcmp(argv[1], "monitor") == 0 && argc >= 3) {
        return cmd_monitor(argv[2], argc > 3 ? argv[3] : NULL);
    }
    if (strcmp(argv[1], "calibrate") == 0 && argc == 4) {
        return cmd_calibrate(argv[2], argv[3]);
    }
    if (strcmp(argv[1], "betaflight") == 0 && argc >= 3) {
        unsigned port = argc > 4 ? (unsigned)strtoul(argv[4], NULL, 10) : BF_PORT_RC;
        return port > 0 && port < 65536 ? cmd_betaflight(argv[2], argc > 3 ? argv[3] : NULL, port) : usage();
    }
    return usage();
}
