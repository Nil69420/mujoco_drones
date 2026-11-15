#include "test_support.h"
#include "urdf_import_config.h"

#include <stdlib.h>
#include <string.h>
#include <unistd.h>

static int write_config(const char *text, char *path, size_t cap) {
    snprintf(path, cap, "/tmp/import_config_test_XXXXXX");
    int fdesc = mkstemp(path);
    if (fdesc < 0) {
        return -1;
    }
    ssize_t wrote = write(fdesc, text, strlen(text));
    close(fdesc);
    return wrote == (ssize_t)strlen(text) ? 0 : -1;
}

static int parse_text(const char *text, actuator_config_t *cfg, char *err, size_t err_cap) {
    char path[128];
    if (write_config(text, path, sizeof(path)) != 0) {
        return -99;
    }
    int status = config_parse(path, cfg, err, err_cap);
    unlink(path);
    return status;
}

static void expect_error(const char *text, const char *needle) {
    actuator_config_t cfg;
    char err[256] = "";
    int status = parse_text(text, &cfg, err, sizeof(err));
    if (status == 0 || strstr(err, needle) == NULL) {
        fprintf(stderr, "expected error containing '%s', got status %d '%s'\n", needle, status, err);
        g_failures++;
    }
}

static void test_valid(void) {
    const char *text =
        "# comment\n"
        "[channels]\n"
        "order = thrust_0, thrust_1 ,spin_0\n"
        "\n"
        "[joint rotor_0_joint]\n"
        "type = motor\n"
        "torque = 0.016\n"
        "ctrlrange = 0 6\n"
        "[joint left_elevon_joint]\n"
        "type = position\n"
        "kp = 12.5\n"
        "[spin rotor_0_joint]\n"
        "type = velocity\n"
        "kv = 0.0001\n"
        "ctrlrange = -838 0\n";
    actuator_config_t cfg;
    char err[256] = "";
    CHECK(parse_text(text, &cfg, err, sizeof(err)) == 0);
    CHECK(cfg.channel_order_count == 3);
    CHECK(strcmp(cfg.channel_order[0], "thrust_0") == 0 && strcmp(cfg.channel_order[2], "spin_0") == 0);
    CHECK(cfg.count == 3);
    CHECK(config_find(&cfg, "channels") == NULL);
    const config_actuator_t *rotor = config_find(&cfg, "rotor_0_joint");
    CHECK(rotor != NULL && rotor->is_motor == 1 && rotor->has_torque && rotor->torque == 0.016);
    CHECK(rotor != NULL && rotor->has_ctrlrange && rotor->ctrlrange[0] == 0.0 && rotor->ctrlrange[1] == 6.0);
    const config_actuator_t *elevon = config_find(&cfg, "left_elevon_joint");
    CHECK(elevon != NULL && elevon->is_motor == 0 && elevon->kp == 12.5);
    const config_actuator_t *spin = config_find_spin(&cfg, "rotor_0_joint");
    CHECK(spin != NULL && spin->is_spin == 1 && spin->kv == 0.0001 && spin->ctrlrange[0] == -838.0);
    CHECK(config_find(&cfg, "nothing") == NULL && config_find_spin(&cfg, "left_elevon_joint") == NULL);
}

static void test_errors(void) {
    expect_error("[joint a]\nkp = 1\n", "has no type");
    expect_error("[joint a\ntype = motor\n", "malformed section header");
    expect_error("[wing a]\ntype = motor\n", "expected '[joint <name>]'");
    expect_error("type = motor\n", "outside of a");
    expect_error("[joint a]\ntype motor\n", "expected 'key = value'");
    expect_error("[joint a]\ntype = rocket\n", "unknown type");
    expect_error("[joint a]\ntype = motor\nkp = abc\n", "invalid kp");
    expect_error("[joint a]\ntype = motor\nkv = 1x\n", "invalid kv");
    expect_error("[joint a]\ntype = motor\ntorque = x\n", "invalid torque");
    expect_error("[joint a]\ntype = motor\nctrlrange = 1\n", "invalid ctrlrange");
    expect_error("[joint a]\ntype = motor\nspeed = 1\n", "unknown key");
    expect_error("[joint a]\ntype = motor\n[joint a]\ntype = motor\n", "duplicate section");
    expect_error("[channels]\nmode = x\n", "unknown key 'mode' in [channels]");
    expect_error("[joint a]\n", "has no type");

    actuator_config_t cfg;
    char err[256] = "";
    CHECK(config_parse("/tmp/definitely_missing_import_config", &cfg, err, sizeof(err)) != 0);
    CHECK(strstr(err, "cannot open config file") != NULL);

    char many[8192] = "";
    for (int idx = 0; idx < 65; idx++) {
        char piece[64];
        snprintf(piece, sizeof(piece), "[joint j%d]\ntype = motor\n", idx);
        strncat(many, piece, sizeof(many) - strlen(many) - 1);
    }
    expect_error(many, "too many joint sections");
}

int main(void) {
    test_valid();
    test_errors();
    if (g_failures != 0) {
        fprintf(stderr, "%d check(s) failed\n", g_failures);
        return 1;
    }
    puts("test_import_config OK");
    return 0;
}
