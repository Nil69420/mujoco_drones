#define _DEFAULT_SOURCE
#include "scenario.h"
#include "test_support.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

static int load(const char *vehicle_keys, scenario_t *out) {
    char path[] = "/tmp/scenario_fx_XXXXXX";
    int fd = mkstemp(path);
    if (fd < 0) {
        return -2;
    }
    FILE *file = fdopen(fd, "w");
    if (file == NULL) {
        close(fd);
        unlink(path);
        return -2;
    }
    fprintf(file,
            "[scenario]\nenvironment = desert\nenvironment_seed = 1\nduration = 5\n"
            "[vehicle a]\nairframe = quad_x\n%s",
            vehicle_keys);
    fclose(file);
    int status = scenario_load(path, out);
    unlink(path);
    return status;
}

int main(void) {
    scenario_t scn;
    CHECK(load("", &scn) == 0 && scn.vehicles[0].effects_mask == 0);
    CHECK(load("fx_accel_deficit = 0.4\nfx_baro_lag = 0.25\nfx_cam_drop = 0.02\nfx_cam_shift = -0.012\n",
               &scn) == 0);
    CHECK(scn.vehicles[0].effects_mask == ASSEMBLY_FX_ALL);
    CHECK_NEAR(scn.vehicles[0].effects.accel_thrust_deficit_mps2, 0.4, 1e-15);
    CHECK_NEAR(scn.vehicles[0].effects.baro_lag_s, 0.25, 1e-15);
    CHECK_NEAR(scn.vehicles[0].effects.camera_drop_probability, 0.02, 1e-15);
    CHECK_NEAR(scn.vehicles[0].effects.camera_time_shift_s, -0.012, 1e-15);
    CHECK(load("fx_accel_deficit = 0.4\n", &scn) != 0);
    CHECK(load("fx_accel_deficit = 0.4\nfx_baro_lag = 0.25\nfx_cam_drop = 0.02\n", &scn) != 0);
    CHECK(load("fx_accel_deficit =\nfx_baro_lag = 0.25\nfx_cam_drop = 0.02\nfx_cam_shift = 0\n", &scn) != 0);
    CHECK(load("fx_accel_deficit = abc\nfx_baro_lag = 0.25\nfx_cam_drop = 0.02\nfx_cam_shift = 0\n", &scn) != 0);
    CHECK(load("fx_accel_deficit = nan\nfx_baro_lag = 0.25\nfx_cam_drop = 0.02\nfx_cam_shift = 0\n", &scn) != 0);
    CHECK(load("fx_accel_deficit = 0.4x\nfx_baro_lag = 0.25\nfx_cam_drop = 0.02\nfx_cam_shift = 0\n", &scn) != 0);
    if (g_failures != 0) {
        fprintf(stderr, "%d check(s) failed\n", g_failures);
        return 1;
    }
    puts("test_scenario_effects OK");
    return 0;
}
