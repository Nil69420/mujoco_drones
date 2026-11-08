#include "cli.h"
#include "test_support.h"

#include <string.h>

static int parse(int argc, char **argv, cli_args_t *out) {
    return cli_parse(argc, argv, out);
}

int main(void) {
    cli_args_t args;
    char *defaults[] = {"prog"};
    CHECK(parse(1, defaults, &args) == 0);
    CHECK(!args.headless && args.duration == 10.0 && args.cam_w == 640 && args.imu_rate == 1000.0);

    char *full[] = {"prog", "--headless", "--duration", "12.5", "--imu", "icm42688p", "--imu-rate",
                    "2000", "--gnss", "zed_f9p", "--gnss-rtk", "--cam-profile", "ar0234",
                    "--cam-width", "1280", "--cam-height", "720", "--cam-fps", "120",
                    "--scenario", "a.scn", "--weather", "storm", "--vibration", "1.5"};
    CHECK(parse((int)(sizeof(full) / sizeof(full[0])), full, &args) == 0);
    CHECK(args.headless && args.duration_set && args.duration == 12.5);
    CHECK(args.imu_name != NULL && strcmp(args.imu_name, "icm42688p") == 0 && args.imu_rate == 2000.0);
    CHECK(args.gnss_rtk && args.cam_mono && strcmp(args.cam_profile, "ar0234") == 0);
    CHECK(args.cam_w == 1280 && args.cam_h == 720 && args.cam_fps == 120.0);
    CHECK(strcmp(args.scenario_path, "a.scn") == 0 && strcmp(args.weather, "storm") == 0);
    CHECK(args.vibration == 1.5);

    char *help[] = {"prog", "--help"};
    CHECK(parse(2, help, &args) > 0);
    char *unknown[] = {"prog", "--nope"};
    CHECK(parse(2, unknown, &args) < 0);
    char *missing[] = {"prog", "--imu"};
    CHECK(parse(2, missing, &args) < 0);
    char *bad_number[] = {"prog", "--duration", "abc"};
    CHECK(parse(3, bad_number, &args) < 0);
    char *trailing[] = {"prog", "--duration", "5x"};
    CHECK(parse(3, trailing, &args) < 0);
    char *not_finite[] = {"prog", "--altitude", "nan"};
    CHECK(parse(3, not_finite, &args) < 0);
    char *fractional_int[] = {"prog", "--cam-width", "10.5"};
    CHECK(parse(3, fractional_int, &args) < 0);
    char *huge_int[] = {"prog", "--cam-width", "99999999999"};
    CHECK(parse(3, huge_int, &args) < 0);
    char *negative_duration[] = {"prog", "--duration", "-3"};
    CHECK(parse(3, negative_duration, &args) < 0);
    char *zero_width[] = {"prog", "--cam-width", "0"};
    CHECK(parse(3, zero_width, &args) < 0);
    char *too_many_rays[] = {"prog", "--lidar-rays", "361"};
    CHECK(parse(3, too_many_rays, &args) < 0);
    char *negative_rate[] = {"prog", "--gnss-rate", "-1"};
    CHECK(parse(3, negative_rate, &args) < 0);
    char *flag_with_value[] = {"prog", "--headless", "yes"};
    CHECK(parse(3, flag_with_value, &args) < 0);

    if (g_failures != 0) {
        fprintf(stderr, "%d check(s) failed\n", g_failures);
        return 1;
    }
    puts("test_cli OK");
    return 0;
}
