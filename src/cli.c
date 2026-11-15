#include "cli.h"

#include "conditions/conditions.h"
#include "sensors/models/baro_model.h"
#include "sensors/models/camera_model.h"
#include "sensors/models/gnss_model.h"
#include "sensors/models/imu_model.h"
#include "sensors/models/mag_model.h"
#include "sensors/models/range_model.h"

#include <errno.h>
#include <math.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

typedef enum { OPT_FLAG, OPT_INT, OPT_DOUBLE, OPT_STRING } opt_kind_t;

typedef struct {
    const char *name;
    opt_kind_t kind;
    size_t offset;
    size_t implies_offset;
    int has_implies;
} opt_t;

#define OPT(opt_name, opt_kind, field) {opt_name, opt_kind, offsetof(cli_args_t, field), 0U, 0}
#define OPT_IMPLIES(opt_name, opt_kind, field, other) \
    {opt_name, opt_kind, offsetof(cli_args_t, field), offsetof(cli_args_t, other), 1}

static const opt_t OPTIONS[] = {
    OPT("--headless", OPT_FLAG, headless),
    OPT_IMPLIES("--duration", OPT_DOUBLE, duration, duration_set),
    OPT("--altitude", OPT_DOUBLE, altitude),
    OPT("--model", OPT_STRING, model_path),
    OPT("--no-ipc", OPT_FLAG, no_ipc),
    OPT("--lidar-rays", OPT_INT, lidar_rays),
    OPT("--cam-width", OPT_INT, cam_w),
    OPT("--cam-height", OPT_INT, cam_h),
    OPT("--cam-fps", OPT_DOUBLE, cam_fps),
    OPT("--cam-mono", OPT_FLAG, cam_mono),
    OPT_IMPLIES("--cam-profile", OPT_STRING, cam_profile, cam_mono),
    OPT("--imu", OPT_STRING, imu_name),
    OPT("--imu-rate", OPT_DOUBLE, imu_rate),
    OPT("--gnss", OPT_STRING, gnss_name),
    OPT("--gnss-rate", OPT_DOUBLE, gnss_rate),
    OPT("--gnss-cold", OPT_FLAG, gnss_cold),
    OPT("--gnss-rtk", OPT_FLAG, gnss_rtk),
    OPT("--baro", OPT_STRING, baro_name),
    OPT("--baro-rate", OPT_DOUBLE, baro_rate),
    OPT("--fx-accel-deficit", OPT_DOUBLE, fx_accel_deficit),
    OPT("--fx-baro-lag", OPT_DOUBLE, fx_baro_lag),
    OPT("--fx-cam-drop", OPT_DOUBLE, fx_cam_drop),
    OPT("--fx-cam-shift", OPT_DOUBLE, fx_cam_shift),
    OPT("--mag", OPT_STRING, mag_name),
    OPT("--mag-rate", OPT_DOUBLE, mag_rate),
    OPT("--rangefinder", OPT_STRING, range_name),
    OPT("--rangefinder-rate", OPT_DOUBLE, ir_rate),
    OPT("--weather", OPT_STRING, weather),
    OPT("--vibration", OPT_DOUBLE, vibration),
    OPT("--list-sensors", OPT_FLAG, list_sensors),
    OPT("--scenario", OPT_STRING, scenario_path),
};

static void set_defaults(cli_args_t *args) {
    memset(args, 0, sizeof(*args));
    args->duration = 10.0;
    args->altitude = 1.0;
    args->lidar_rays = 36;
    args->cam_w = 640;
    args->cam_h = 360;
    args->cam_fps = 8.0;
    args->imu_rate = 1000.0;
    args->gnss_rate = 10.0;
    args->ir_rate = 100.0;
    args->baro_rate = 50.0;
    args->fx_accel_deficit = NAN;
    args->fx_baro_lag = NAN;
    args->fx_cam_drop = NAN;
    args->fx_cam_shift = NAN;
}

static int parse_number(const char *text, double *out) {
    char *end = NULL;
    errno = 0;
    double value = strtod(text, &end);
    if (end == text || *end != '\0' || errno != 0 || !isfinite(value)) {
        return -1;
    }
    *out = value;
    return 0;
}

static int apply_option(const opt_t *opt, const char *value, const char *name, cli_args_t *args) {
    char *base = (char *)args;
    if (opt->kind == OPT_FLAG) {
        *(bool *)(void *)(base + opt->offset) = true;
    } else if (opt->kind == OPT_STRING) {
        *(const char **)(void *)(base + opt->offset) = value;
    } else {
        double number = 0.0;
        if (parse_number(value, &number) != 0) {
            fprintf(stderr, "Invalid number for %s: %s\n", name, value);
            return -1;
        }
        if (opt->kind == OPT_INT) {
            if (number < -2147483648.0 || number > 2147483647.0 || number != floor(number)) {
                fprintf(stderr, "Invalid integer for %s: %s\n", name, value);
                return -1;
            }
            *(int *)(void *)(base + opt->offset) = (int)number;
        } else {
            *(double *)(void *)(base + opt->offset) = number;
        }
    }
    if (opt->has_implies) {
        *(bool *)(void *)(base + opt->implies_offset) = true;
    }
    return 0;
}

static const opt_t *find_option(const char *name) {
    for (size_t idx = 0; idx < sizeof(OPTIONS) / sizeof(OPTIONS[0]); idx++) {
        if (strcmp(OPTIONS[idx].name, name) == 0) {
            return &OPTIONS[idx];
        }
    }
    return NULL;
}

static int validate(const cli_args_t *args) {
    int bad = 0;
    bad |= args->duration <= 0.0;
    bad |= args->cam_w < 1 || args->cam_h < 1 || args->lidar_rays < 1 || args->lidar_rays > 360;
    bad |= args->cam_fps < 0.0 || args->imu_rate < 0.0 || args->gnss_rate < 0.0;
    bad |= args->baro_rate < 0.0 || args->mag_rate < 0.0 || args->ir_rate < 0.0;
    bad |= args->vibration < 0.0;
    int fx_given = !isnan(args->fx_accel_deficit) + !isnan(args->fx_baro_lag) +
                   !isnan(args->fx_cam_drop) + !isnan(args->fx_cam_shift);
    if (fx_given != 0 && fx_given != 4) {
        fprintf(stderr, "Airframe effects have no defaults: give all of --fx-accel-deficit, "
                        "--fx-baro-lag, --fx-cam-drop and --fx-cam-shift\n");
        return -1;
    }
    if (bad) {
        fprintf(stderr, "Option value out of range\n");
        return -1;
    }
    return 0;
}

int cli_parse(int argc, char **argv, cli_args_t *args) {
    set_defaults(args);
    for (int idx = 1; idx < argc; idx++) {
        const char *name = argv[idx];
        if (strcmp(name, "-h") == 0 || strcmp(name, "--help") == 0) {
            cli_print_usage(argv[0]);
            return 1;
        }
        const opt_t *opt = find_option(name);
        if (opt == NULL) {
            fprintf(stderr, "Unknown option: %s\n", name);
            cli_print_usage(argv[0]);
            return -1;
        }
        const char *value = NULL;
        if (opt->kind != OPT_FLAG) {
            if (idx + 1 >= argc) {
                fprintf(stderr, "Option %s needs a value\n", name);
                return -1;
            }
            value = argv[++idx];
        }
        if (apply_option(opt, value, name, args) != 0) {
            return -1;
        }
    }
    return validate(args);
}

void cli_print_usage(const char *prog) {
    printf("Usage: %s [OPTIONS]\n\n", prog);
    printf("Hummingbird quadrotor simulation (MuJoCo)\n\n");
    printf("Options:\n");
    printf("  --headless       Run without visualization\n");
    printf("  --duration SEC   Sim duration in seconds (headless, default 10)\n");
    printf("  --altitude M     Initial target altitude (default 1.0)\n");
    printf("  --model PATH     Path to MJCF model file\n");
    printf("  --no-ipc         Disable IPC transport\n");
    printf("  --lidar-rays N   Number of LiDAR rays (default 36)\n");
    printf("  --cam-width W    Camera width in pixels (default 640)\n");
    printf("  --cam-height H   Camera height in pixels (default 360)\n");
    printf("  --cam-fps F      Camera FPS (default 8.0)\n");
    printf("  --cam-mono       Publish a mono8 image through the camera sensor model\n");
    printf("  --cam-profile N  Camera sensor profile (see --list-sensors); implies --cam-mono\n");
    printf("  --scenario FILE  Run a composable multi-vehicle scenario\n");
    printf("  --imu NAME       IMU profile; --imu-rate HZ (default 1000)\n");
    printf("  --gnss NAME      GNSS profile; --gnss-rate HZ (default 10); --gnss-cold starts without\n");
    printf("                   a fix; --gnss-rtk enables RTK corrections\n");
    printf("  --baro NAME      Barometer profile; --baro-rate HZ (default 50)\n");
    printf("  --fx-accel-deficit MPS2 --fx-baro-lag S --fx-cam-drop P --fx-cam-shift S\n"
           "                   Measured airframe effects; all four or none, no defaults\n");
    printf("  --mag NAME       Magnetometer profile; --mag-rate HZ (default min(100, profile max))\n");
    printf("  --rangefinder N  Downward rangefinder profile; --rangefinder-rate HZ (default 100)\n");
    printf("  --weather NAME   Weather preset (ambient temperature, pressure, wind)\n");
    printf("  --vibration S    Motor vibration scale (0 = off, 1 = nominal)\n");
    printf("  --list-sensors   List available sensor, weather and camera profiles and exit\n");
    printf("  -h, --help       Show this help\n\n");
    printf("Interactive controls:\n");
    printf("  W/A/S/D          Move target XY\n");
    printf("  Up/Down          Raise/lower altitude\n");
    printf("  Left/Right       Yaw target\n");
    printf("  R                Reset target\n");
    printf("  Backspace        Reset simulation\n");
    printf("  Q / Escape       Quit\n");
    printf("  Mouse            Camera control\n");
}

void cli_list_sensor_profiles(void) {
    printf("IMU (--imu):\n");
    for (size_t idx = 0; idx < IMU_PROFILE_COUNT; idx++) {
        printf("  %-14s %-14s max %.0f Hz\n", IMU_PROFILES[idx].name, IMU_PROFILES[idx].grade,
               IMU_PROFILES[idx].max_odr_hz);
    }
    printf("GNSS (--gnss):\n");
    for (size_t idx = 0; idx < GNSS_PROFILE_COUNT; idx++) {
        printf("  %-14s %-14s max %.0f Hz\n", GNSS_PROFILES[idx].name, GNSS_PROFILES[idx].grade,
               GNSS_PROFILES[idx].max_rate_hz);
    }
    printf("Magnetometer (--mag):\n");
    for (size_t idx = 0; idx < MAG_PROFILE_COUNT; idx++) {
        printf("  %-14s %-14s max %.0f Hz\n", MAG_PROFILES[idx].name, MAG_PROFILES[idx].grade,
               MAG_PROFILES[idx].max_rate_hz);
    }
    printf("Barometer (--baro):\n");
    for (size_t idx = 0; idx < BARO_PROFILE_COUNT; idx++) {
        printf("  %-14s %-14s max %.0f Hz\n", BARO_PROFILES[idx].name, BARO_PROFILES[idx].grade,
               BARO_PROFILES[idx].max_rate_hz);
    }
    printf("Rangefinder (--rangefinder):\n");
    for (size_t idx = 0; idx < RANGE_PROFILE_COUNT; idx++) {
        printf("  %-14s %-14s %.2f..%.0f m, max %.0f Hz\n", RANGE_PROFILES[idx].name,
               RANGE_PROFILES[idx].grade, RANGE_PROFILES[idx].min_range_m,
               RANGE_PROFILES[idx].max_range_90_m, RANGE_PROFILES[idx].max_rate_hz);
    }
    printf("Camera (--cam-profile):\n");
    for (size_t idx = 0; idx < CAMERA_PROFILE_COUNT; idx++) {
        printf("  %-14s %-14s max %dx%d, %.0f fps\n", CAMERA_PROFILES[idx].name,
               CAMERA_PROFILES[idx].grade, CAMERA_PROFILES[idx].max_width,
               CAMERA_PROFILES[idx].max_height, CAMERA_PROFILES[idx].max_fps);
    }
    printf("Weather (--weather):\n");
    for (size_t idx = 0; idx < CONDITIONS_PRESET_COUNT; idx++) {
        printf("  %-14s %5.1f C, wind %.1f m/s\n", CONDITIONS_PRESETS[idx].name,
               CONDITIONS_PRESETS[idx].ambient_temp_c, CONDITIONS_PRESETS[idx].wind_speed_20ft_ms);
    }
}
