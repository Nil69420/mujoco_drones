#ifndef MUJOCO_DRONES_CLI_H
#define MUJOCO_DRONES_CLI_H

#include <stdbool.h>

typedef struct {
    bool headless;
    bool duration_set;
    double duration;
    double altitude;
    const char *model_path;
    bool no_ipc;
    int lidar_rays;
    int cam_w;
    int cam_h;
    double cam_fps;
    const char *imu_name;
    const char *gnss_name;
    const char *baro_name;
    const char *mag_name;
    const char *range_name;
    double ir_rate;
    const char *cam_profile;
    bool cam_mono;
    double mag_rate;
    bool gnss_cold;
    bool gnss_rtk;
    const char *weather;
    double vibration;
    double imu_rate;
    double gnss_rate;
    double baro_rate;
    double fx_accel_deficit;
    double fx_baro_lag;
    double fx_cam_drop;
    double fx_cam_shift;
    bool list_sensors;
    const char *scenario_path;
} cli_args_t;

int cli_parse(int argc, char **argv, cli_args_t *args);
void cli_print_usage(const char *prog);
void cli_list_sensor_profiles(void);

#endif
