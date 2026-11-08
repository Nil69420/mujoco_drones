#include "types.h"
#include "controller.h"
#include "viewer.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifdef ENABLE_IPC
#include "transport/transport_renoir.h"
#include "sensors/sensors.h"
#include "scenario.h"
#ifdef ENABLE_FOXGLOVE
#include "foxglove/foxglove.h"
#endif
#endif

static void print_usage(const char *prog) {
    printf("Usage: %s [OPTIONS]\n\n", prog);
    printf("Hummingbird quadrotor simulation (MuJoCo)\n\n");
    printf("Options:\n");
    printf("  --headless       Run without visualization\n");
    printf("  --duration SEC   Sim duration in seconds (headless, default 10)\n");
    printf("  --altitude M     Initial target altitude (default 1.0)\n");
    printf("  --model PATH     Path to MJCF model file\n");
#ifdef ENABLE_IPC
    printf("  --no-ipc         Disable IPC transport\n");
    printf("  --lidar-rays N   Number of LiDAR rays (default 36)\n");
    printf("  --cam-width W    Camera width in pixels (default 640)\n");
    printf("  --cam-height H   Camera height in pixels (default 360)\n");
    printf("  --cam-fps F      Camera FPS (default 8.0)\n");
    printf("  --scenario FILE  Run a composable multi-vehicle scenario\n");
    printf("  --imu NAME       IMU profile (see --list-sensors); --imu-rate HZ (default 1000)\n");
    printf("  --gnss NAME      GNSS profile; --gnss-rate HZ (default 10)\n");
    printf("  --baro NAME      Barometer profile; --baro-rate HZ (default 50)\n");
    printf("  --mag NAME       Magnetometer profile; --mag-rate HZ (default 100)\n");
    printf("  --weather NAME   Weather preset (ambient temperature and pressure)\n");
    printf("  --vibration S    Motor vibration scale (0 = off, 1 = nominal)\n");
    printf("  --cam-mono        Publish a mono8 image through the camera sensor model\n");
    printf("  --cam-profile N  Camera sensor profile (ar0234, ov9281, ideal); implies --cam-mono\n");
    printf("  --list-sensors   List available sensor profiles and exit\n");
#endif
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

static const char *find_model(void) {
    static const char *candidates[] = {
        "model/scene.xml",
        "../model/scene.xml",
        "scene.xml",
        "model/hummingbird.xml",
        "../model/hummingbird.xml",
        NULL
    };
    for (int i = 0; candidates[i]; i++) {
        FILE *f = fopen(candidates[i], "r");
        if (f) { fclose(f); return candidates[i]; }
    }
    return NULL;
}

typedef struct {
    bool        headless;
    bool        duration_set;
    double      duration;
    double      altitude;
    const char *model_path;
#ifdef ENABLE_IPC
    bool        no_ipc;
    int         lidar_rays;
    int         cam_w;
    int         cam_h;
    double      cam_fps;
    const char *imu_name;
    const char *gnss_name;
    const char *baro_name;
    const char *mag_name;
    const char *cam_profile;
    bool        cam_mono;
    double      mag_rate;
    const char *weather;
    double      vibration;
    double      imu_rate;
    double      gnss_rate;
    double      baro_rate;
    bool        list_sensors;
    const char *scenario_path;
#endif
} cli_args_t;

#ifdef ENABLE_IPC
static void list_sensor_profiles(void) {
    printf("IMU (--imu):\n");
    for (size_t i = 0; i < IMU_PROFILE_COUNT; i++)
        printf("  %-12s %-12s max %.0f Hz\n", IMU_PROFILES[i].name, IMU_PROFILES[i].grade, IMU_PROFILES[i].max_odr_hz);
    printf("GNSS (--gnss):\n");
    for (size_t i = 0; i < GNSS_PROFILE_COUNT; i++)
        printf("  %-12s %-12s max %.0f Hz\n", GNSS_PROFILES[i].name, GNSS_PROFILES[i].grade, GNSS_PROFILES[i].max_rate_hz);
    printf("Magnetometer (--mag):\n");
    for (size_t i = 0; i < MAG_PROFILE_COUNT; i++)
        printf("  %-12s %-12s max %.0f Hz\n", MAG_PROFILES[i].name, MAG_PROFILES[i].grade, MAG_PROFILES[i].max_rate_hz);
    printf("Weather (--weather):\n");
    for (size_t i = 0; i < CONDITIONS_PRESET_COUNT; i++)
        printf("  %-14s %5.1f C, wind %.1f m/s\n", CONDITIONS_PRESETS[i].name, CONDITIONS_PRESETS[i].ambient_temp_c, CONDITIONS_PRESETS[i].wind_speed_20ft_ms);
    printf("Camera (--cam-profile):\n");
    for (size_t i = 0; i < CAMERA_PROFILE_COUNT; i++)
        printf("  %-12s %-12s max %ux%u, %.0f fps\n", CAMERA_PROFILES[i].name, CAMERA_PROFILES[i].grade, (unsigned)CAMERA_PROFILES[i].max_width, (unsigned)CAMERA_PROFILES[i].max_height, CAMERA_PROFILES[i].max_fps);
    printf("Barometer (--baro):\n");
    for (size_t i = 0; i < BARO_PROFILE_COUNT; i++)
        printf("  %-12s %-12s max %.0f Hz\n", BARO_PROFILES[i].name, BARO_PROFILES[i].grade, BARO_PROFILES[i].max_rate_hz);
}
#endif

static int parse_args(int argc, char **argv, cli_args_t *args) {
    args->headless   = false;
    args->duration_set = false;
    args->duration   = 10.0;
    args->altitude   = 1.0;
    args->model_path = NULL;
#ifdef ENABLE_IPC
    args->no_ipc     = false;
    args->lidar_rays = 36;
    args->cam_w      = 640;
    args->cam_h      = 360;
    args->cam_fps    = 8.0;
    args->imu_rate   = 1000.0;
    args->gnss_rate  = 10.0;
    args->mag_rate   = 100.0;
    args->baro_rate  = 50.0;
    args->scenario_path = NULL;
#endif

    for (int i = 1; i < argc; i++) {
        if      (!strcmp(argv[i], "--headless"))                  { args->headless = true; }
        else if (!strcmp(argv[i], "--duration") && i+1 < argc)   { args->duration   = strtod(argv[++i], NULL); args->duration_set = true; }
        else if (!strcmp(argv[i], "--altitude") && i+1 < argc)   { args->altitude   = strtod(argv[++i], NULL); }
        else if (!strcmp(argv[i], "--model")    && i+1 < argc)   { args->model_path = argv[++i]; }
#ifdef ENABLE_IPC
        else if (!strcmp(argv[i], "--no-ipc"))                    { args->no_ipc = true; }
        else if (!strcmp(argv[i], "--lidar-rays") && i+1 < argc) { args->lidar_rays = (int)strtol(argv[++i], NULL, 10); }
        else if (!strcmp(argv[i], "--cam-width")  && i+1 < argc) { args->cam_w = (int)strtol(argv[++i], NULL, 10); }
        else if (!strcmp(argv[i], "--cam-height") && i+1 < argc) { args->cam_h = (int)strtol(argv[++i], NULL, 10); }
        else if (!strcmp(argv[i], "--imu")        && i+1 < argc) { args->imu_name = argv[++i]; }
        else if (!strcmp(argv[i], "--gnss")       && i+1 < argc) { args->gnss_name = argv[++i]; }
        else if (!strcmp(argv[i], "--baro")       && i+1 < argc) { args->baro_name = argv[++i]; }
        else if (!strcmp(argv[i], "--mag")        && i+1 < argc) { args->mag_name = argv[++i]; }
        else if (!strcmp(argv[i], "--mag-rate")   && i+1 < argc) { args->mag_rate = strtod(argv[++i], NULL); }
        else if (!strcmp(argv[i], "--cam-profile") && i+1 < argc) { args->cam_profile = argv[++i]; args->cam_mono = true; }
        else if (!strcmp(argv[i], "--cam-mono")) { args->cam_mono = true; }
        else if (!strcmp(argv[i], "--weather")    && i+1 < argc) { args->weather = argv[++i]; }
        else if (!strcmp(argv[i], "--vibration")  && i+1 < argc) { args->vibration = strtod(argv[++i], NULL); }
        else if (!strcmp(argv[i], "--imu-rate")   && i+1 < argc) { args->imu_rate = strtod(argv[++i], NULL); }
        else if (!strcmp(argv[i], "--gnss-rate")  && i+1 < argc) { args->gnss_rate = strtod(argv[++i], NULL); }
        else if (!strcmp(argv[i], "--baro-rate")  && i+1 < argc) { args->baro_rate = strtod(argv[++i], NULL); }
        else if (!strcmp(argv[i], "--list-sensors")) { args->list_sensors = true; }
        else if (!strcmp(argv[i], "--cam-fps")    && i+1 < argc) { args->cam_fps = strtod(argv[++i], NULL); }
        else if (!strcmp(argv[i], "--scenario")   && i+1 < argc) { args->scenario_path = argv[++i]; }
#endif
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

#ifdef ENABLE_FOXGLOVE
static void apply_camera_cap(int cam_w, int cam_h, sensor_config_t *scfg) {
    if (cam_w < 1) cam_w = 1;
    if (cam_h < 1) cam_h = 1;
    bool clamped = false;
    if (cam_w > CAM_BUF_MAX_W) {
        cam_w = CAM_BUF_MAX_W;
        clamped = true;
    }
    if (cam_h > CAM_BUF_MAX_H) {
        cam_h = CAM_BUF_MAX_H;
        clamped = true;
    }
    if (clamped) {
        fprintf(stderr, "[main] WARNING: camera resolution reduced to %dx%d "
                        "(Foxglove bridge cap)\n",
                cam_w, cam_h);
    }
    scfg->camera_width  = (uint16_t)cam_w;
    scfg->camera_height = (uint16_t)cam_h;
}
#endif

static int run_headless(sim_t *sim, double duration) {
    printf("Running headless for %.1f seconds...\n", duration);
    printf("Target: altitude=%.1f m, position=(%.1f, %.1f)\n\n",
           sim->target.z, sim->target.x, sim->target.y);

    int print_every = (int)(0.1 / sim->model->opt.timestep);
    int step = 0;

    while (sim->data->time < duration) {
        ctrl_update(sim);
        mj_step(sim->model, sim->data);
#ifdef ENABLE_IPC
        if (sim->ipc_enabled) {
            sensor_update(&sim->sensors, sim->model, sim->data, &sim->target);
            sensor_render_camera(&sim->sensors, sim->model, sim->data);
        }
#endif
        if (++step % print_every == 0) {
            viewer_print_telemetry(sim);
        }
    }

    printf("\n\nFinal state:\n");
    printf("  Position: (%.4f, %.4f, %.4f)\n",
           sim->data->qpos[0], sim->data->qpos[1], sim->data->qpos[2]);

    double roll = 0.0, pitch = 0.0, yaw = 0.0;
    quat_to_euler(&sim->data->qpos[3], &roll, &pitch, &yaw);
    printf("  RPY:      (%.2f, %.2f, %.2f) deg\n",
           roll * 180.0/M_PI, pitch * 180.0/M_PI, yaw * 180.0/M_PI);
    printf("  Velocity: (%.4f, %.4f, %.4f)\n",
           sim->data->qvel[0], sim->data->qvel[1], sim->data->qvel[2]);

    return 0;
}

int main(int argc, char **argv) {
    cli_args_t args = {0};
    int parse_rc = parse_args(argc, argv, &args);
    if (parse_rc > 0) return 0;
    if (parse_rc < 0) return 1;

#ifdef ENABLE_IPC
    if (args.list_sensors) {
        list_sensor_profiles();
        return 0;
    }
    if (args.scenario_path) {
        return scenario_main_run(args.scenario_path, args.headless,
                                 args.duration_set ? args.duration : 0.0);
    }
#endif

    const char *model_path = args.model_path;
    if (!model_path) model_path = find_model();
    if (!model_path) {
        fprintf(stderr, "ERROR: Could not find scene.xml. Use --model.\n");
        return 1;
    }

    sim_t sim = {0};
    sim.gains  = ctrl_default_gains();
    sim.target = (setpoint_t){ .x = 0, .y = 0, .z = args.altitude, .yaw = 0 };

    char error[1000] = "";
    sim.model = mj_loadXML(model_path, NULL, error, sizeof(error));
    if (!sim.model) {
        fprintf(stderr, "ERROR loading model: %s\n", error);
        return 1;
    }
    sim.data = mj_makeData(sim.model);

    printf("Model loaded: %s\n", model_path);
    printf("  bodies: %ld, joints: %ld, actuators: %ld, sensors: %ld\n",
           sim.model->nbody, sim.model->njnt, sim.model->nu, sim.model->nsensor);
    printf("  timestep: %.4f s (%.0f Hz)\n",
           sim.model->opt.timestep, 1.0 / sim.model->opt.timestep);

    if (ctrl_resolve_actuators(&sim) != 0) {
        mj_deleteData(sim.data);
        mj_deleteModel(sim.model);
        return 1;
    }
    ctrl_reset(&sim);

#ifdef ENABLE_IPC
    sim.ipc_enabled = !args.no_ipc;
    if (sim.ipc_enabled) {
        if (transport_renoir_create(&sim.transport) != 0 ||
            transport_init(&sim.transport) != 0) {
            fprintf(stderr, "ERROR: failed to initialize IPC transport\n");
            sim.ipc_enabled = false;
        } else {
            sensor_config_t scfg = sensor_default_config();
            scfg.lidar_num_rays = (uint16_t)args.lidar_rays;

#ifdef ENABLE_FOXGLOVE
            apply_camera_cap(args.cam_w, args.cam_h, &scfg);
#else
            scfg.camera_width  = (uint16_t)args.cam_w;
            scfg.camera_height = (uint16_t)args.cam_h;
#endif
            scfg.camera_rate    = args.cam_fps;
            scfg.imu_profile    = args.imu_name;
            scfg.gnss_profile   = args.gnss_name;
            scfg.baro_profile   = args.baro_name;
            scfg.mag_profile    = args.mag_name;
            scfg.mag_rate       = args.mag_rate;
            scfg.weather        = args.weather;
            scfg.vibration_scale = args.vibration;
            scfg.imu_rate       = args.imu_rate;
            scfg.gnss_rate      = args.gnss_rate;
            scfg.baro_rate      = args.baro_rate;

            scfg.camera_profile = args.cam_profile;
            scfg.camera_mono    = args.cam_mono;
            scfg.camera_headless = args.headless;
#ifdef ENABLE_FOXGLOVE
            scfg.camera_legacy_topic = true;
#endif
            if (args.headless && !args.cam_mono) {
                scfg.enable.camera = false;
            }
#ifndef ENABLE_FOXGLOVE
            if (!args.headless) {
                fprintf(stderr, "[main] WARNING: camera stream disabled "
                                "(no Foxglove bridge to consume it)\n");
                scfg.enable.camera = false;
            }
#endif

            if (sensor_init(&sim.sensors, sim.model,
                            &sim.transport, &scfg, NULL) != 0) {
                fprintf(stderr, "ERROR: failed to initialize sensors\n");
                if (args.imu_name != NULL || args.gnss_name != NULL || args.baro_name != NULL ||
                    args.mag_name != NULL || args.weather != NULL || args.vibration > 0.0 ||
                    args.cam_mono) {
                    transport_shutdown(&sim.transport);
                    mj_deleteData(sim.data);
                    mj_deleteModel(sim.model);
                    return 1;
                }
                sim.ipc_enabled = false;
            }
        }
    }

#ifdef ENABLE_FOXGLOVE
    foxglove_bridge_t *fg = NULL;
    if (sim.ipc_enabled) {
        fg = foxglove_create(&sim.transport, FG_DEFAULT_PORT);
        if (fg && foxglove_start(fg) != 0) {
            foxglove_destroy(fg);
            fg = NULL;
        }
    }
#endif
#endif

    int rc = 0;
    if (args.headless) {
        rc = run_headless(&sim, args.duration);
    } else {
#ifdef NO_GLFW
        fprintf(stderr, "ERROR: Built without GLFW. Use --headless.\n");
        rc = 1;
#else
        if (viewer_init(&sim) != 0) {
            rc = 1;
        } else {
            printf("Interactive mode. Q to quit.\n");
            viewer_loop(&sim);
            viewer_close();
            printf("\n");
            rc = 0;
        }
#endif
    }

#ifdef ENABLE_IPC
    if (sim.ipc_enabled) {
#ifdef ENABLE_FOXGLOVE
        foxglove_destroy(fg);
#endif
        sensor_cleanup(&sim.sensors);
        transport_shutdown(&sim.transport);
    }
#endif

    mj_deleteData(sim.data);
    mj_deleteModel(sim.model);
    return rc;
}
