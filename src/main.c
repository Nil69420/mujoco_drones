#include "types.h"
#include "controller.h"
#include "viewer.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifdef ENABLE_IPC
#include "transport/transport_renoir.h"
#include "sensors/sensors.h"
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
    printf("  --cam-width W    Camera width in pixels (default 320)\n");
    printf("  --cam-height H   Camera height in pixels (default 240)\n");
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
    double      duration;
    double      altitude;
    const char *model_path;
#ifdef ENABLE_IPC
    bool        no_ipc;
    int         lidar_rays;
    int         cam_w;
    int         cam_h;
#endif
} cli_args_t;

static int parse_args(int argc, char **argv, cli_args_t *args) {
    args->headless   = false;
    args->duration   = 10.0;
    args->altitude   = 1.0;
    args->model_path = NULL;
#ifdef ENABLE_IPC
    args->no_ipc     = false;
    args->lidar_rays = 36;
    args->cam_w      = 320;
    args->cam_h      = 240;
#endif

    for (int i = 1; i < argc; i++) {
        if      (!strcmp(argv[i], "--headless"))                  { args->headless = true; }
        else if (!strcmp(argv[i], "--duration") && i+1 < argc)   { args->duration   = strtod(argv[++i], NULL); }
        else if (!strcmp(argv[i], "--altitude") && i+1 < argc)   { args->altitude   = strtod(argv[++i], NULL); }
        else if (!strcmp(argv[i], "--model")    && i+1 < argc)   { args->model_path = argv[++i]; }
#ifdef ENABLE_IPC
        else if (!strcmp(argv[i], "--no-ipc"))                    { args->no_ipc = true; }
        else if (!strcmp(argv[i], "--lidar-rays") && i+1 < argc) { args->lidar_rays = (int)strtol(argv[++i], NULL, 10); }
        else if (!strcmp(argv[i], "--cam-width")  && i+1 < argc) { args->cam_w = (int)strtol(argv[++i], NULL, 10); }
        else if (!strcmp(argv[i], "--cam-height") && i+1 < argc) { args->cam_h = (int)strtol(argv[++i], NULL, 10); }
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
            scfg.camera_width   = (uint16_t)args.cam_w;
            scfg.camera_height  = (uint16_t)args.cam_h;

            if (args.headless) {
                scfg.enable.camera = false;
            }

            if (sensor_init(&sim.sensors, sim.model,
                            &sim.transport, &scfg) != 0) {
                fprintf(stderr, "ERROR: failed to initialize sensors\n");
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
