#include "cli.h"
#include "types.h"
#include "controller.h"
#include "viewer.h"

#include <math.h>
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

#ifdef ENABLE_IPC
static void build_sensor_config(const cli_args_t *args, sensor_config_t *scfg,
                                airframe_effects_params_t *effects) {
    *scfg = sensor_default_config();
    scfg->lidar_num_rays = (uint16_t)args->lidar_rays;

#ifdef ENABLE_FOXGLOVE
    apply_camera_cap(args->cam_w, args->cam_h, scfg);
#else
    scfg->camera_width  = (uint16_t)args->cam_w;
    scfg->camera_height = (uint16_t)args->cam_h;
#endif
    scfg->camera_rate    = args->cam_fps;
    scfg->imu_profile    = args->imu_name;
    scfg->gnss_profile   = args->gnss_name;
    scfg->gnss_cold_start = args->gnss_cold;
    scfg->rtk_corrections = args->gnss_rtk ? 1 : 0;
    scfg->baro_profile   = args->baro_name;
    scfg->mag_profile    = args->mag_name;
    scfg->range_profile  = args->range_name;
    scfg->infrared_rate  = args->ir_rate;
    scfg->mag_rate       = args->mag_rate;
    scfg->weather        = args->weather;
    scfg->vibration_scale = args->vibration;
    scfg->imu_rate       = args->imu_rate;
    scfg->gnss_rate      = args->gnss_rate;
    scfg->baro_rate      = args->baro_rate;
        if (!isnan(args->fx_accel_deficit)) {
        effects->accel_thrust_deficit_mps2 = args->fx_accel_deficit;
        effects->baro_lag_s = args->fx_baro_lag;
        effects->camera_drop_probability = args->fx_cam_drop;
        effects->camera_time_shift_s = args->fx_cam_shift;
        effects->provenance = PROVENANCE_ASSUMED;
        scfg->effects = effects;
    }

    scfg->camera_profile = args->cam_profile;
    scfg->camera_mono    = args->cam_mono;
    scfg->camera_headless = args->headless;
#ifdef ENABLE_FOXGLOVE
    scfg->camera_legacy_topic = true;
#endif
    if (args->headless && !args->cam_mono) {
        scfg->enable.camera = false;
    }
#ifndef ENABLE_FOXGLOVE
    if (!args->headless) {
        fprintf(stderr, "[main] WARNING: camera stream disabled "
                        "(no Foxglove bridge to consume it)\n");
        scfg->enable.camera = false;
    }
#endif
}

static bool sensors_requested(const cli_args_t *args) {
    return args->imu_name != NULL || args->gnss_name != NULL || args->baro_name != NULL ||
           args->mag_name != NULL || args->range_name != NULL || args->weather != NULL ||
           args->vibration > 0.0 || args->cam_mono || !isnan(args->fx_accel_deficit);
}
#endif

int main(int argc, char **argv) {
    cli_args_t args = {0};
    int parse_rc = cli_parse(argc, argv, &args);
    if (parse_rc > 0) return 0;
    if (parse_rc < 0) return 1;

#ifdef ENABLE_IPC
    if (args.list_sensors) {
        cli_list_sensor_profiles();
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
            sensor_config_t scfg;
            airframe_effects_params_t effects;
            build_sensor_config(&args, &scfg, &effects);

            if (sensor_init(&sim.sensors, sim.model,
                            &sim.transport, &scfg, NULL) != 0) {
                fprintf(stderr, "ERROR: failed to initialize sensors\n");
                if (sensors_requested(&args)) {
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
