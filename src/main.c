/*
 * main.c - Entry point for mujoco_drones
 *
 * Keeps main() small: parse args → load model → run sim.
 * All logic lives in controller.c and viewer.c.
 */

#include "types.h"
#include "controller.h"
#include "viewer.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* ================================================================ */
static void print_usage(const char *prog) {
    printf("Usage: %s [OPTIONS]\n\n", prog);
    printf("Hummingbird quadrotor simulation (MuJoCo)\n\n");
    printf("Options:\n");
    printf("  --headless       Run without visualization\n");
    printf("  --duration SEC   Sim duration in seconds (headless, default 10)\n");
    printf("  --altitude M     Initial target altitude (default 1.0)\n");
    printf("  --model PATH     Path to MJCF model file\n");
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

/* ================================================================ */
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

/* ================================================================ */
static int run_headless(sim_t *sim, double duration) {
    printf("Running headless for %.1f seconds...\n", duration);
    printf("Target: altitude=%.1f m, position=(%.1f, %.1f)\n\n",
           sim->target.z, sim->target.x, sim->target.y);

    int print_every = (int)(0.1 / sim->model->opt.timestep);
    int step = 0;

    while (sim->data->time < duration) {
        mj_step(sim->model, sim->data);
        if (++step % print_every == 0)
            viewer_print_telemetry(sim);
    }

    printf("\n\nFinal state:\n");
    printf("  Position: (%.4f, %.4f, %.4f)\n",
           sim->data->qpos[0], sim->data->qpos[1], sim->data->qpos[2]);

    double roll, pitch, yaw;
    quat_to_euler(&sim->data->qpos[3], &roll, &pitch, &yaw);
    printf("  RPY:      (%.2f, %.2f, %.2f) deg\n",
           roll * 180.0/M_PI, pitch * 180.0/M_PI, yaw * 180.0/M_PI);
    printf("  Velocity: (%.4f, %.4f, %.4f)\n",
           sim->data->qvel[0], sim->data->qvel[1], sim->data->qvel[2]);

    return 0;
}

/* ================================================================ */
int main(int argc, char **argv) {
    bool        headless   = false;
    double      duration   = 10.0;
    const char *model_path = NULL;
    double      altitude   = 1.0;

    /* ---- Parse arguments ---- */
    for (int i = 1; i < argc; i++) {
        if      (!strcmp(argv[i], "--headless"))                  headless = true;
        else if (!strcmp(argv[i], "--duration") && i+1 < argc)   duration   = atof(argv[++i]);
        else if (!strcmp(argv[i], "--altitude") && i+1 < argc)   altitude   = atof(argv[++i]);
        else if (!strcmp(argv[i], "--model")    && i+1 < argc)   model_path = argv[++i];
        else if (!strcmp(argv[i], "-h") || !strcmp(argv[i], "--help")) {
            print_usage(argv[0]);
            return 0;
        } else {
            fprintf(stderr, "Unknown option: %s\n", argv[i]);
            print_usage(argv[0]);
            return 1;
        }
    }

    /* ---- Locate model ---- */
    if (!model_path) model_path = find_model();
    if (!model_path) {
        fprintf(stderr, "ERROR: Could not find scene.xml. Use --model.\n");
        return 1;
    }

    /* ---- Initialize sim context ---- */
    sim_t sim = {0};
    sim.gains  = ctrl_default_gains();
    sim.target = (setpoint_t){ .x = 0, .y = 0, .z = altitude, .yaw = 0 };

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

    /* ---- Wire up controller ---- */
    if (ctrl_resolve_actuators(&sim) != 0) {
        mj_deleteData(sim.data);
        mj_deleteModel(sim.model);
        return 1;
    }
    ctrl_reset(&sim);
    mjcb_control = ctrl_update;

    /* ---- Run ---- */
    int rc;
    if (headless) {
        rc = run_headless(&sim, duration);
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

    /* ---- Cleanup ---- */
    mj_deleteData(sim.data);
    mj_deleteModel(sim.model);
    return rc;
}
