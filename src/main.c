/*
 * mujoco_drones - Hummingbird Quadrotor Simulation
 *
 * MuJoCo-based simulation of the AscTec Hummingbird quadrotor,
 * converted from the RotorS/Gazebo URDF model (ETH ASL).
 *
 * Features:
 *   - Full 6-DOF dynamics via MuJoCo freejoint
 *   - PD attitude + altitude controller
 *   - Motor mixing for + configuration
 *   - GLFW interactive visualization with camera controls
 *   - Headless mode for batch simulation
 *
 * Build:
 *   mkdir build && cd build
 *   cmake .. && make
 *
 * Run:
 *   ./hummingbird              (interactive with viewer)
 *   ./hummingbird --headless   (console only)
 */

#include <mujoco/mujoco.h>

#ifndef NO_GLFW
#include <GLFW/glfw3.h>
#endif

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>

/* ================================================================
 * Physical constants from hummingbird.xacro / hummingbird.yaml
 * ================================================================ */
#define GRAVITY          9.81
#define MASS             0.68       /* kg   (total with rotors: 0.68 + 4*0.009) */
#define TOTAL_MASS       0.716      /* kg   (from yaml, used for hover calc)    */
#define ARM_LENGTH       0.17       /* m    */
#define MOMENT_CONSTANT  0.016      /* m    torque = km * thrust                */
#define MOTOR_KF         8.54858e-6 /* kg*m/s^2  thrust = kf * omega^2         */
#define MAX_OMEGA        838.0      /* rad/s                                    */
#define MAX_THRUST       (MOTOR_KF * MAX_OMEGA * MAX_OMEGA)  /* ~6.0 N/rotor   */
#define HOVER_THRUST     (TOTAL_MASS * GRAVITY / 4.0)        /* ~1.76 N/rotor  */
#define NUM_ROTORS       4

/* ================================================================
 * Controller gains
 * ================================================================ */
static struct {
    /* Altitude (Z) */
    double kp_z;
    double kd_z;
    double ki_z;

    /* Roll (X rotation) */
    double kp_roll;
    double kd_roll;

    /* Pitch (Y rotation) */
    double kp_pitch;
    double kd_pitch;

    /* Yaw (Z rotation) */
    double kp_yaw;
    double kd_yaw;

    /* XY position */
    double kp_xy;
    double kd_xy;
} gains = {
    .kp_z     = 20.0,
    .kd_z     = 10.0,
    .ki_z     =  0.5,

    .kp_roll  = 10.0,
    .kd_roll  =  3.0,

    .kp_pitch = 10.0,
    .kd_pitch =  3.0,

    .kp_yaw   =  5.0,
    .kd_yaw   =  1.5,

    .kp_xy    =  2.0,
    .kd_xy    =  1.5,
};

/* ================================================================
 * Setpoint / target state
 * ================================================================ */
static struct {
    double x, y, z;
    double yaw;
} target = {
    .x   = 0.0,
    .y   = 0.0,
    .z   = 1.0,
    .yaw = 0.0,
};

/* ================================================================
 * Simulation state
 * ================================================================ */
static mjModel* m = NULL;
static mjData*  d = NULL;

/* Actuator indices (resolved at init) */
static int act_thrust[4];
static int act_spin[4];

/* Integral error for altitude */
static double z_integral = 0.0;

/* Visualization state */
#ifndef NO_GLFW
static mjvCamera  cam;
static mjvOption   opt;
static mjvScene   scn;
static mjrContext  con;

/* Mouse interaction */
static bool button_left   = false;
static bool button_middle = false;
static bool button_right  = false;
static double lastx = 0, lasty = 0;
#endif

/* ================================================================
 * Utility: clamp
 * ================================================================ */
static inline double clampd(double v, double lo, double hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

/* ================================================================
 * Quaternion to Euler (ZYX convention)
 * MuJoCo quaternion: [w, x, y, z]
 * ================================================================ */
static void quat_to_euler(const double q[4], double *roll, double *pitch, double *yaw) {
    double w = q[0], x = q[1], y = q[2], z = q[3];

    /* Roll (x-axis rotation) */
    double sinr = 2.0 * (w * x + y * z);
    double cosr = 1.0 - 2.0 * (x * x + y * y);
    *roll = atan2(sinr, cosr);

    /* Pitch (y-axis rotation) */
    double sinp = 2.0 * (w * y - z * x);
    if (fabs(sinp) >= 1.0)
        *pitch = copysign(M_PI / 2.0, sinp);
    else
        *pitch = asin(sinp);

    /* Yaw (z-axis rotation) */
    double siny = 2.0 * (w * z + x * y);
    double cosy = 1.0 - 2.0 * (y * y + z * z);
    *yaw = atan2(siny, cosy);
}

/* ================================================================
 * Motor mixer for + configuration
 *
 * Layout (top view):
 *        Rotor 0 (front, +X, CW)
 *            |
 *  Rotor 1 --+-- Rotor 3   (left +Y CCW, right -Y CCW)
 *            |
 *        Rotor 2 (back, -X, CW)
 *
 * Torque equations (derived from cross products r × F at each site):
 *   roll_torque  (τ_x) = L * (F1 - F3)
 *   pitch_torque (τ_y) = L * (F2 - F0)
 *   yaw_torque   (τ_z) = c * (-F0 + F1 - F2 + F3)
 *   total_thrust (T)   = F0 + F1 + F2 + F3
 *
 * Inverse:
 *   F0 = T/4 - τ_y/(2L) - τ_z/(4c)
 *   F1 = T/4 + τ_x/(2L) + τ_z/(4c)
 *   F2 = T/4 + τ_y/(2L) - τ_z/(4c)
 *   F3 = T/4 - τ_x/(2L) + τ_z/(4c)
 * ================================================================ */
static void mixer(double total_thrust,
                  double tau_roll, double tau_pitch, double tau_yaw,
                  double f_out[4])
{
    double L  = ARM_LENGTH;
    double km = MOMENT_CONSTANT;

    f_out[0] = total_thrust / 4.0 - tau_pitch / (2.0 * L) - tau_yaw / (4.0 * km);
    f_out[1] = total_thrust / 4.0 + tau_roll  / (2.0 * L) + tau_yaw / (4.0 * km);
    f_out[2] = total_thrust / 4.0 + tau_pitch / (2.0 * L) - tau_yaw / (4.0 * km);
    f_out[3] = total_thrust / 4.0 - tau_roll  / (2.0 * L) + tau_yaw / (4.0 * km);

    /* Clamp to physical limits */
    for (int i = 0; i < 4; i++)
        f_out[i] = clampd(f_out[i], 0.0, MAX_THRUST);
}

/* ================================================================
 * Thrust to angular velocity (for visual spin)
 *   thrust = kf * omega^2  →  omega = sqrt(thrust / kf)
 * ================================================================ */
static double thrust_to_omega(double thrust) {
    if (thrust <= 0.0) return 0.0;
    return sqrt(thrust / MOTOR_KF);
}

/* ================================================================
 * Controller callback
 *
 * Full attitude + altitude + position PD controller.
 * Runs at the simulation timestep (~500 Hz at dt=0.002).
 * ================================================================ */
static void controller(const mjModel* model, mjData* data) {
    /* ---- Read state ---- */
    /* Freejoint: qpos = [x y z qw qx qy qz ...], qvel = [vx vy vz wx wy wz ...] */
    double px = data->qpos[0];
    double py = data->qpos[1];
    double pz = data->qpos[2];

    double vx = data->qvel[0];
    double vy = data->qvel[1];
    double vz = data->qvel[2];

    double wx = data->qvel[3];
    double wy = data->qvel[4];
    double wz = data->qvel[5];

    double roll, pitch, yaw;
    quat_to_euler(&data->qpos[3], &roll, &pitch, &yaw);

    /* ---- Altitude controller ---- */
    double z_error = target.z - pz;
    z_integral += z_error * model->opt.timestep;
    z_integral = clampd(z_integral, -2.0, 2.0);

    double thrust_cmd = TOTAL_MASS * GRAVITY
                      + gains.kp_z * z_error
                      + gains.kd_z * (-vz)
                      + gains.ki_z * z_integral;

    thrust_cmd = clampd(thrust_cmd, 0.0, 4.0 * MAX_THRUST);

    /* ---- Outer loop: XY position → desired roll/pitch ----
     * Transform XY errors into body-frame desired angles.
     * Desired pitch to move in +X: negative pitch (nose down).
     * Desired roll to move in +Y: positive roll (left up).
     */
    double ax_cmd = gains.kp_xy * (target.x - px) + gains.kd_xy * (-vx);
    double ay_cmd = gains.kp_xy * (target.y - py) + gains.kd_xy * (-vy);

    /* Rotate world-frame acceleration commands into body yaw frame */
    double cos_yaw = cos(yaw);
    double sin_yaw = sin(yaw);
    double ax_body =  cos_yaw * ax_cmd + sin_yaw * ay_cmd;
    double ay_body = -sin_yaw * ax_cmd + cos_yaw * ay_cmd;

    /* Desired angles (small-angle linearization) */
    double desired_pitch = clampd(-ax_body / GRAVITY, -0.3, 0.3);
    double desired_roll  = clampd( ay_body / GRAVITY, -0.3, 0.3);

    /* ---- Inner loop: attitude PD ---- */
    double tau_roll  = gains.kp_roll  * (desired_roll  - roll)  + gains.kd_roll  * (-wx);
    double tau_pitch = gains.kp_pitch * (desired_pitch - pitch) + gains.kd_pitch * (-wy);
    double tau_yaw   = gains.kp_yaw   * (target.yaw   - yaw)   + gains.kd_yaw   * (-wz);

    /* ---- Motor mixing ---- */
    double forces[4];
    mixer(thrust_cmd, tau_roll, tau_pitch, tau_yaw, forces);

    /* ---- Apply to actuators ---- */
    for (int i = 0; i < 4; i++) {
        data->ctrl[act_thrust[i]] = forces[i];

        /* Visual spin: CW rotors spin negative, CCW spin positive */
        double omega = thrust_to_omega(forces[i]);
        if (i == 0 || i == 2)
            data->ctrl[act_spin[i]] = -omega;   /* CW  */
        else
            data->ctrl[act_spin[i]] =  omega;   /* CCW */
    }
}

/* ================================================================
 * Resolve actuator indices by name
 * ================================================================ */
static void resolve_actuators(void) {
    char name[32];
    for (int i = 0; i < 4; i++) {
        snprintf(name, sizeof(name), "thrust_%d", i);
        act_thrust[i] = mj_name2id(m, mjOBJ_ACTUATOR, name);
        if (act_thrust[i] < 0) {
            fprintf(stderr, "ERROR: actuator '%s' not found in model\n", name);
            exit(1);
        }

        snprintf(name, sizeof(name), "spin_%d", i);
        act_spin[i] = mj_name2id(m, mjOBJ_ACTUATOR, name);
        if (act_spin[i] < 0) {
            fprintf(stderr, "ERROR: actuator '%s' not found in model\n", name);
            exit(1);
        }
    }
}

/* ================================================================
 * GLFW callbacks
 * ================================================================ */
#ifndef NO_GLFW
static void keyboard(GLFWwindow* window, int key, int scancode, int act, int mods) {
    (void)scancode; (void)mods;
    if (act != GLFW_PRESS) return;

    switch (key) {
    case GLFW_KEY_ESCAPE:
    case GLFW_KEY_Q:
        glfwSetWindowShouldClose(window, 1);
        break;

    case GLFW_KEY_BACKSPACE:
        /* Reset simulation */
        mj_resetData(m, d);
        z_integral = 0.0;
        break;

    /* Target altitude */
    case GLFW_KEY_UP:    target.z += 0.25; break;
    case GLFW_KEY_DOWN:  target.z -= 0.25; if (target.z < 0.2) target.z = 0.2; break;

    /* Target XY position */
    case GLFW_KEY_W:     target.x += 0.5; break;
    case GLFW_KEY_S:     target.x -= 0.5; break;
    case GLFW_KEY_A:     target.y += 0.5; break;
    case GLFW_KEY_D:     target.y -= 0.5; break;

    /* Target yaw */
    case GLFW_KEY_LEFT:  target.yaw += 0.2; break;
    case GLFW_KEY_RIGHT: target.yaw -= 0.2; break;

    /* Reset target */
    case GLFW_KEY_R:
        target.x = 0; target.y = 0; target.z = 1.0; target.yaw = 0;
        break;
    }
}

static void mouse_button(GLFWwindow* window, int button, int act, int mods) {
    (void)mods;
    button_left   = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT)   == GLFW_PRESS);
    button_middle = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS);
    button_right  = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT)  == GLFW_PRESS);
    glfwGetCursorPos(window, &lastx, &lasty);
    (void)button; (void)act;
}

static void mouse_move(GLFWwindow* window, double xpos, double ypos) {
    double dx = xpos - lastx;
    double dy = ypos - lasty;
    lastx = xpos;
    lasty = ypos;

    if (!button_left && !button_middle && !button_right) return;

    int width, height;
    glfwGetWindowSize(window, &width, &height);

    bool shift = (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS ||
                  glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS);

    mjtMouse action;
    if (button_right)
        action = shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
    else if (button_left)
        action = shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
    else
        action = mjMOUSE_ZOOM;

    mjv_moveCamera(m, action, dx / (double)width, dy / (double)height, &scn, &cam);
}

static void scroll(GLFWwindow* window, double xoffset, double yoffset) {
    (void)window; (void)xoffset;
    mjv_moveCamera(m, mjMOUSE_ZOOM, 0.0, -0.05 * yoffset, &scn, &cam);
}
#endif /* NO_GLFW */

/* ================================================================
 * Print telemetry to console
 * ================================================================ */
static void print_telemetry(double sim_time) {
    double roll, pitch, yaw;
    quat_to_euler(&d->qpos[3], &roll, &pitch, &yaw);

    printf("\rt=%.2f | pos=(%.2f, %.2f, %.2f) | rpy=(%.1f°, %.1f°, %.1f°) | "
           "tgt=(%.1f, %.1f, %.1f) | F=[%.2f %.2f %.2f %.2f]",
           sim_time,
           d->qpos[0], d->qpos[1], d->qpos[2],
           roll * 180.0/M_PI, pitch * 180.0/M_PI, yaw * 180.0/M_PI,
           target.x, target.y, target.z,
           d->ctrl[act_thrust[0]], d->ctrl[act_thrust[1]],
           d->ctrl[act_thrust[2]], d->ctrl[act_thrust[3]]);
    fflush(stdout);
}

/* ================================================================
 * Usage
 * ================================================================ */
static void print_usage(const char* prog) {
    printf("Usage: %s [OPTIONS]\n\n", prog);
    printf("Hummingbird quadrotor simulation (MuJoCo)\n\n");
    printf("Options:\n");
    printf("  --headless       Run without visualization\n");
    printf("  --duration SEC   Simulation duration in seconds (headless, default 10)\n");
    printf("  --altitude M     Initial target altitude in meters (default 1.0)\n");
    printf("  --model PATH     Path to MJCF model file\n");
    printf("  -h, --help       Show this help\n");
    printf("\nInteractive controls:\n");
    printf("  W/A/S/D          Move target XY position\n");
    printf("  Up/Down arrows   Raise/lower target altitude\n");
    printf("  Left/Right       Rotate target yaw\n");
    printf("  R                Reset target to origin\n");
    printf("  Backspace        Reset simulation\n");
    printf("  Q / Escape       Quit\n");
    printf("  Mouse            Rotate/pan/zoom camera\n");
}

/* ================================================================
 * Main
 * ================================================================ */
int main(int argc, char** argv) {
    /* ---- Parse arguments ---- */
    bool headless = false;
    double duration = 10.0;
    const char* model_path = NULL;

    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--headless") == 0) {
            headless = true;
        } else if (strcmp(argv[i], "--duration") == 0 && i + 1 < argc) {
            duration = atof(argv[++i]);
        } else if (strcmp(argv[i], "--altitude") == 0 && i + 1 < argc) {
            target.z = atof(argv[++i]);
        } else if (strcmp(argv[i], "--model") == 0 && i + 1 < argc) {
            model_path = argv[++i];
        } else if (strcmp(argv[i], "-h") == 0 || strcmp(argv[i], "--help") == 0) {
            print_usage(argv[0]);
            return 0;
        } else {
            fprintf(stderr, "Unknown option: %s\n", argv[i]);
            print_usage(argv[0]);
            return 1;
        }
    }

    /* ---- Find model file ---- */
    if (!model_path) {
        /* Try relative paths from likely run locations */
        const char* candidates[] = {
            "model/scene.xml",
            "../model/scene.xml",
            "scene.xml",
            "model/hummingbird.xml",
            "../model/hummingbird.xml",
            NULL
        };
        for (int i = 0; candidates[i]; i++) {
            FILE* f = fopen(candidates[i], "r");
            if (f) { fclose(f); model_path = candidates[i]; break; }
        }
        if (!model_path) {
            fprintf(stderr, "ERROR: Could not find scene.xml or hummingbird.xml. "
                    "Use --model to specify path.\n");
            return 1;
        }
    }

    /* ---- Load model ---- */
    char error[1000] = "";
    m = mj_loadXML(model_path, NULL, error, sizeof(error));
    if (!m) {
        fprintf(stderr, "ERROR loading model: %s\n", error);
        return 1;
    }
    d = mj_makeData(m);

    printf("Model loaded: %s\n", model_path);
    printf("  bodies: %ld, joints: %ld, actuators: %ld, sensors: %ld\n",
           m->nbody, m->njnt, m->nu, m->nsensor);
    printf("  timestep: %.4f s (%.0f Hz)\n", m->opt.timestep, 1.0 / m->opt.timestep);

    /* ---- Resolve actuator IDs ---- */
    resolve_actuators();

    /* ---- Install controller callback ---- */
    mjcb_control = controller;

    if (headless) {
        /* ============================================================
         * Headless mode: just step the simulation and print telemetry
         * ============================================================ */
        printf("Running headless for %.1f seconds...\n", duration);
        printf("Target: altitude=%.1f m, position=(%.1f, %.1f)\n\n",
               target.z, target.x, target.y);

        int print_every = (int)(0.1 / m->opt.timestep);   /* print at ~10 Hz */
        int step = 0;

        while (d->time < duration) {
            mj_step(m, d);
            step++;
            if (step % print_every == 0)
                print_telemetry(d->time);
        }

        printf("\n\nFinal state:\n");
        printf("  Position: (%.4f, %.4f, %.4f)\n", d->qpos[0], d->qpos[1], d->qpos[2]);
        double roll, pitch, yaw;
        quat_to_euler(&d->qpos[3], &roll, &pitch, &yaw);
        printf("  RPY:      (%.2f°, %.2f°, %.2f°)\n",
               roll*180/M_PI, pitch*180/M_PI, yaw*180/M_PI);
        printf("  Velocity: (%.4f, %.4f, %.4f)\n", d->qvel[0], d->qvel[1], d->qvel[2]);

    } else {
#ifdef NO_GLFW
        fprintf(stderr, "ERROR: Built without GLFW. Use --headless.\n");
        mj_deleteData(d);
        mj_deleteModel(m);
        return 1;
#else
        /* ============================================================
         * Interactive mode: GLFW window with MuJoCo rendering
         * ============================================================ */
        if (!glfwInit()) {
            fprintf(stderr, "ERROR: could not initialize GLFW\n");
            mj_deleteData(d);
            mj_deleteModel(m);
            return 1;
        }

        GLFWwindow* window = glfwCreateWindow(1280, 720, "Hummingbird - mujoco_drones", NULL, NULL);
        if (!window) {
            fprintf(stderr, "ERROR: could not create GLFW window\n");
            glfwTerminate();
            mj_deleteData(d);
            mj_deleteModel(m);
            return 1;
        }
        glfwMakeContextCurrent(window);
        glfwSwapInterval(1);

        /* Initialize visualization */
        mjv_defaultCamera(&cam);
        mjv_defaultOption(&opt);
        mjv_defaultScene(&scn);
        mjr_defaultContext(&con);

        mjv_makeScene(m, &scn, 10000);
        mjr_makeContext(m, &con, mjFONTSCALE_150);

        /* Camera setup: look at the drone from behind and above */
        cam.type = mjCAMERA_FREE;
        cam.distance = 4.0;
        cam.azimuth = 135.0;
        cam.elevation = -25.0;
        cam.lookat[0] = 0.0;
        cam.lookat[1] = 0.0;
        cam.lookat[2] = 1.0;

        /* Callbacks */
        glfwSetKeyCallback(window, keyboard);
        glfwSetCursorPosCallback(window, mouse_move);
        glfwSetMouseButtonCallback(window, mouse_button);
        glfwSetScrollCallback(window, scroll);

        printf("Interactive mode. Press H for controls, Q to quit.\n");

        int telemetry_counter = 0;

        while (!glfwWindowShouldClose(window)) {
            /* Step simulation until real-time */
            mjtNum sim_start = d->time;
            while (d->time - sim_start < 1.0 / 60.0)
                mj_step(m, d);

            /* Print telemetry at ~10 Hz */
            telemetry_counter++;
            if (telemetry_counter % 6 == 0)
                print_telemetry(d->time);

            /* Render */
            int width, height;
            mjrRect viewport = {0, 0, 0, 0};
            glfwGetFramebufferSize(window, &width, &height);
            viewport.width  = width;
            viewport.height = height;

            mjv_updateScene(m, d, &opt, NULL, &cam, mjCAT_ALL, &scn);
            mjr_render(viewport, &scn, &con);

            /* HUD overlay */
            char overlay[512];
            snprintf(overlay, sizeof(overlay),
                     "Target: (%.1f, %.1f, %.1f) yaw=%.0f°\n"
                     "Pos:    (%.2f, %.2f, %.2f)\n"
                     "Time:   %.1f s",
                     target.x, target.y, target.z, target.yaw * 180 / M_PI,
                     d->qpos[0], d->qpos[1], d->qpos[2],
                     d->time);
            mjr_overlay(mjFONT_NORMAL, mjGRID_TOPLEFT, viewport, overlay, NULL, &con);

            glfwSwapBuffers(window);
            glfwPollEvents();
        }

        /* Cleanup visualization */
        mjv_freeScene(&scn);
        mjr_freeContext(&con);
        glfwDestroyWindow(window);
        glfwTerminate();
        printf("\n");
#endif /* NO_GLFW */
    }

    /* Cleanup simulation */
    mj_deleteData(d);
    mj_deleteModel(m);

    return 0;
}
