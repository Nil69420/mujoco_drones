#include "viewer.h"
#include "controller.h"

#include <math.h>
#include <stdio.h>

#ifndef NO_GLFW
#include <GLFW/glfw3.h>

static mjvCamera  cam;    // NOLINT(cppcoreguidelines-avoid-non-const-global-variables)
static mjvOption   opt;   // NOLINT(cppcoreguidelines-avoid-non-const-global-variables)
static mjvScene   scn;    // NOLINT(cppcoreguidelines-avoid-non-const-global-variables)
static mjrContext  con;   // NOLINT(cppcoreguidelines-avoid-non-const-global-variables)
static GLFWwindow *window = NULL;  // NOLINT(cppcoreguidelines-avoid-non-const-global-variables)

static bool button_left   = false;   // NOLINT(cppcoreguidelines-avoid-non-const-global-variables)
static bool button_middle = false;   // NOLINT(cppcoreguidelines-avoid-non-const-global-variables)
static bool button_right  = false;   // NOLINT(cppcoreguidelines-avoid-non-const-global-variables)
static double lastx = 0, lasty = 0;  // NOLINT(cppcoreguidelines-avoid-non-const-global-variables)

static void cb_keyboard(GLFWwindow *win, int key, int scancode, int act, int mods) {
    (void)scancode; (void)mods;
    if (act != GLFW_PRESS) return;

    sim_t *sim = (sim_t *)glfwGetWindowUserPointer(win);

    switch (key) {
    case GLFW_KEY_ESCAPE:
    case GLFW_KEY_Q:
        glfwSetWindowShouldClose(win, 1);
        break;

    case GLFW_KEY_BACKSPACE:
        mj_resetData(sim->model, sim->data);
        ctrl_reset(sim);
        break;

    case GLFW_KEY_UP:   sim->target.z += 0.25; break;
    case GLFW_KEY_DOWN:
        sim->target.z -= 0.25;
        if (sim->target.z < 0.2) sim->target.z = 0.2;
        break;

    case GLFW_KEY_W: sim->target.x += 0.5; break;
    case GLFW_KEY_S: sim->target.x -= 0.5; break;
    case GLFW_KEY_A: sim->target.y += 0.5; break;
    case GLFW_KEY_D: sim->target.y -= 0.5; break;

    case GLFW_KEY_LEFT:  sim->target.yaw += 0.2; break;
    case GLFW_KEY_RIGHT: sim->target.yaw -= 0.2; break;

    case GLFW_KEY_R:
        sim->target.x = 0; sim->target.y = 0;
        sim->target.z = 1.0; sim->target.yaw = 0;
        break;

    default: break;
    }
}

static void cb_mouse_button(GLFWwindow *win, int button, int act, int mods) {
    (void)button; (void)act; (void)mods;
    button_left   = (glfwGetMouseButton(win, GLFW_MOUSE_BUTTON_LEFT)   == GLFW_PRESS);
    button_middle = (glfwGetMouseButton(win, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS);
    button_right  = (glfwGetMouseButton(win, GLFW_MOUSE_BUTTON_RIGHT)  == GLFW_PRESS);
    glfwGetCursorPos(win, &lastx, &lasty);
}

static void cb_mouse_move(GLFWwindow *win, double xpos, double ypos) {
    double dx = xpos - lastx;
    double dy = ypos - lasty;
    lastx = xpos;
    lasty = ypos;

    if (!button_left && !button_middle && !button_right) return;

    int width = 0, height = 0;
    glfwGetWindowSize(win, &width, &height);

    bool shift = (glfwGetKey(win, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS ||
                  glfwGetKey(win, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS);

    mjtMouse action = mjMOUSE_ZOOM;
    if (button_right) {
        action = shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
    } else if (button_left) {
        action = shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
    } else {
        action = mjMOUSE_ZOOM;
    }

    sim_t *sim = (sim_t *)glfwGetWindowUserPointer(win);
    mjv_moveCamera(sim->model, (int)action,
                   dx / (double)width, dy / (double)height, &scn, &cam);
}

static void cb_scroll(GLFWwindow *win, double xoffset, double yoffset) {
    (void)xoffset;
    sim_t *sim = (sim_t *)glfwGetWindowUserPointer(win);
    mjv_moveCamera(sim->model, mjMOUSE_ZOOM,
                   0.0, -0.05 * yoffset, &scn, &cam);
}

int viewer_init(sim_t *sim) {
    if (!glfwInit()) {
        fprintf(stderr, "ERROR: could not initialize GLFW\n");
        return -1;
    }

    window = glfwCreateWindow(1280, 720, "Hummingbird - mujoco_drones", NULL, NULL);
    if (!window) {
        fprintf(stderr, "ERROR: could not create GLFW window\n");
        glfwTerminate();
        return -1;
    }
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);
    glfwSetWindowUserPointer(window, sim);

    mjv_defaultCamera(&cam);
    mjv_defaultOption(&opt);
    mjv_defaultScene(&scn);
    mjr_defaultContext(&con);

    mjv_makeScene(sim->model, &scn, 10000);
    mjr_makeContext(sim->model, &con, mjFONTSCALE_150);

    cam.type      = mjCAMERA_FREE;
    cam.distance  = 4.0;
    cam.azimuth   = 135.0;
    cam.elevation = -25.0;
    cam.lookat[0] = 0.0;
    cam.lookat[1] = 0.0;
    cam.lookat[2] = 1.0;

    glfwSetKeyCallback(window,         cb_keyboard);
    glfwSetCursorPosCallback(window,    cb_mouse_move);
    glfwSetMouseButtonCallback(window,  cb_mouse_button);
    glfwSetScrollCallback(window,       cb_scroll);

    return 0;
}

void viewer_loop(sim_t *sim) {
    int telemetry_counter = 0;

    while (!glfwWindowShouldClose(window)) {
        mjtNum sim_start = sim->data->time;
        while (sim->data->time - sim_start < 1.0 / 60.0) {
            ctrl_update(sim);
            mj_step(sim->model, sim->data);
#ifdef ENABLE_IPC
            if (sim->ipc_enabled) {
                sensor_update(&sim->sensors, sim->model, sim->data,
                              &sim->target);
            }
#endif
        }

#ifdef ENABLE_IPC
        if (sim->ipc_enabled && sim->sensors.cam_due) {
            sensor_render_camera(&sim->sensors, sim->model, sim->data);
        }
#endif

        if (++telemetry_counter % 6 == 0) {
            viewer_print_telemetry(sim);
        }

        int width = 0, height = 0;
        glfwGetFramebufferSize(window, &width, &height);
        mjrRect viewport = {0, 0, width, height};

        mjv_updateScene(sim->model, sim->data, &opt, NULL, &cam, mjCAT_ALL, &scn);
        mjr_render(viewport, &scn, &con);

        char overlay[2048];
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wformat-truncation"
        snprintf(overlay, sizeof(overlay),
                 "Target: (%.1f, %.1f, %.1f) yaw=%.0f deg\n"
                 "Pos:    (%.2f, %.2f, %.2f)\n"
                 "Time:   %.1f s",
                 sim->target.x, sim->target.y, sim->target.z,
                 sim->target.yaw * 180.0 / M_PI,
                 sim->data->qpos[0], sim->data->qpos[1], sim->data->qpos[2],
                 sim->data->time);
#pragma GCC diagnostic pop
        mjr_overlay(mjFONT_NORMAL, mjGRID_TOPLEFT, viewport, overlay, NULL, &con);

        glfwSwapBuffers(window);
        glfwPollEvents();
    }
}

void viewer_close(void) {
    mjv_freeScene(&scn);
    mjr_freeContext(&con);
    if (window) glfwDestroyWindow(window);
    glfwTerminate();
    window = NULL;
}

#endif

void viewer_print_telemetry(const sim_t *sim) {
    double roll = 0.0, pitch = 0.0, yaw = 0.0;
    quat_to_euler(&sim->data->qpos[3], &roll, &pitch, &yaw);

    const ctrl_state_t *cs = &sim->ctrl;

    printf("\rt=%.2f | pos=(%.2f, %.2f, %.2f) | rpy=(%.1f, %.1f, %.1f) | "
           "tgt=(%.1f, %.1f, %.1f) | F=[%.2f %.2f %.2f %.2f]",
           sim->data->time,
           sim->data->qpos[0], sim->data->qpos[1], sim->data->qpos[2],
           roll * 180.0/M_PI, pitch * 180.0/M_PI, yaw * 180.0/M_PI,
           sim->target.x, sim->target.y, sim->target.z,
           sim->data->ctrl[cs->act_thrust[0]], sim->data->ctrl[cs->act_thrust[1]],
           sim->data->ctrl[cs->act_thrust[2]], sim->data->ctrl[cs->act_thrust[3]]);
    fflush(stdout);
}
