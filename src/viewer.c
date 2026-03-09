/*
 * viewer.c - GLFW/MuJoCo interactive viewer + telemetry
 */

#include "viewer.h"
#include "controller.h"

#include <math.h>
#include <stdio.h>

#ifndef NO_GLFW
#include <GLFW/glfw3.h>

/* ================================================================
 * Module state
 * ================================================================ */
static mjvCamera  cam;
static mjvOption   opt;
static mjvScene   scn;
static mjrContext  con;
static GLFWwindow *window = NULL;

static bool button_left   = false;
static bool button_middle = false;
static bool button_right  = false;
static double lastx = 0, lasty = 0;

/* Pointer back to the sim for use in callbacks */
static sim_t *g_viewer_sim = NULL;

/* ================================================================
 * GLFW callbacks
 * ================================================================ */
static void cb_keyboard(GLFWwindow *win, int key, int scancode, int act, int mods) {
    (void)scancode; (void)mods;
    if (act != GLFW_PRESS) return;

    sim_t *sim = g_viewer_sim;

    switch (key) {
    case GLFW_KEY_ESCAPE:
    case GLFW_KEY_Q:
        glfwSetWindowShouldClose(win, 1);
        break;

    case GLFW_KEY_BACKSPACE:
        mj_resetData(sim->model, sim->data);
        ctrl_reset(sim);
        break;

    /* Target altitude */
    case GLFW_KEY_UP:   sim->target.z += 0.25; break;
    case GLFW_KEY_DOWN:
        sim->target.z -= 0.25;
        if (sim->target.z < 0.2) sim->target.z = 0.2;
        break;

    /* Target XY */
    case GLFW_KEY_W: sim->target.x += 0.5; break;
    case GLFW_KEY_S: sim->target.x -= 0.5; break;
    case GLFW_KEY_A: sim->target.y += 0.5; break;
    case GLFW_KEY_D: sim->target.y -= 0.5; break;

    /* Target yaw */
    case GLFW_KEY_LEFT:  sim->target.yaw += 0.2; break;
    case GLFW_KEY_RIGHT: sim->target.yaw -= 0.2; break;

    /* Reset target */
    case GLFW_KEY_R:
        sim->target.x = 0; sim->target.y = 0;
        sim->target.z = 1.0; sim->target.yaw = 0;
        break;
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

    int width, height;
    glfwGetWindowSize(win, &width, &height);

    bool shift = (glfwGetKey(win, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS ||
                  glfwGetKey(win, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS);

    mjtMouse action;
    if (button_right)
        action = shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
    else if (button_left)
        action = shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
    else
        action = mjMOUSE_ZOOM;

    mjv_moveCamera(g_viewer_sim->model, action,
                   dx / (double)width, dy / (double)height, &scn, &cam);
}

static void cb_scroll(GLFWwindow *win, double xoffset, double yoffset) {
    (void)win; (void)xoffset;
    mjv_moveCamera(g_viewer_sim->model, mjMOUSE_ZOOM,
                   0.0, -0.05 * yoffset, &scn, &cam);
}

/* ================================================================
 * Public API
 * ================================================================ */
int viewer_init(sim_t *sim) {
    g_viewer_sim = sim;

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
        /* Step simulation to maintain real-time pace */
        mjtNum sim_start = sim->data->time;
        while (sim->data->time - sim_start < 1.0 / 60.0)
            mj_step(sim->model, sim->data);

        /* Telemetry at ~10 Hz */
        if (++telemetry_counter % 6 == 0)
            viewer_print_telemetry(sim);

        /* Render */
        int width, height;
        glfwGetFramebufferSize(window, &width, &height);
        mjrRect viewport = {0, 0, width, height};

        mjv_updateScene(sim->model, sim->data, &opt, NULL, &cam, mjCAT_ALL, &scn);
        mjr_render(viewport, &scn, &con);

        /* HUD overlay */
        char overlay[512];
        snprintf(overlay, sizeof(overlay),
                 "Target: (%.1f, %.1f, %.1f) yaw=%.0f deg\n"
                 "Pos:    (%.2f, %.2f, %.2f)\n"
                 "Time:   %.1f s",
                 sim->target.x, sim->target.y, sim->target.z,
                 sim->target.yaw * 180.0 / M_PI,
                 sim->data->qpos[0], sim->data->qpos[1], sim->data->qpos[2],
                 sim->data->time);
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

#endif /* NO_GLFW */

/* ================================================================
 * Telemetry (always available)
 * ================================================================ */
void viewer_print_telemetry(const sim_t *sim) {
    double roll, pitch, yaw;
    quat_to_euler(&sim->data->qpos[3], &roll, &pitch, &yaw);

    const ctrl_state_t *c = &sim->ctrl;

    printf("\rt=%.2f | pos=(%.2f, %.2f, %.2f) | rpy=(%.1f, %.1f, %.1f) | "
           "tgt=(%.1f, %.1f, %.1f) | F=[%.2f %.2f %.2f %.2f]",
           sim->data->time,
           sim->data->qpos[0], sim->data->qpos[1], sim->data->qpos[2],
           roll * 180.0/M_PI, pitch * 180.0/M_PI, yaw * 180.0/M_PI,
           sim->target.x, sim->target.y, sim->target.z,
           sim->data->ctrl[c->act_thrust[0]], sim->data->ctrl[c->act_thrust[1]],
           sim->data->ctrl[c->act_thrust[2]], sim->data->ctrl[c->act_thrust[3]]);
    fflush(stdout);
}
