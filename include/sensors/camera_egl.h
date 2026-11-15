#ifndef MUJOCO_DRONES_CAMERA_EGL_H
#define MUJOCO_DRONES_CAMERA_EGL_H

typedef struct {
    void *display;
    void *context;
    int open;
} camera_egl_t;

int camera_egl_open(camera_egl_t *egl);
void camera_egl_close(camera_egl_t *egl);

#endif
