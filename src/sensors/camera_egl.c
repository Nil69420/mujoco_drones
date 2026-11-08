#include "sensors/camera_egl.h"

#include <stdio.h>
#include <string.h>

#ifdef HAVE_EGL

#include <EGL/egl.h>
#include <EGL/eglext.h>

enum { MAX_DEVICES = 16 };

static EGLDisplay open_device_display(void) {
    PFNEGLQUERYDEVICESEXTPROC query =
        (PFNEGLQUERYDEVICESEXTPROC)eglGetProcAddress("eglQueryDevicesEXT");
    PFNEGLGETPLATFORMDISPLAYEXTPROC platform =
        (PFNEGLGETPLATFORMDISPLAYEXTPROC)eglGetProcAddress("eglGetPlatformDisplayEXT");
    if (query == NULL || platform == NULL) {
        return EGL_NO_DISPLAY;
    }
    EGLDeviceEXT devices[MAX_DEVICES];
    EGLint count = 0;
    if (!query(MAX_DEVICES, devices, &count)) {
        return EGL_NO_DISPLAY;
    }
    for (EGLint i = 0; i < count; i++) {
        EGLDisplay display = platform(EGL_PLATFORM_DEVICE_EXT, devices[i], NULL);
        EGLint major = 0;
        EGLint minor = 0;
        if (display != EGL_NO_DISPLAY && eglInitialize(display, &major, &minor)) {
            return display;
        }
    }
    return EGL_NO_DISPLAY;
}

int camera_egl_open(camera_egl_t *egl) {
    memset(egl, 0, sizeof(*egl));
    EGLDisplay display = open_device_display();
    if (display == EGL_NO_DISPLAY) {
        fprintf(stderr, "[camera] no usable EGL device for offscreen rendering\n");
        return -1;
    }
    const EGLint attrs[] = {EGL_SURFACE_TYPE,   EGL_PBUFFER_BIT, EGL_RED_SIZE,   8,
                            EGL_GREEN_SIZE,     8,               EGL_BLUE_SIZE,  8,
                            EGL_DEPTH_SIZE,     24,              EGL_RENDERABLE_TYPE,
                            EGL_OPENGL_BIT,     EGL_NONE};
    EGLConfig config;
    EGLint found = 0;
    if (!eglChooseConfig(display, attrs, &config, 1, &found) || found < 1 ||
        !eglBindAPI(EGL_OPENGL_API)) {
        fprintf(stderr, "[camera] EGL has no desktop OpenGL configuration\n");
        eglTerminate(display);
        return -1;
    }
    EGLContext context = eglCreateContext(display, config, EGL_NO_CONTEXT, NULL);
    if (context == EGL_NO_CONTEXT ||
        !eglMakeCurrent(display, EGL_NO_SURFACE, EGL_NO_SURFACE, context)) {
        fprintf(stderr, "[camera] could not create a current EGL context\n");
        eglTerminate(display);
        return -1;
    }
    egl->display = display;
    egl->context = context;
    egl->open = 1;
    return 0;
}

void camera_egl_close(camera_egl_t *egl) {
    if (!egl->open) {
        return;
    }
    eglMakeCurrent(egl->display, EGL_NO_SURFACE, EGL_NO_SURFACE, EGL_NO_CONTEXT);
    eglDestroyContext(egl->display, egl->context);
    eglTerminate(egl->display);
    memset(egl, 0, sizeof(*egl));
}

#else

int camera_egl_open(camera_egl_t *egl) {
    memset(egl, 0, sizeof(*egl));
    fprintf(stderr, "[camera] built without EGL: headless camera unavailable\n");
    return -1;
}

void camera_egl_close(camera_egl_t *egl) {
    memset(egl, 0, sizeof(*egl));
}

#endif
