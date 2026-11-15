#ifndef MUJOCO_DRONES_TEST_SUPPORT_H
#define MUJOCO_DRONES_TEST_SUPPORT_H

#include <math.h>
#include <stdio.h>

static int g_failures = 0;

#define CHECK(cond)                                                              \
    do {                                                                         \
        if (!(cond)) {                                                           \
            fprintf(stderr, "FAIL %s:%d %s\n", __FILE__, __LINE__, #cond);       \
            g_failures++;                                                        \
        }                                                                        \
    } while (0)

#define CHECK_NEAR(actual, expected, tol)                                        \
    do {                                                                         \
        double a_ = (actual);                                                    \
        double e_ = (expected);                                                  \
        if (fabs(a_ - e_) > (tol)) {                                             \
            fprintf(stderr, "FAIL %s:%d %s = %.9g, expected %.9g +- %.3g\n",     \
                    __FILE__, __LINE__, #actual, a_, e_, (double)(tol));         \
            g_failures++;                                                        \
        }                                                                        \
    } while (0)

#define CHECK_REL(actual, expected, rel)                                         \
    CHECK_NEAR((actual), (expected), fabs(expected) * (rel))

#endif
