#ifndef MUJOCO_DRONES_NOISE_H
#define MUJOCO_DRONES_NOISE_H

#include <math.h>
#include <stdlib.h>
#include <stdint.h>

static inline void noise_seed(uint64_t seed) {
    srand48((long)seed);
}

static inline double noise_gaussian(double stddev) {
    double u1 = drand48();
    double u2 = drand48();
    if (u1 < 1e-15) u1 = 1e-15;
    return stddev * sqrt(-2.0 * log(u1)) * cos(2.0 * M_PI * u2);
}

static inline double noise_random_walk(double *state, double stddev, double dt) {
    *state += noise_gaussian(stddev * sqrt(dt));
    return *state;
}

#endif
