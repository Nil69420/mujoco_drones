#ifndef MUJOCO_DRONES_NOISE_H
#define MUJOCO_DRONES_NOISE_H

#include <math.h>
#include <stdint.h>

typedef struct {
    uint64_t state;
} noise_rng_t;

static inline void noise_seed(noise_rng_t *rng, uint64_t seed) {
    rng->state = seed;
}

static inline double noise_uniform(noise_rng_t *rng) {
    uint64_t z = (rng->state += 0x9e3779b97f4a7c15ULL);
    z = (z ^ (z >> 30)) * 0xbf58476d1ce4e5b9ULL;
    z = (z ^ (z >> 27)) * 0x94d049bb133111ebULL;
    z ^= z >> 31;
    return (double)(z >> 11) * 0x1.0p-53;
}

static inline double noise_gaussian(noise_rng_t *rng, double stddev) {
    double u1 = noise_uniform(rng);
    double u2 = noise_uniform(rng);
    if (u1 < 1e-15) u1 = 1e-15;
    return stddev * sqrt(-2.0 * log(u1)) * cos(2.0 * M_PI * u2);
}

static inline double noise_random_walk(noise_rng_t *rng, double *state,
                                       double stddev, double dt) {
    *state += noise_gaussian(rng, stddev * sqrt(dt));
    return *state;
}

#endif
