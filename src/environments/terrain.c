#include "environments/biome.h"

#include <math.h>

static uint32_t hash2(int32_t x, int32_t y, uint32_t seed) {
    uint32_t h = seed ^ 0x9E3779B9U;
    h ^= (uint32_t)x * 0x85EBCA6BU;
    h = (h << 13) | (h >> 19);
    h *= 0xC2B2AE35U;
    h ^= (uint32_t)y * 0x27D4EB2FU;
    h = (h << 15) | (h >> 17);
    h *= 0x165667B1U;
    h ^= h >> 16;
    h *= 0x85EBCA6BU;
    h ^= h >> 13;
    return h;
}

static double lattice(int32_t x, int32_t y, uint32_t seed) {
    return (double)(hash2(x, y, seed) >> 8) / 8388608.0 - 1.0;
}

static double smooth(double t) {
    return t * t * (3.0 - 2.0 * t);
}

static double value_noise(double x, double y, uint32_t seed) {
    double fx = floor(x);
    double fy = floor(y);
    int32_t ix = (int32_t)fx;
    int32_t iy = (int32_t)fy;
    double tx = smooth(x - fx);
    double ty = smooth(y - fy);
    double a = lattice(ix, iy, seed);
    double b = lattice(ix + 1, iy, seed);
    double c = lattice(ix, iy + 1, seed);
    double d = lattice(ix + 1, iy + 1, seed);
    double top = a + (b - a) * tx;
    double bottom = c + (d - c) * tx;
    return top + (bottom - top) * ty;
}

static double fbm(const biome_terrain_t *t, unsigned seed, double x, double y) {
    double sum = 0.0;
    double amp = 1.0;
    double norm = 0.0;
    double freq = 1.0 / t->wavelength_m;
    for (int o = 0; o < t->octaves; o++) {
        double n = value_noise(x * freq, y * freq, seed + (unsigned)o * 7919U);
        if (t->ridged > 0.0) {
            double r = 1.0 - fabs(n);
            n = (1.0 - t->ridged) * n + t->ridged * (2.0 * r * r - 1.0);
        }
        sum += amp * n;
        norm += amp;
        amp *= 0.5;
        freq *= 2.0;
    }
    return norm > 0.0 ? sum / norm : 0.0;
}

double biome_terrain_height(const biome_terrain_t *t, unsigned seed, double x, double y) {
    double h = t->base_height_m;
    if (t->amplitude_m > 0.0 && t->wavelength_m > 0.0) {
        h += t->amplitude_m * fbm(t, seed, x, y);
    }
    if (t->ramp_m != 0.0) {
        h += t->ramp_m * x / t->half_extent_m;
    }
    if (t->swell_m > 0.0 && t->swell_wavelength_m > 0.0) {
        double k = 2.0 * M_PI / t->swell_wavelength_m;
        h += t->swell_m * (0.6 * sin(k * x + 0.7 * sin(0.3 * k * y)) +
                           0.4 * sin(0.8 * k * y + 1.3 + 0.5 * sin(0.2 * k * x)));
    }
    if (t->pad_radius_m > 0.0) {
        double r = sqrt(x * x + y * y);
        double blend = 0.0;
        if (r <= t->pad_radius_m) {
            blend = 1.0;
        } else if (r < 2.0 * t->pad_radius_m) {
            blend = 1.0 - smooth((r - t->pad_radius_m) / t->pad_radius_m);
        }
        h = h * (1.0 - blend);
    }
    return h;
}
