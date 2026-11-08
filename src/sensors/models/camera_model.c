#include "sensors/models/camera_model.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

enum { LUM_R = 77, LUM_G = 150, LUM_B = 29 };

#define WHITE_FRACTION_OF_WELL 0.9
#define LENS_REFERENCE_TEMP_C 20.0
#define HOT_PIXEL_MIN_SCALE 20.0
#define HOT_PIXEL_SPREAD 400.0
#define DEFOCUS_MAX_WEIGHT 0.33f

static uint32_t next_u32(uint64_t *state) {
    *state = *state * 6364136223846793005ULL + 1442695040888963407ULL;
    return (uint32_t)(*state >> 32);
}

double camera_dark_electrons(const camera_profile_t *profile, double temp_c, double exposure_s) {
    double doublings = (temp_c - profile->dark_ref_temp_c) / profile->dark_doubling_k;
    return profile->dark_e_per_s * exposure_s * pow(2.0, doublings);
}

int camera_model_init(camera_model_t *model, const camera_profile_t *profile, int width,
                      int height, uint64_t seed) {
    if (profile == NULL || width <= 0 || height <= 0 || width > profile->max_width ||
        height > profile->max_height) {
        return -1;
    }
    memset(model, 0, sizeof(*model));
    model->profile = profile;
    model->width = width;
    model->height = height;
    model->rng_state = seed ^ 0x9E3779B97F4A7C15ULL;
    size_t pixels = (size_t)width * (size_t)height;
    model->gain = malloc(pixels * sizeof(float));
    model->fpn = malloc(pixels * sizeof(float));
    model->noise_table = malloc(2U * CAMERA_NOISE_TABLE * sizeof(float));
    model->row_scratch = malloc((size_t)width);
    model->row_float = malloc((size_t)width * sizeof(float));
    size_t hot = (size_t)floor(profile->hot_pixel_fraction * (double)pixels);
    model->hot_index = hot > 0 ? malloc(hot * sizeof(uint32_t)) : NULL;
    model->hot_scale = hot > 0 ? malloc(hot * sizeof(float)) : NULL;
    if (model->gain == NULL || model->fpn == NULL || model->noise_table == NULL ||
        model->row_scratch == NULL || model->row_float == NULL || (hot > 0 && (model->hot_index == NULL || model->hot_scale == NULL))) {
        camera_model_free(model);
        return -1;
    }
    noise_rng_t rng;
    noise_seed(&rng, seed);
    for (size_t i = 0; i < CAMERA_NOISE_TABLE; i++) {
        model->noise_table[i] = (float)noise_gaussian(&rng, 1.0);
    }
    memcpy(model->noise_table + CAMERA_NOISE_TABLE, model->noise_table,
           CAMERA_NOISE_TABLE * sizeof(float));
    double cx = 0.5 * (double)(width - 1);
    double cy = 0.5 * (double)(height - 1);
    double r2max = cx * cx + cy * cy;
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            size_t i = (size_t)y * (size_t)width + (size_t)x;
            double dx = (double)x - cx;
            double dy = (double)y - cy;
            double r2 = (dx * dx + dy * dy) / r2max;
            double vig = 1.0 / ((1.0 + profile->vignette_k * r2) * (1.0 + profile->vignette_k * r2));
            double prnu = 1.0 + noise_gaussian(&rng, profile->prnu_sigma);
            model->gain[i] = (float)(vig * prnu);
            model->fpn[i] = (float)noise_gaussian(&rng, profile->fpn_e);
        }
    }
    model->hot_count = hot;
    for (size_t k = 0; k < hot; k++) {
        model->hot_index[k] = next_u32(&model->rng_state) % (uint32_t)pixels;
        double u = (double)(next_u32(&model->rng_state) >> 8) / 16777216.0;
        model->hot_scale[k] = (float)(HOT_PIXEL_MIN_SCALE + HOT_PIXEL_SPREAD * u * u * u);
    }
    return 0;
}

void camera_model_free(camera_model_t *model) {
    free(model->gain);
    free(model->fpn);
    free(model->noise_table);
    free(model->row_scratch);
    free(model->row_float);
    free(model->hot_index);
    free(model->hot_scale);
    memset(model, 0, sizeof(*model));
}

static void defocus_rows(camera_model_t *model, uint8_t *img, float weight) {
    size_t w = (size_t)model->width;
    float centre = 1.0F - 2.0F * weight;
    for (int y = 0; y < model->height; y++) {
        uint8_t *row = img + (size_t)y * w;
        memcpy(model->row_scratch, row, w);
        const uint8_t *src = model->row_scratch;
        row[0] = (uint8_t)(centre * (float)src[0] + 2.0F * weight * (float)src[w > 1 ? 1 : 0] + 0.5F);
        for (size_t x = 1; x + 1 < w; x++) {
            row[x] = (uint8_t)(centre * (float)src[x] + weight * ((float)src[x - 1] + (float)src[x + 1]) + 0.5F);
        }
        if (w > 1) {
            row[w - 1] = (uint8_t)(centre * (float)src[w - 1] + 2.0F * weight * (float)src[w - 2] + 0.5F);
        }
    }
}

void camera_model_apply(camera_model_t *model, const uint8_t *rgb_bottom_up, uint8_t *mono_top_down,
                        double sensor_temp_c, double lens_temp_c, double exposure_s) {
    const camera_profile_t *p = model->profile;
    size_t w = (size_t)model->width;
    size_t h = (size_t)model->height;
    float well = (float)p->full_well_e;
    float scale_e = (float)WHITE_FRACTION_OF_WELL * well / 255.0F;
    float dark = (float)camera_dark_electrons(p, sensor_temp_c, exposure_s);
    float read2 = (float)(p->read_noise_e * p->read_noise_e);
    float to_dn = 255.0F / well;
    float dark_plus_read2 = dark + read2;
    for (size_t y = 0; y < h; y++) {
        const float *noise = model->noise_table + (next_u32(&model->rng_state) >> 16);
        const uint8_t *src = rgb_bottom_up + (h - 1U - y) * w * 3U;
        uint8_t *dst = mono_top_down + y * w;
        const float *g = model->gain + (h - 1U - y) * w;
        const float *f = model->fpn + (h - 1U - y) * w;
        float *lumrow = model->row_float;
        for (size_t x = 0; x < w; x++) {
            lumrow[x] = (float)((LUM_R * (unsigned)src[3U * x] + LUM_G * (unsigned)src[3U * x + 1U] +
                                 LUM_B * (unsigned)src[3U * x + 2U]) >>
                                8);
        }
        for (size_t x = 0; x < w; x++) {
            float signal = lumrow[x] * scale_e * g[x];
            float var = signal + dark_plus_read2;
            float e = signal + dark + f[x] + sqrtf(var) * noise[x];
            lumrow[x] = fminf(fmaxf(e * to_dn, 0.0F), 255.0F) + 0.5F;
        }
        for (size_t x = 0; x < w; x++) {
            dst[x] = (uint8_t)lumrow[x];
        }
    }
    for (size_t k = 0; k < model->hot_count; k++) {
        size_t i = model->hot_index[k];
        size_t row_flipped = i / w;
        size_t col = i % w;
        size_t y = h - 1U - row_flipped;
        float extra = dark * model->hot_scale[k] * to_dn;
        float v = (float)mono_top_down[y * w + col] + extra;
        mono_top_down[y * w + col] = (uint8_t)(v > 255.0F ? 255.0F : v);
    }
    double dt_lens = lens_temp_c - LENS_REFERENCE_TEMP_C;
    float weight = (float)(0.25 * p->defocus_px_per_k * fabs(dt_lens));
    if (weight > 0.02F) {
        defocus_rows(model, mono_top_down, weight > DEFOCUS_MAX_WEIGHT ? DEFOCUS_MAX_WEIGHT : weight);
    }
}

const camera_profile_t *camera_profile_find(const char *name) {
    if (name == NULL) {
        return NULL;
    }
    for (size_t i = 0; i < CAMERA_PROFILE_COUNT; i++) {
        if (strcmp(CAMERA_PROFILES[i].name, name) == 0) {
            return &CAMERA_PROFILES[i];
        }
    }
    return NULL;
}
