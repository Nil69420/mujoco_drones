#ifndef MUJOCO_DRONES_CAMERA_MODEL_H
#define MUJOCO_DRONES_CAMERA_MODEL_H

#include <stddef.h>
#include <stdint.h>

#include "sensors/models/provenance.h"
#include "sensors/noise.h"

typedef struct {
    const char *name;
    const char *grade;
    const char *source;
    provenance_t noise_provenance;
    provenance_t other_provenance;
    int max_width;
    int max_height;
    double max_fps;
    int global_shutter;
    double full_well_e;
    double read_noise_e;
    double dark_e_per_s;
    double dark_ref_temp_c;
    double dark_doubling_k;
    double prnu_sigma;
    double fpn_e;
    double vignette_k;
    double hot_pixel_fraction;
    double defocus_px_per_k;
} camera_profile_t;

enum { CAMERA_NOISE_TABLE = 65536 };

typedef struct {
    const camera_profile_t *profile;
    int width;
    int height;
    uint64_t rng_state;
    float *gain;
    float *fpn;
    uint32_t *hot_index;
    float *hot_scale;
    size_t hot_count;
    float *noise_table;
    uint8_t *row_scratch;
    float *row_float;
} camera_model_t;

int camera_model_init(camera_model_t *model, const camera_profile_t *profile, int width,
                      int height, uint64_t seed);
void camera_model_free(camera_model_t *model);

void camera_model_apply(camera_model_t *model, const uint8_t *rgb_bottom_up, uint8_t *mono_top_down,
                        double sensor_temp_c, double lens_temp_c, double exposure_s);

double camera_dark_electrons(const camera_profile_t *profile, double temp_c, double exposure_s);

extern const camera_profile_t CAMERA_PROFILES[];
extern const size_t CAMERA_PROFILE_COUNT;
const camera_profile_t *camera_profile_find(const char *name);

#endif
