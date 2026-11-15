#ifndef MUJOCO_DRONES_GNSS_MODEL_H
#define MUJOCO_DRONES_GNSS_MODEL_H

#include <stddef.h>
#include <stdint.h>

#include "sensors/models/provenance.h"
#include "sensors/noise.h"

enum {
    GNSS_QUEUE = 64,
};
typedef enum {
    GNSS_FIX_NONE = 0,
    GNSS_FIX_NO_FIX = 1,
    GNSS_FIX_3D = 3,
    GNSS_FIX_RTK_FLOAT = 5,
    GNSS_FIX_RTK_FIXED = 6
} gnss_fix_t;

typedef struct {
    const char *name;
    const char *grade;
    const char *source;
    provenance_t accuracy_provenance;
    provenance_t other_provenance;
    double horizontal_cep_m;
    double vertical_to_horizontal;
    double velocity_sigma_ms;
    double max_rate_hz;
    double corr_time_s;
    double white_fraction;
    double latency_s;
    double ttff_cold_s;
    int rtk_capable;
    double rtk_cep_m;
    double rtk_converge_s;
} gnss_profile_t;

typedef struct {
    double sky_visibility;
    double multipath_scale;
    int rtk_corrections;
} gnss_env_t;

typedef struct {
    double stamp_s;
    int fix;
    int satellites;
    double hdop;
    double pos_enu[3];
    double vel_enu[3];
} gnss_sample_t;

typedef struct {
    const gnss_profile_t *profile;
    noise_rng_t rng;
    double dt;
    double period_s;
    double next_sample_s;
    double decay;
    double gm[3];
    double rtk_clock_s;
    gnss_sample_t queue[GNSS_QUEUE];
    size_t head;
    size_t count;
    int warm_start;
} gnss_model_t;

int gnss_model_init(gnss_model_t *model, const gnss_profile_t *profile, double rate_hz,
                    double sim_dt, uint64_t seed);
int gnss_model_step(gnss_model_t *model, double t_s, const double truth_pos_enu[3],
                    const double truth_vel_enu[3], const gnss_env_t *env, gnss_sample_t *out);

extern const gnss_profile_t GNSS_PROFILES[];
extern const size_t GNSS_PROFILE_COUNT;
const gnss_profile_t *gnss_profile_find(const char *name);

#endif
