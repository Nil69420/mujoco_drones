#include "sensors/models/gnss_model.h"

#include <math.h>
#include <string.h>

#define CEP_TO_SIGMA 1.1774
#define RTK_FLOAT_SIGMA_M 0.3
#define MIN_VISIBILITY_FIX 0.2
#define RTK_MIN_VISIBILITY 0.7
#define RTK_MAX_MULTIPATH 2.0

int gnss_model_init(gnss_model_t *model, const gnss_profile_t *profile, double rate_hz,
                    double sim_dt, uint64_t seed) {
    if (profile == NULL || rate_hz <= 0.0 || rate_hz > profile->max_rate_hz || sim_dt <= 0.0) {
        return -1;
    }
    memset(model, 0, sizeof(*model));
    model->profile = profile;
    noise_seed(&model->rng, seed);
    model->dt = sim_dt;
    model->period_s = 1.0 / rate_hz;
    model->next_sample_s = 0.0;
    model->decay = exp(-sim_dt / profile->corr_time_s);
    double sigma = profile->horizontal_cep_m / CEP_TO_SIGMA;
    double gm_sigma = sigma * sqrt(1.0 - profile->white_fraction);
    for (int i = 0; i < 3; i++) {
        double scale = i == 2 ? profile->vertical_to_horizontal : 1.0;
        model->gm[i] = noise_gaussian(&model->rng, gm_sigma * scale);
    }
    return 0;
}

static void queue_push(gnss_model_t *m, const gnss_sample_t *s) {
    if (m->count == GNSS_QUEUE) {
        m->head = (m->head + 1) % GNSS_QUEUE;
        m->count--;
    }
    m->queue[(m->head + m->count) % GNSS_QUEUE] = *s;
    m->count++;
}

static void make_sample(gnss_model_t *m, double t_s, const double pos[3], const double vel[3],
                        const gnss_env_t *env) {
    const gnss_profile_t *p = m->profile;
    gnss_sample_t s;
    memset(&s, 0, sizeof(s));
    s.stamp_s = t_s;
    double vis = env->sky_visibility < 0.0 ? 0.0 : (env->sky_visibility > 1.0 ? 1.0 : env->sky_visibility);
    double mult = env->multipath_scale < 1.0 ? 1.0 : env->multipath_scale;
    s.satellites = (int)lround(6.0 + 20.0 * vis);
    s.hdop = 0.6 + 2.4 * (1.0 - vis) + 0.2 * (mult - 1.0);
    if (t_s < p->ttff_cold_s || vis < MIN_VISIBILITY_FIX) {
        s.fix = t_s < p->ttff_cold_s ? GNSS_FIX_NONE : GNSS_FIX_NO_FIX;
        s.satellites = s.fix == GNSS_FIX_NONE ? 0 : s.satellites / 3;
        queue_push(m, &s);
        return;
    }
    double sigma = p->horizontal_cep_m / CEP_TO_SIGMA;
    s.fix = GNSS_FIX_3D;
    int use_gm = 1;
    if (p->rtk_capable && env->rtk_corrections && vis >= RTK_MIN_VISIBILITY && mult <= RTK_MAX_MULTIPATH) {
        if (m->rtk_clock_s >= p->rtk_converge_s) {
            s.fix = GNSS_FIX_RTK_FIXED;
            sigma = p->rtk_cep_m / CEP_TO_SIGMA;
            use_gm = 0;
        } else {
            s.fix = GNSS_FIX_RTK_FLOAT;
            sigma = RTK_FLOAT_SIGMA_M;
            use_gm = 0;
        }
    }
    double white_sigma = use_gm ? sigma * sqrt(p->white_fraction) : sigma;
    for (int i = 0; i < 3; i++) {
        double axis_scale = (i == 2 ? p->vertical_to_horizontal : 1.0) * mult;
        double err = noise_gaussian(&m->rng, white_sigma * axis_scale);
        if (use_gm) {
            err += m->gm[i] * mult;
        }
        s.pos_enu[i] = pos[i] + err;
        s.vel_enu[i] = vel[i] + noise_gaussian(&m->rng, p->velocity_sigma_ms * mult);
    }
    queue_push(m, &s);
}

int gnss_model_step(gnss_model_t *model, double t_s, const double truth_pos_enu[3],
                    const double truth_vel_enu[3], const gnss_env_t *env, gnss_sample_t *out) {
    const gnss_profile_t *p = model->profile;
    double sigma = p->horizontal_cep_m / CEP_TO_SIGMA;
    double gm_sigma = sigma * sqrt(1.0 - p->white_fraction);
    double drive = sqrt(1.0 - model->decay * model->decay);
    for (int i = 0; i < 3; i++) {
        double scale = i == 2 ? p->vertical_to_horizontal : 1.0;
        model->gm[i] = model->decay * model->gm[i] +
                       noise_gaussian(&model->rng, gm_sigma * scale * drive);
    }
    int rtk_ok = p->rtk_capable && env->rtk_corrections &&
                 env->sky_visibility >= RTK_MIN_VISIBILITY &&
                 env->multipath_scale <= RTK_MAX_MULTIPATH;
    model->rtk_clock_s = rtk_ok ? model->rtk_clock_s + model->dt : 0.0;
    if (t_s + 0.5 * model->dt >= model->next_sample_s) {
        make_sample(model, t_s, truth_pos_enu, truth_vel_enu, env);
        model->next_sample_s += model->period_s;
    }
    if (model->count > 0 && t_s - model->queue[model->head].stamp_s + 1e-9 >= p->latency_s) {
        *out = model->queue[model->head];
        model->head = (model->head + 1) % GNSS_QUEUE;
        model->count--;
        return 1;
    }
    return 0;
}

const gnss_profile_t *gnss_profile_find(const char *name) {
    if (name == NULL) {
        return NULL;
    }
    for (size_t i = 0; i < GNSS_PROFILE_COUNT; i++) {
        if (strcmp(GNSS_PROFILES[i].name, name) == 0) {
            return &GNSS_PROFILES[i];
        }
    }
    return NULL;
}
