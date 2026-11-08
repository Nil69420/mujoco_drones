#include "sensors/models/gnss_model.h"
#include "test_support.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

#define DT 0.01

static gnss_env_t open_sky(void) {
    gnss_env_t env = {1.0, 1.0, 0};
    return env;
}

static void test_limits(void) {
    gnss_model_t m;
    const gnss_profile_t *p = gnss_profile_find("neo_m8n");
    CHECK(p != NULL);
    CHECK(gnss_model_init(&m, p, 10.0, DT, 1) == 0);
    CHECK(gnss_model_init(&m, p, 19.0, DT, 1) != 0);
    CHECK(gnss_model_init(&m, p, 0.0, DT, 1) != 0);
    CHECK(gnss_model_init(&m, NULL, 10.0, DT, 1) != 0);
    CHECK(gnss_model_init(&m, p, 10.0, 0.0, 1) != 0);
    CHECK(gnss_profile_find("nope") == NULL);
    CHECK(gnss_profile_find(NULL) == NULL);
}

static void test_cold_start_and_rate(void) {
    gnss_model_t m;
    CHECK(gnss_model_init(&m, gnss_profile_find("neo_m8n"), 10.0, DT, 3) == 0);
    gnss_env_t env = open_sky();
    const double pos[3] = {0.0, 0.0, 0.0};
    const double vel[3] = {0.0, 0.0, 0.0};
    gnss_sample_t s;
    int outputs = 0;
    int early_fix = 0;
    int late_no_fix = 0;
    for (int i = 0; i < 12000; i++) {
        double t = i * DT;
        if (gnss_model_step(&m, t, pos, vel, &env, &s)) {
            outputs++;
            if (s.stamp_s < 25.5 && s.fix != GNSS_FIX_NONE) {
                early_fix++;
            }
            if (s.stamp_s > 27.0 && s.fix != GNSS_FIX_3D) {
                late_no_fix++;
            }
        }
    }
    CHECK(early_fix == 0);
    CHECK(late_no_fix == 0);
    CHECK(outputs >= 1195 && outputs <= 1200);
}

static void test_warm_start(void) {
    gnss_model_t m;
    CHECK(gnss_model_init(&m, gnss_profile_find("zed_f9p"), 10.0, DT, 8) == 0);
    m.warm_start = 1;
    gnss_env_t env = open_sky();
    const double pos[3] = {1.0, 2.0, 3.0};
    const double vel[3] = {0.0, 0.0, 0.0};
    gnss_sample_t s;
    int first_fix_ms = -1;
    for (int i = 0; i < 2000 && first_fix_ms < 0; i++) {
        if (gnss_model_step(&m, i * DT, pos, vel, &env, &s) && s.fix >= GNSS_FIX_3D) {
            first_fix_ms = i;
        }
    }
    CHECK(first_fix_ms >= 0 && first_fix_ms < 200);
}

static void test_accuracy_and_correlation(void) {
    const gnss_profile_t *p = gnss_profile_find("neo_m8n");
    gnss_model_t m;
    CHECK(gnss_model_init(&m, p, 10.0, DT, 4242) == 0);
    gnss_env_t env = open_sky();
    const double pos[3] = {0.0, 0.0, 0.0};
    const double vel[3] = {0.0, 0.0, 0.0};
    gnss_sample_t s;
    const int steps = 6000000;
    size_t keep = 0;
    double *east = malloc(sizeof(double) * 700000);
    CHECK(east != NULL);
    if (east == NULL) {
        return;
    }
    long inside = 0;
    long total = 0;
    double sumsq_e = 0.0;
    double sumsq_u = 0.0;
    for (int i = 0; i < steps; i++) {
        if (gnss_model_step(&m, i * DT, pos, vel, &env, &s) && s.fix == GNSS_FIX_3D) {
            double r = hypot(s.pos_enu[0], s.pos_enu[1]);
            total++;
            if (r <= p->horizontal_cep_m) {
                inside++;
            }
            sumsq_e += s.pos_enu[0] * s.pos_enu[0];
            sumsq_u += s.pos_enu[2] * s.pos_enu[2];
            if (total % 10 == 0 && keep < 700000) {
                east[keep++] = s.pos_enu[0];
            }
        }
    }
    CHECK_NEAR((double)inside / (double)total, 0.5, 0.09);
    double sigma = p->horizontal_cep_m / 1.1774;
    CHECK_REL(sqrt(sumsq_e / (double)total), sigma, 0.14);
    CHECK_REL(sqrt(sumsq_u / (double)total), sigma * p->vertical_to_horizontal, 0.14);

    size_t lag = (size_t)p->corr_time_s;
    double num = 0.0;
    double den = 0.0;
    for (size_t k = 0; k + lag < keep; k++) {
        num += east[k] * east[k + lag];
        den += east[k] * east[k];
    }
    double expected = (1.0 - p->white_fraction) * exp(-1.0);
    CHECK_NEAR(num / den, expected, 0.1);
    free(east);
}

static void test_latency(void) {
    gnss_profile_t p = *gnss_profile_find("ideal");
    p.latency_s = 0.3;
    gnss_model_t m;
    CHECK(gnss_model_init(&m, &p, 10.0, DT, 5) == 0);
    gnss_env_t env = open_sky();
    double pos[3] = {0.0, 0.0, 0.0};
    const double vel[3] = {10.0, 0.0, 0.0};
    gnss_sample_t s;
    int checked = 0;
    for (int i = 0; i < 2000; i++) {
        double t = i * DT;
        pos[0] = 10.0 * t;
        if (gnss_model_step(&m, t, pos, vel, &env, &s)) {
            CHECK_NEAR(t - s.stamp_s, 0.3, DT);
            CHECK_NEAR(s.pos_enu[0], 10.0 * s.stamp_s, 1e-3);
            checked++;
        }
    }
    CHECK(checked > 150);
}

static double run_sigma(gnss_env_t env, const gnss_profile_t *p, int *fix_out) {
    gnss_model_t m;
    CHECK(gnss_model_init(&m, p, 10.0, DT, 9) == 0);
    const double pos[3] = {0.0, 0.0, 0.0};
    const double vel[3] = {0.0, 0.0, 0.0};
    gnss_sample_t s;
    double sumsq = 0.0;
    long n = 0;
    for (int i = 0; i < 4000000; i++) {
        if (gnss_model_step(&m, i * DT, pos, vel, &env, &s) && s.stamp_s > 60.0) {
            *fix_out = s.fix;
            sumsq += s.pos_enu[0] * s.pos_enu[0];
            n++;
        }
    }
    return n > 0 ? sqrt(sumsq / (double)n) : -1.0;
}

static void test_environment_effects(void) {
    const gnss_profile_t *p = gnss_profile_find("neo_m8n");
    int fix = 0;
    gnss_env_t open = open_sky();
    double base = run_sigma(open, p, &fix);
    gnss_env_t canyon = {0.6, 4.0, 0};
    double bad = run_sigma(canyon, p, &fix);
    CHECK_REL(bad / base, 4.0, 0.35);
    gnss_env_t blocked = {0.1, 1.0, 0};
    gnss_model_t m;
    CHECK(gnss_model_init(&m, p, 10.0, DT, 1) == 0);
    const double pos[3] = {0.0, 0.0, 0.0};
    gnss_sample_t s;
    int saw_no_fix = 0;
    for (int i = 0; i < 6000; i++) {
        if (gnss_model_step(&m, 40.0 + i * DT, pos, pos, &blocked, &s)) {
            saw_no_fix += s.fix == GNSS_FIX_NO_FIX;
        }
    }
    CHECK(saw_no_fix > 0);
}

static void test_rtk(void) {
    const gnss_profile_t *p = gnss_profile_find("zed_f9p");
    gnss_model_t m;
    CHECK(gnss_model_init(&m, p, 20.0, DT, 11) == 0);
    gnss_env_t env = {1.0, 1.0, 1};
    const double pos[3] = {0.0, 0.0, 0.0};
    gnss_sample_t s;
    int saw_float = 0;
    int fixed_n = 0;
    double sumsq = 0.0;
    for (int i = 0; i < 12000; i++) {
        double t = i * DT;
        if (gnss_model_step(&m, t, pos, pos, &env, &s)) {
            if (s.fix == GNSS_FIX_RTK_FLOAT) {
                saw_float++;
            }
            if (s.fix == GNSS_FIX_RTK_FIXED && s.stamp_s > 30.0) {
                fixed_n++;
                sumsq += s.pos_enu[0] * s.pos_enu[0];
            }
        }
    }
    CHECK(fixed_n > 1000);
    CHECK_REL(sqrt(sumsq / fixed_n), p->rtk_cep_m / 1.1774, 0.15);
    (void)saw_float;

    gnss_env_t degraded = {0.5, 1.0, 1};
    int lost = 0;
    for (int i = 0; i < 500; i++) {
        if (gnss_model_step(&m, 120.0 + i * DT, pos, pos, &degraded, &s)) {
            lost += s.fix == GNSS_FIX_3D;
        }
    }
    CHECK(lost > 0);
    int float_seen = 0;
    int fixed_early = 0;
    for (int i = 0; i < 1000; i++) {
        double t = 125.0 + i * DT;
        if (gnss_model_step(&m, t, pos, pos, &env, &s)) {
            float_seen += s.fix == GNSS_FIX_RTK_FLOAT;
            fixed_early += (s.fix == GNSS_FIX_RTK_FIXED && s.stamp_s < 125.0 + 14.0);
        }
    }
    CHECK(float_seen > 20);
    CHECK(fixed_early == 0);
}

static void test_determinism(void) {
    const gnss_profile_t *p = gnss_profile_find("neo_m8n");
    gnss_model_t a;
    gnss_model_t b;
    gnss_model_t c;
    CHECK(gnss_model_init(&a, p, 10.0, DT, 1) == 0);
    CHECK(gnss_model_init(&b, p, 10.0, DT, 1) == 0);
    CHECK(gnss_model_init(&c, p, 10.0, DT, 2) == 0);
    gnss_env_t env = open_sky();
    const double z[3] = {0.0, 0.0, 0.0};
    gnss_sample_t sa;
    gnss_sample_t sb;
    gnss_sample_t sc;
    memset(&sa, 0, sizeof(sa));
    memset(&sb, 0, sizeof(sb));
    memset(&sc, 0, sizeof(sc));
    for (int i = 0; i < 4000; i++) {
        double t = 30.0 + i * DT;
        (void)gnss_model_step(&a, t, z, z, &env, &sa);
        (void)gnss_model_step(&b, t, z, z, &env, &sb);
        (void)gnss_model_step(&c, t, z, z, &env, &sc);
    }
    CHECK(memcmp(sa.pos_enu, sb.pos_enu, sizeof(sa.pos_enu)) == 0);
    CHECK(memcmp(sa.pos_enu, sc.pos_enu, sizeof(sa.pos_enu)) != 0);
}

int main(void) {
    test_limits();
    test_cold_start_and_rate();
    test_warm_start();
    test_latency();
    test_determinism();
    test_rtk();
    test_environment_effects();
    test_accuracy_and_correlation();
    if (g_failures != 0) {
        fprintf(stderr, "%d check(s) failed\n", g_failures);
        return 1;
    }
    puts("test_gnss_model OK");
    return 0;
}
