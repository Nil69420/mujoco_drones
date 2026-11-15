#include "conditions/conditions.h"
#include "sensors/models/baro_model.h"
#include "sensors/models/gnss_model.h"
#include "sensors/models/imu_model.h"
#include "sensors/models/mag_model.h"
#include "test_support.h"
#include "vehicle/estimator_ops.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

#define DT 0.001
#define R_EARTH 6371000.0
#define LAT0 47.3667
#define LON0 8.55
#define ALT0 500.0

typedef struct {
    double p[3];
    double v[3];
    double a[3];
    double R[3][3];
} truth_t;

static void truth_at(double t, truth_t *o) {
    double w = 2.0 * M_PI / 45.0;
    double radius = 25.0;
    o->p[0] = radius * cos(w * t);
    o->p[1] = radius * sin(w * t);
    double vz_amp = 4.0;
    o->p[2] = 30.0 + vz_amp * sin(0.2 * t);
    o->v[0] = -radius * w * sin(w * t);
    o->v[1] = radius * w * cos(w * t);
    o->v[2] = 0.2 * vz_amp * cos(0.2 * t);
    o->a[0] = -radius * w * w * cos(w * t);
    o->a[1] = -radius * w * w * sin(w * t);
    o->a[2] = -0.04 * vz_amp * sin(0.2 * t);
    double zb[3] = {o->a[0], o->a[1], o->a[2] + 9.80665};
    double n = sqrt(zb[0] * zb[0] + zb[1] * zb[1] + zb[2] * zb[2]);
    for (int i = 0; i < 3; i++) {
        zb[i] /= n;
    }
    double yaw = 0.6 + 0.05 * t;
    double xc[3] = {cos(yaw), sin(yaw), 0.0};
    double yb[3] = {zb[1] * xc[2] - zb[2] * xc[1], zb[2] * xc[0] - zb[0] * xc[2],
                    zb[0] * xc[1] - zb[1] * xc[0]};
    double yn = sqrt(yb[0] * yb[0] + yb[1] * yb[1] + yb[2] * yb[2]);
    for (int i = 0; i < 3; i++) {
        yb[i] /= yn;
    }
    double xb[3] = {yb[1] * zb[2] - yb[2] * zb[1], yb[2] * zb[0] - yb[0] * zb[2],
                    yb[0] * zb[1] - yb[1] * zb[0]};
    for (int i = 0; i < 3; i++) {
        o->R[i][0] = xb[i];
        o->R[i][1] = yb[i];
        o->R[i][2] = zb[i];
    }
}

static void to_quat(double R[3][3], double q[4]) {
    double tr = R[0][0] + R[1][1] + R[2][2];
    if (tr > 0.0) {
        double s = sqrt(tr + 1.0) * 2.0;
        q[0] = 0.25 * s;
        q[1] = (R[2][1] - R[1][2]) / s;
        q[2] = (R[0][2] - R[2][0]) / s;
        q[3] = (R[1][0] - R[0][1]) / s;
    } else if (R[0][0] > R[1][1] && R[0][0] > R[2][2]) {
        double s = sqrt(1.0 + R[0][0] - R[1][1] - R[2][2]) * 2.0;
        q[0] = (R[2][1] - R[1][2]) / s;
        q[1] = 0.25 * s;
        q[2] = (R[0][1] + R[1][0]) / s;
        q[3] = (R[0][2] + R[2][0]) / s;
    } else if (R[1][1] > R[2][2]) {
        double s = sqrt(1.0 + R[1][1] - R[0][0] - R[2][2]) * 2.0;
        q[0] = (R[0][2] - R[2][0]) / s;
        q[1] = (R[0][1] + R[1][0]) / s;
        q[2] = 0.25 * s;
        q[3] = (R[1][2] + R[2][1]) / s;
    } else {
        double s = sqrt(1.0 + R[2][2] - R[0][0] - R[1][1]) * 2.0;
        q[0] = (R[1][0] - R[0][1]) / s;
        q[1] = (R[0][2] + R[2][0]) / s;
        q[2] = (R[1][2] + R[2][1]) / s;
        q[3] = 0.25 * s;
    }
}

static double z_axis_angle(const double qa[4], const double qb[4]) {
    double za[3] = {2.0 * (qa[1] * qa[3] + qa[0] * qa[2]), 2.0 * (qa[2] * qa[3] - qa[0] * qa[1]),
                    1.0 - 2.0 * (qa[1] * qa[1] + qa[2] * qa[2])};
    double zb[3] = {2.0 * (qb[1] * qb[3] + qb[0] * qb[2]), 2.0 * (qb[2] * qb[3] - qb[0] * qb[1]),
                    1.0 - 2.0 * (qb[1] * qb[1] + qb[2] * qb[2])};
    double dot = za[0] * zb[0] + za[1] * zb[1] + za[2] * zb[2];
    dot = dot > 1.0 ? 1.0 : (dot < -1.0 ? -1.0 : dot);
    return acos(dot);
}

static double yaw_of(const double q[4]) {
    return atan2(2.0 * (q[0] * q[3] + q[1] * q[2]), 1.0 - 2.0 * (q[2] * q[2] + q[3] * q[3]));
}

static double wrap(double a) {
    while (a > M_PI) {
        a -= 2.0 * M_PI;
    }
    while (a < -M_PI) {
        a += 2.0 * M_PI;
    }
    return a;
}

typedef struct {
    double pos_rms;
    double raw_rms;
    double vel_rms;
    double yaw_err_deg;
    double tilt_err_deg;
    double outage_end_error;
    double early_z_max;
    int valid_at_end;
} result_t;

static result_t run(const char *imu_name, const char *gnss_name, const char *baro_name, double seconds,
                    double outage_start, double outage_len, int use_mag, int white_gnss) {
    result_t res;
    memset(&res, 0, sizeof(res));
    imu_model_t imu;
    gnss_model_t gnss;
    baro_model_t baro;
    mag_model_t mag;
    CHECK(imu_model_init(&imu, imu_profile_find(imu_name), 1000.0, 11) == 0);
    gnss_profile_t gnss_profile = *gnss_profile_find(gnss_name);
    if (white_gnss) {
        gnss_profile.white_fraction = 1.0;
    }
    if (white_gnss == 2) {
        gnss_profile.latency_s = 0.4;
        gnss_profile.horizontal_cep_m = 0.3;
    }
    CHECK(gnss_model_init(&gnss, &gnss_profile, 10.0, DT, 12) == 0);
    gnss.warm_start = 1;
    CHECK(baro_model_init(&baro, baro_profile_find(baro_name), 50.0, 13) == 0);
    CHECK(mag_model_init(&mag, mag_profile_find("rm3100"), 100.0, 14) == 0);

    const estimator_ops_t *ops = estimator_registry_find("kf_ins");
    CHECK(ops != NULL);
    estimator_t est;
    memset(&est, 0, sizeof(est));
    CHECK(ops->init(&est) == 0);

    double field_ut[3] = {0.9, 21.4, -42.3};
    int steps = (int)(seconds / DT);
    double axis_mean[3] = {0.0, 0.0, 0.0};
    double axis_sq[3] = {0.0, 0.0, 0.0};
    double sum_raw = 0.0;
    int n_raw = 0;
    double sum_p = 0.0;
    double sum_v = 0.0;
    double sum_yaw = 0.0;
    double sum_tilt = 0.0;
    int n_eval = 0;
    truth_t prev;
    truth_at(0.0, &prev);
    double mag_hold[3] = {0.0, 0.0, 0.0};
    for (int k = 1; k <= steps; k++) {
        double t = k * DT;
        truth_t cur;
        truth_at(t, &cur);
        double omega[3];
        double rt_rdot[3][3];
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                rt_rdot[i][j] = 0.0;
                for (int m = 0; m < 3; m++) {
                    rt_rdot[i][j] += cur.R[m][i] * (cur.R[m][j] - prev.R[m][j]) / DT;
                }
            }
        }
        omega[0] = rt_rdot[2][1];
        omega[1] = rt_rdot[0][2];
        omega[2] = rt_rdot[1][0];
        double fw[3] = {cur.a[0], cur.a[1], cur.a[2] + 9.80665};
        double fb[3];
        for (int i = 0; i < 3; i++) {
            fb[i] = cur.R[0][i] * fw[0] + cur.R[1][i] * fw[1] + cur.R[2][i] * fw[2];
        }
        sensor_snapshot_t snap;
        memset(&snap, 0, sizeof(snap));
        snap.has_origin = 1;
        snap.origin_lat_deg = LAT0;
        snap.origin_lon_deg = LON0;
        snap.origin_alt_m = ALT0;
        snap.has_imu = 1;
        snap.imu.header.timestamp_ns = (uint64_t)(t * 1e9);
        const double zero[3] = {0.0, 0.0, 0.0};
        imu_model_step(&imu, omega, fb, 25.0, zero, zero, snap.imu.gyro, snap.imu.accel);
        if (use_mag && k % 10 == 1) {
            double field_nt[3];
            for (int i = 0; i < 3; i++) {
                field_nt[i] = 1000.0 * (cur.R[0][i] * field_ut[0] + cur.R[1][i] * field_ut[1] +
                                        cur.R[2][i] * field_ut[2]);
            }
            double out_nt[3];
            mag_model_step(&mag, field_nt, 6.0, 25.0, out_nt);
            for (int i = 0; i < 3; i++) {
                mag_hold[i] = out_nt[i] * 1e-3;
            }
        }
        memcpy(snap.imu.mag, mag_hold, sizeof(mag_hold));

        gnss_env_t env = {1.0, 1.0, 0};
        if (t >= outage_start && t < outage_start + outage_len) {
            env.sky_visibility = 0.0;
        }
        gnss_sample_t gs;
        if (gnss_model_step(&gnss, t, cur.p, cur.v, &env, &gs)) {
            snap.has_gnss = 1;
            snap.gnss.header.timestamp_ns = (uint64_t)(gs.stamp_s * 1e9) + 1U;
            snap.gnss.latitude = LAT0 + gs.pos_enu[1] / R_EARTH * 180.0 / M_PI;
            snap.gnss.longitude = LON0 + gs.pos_enu[0] / (R_EARTH * cos(LAT0 * M_PI / 180.0)) * 180.0 / M_PI;
            snap.gnss.altitude = ALT0 + gs.pos_enu[2];
            memcpy(snap.gnss.velocity, gs.vel_enu, sizeof(gs.vel_enu));
            snap.gnss.fix_type = gs.fix >= GNSS_FIX_3D ? 3 : 0;
            snap.gnss.num_satellites = (uint8_t)gs.satellites;
            snap.gnss.hdop = gs.hdop;
            if (t > 40.0 && gs.fix >= GNSS_FIX_3D && outage_len == 0.0) {
                truth_t at_stamp;
                truth_at(gs.stamp_s, &at_stamp);
                for (int i = 0; i < 3; i++) {
                    sum_raw += (gs.pos_enu[i] - at_stamp.p[i]) * (gs.pos_enu[i] - at_stamp.p[i]);
                }
                n_raw++;
            }
        }
        if (k % 20 == 0) {
            snap.has_baro = 1;
            snap.baro.header.timestamp_ns = (uint64_t)(t * 1e9);
            double pa = baro_model_step(&baro, atmosphere_at(15.0, 101325.0, ALT0 + cur.p[2]).pressure_pa, 25.0, 0.5);
            snap.baro.altitude_m = baro_pressure_to_altitude(pa, 101325.0);
            snap.baro.pressure_pa = pa;
        }
        ops->ingest(&est, &snap, DT);
        estimated_state_t es;
        ops->get_state(&est, &es);
        if (t > 40.0 && es.valid && (t < outage_start || t > outage_start + outage_len + 20.0 || outage_len == 0.0)) {
            double e2 = 0.0;
            double v2 = 0.0;
            for (int i = 0; i < 3; i++) {
                e2 += (es.pos[i] - cur.p[i]) * (es.pos[i] - cur.p[i]);
                v2 += (es.vel[i] - cur.v[i]) * (es.vel[i] - cur.v[i]);
            }
            for (int i = 0; i < 3; i++) {
                double d = es.pos[i] - cur.p[i];
                axis_mean[i] += d;
                axis_sq[i] += d * d;
            }
            sum_p += e2;
            sum_v += v2;
            double qt[4];
            to_quat(cur.R, qt);
            double yaw_err = wrap(yaw_of(es.quat) - yaw_of(qt));
            sum_yaw += yaw_err * yaw_err;
            double tilt = z_axis_angle(es.quat, qt);
            sum_tilt += tilt * tilt;
            n_eval++;
        }
        if (t > 2.0 && t < 20.0 && es.valid) {
            double dz = fabs(es.pos[2] - cur.p[2]);
            res.early_z_max = dz > res.early_z_max ? dz : res.early_z_max;
        }
        if (outage_len > 0.0 && fabs(t - (outage_start + outage_len)) < 0.5 * DT && es.valid) {
            double e2 = 0.0;
            for (int i = 0; i < 3; i++) {
                e2 += (es.pos[i] - cur.p[i]) * (es.pos[i] - cur.p[i]);
            }
            res.outage_end_error = sqrt(e2);
        }
        prev = cur;
    }
    estimated_state_t last;
    ops->get_state(&est, &last);
    res.valid_at_end = last.valid;
    if (n_eval > 0) {
        res.pos_rms = sqrt(sum_p / n_eval);
        res.raw_rms = n_raw > 0 ? sqrt(sum_raw / n_raw) : 0.0;
        res.vel_rms = sqrt(sum_v / n_eval);
        res.yaw_err_deg = sqrt(sum_yaw / n_eval) * 180.0 / M_PI;
        res.tilt_err_deg = sqrt(sum_tilt / n_eval) * 180.0 / M_PI;
    }
    if (n_eval > 0 && getenv("KF_DEBUG") != NULL) {
        for (int i = 0; i < 3; i++) {
            printf("  axis %d: mean %.2f rms %.2f\n", i, axis_mean[i] / n_eval, sqrt(axis_sq[i] / n_eval));
        }
    }
    ops->destroy(&est);
    return res;
}

int main(void) {
    result_t a = run("mpu6050", "neo_m8n", "ms5611", 180.0, 0.0, 0.0, 1, 0);
    printf("mpu6050+neo_m8n+ms5611: pos %.2f m (raw gnss %.2f) vel %.3f m/s yaw %.2f deg tilt %.2f deg\n",
           a.pos_rms, a.raw_rms, a.vel_rms, a.yaw_err_deg, a.tilt_err_deg);
    CHECK(a.valid_at_end);
    CHECK(a.pos_rms < 1.15 * a.raw_rms);
    CHECK(a.vel_rms < 0.25);
    CHECK(a.yaw_err_deg < 8.0);
    CHECK(a.tilt_err_deg < 3.5);

    result_t b = run("adis16470", "zed_f9p", "dps310", 180.0, 0.0, 0.0, 1, 0);
    printf("adis16470+zed_f9p+dps310: pos %.2f m (raw gnss %.2f) vel %.3f m/s yaw %.2f deg tilt %.2f deg\n",
           b.pos_rms, b.raw_rms, b.vel_rms, b.yaw_err_deg, b.tilt_err_deg);
    CHECK(b.pos_rms < 1.15 * b.raw_rms);
    CHECK(b.vel_rms < 0.1);
    CHECK(b.tilt_err_deg < 2.0);
    CHECK(b.yaw_err_deg < 6.0);
    CHECK(b.early_z_max < 2.5);

    result_t w = run("adis16470", "zed_f9p", "dps310", 180.0, 0.0, 0.0, 1, 1);
    printf("uncorrelated gnss noise: pos %.2f m (raw gnss %.2f)\n", w.pos_rms, w.raw_rms);
    CHECK(w.pos_rms < 0.6 * w.raw_rms);

    result_t l = run("adis16470", "zed_f9p", "dps310", 120.0, 0.0, 0.0, 1, 2);
    printf("0.4 s GNSS latency, 0.3 m noise: pos %.2f m (raw gnss %.2f) vel %.3f m/s\n", l.pos_rms,
           l.raw_rms, l.vel_rms);
    CHECK(l.pos_rms < 0.55);
    CHECK(l.vel_rms < 0.12);

    result_t c = run("adis16470", "zed_f9p", "dps310", 140.0, 60.0, 15.0, 1, 0);
    printf("15 s GNSS outage with adis16470: error at end of outage %.2f m\n", c.outage_end_error);
    CHECK(c.outage_end_error < 15.0);
    CHECK(c.valid_at_end);

    result_t d = run("mpu6050", "neo_m8n", "ms5611", 180.0, 0.0, 0.0, 0, 0);
    printf("no magnetometer: pos %.2f m (raw gnss %.2f) tilt %.2f deg\n", d.pos_rms, d.raw_rms,
           d.tilt_err_deg);
    CHECK(d.pos_rms < 1.3 * d.raw_rms);
    CHECK(d.tilt_err_deg < 4.0);

    if (g_failures != 0) {
        fprintf(stderr, "%d check(s) failed\n", g_failures);
        return 1;
    }
    puts("test_kf_ins OK");
    return 0;
}
