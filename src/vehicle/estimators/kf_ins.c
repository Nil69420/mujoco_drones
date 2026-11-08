#include "vehicle/estimator_ops.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

#define KF_G 9.80665
#define KF_R_EARTH 6371000.0
#define KF_HISTORY 512
#define KF_MIN_SATS 6
#define KF_FIELD_MIN_UT 5.0
#define KF_ACC_GAIN 0.6
#define KF_MAG_GAIN 0.25
#define KF_ACC_TRUST_BAND 0.15
#define KF_ACCEL_SIGMA 0.10
#define KF_ACCEL_BIAS_WALK 0.004
#define KF_BARO_BIAS_WALK 0.03
#define KF_GNSS_POS_SIGMA 1.0
#define KF_GNSS_VEL_SIGMA 0.15
#define KF_BARO_SIGMA 0.6
#define KF_INIT_POS_VAR 4.0
#define KF_INIT_VEL_VAR 1.0
#define KF_INIT_BIAS_VAR 0.25
#define KF_INIT_BARO_VAR 100.0
#define KF_BARO_START_VAR 4.0
#define KF_ACCEL_LAG_S 0.6
#define KF_INIT_G_BAND 0.12
#define KF_INIT_TIMEOUT_S 0.25
#define KF_MIN_DT 1e-4
#define KF_MAX_DT 0.1

extern const estimator_ops_t kf_ins_ops;

typedef struct {
    double t_ns;
    double p[3];
    double v[3];
} hist_t;

typedef struct {
    int n;
    double x[4];
    double P[4][4];
} axis_t;

typedef struct {
    double quat[4];
    double angvel[3];
    double mag_ref[3];
    int mag_ref_set;
    axis_t axis[3];
    hist_t hist[KF_HISTORY];
    int hist_head;
    int hist_count;
    double lat0_rad;
    double lon0_rad;
    double alt0;
    int origin_set;
    int pos_init;
    int attitude_init;
    int yaw_init;
    uint64_t first_imu_ns;
    uint64_t last_imu_ns;
    uint64_t stamp_ns;
    double last_baro_alt;
    int baro_seen;
} kf_state_t;

static void quat_mul(double out[4], const double a[4], const double b[4]) {
    out[0] = a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3];
    out[1] = a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2];
    out[2] = a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1];
    out[3] = a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0];
}

static void quat_to_matrix(const double q[4], double r[3][3]) {
    double w = q[0];
    double x = q[1];
    double y = q[2];
    double z = q[3];
    r[0][0] = 1.0 - 2.0 * (y * y + z * z);
    r[0][1] = 2.0 * (x * y - w * z);
    r[0][2] = 2.0 * (x * z + w * y);
    r[1][0] = 2.0 * (x * y + w * z);
    r[1][1] = 1.0 - 2.0 * (x * x + z * z);
    r[1][2] = 2.0 * (y * z - w * x);
    r[2][0] = 2.0 * (x * z - w * y);
    r[2][1] = 2.0 * (y * z + w * x);
    r[2][2] = 1.0 - 2.0 * (x * x + y * y);
}

static void mat_vec(double out[3], double r[3][3], const double v[3]) {
    for (int i = 0; i < 3; i++) {
        out[i] = r[i][0] * v[0] + r[i][1] * v[1] + r[i][2] * v[2];
    }
}

static void mat_t_vec(double out[3], double r[3][3], const double v[3]) {
    for (int i = 0; i < 3; i++) {
        out[i] = r[0][i] * v[0] + r[1][i] * v[1] + r[2][i] * v[2];
    }
}

static double norm3(const double v[3]) {
    return sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

static void cross3(double out[3], const double a[3], const double b[3]) {
    out[0] = a[1] * b[2] - a[2] * b[1];
    out[1] = a[2] * b[0] - a[0] * b[2];
    out[2] = a[0] * b[1] - a[1] * b[0];
}

static void normalise_quat(double q[4]) {
    double n = sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    if (n < 1e-12) {
        q[0] = 1.0;
        q[1] = q[2] = q[3] = 0.0;
        return;
    }
    for (int i = 0; i < 4; i++) {
        q[i] /= n;
    }
}

static void axis_init(axis_t *a, int n, double p0, double v0) {
    memset(a, 0, sizeof(*a));
    a->n = n;
    a->x[0] = p0;
    a->x[1] = v0;
    a->P[0][0] = KF_INIT_POS_VAR;
    a->P[1][1] = KF_INIT_VEL_VAR;
    a->P[2][2] = KF_INIT_BIAS_VAR;
    if (n > 3) {
        a->P[3][3] = KF_INIT_BARO_VAR;
    }
}

static void axis_predict(axis_t *a, double accel, double dt) {
    int n = a->n;
    double f[4][4] = {{0}};
    for (int i = 0; i < n; i++) {
        f[i][i] = 1.0;
    }
    f[0][1] = dt;
    f[0][2] = -0.5 * dt * dt;
    f[1][2] = -dt;
    double u = accel - a->x[2];
    a->x[0] += a->x[1] * dt + 0.5 * u * dt * dt;
    a->x[1] += u * dt;
    double fp[4][4] = {{0}};
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
            for (int k = 0; k < n; k++) {
                fp[i][j] += f[i][k] * a->P[k][j];
            }
        }
    }
    double pn[4][4] = {{0}};
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
            for (int k = 0; k < n; k++) {
                pn[i][j] += fp[i][k] * f[j][k];
            }
        }
    }
    double g[2] = {0.5 * dt * dt, dt};
    double q2 = KF_ACCEL_SIGMA * KF_ACCEL_SIGMA;
    for (int i = 0; i < 2; i++) {
        for (int j = 0; j < 2; j++) {
            pn[i][j] += g[i] * g[j] * q2;
        }
    }
    pn[2][2] += KF_ACCEL_BIAS_WALK * KF_ACCEL_BIAS_WALK * dt;
    if (n > 3) {
        pn[3][3] += KF_BARO_BIAS_WALK * KF_BARO_BIAS_WALK * dt;
    }
    memcpy(a->P, pn, sizeof(pn));
}

static void axis_update(axis_t *a, const double h[4], double innovation, double r) {
    int n = a->n;
    double ph[4] = {0};
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
            ph[i] += a->P[i][j] * h[j];
        }
    }
    double s = r;
    for (int i = 0; i < n; i++) {
        s += h[i] * ph[i];
    }
    if (s < 1e-12) {
        return;
    }
    double k[4];
    for (int i = 0; i < n; i++) {
        k[i] = ph[i] / s;
        a->x[i] += k[i] * innovation;
    }
    double ikh[4][4] = {{0}};
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
            ikh[i][j] = (i == j ? 1.0 : 0.0) - k[i] * h[j];
        }
    }
    double t1[4][4] = {{0}};
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
            for (int m = 0; m < n; m++) {
                t1[i][j] += ikh[i][m] * a->P[m][j];
            }
        }
    }
    double pn[4][4] = {{0}};
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
            for (int m = 0; m < n; m++) {
                pn[i][j] += t1[i][m] * ikh[j][m];
            }
            pn[i][j] += k[i] * r * k[j];
        }
    }
    memcpy(a->P, pn, sizeof(pn));
}

static const hist_t *history_at(const kf_state_t *st, double t_ns) {
    const hist_t *best = NULL;
    double best_err = 1e30;
    for (int i = 0; i < st->hist_count; i++) {
        int idx = (st->hist_head - 1 - i + KF_HISTORY) % KF_HISTORY;
        double err = fabs(st->hist[idx].t_ns - t_ns);
        if (err < best_err) {
            best_err = err;
            best = &st->hist[idx];
        }
        if (st->hist[idx].t_ns < t_ns) {
            break;
        }
    }
    return best;
}

static void history_push(kf_state_t *st, double t_ns) {
    hist_t *h = &st->hist[st->hist_head];
    h->t_ns = t_ns;
    for (int i = 0; i < 3; i++) {
        h->p[i] = st->axis[i].x[0];
        h->v[i] = st->axis[i].x[1];
    }
    st->hist_head = (st->hist_head + 1) % KF_HISTORY;
    if (st->hist_count < KF_HISTORY) {
        st->hist_count++;
    }
}

static void attitude_update(kf_state_t *st, const sensor_imu_t *imu, const double a_est[3], double dt) {
    double f[3] = {imu->accel[0], imu->accel[1], imu->accel[2]};
    double w[3] = {imu->gyro[0], imu->gyro[1], imu->gyro[2]};
    double r[3][3];
    quat_to_matrix(st->quat, r);
    double corr[3] = {0.0, 0.0, 0.0};
    double fn = norm3(f);
    if (fn > 1e-3) {
        double expected_w[3] = {a_est[0], a_est[1], a_est[2] + KF_G};
        double expected_b[3];
        mat_t_vec(expected_b, r, expected_w);
        double en = norm3(expected_b);
        double trust = en > 1e-3 && fabs(fn - en) / KF_G < KF_ACC_TRUST_BAND ? 1.0 : 0.0;
        double meas[3] = {f[0] / fn, f[1] / fn, f[2] / fn};
        double up_body[3] = {0.0, 0.0, 1.0};
        if (en > 1e-3) {
            up_body[0] = expected_b[0] / en;
            up_body[1] = expected_b[1] / en;
            up_body[2] = expected_b[2] / en;
        }
        double e[3];
        cross3(e, meas, up_body);
        for (int i = 0; i < 3; i++) {
            corr[i] += KF_ACC_GAIN * trust * e[i];
        }
    }
    double mn = norm3(imu->mag);
    if (mn > KF_FIELD_MIN_UT) {
        double mb[3] = {imu->mag[0], imu->mag[1], imu->mag[2]};
        if (!st->mag_ref_set) {
            mat_vec(st->mag_ref, r, mb);
            st->mag_ref_set = 1;
        } else {
            double est_body[3];
            mat_t_vec(est_body, r, st->mag_ref);
            double en = norm3(est_body);
            if (en > 1e-6) {
                double a[3] = {mb[0] / mn, mb[1] / mn, mb[2] / mn};
                double b[3] = {est_body[0] / en, est_body[1] / en, est_body[2] / en};
                double e[3];
                cross3(e, a, b);
                double ew[3];
                mat_vec(ew, r, e);
                double yaw_only[3] = {0.0, 0.0, ew[2]};
                double eb[3];
                mat_t_vec(eb, r, yaw_only);
                for (int i = 0; i < 3; i++) {
                    corr[i] += KF_MAG_GAIN * eb[i];
                }
            }
        }
    }
    double ww[3] = {w[0] + corr[0], w[1] + corr[1], w[2] + corr[2]};
    double dq[4] = {1.0, 0.5 * dt * ww[0], 0.5 * dt * ww[1], 0.5 * dt * ww[2]};
    double qn[4];
    quat_mul(qn, st->quat, dq);
    memcpy(st->quat, qn, sizeof(qn));
    normalise_quat(st->quat);
    memcpy(st->angvel, imu->gyro, sizeof(st->angvel));
}

static void init_yaw_from_mag(kf_state_t *st, const sensor_imu_t *imu) {
    double mn = norm3(imu->mag);
    if (mn <= KF_FIELD_MIN_UT) {
        return;
    }
    double r[3][3];
    quat_to_matrix(st->quat, r);
    double mb[3] = {imu->mag[0], imu->mag[1], imu->mag[2]};
    double mw[3];
    mat_vec(mw, r, mb);
    double psi = M_PI / 2.0 - atan2(mw[1], mw[0]);
    double qz[4] = {cos(0.5 * psi), 0.0, 0.0, sin(0.5 * psi)};
    double qn[4];
    quat_mul(qn, qz, st->quat);
    memcpy(st->quat, qn, sizeof(qn));
    normalise_quat(st->quat);
}

static void init_attitude_from_accel(kf_state_t *st, const sensor_imu_t *imu) {
    double fn = norm3(imu->accel);
    int usable = fabs(fn - KF_G) < KF_INIT_G_BAND * KF_G;
    double waited = (double)(imu->header.timestamp_ns - st->first_imu_ns) * 1e-9;
    if (!usable && waited < KF_INIT_TIMEOUT_S) {
        return;
    }
    double up[3] = {0.0, 0.0, 1.0};
    if (usable) {
        up[0] = imu->accel[0] / fn;
        up[1] = imu->accel[1] / fn;
        up[2] = imu->accel[2] / fn;
    }
    double roll = atan2(up[1], up[2]);
    double pitch = atan2(-up[0], sqrt(up[1] * up[1] + up[2] * up[2]));
    double cr = cos(0.5 * roll);
    double sr = sin(0.5 * roll);
    double cp = cos(0.5 * pitch);
    double sp = sin(0.5 * pitch);
    st->quat[0] = cr * cp;
    st->quat[1] = sr * cp;
    st->quat[2] = cr * sp;
    st->quat[3] = -sr * sp;
    normalise_quat(st->quat);
    st->attitude_init = 1;
}

static void gnss_to_local(const kf_state_t *st, const sensor_gnss_t *g, double p[3], double v[3]) {
    p[0] = (g->longitude * M_PI / 180.0 - st->lon0_rad) * KF_R_EARTH * cos(st->lat0_rad);
    p[1] = (g->latitude * M_PI / 180.0 - st->lat0_rad) * KF_R_EARTH;
    p[2] = g->altitude - st->alt0;
    memcpy(v, g->velocity, 3 * sizeof(double));
}

static void fuse_gnss(kf_state_t *st, const sensor_gnss_t *g) {
    double p[3];
    double v[3];
    gnss_to_local(st, g, p, v);
    if (!st->pos_init) {
        for (int i = 0; i < 3; i++) {
            axis_init(&st->axis[i], i == 2 ? 4 : 3, p[i], v[i]);
        }
        if (st->baro_seen) {
            st->axis[2].x[3] = (st->last_baro_alt - st->alt0) - p[2];
            st->axis[2].P[3][3] = KF_BARO_START_VAR;
        }
        st->pos_init = 1;
        return;
    }
    const hist_t *h = history_at(st, (double)g->header.timestamp_ns);
    double sigma_scale = g->hdop > 1.0 ? g->hdop : 1.0;
    for (int i = 0; i < 3; i++) {
        axis_t *a = &st->axis[i];
        double hp[4] = {1.0, 0.0, 0.0, 0.0};
        double hv[4] = {0.0, 1.0, 0.0, 0.0};
        double p_ref = h != NULL ? h->p[i] : a->x[0];
        double v_ref = h != NULL ? h->v[i] : a->x[1];
        double rp = KF_GNSS_POS_SIGMA * sigma_scale * (i == 2 ? 1.5 : 1.0);
        axis_update(a, hp, p[i] - p_ref, rp * rp);
        axis_update(a, hv, v[i] - v_ref, KF_GNSS_VEL_SIGMA * KF_GNSS_VEL_SIGMA);
    }
    st->stamp_ns = g->header.timestamp_ns;
}

static void fuse_baro(kf_state_t *st, const sensor_baro_t *b) {
    st->last_baro_alt = b->altitude_m;
    st->baro_seen = 1;
    if (!st->pos_init) {
        return;
    }
    axis_t *a = &st->axis[2];
    double z = b->altitude_m - st->alt0;
    double h[4] = {1.0, 0.0, 0.0, 1.0};
    double predicted = a->x[0] + a->x[3];
    axis_update(a, h, z - predicted, KF_BARO_SIGMA * KF_BARO_SIGMA);
}

static int kf_init(estimator_t *est) {
    kf_state_t *st = calloc(1, sizeof(*st));
    if (st == NULL) {
        return -1;
    }
    st->quat[0] = 1.0;
    est->ops = &kf_ins_ops;
    est->ctx = st;
    return 0;
}

static void kf_ingest(estimator_t *est, const sensor_snapshot_t *snap, double dt) {
    kf_state_t *st = est->ctx;
    if (st == NULL || snap == NULL) {
        return;
    }
    if (!st->origin_set && snap->has_origin) {
        st->lat0_rad = snap->origin_lat_deg * M_PI / 180.0;
        st->lon0_rad = snap->origin_lon_deg * M_PI / 180.0;
        st->alt0 = snap->origin_alt_m;
        st->origin_set = 1;
    }
    if (!st->origin_set) {
        return;
    }
    if (snap->has_imu) {
        double step = dt;
        if (st->last_imu_ns != 0 && snap->imu.header.timestamp_ns > st->last_imu_ns) {
            step = (double)(snap->imu.header.timestamp_ns - st->last_imu_ns) * 1e-9;
        }
        if (step < KF_MIN_DT) {
            step = KF_MIN_DT;
        } else if (step > KF_MAX_DT) {
            step = KF_MAX_DT;
        }
        st->last_imu_ns = snap->imu.header.timestamp_ns;
        if (st->first_imu_ns == 0) {
            st->first_imu_ns = snap->imu.header.timestamp_ns > 0 ? snap->imu.header.timestamp_ns : 1U;
        }
        if (!st->attitude_init) {
            init_attitude_from_accel(st, &snap->imu);
        } else {
            if (!st->yaw_init && norm3(snap->imu.mag) > KF_FIELD_MIN_UT) {
                init_yaw_from_mag(st, &snap->imu);
                st->yaw_init = 1;
            }
            double a_est[3] = {0.0, 0.0, 0.0};
            if (st->pos_init) {
                const hist_t *old = history_at(st, (double)snap->imu.header.timestamp_ns - KF_ACCEL_LAG_S * 1e9);
                if (old != NULL && (double)snap->imu.header.timestamp_ns - old->t_ns > 0.5 * KF_ACCEL_LAG_S * 1e9) {
                    double span = ((double)snap->imu.header.timestamp_ns - old->t_ns) * 1e-9;
                    for (int i = 0; i < 3; i++) {
                        a_est[i] = (st->axis[i].x[1] - old->v[i]) / span;
                    }
                }
            }
            attitude_update(st, &snap->imu, a_est, step);
        }
        if (st->pos_init && st->attitude_init) {
            double r[3][3];
            quat_to_matrix(st->quat, r);
            double aw[3];
            mat_vec(aw, r, snap->imu.accel);
            aw[2] -= KF_G;
            for (int i = 0; i < 3; i++) {
                axis_predict(&st->axis[i], aw[i], step);
            }
            history_push(st, (double)snap->imu.header.timestamp_ns);
        }
    }
    if (snap->has_gnss && snap->gnss.fix_type >= 3 && snap->gnss.num_satellites >= KF_MIN_SATS) {
        fuse_gnss(st, &snap->gnss);
    }
    if (snap->has_baro) {
        fuse_baro(st, &snap->baro);
    }
}

static void kf_get_state(const estimator_t *est, estimated_state_t *out) {
    const kf_state_t *st = est->ctx;
    if (st == NULL || out == NULL) {
        return;
    }
    memset(out, 0, sizeof(*out));
    for (int i = 0; i < 3; i++) {
        out->pos[i] = st->axis[i].x[0];
        out->vel[i] = st->axis[i].x[1];
    }
    memcpy(out->quat, st->quat, sizeof(out->quat));
    memcpy(out->angvel, st->angvel, sizeof(out->angvel));
    out->timestamp_ns = st->last_imu_ns;
    out->valid = st->pos_init && st->attitude_init;
}

static void kf_reset(estimator_t *est) {
    kf_state_t *st = est->ctx;
    if (st == NULL) {
        return;
    }
    memset(st, 0, sizeof(*st));
    st->quat[0] = 1.0;
}

static void kf_destroy(estimator_t *est) {
    if (est == NULL) {
        return;
    }
    free(est->ctx);
    est->ctx = NULL;
}

const estimator_ops_t kf_ins_ops = {
    .name = "kf_ins",
    .init = kf_init,
    .ingest = kf_ingest,
    .get_state = kf_get_state,
    .reset = kf_reset,
    .destroy = kf_destroy,
};
