#include "vehicle/estimator_ops.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

#define CF_GRAVITY     9.81
#define CF_R_EARTH     6371000.0
#define CF_ATT_GAIN    0.05
#define CF_MAX_TILT_ERR 0.5

extern const estimator_ops_t complementary_filter_ops;

typedef struct {
    double origin_lat_rad;
    double origin_lon_rad;
    double origin_alt;
    bool   origin_set;
    double pos[3];
    double vel[3];
    double quat[4];
    double angvel[3];
    uint64_t timestamp_ns;
} complementary_filter_state_t;

static void cf_to_local(const complementary_filter_state_t *st,
                        const sensor_gnss_t *gnss, double out[3]) {
    double lat_rad = gnss->latitude  * M_PI / 180.0;
    double lon_rad = gnss->longitude * M_PI / 180.0;
    out[0] = (lon_rad - st->origin_lon_rad) * CF_R_EARTH * cos(st->origin_lat_rad);
    out[1] = (lat_rad - st->origin_lat_rad) * CF_R_EARTH;
    out[2] = gnss->altitude - st->origin_alt;
}

static void cf_quat_mul(double out[4], const double a[4], const double b[4]) {
    out[0] = a[0]*b[0] - a[1]*b[1] - a[2]*b[2] - a[3]*b[3];
    out[1] = a[0]*b[1] + a[1]*b[0] + a[2]*b[3] - a[3]*b[2];
    out[2] = a[0]*b[2] - a[1]*b[3] + a[2]*b[0] + a[3]*b[1];
    out[3] = a[0]*b[3] + a[1]*b[2] - a[2]*b[1] + a[3]*b[0];
}

static void cf_rotate_vec(double out[3], const double q[4], const double v[3]) {
    double qv[4] = { q[0], q[1], q[2], q[3] };
    double qinv[4] = { q[0], -q[1], -q[2], -q[3] };
    double vq[4] = { 0.0, v[0], v[1], v[2] };
    double tmp[4];
    cf_quat_mul(tmp, qv, vq);
    double res[4];
    cf_quat_mul(res, tmp, qinv);
    out[0] = res[1];
    out[1] = res[2];
    out[2] = res[3];
}

static int cf_init(estimator_t *est) {
    complementary_filter_state_t *st = calloc(1, sizeof(*st));
    if (!st) return -1;
    st->quat[0] = 1.0;
    est->ops = &complementary_filter_ops;
    est->ctx = st;
    return 0;
}

static void cf_ingest(estimator_t *est, const sensor_snapshot_t *snap,
                      double dt) {
    complementary_filter_state_t *st = est->ctx;
    if (!st || !snap) return;

    if (!st->origin_set && snap->has_gnss) {
        st->origin_lat_rad = snap->gnss.latitude  * M_PI / 180.0;
        st->origin_lon_rad = snap->gnss.longitude * M_PI / 180.0;
        st->origin_alt     = snap->gnss.altitude;
        st->origin_set     = true;
    }

    if (snap->has_gnss) {
        cf_to_local(st, &snap->gnss, st->pos);
        memcpy(st->vel, snap->gnss.velocity, 3 * sizeof(double));
        st->timestamp_ns = snap->gnss.header.timestamp_ns;
    }

    if (snap->has_imu) {
        double acc_norm = sqrt(snap->imu.accel[0] * snap->imu.accel[0] +
                               snap->imu.accel[1] * snap->imu.accel[1] +
                               snap->imu.accel[2] * snap->imu.accel[2]);
        double wx = snap->imu.gyro[0];
        double wy = snap->imu.gyro[1];
        double wz = snap->imu.gyro[2];

        if (acc_norm > 1e-3 && st->origin_set) {
            double g_body[3];
            cf_rotate_vec(g_body, st->quat, (double[3]){ 0.0, 0.0, -CF_GRAVITY });
            double ax = snap->imu.accel[0] / acc_norm * CF_GRAVITY;
            double ay = snap->imu.accel[1] / acc_norm * CF_GRAVITY;
            double az = snap->imu.accel[2] / acc_norm * CF_GRAVITY;
            double err[3] = {
                g_body[1] * az - g_body[2] * ay,
                g_body[2] * ax - g_body[0] * az,
                g_body[0] * ay - g_body[1] * ax,
            };
            double e_mag = sqrt(err[0]*err[0] + err[1]*err[1] + err[2]*err[2]);
            if (e_mag > CF_MAX_TILT_ERR) {
                double scale = CF_MAX_TILT_ERR / e_mag;
                err[0] *= scale; err[1] *= scale; err[2] *= scale;
            }
            wx += CF_ATT_GAIN * err[0];
            wy += CF_ATT_GAIN * err[1];
            wz += CF_ATT_GAIN * err[2];
        }

        double half = 0.5 * dt;
        double dq[4] = {
            1.0,
            wx * half,
            wy * half,
            wz * half,
        };
        double norm = sqrt(dq[0]*dq[0] + dq[1]*dq[1] + dq[2]*dq[2] + dq[3]*dq[3]);
        if (norm > 1e-12) {
            dq[0] /= norm; dq[1] /= norm; dq[2] /= norm; dq[3] /= norm;
        }
        double qnew[4];
        cf_quat_mul(qnew, st->quat, dq);
        norm = sqrt(qnew[0]*qnew[0] + qnew[1]*qnew[1] +
                    qnew[2]*qnew[2] + qnew[3]*qnew[3]);
        if (norm > 1e-12) {
            st->quat[0] = qnew[0]/norm; st->quat[1] = qnew[1]/norm;
            st->quat[2] = qnew[2]/norm; st->quat[3] = qnew[3]/norm;
        }

        if (!snap->has_gnss) {
            double world_a[3];
            cf_rotate_vec(world_a, st->quat, snap->imu.accel);
            world_a[2] -= CF_GRAVITY;
            st->vel[0] += world_a[0] * dt;
            st->vel[1] += world_a[1] * dt;
            st->vel[2] += world_a[2] * dt;
            st->pos[0] += st->vel[0] * dt;
            st->pos[1] += st->vel[1] * dt;
            st->pos[2] += st->vel[2] * dt;
        }

        memcpy(st->angvel, snap->imu.gyro, 3 * sizeof(double));
        if (!snap->has_gnss) {
            st->timestamp_ns = snap->imu.header.timestamp_ns;
        }
    }

    if (snap->has_baro && !snap->has_gnss) {
        st->pos[2] = snap->baro.altitude_m - st->origin_alt;
    }
}

static void cf_get_state(const estimator_t *est, estimated_state_t *out) {
    const complementary_filter_state_t *st = est->ctx;
    if (!st || !out) return;

    memset(out, 0, sizeof(*out));
    memcpy(out->pos, st->pos, 3 * sizeof(double));
    memcpy(out->vel, st->vel, 3 * sizeof(double));
    memcpy(out->quat, st->quat, 4 * sizeof(double));
    memcpy(out->angvel, st->angvel, 3 * sizeof(double));
    out->timestamp_ns = st->timestamp_ns;
    out->valid = st->timestamp_ns != 0;
}

static void cf_reset(estimator_t *est) {
    complementary_filter_state_t *st = est->ctx;
    if (!st) return;
    memset(st, 0, sizeof(*st));
    st->quat[0] = 1.0;
}

static void cf_destroy(estimator_t *est) {
    if (!est) return;
    free(est->ctx);
    est->ctx = NULL;
}

const estimator_ops_t complementary_filter_ops = {
    .name      = "complementary_filter",
    .init      = cf_init,
    .ingest    = cf_ingest,
    .get_state = cf_get_state,
    .reset     = cf_reset,
    .destroy   = cf_destroy,
};
