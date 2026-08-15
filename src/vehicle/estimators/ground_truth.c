#include "vehicle/estimator_ops.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

#define GT_R_EARTH 6371000.0

extern const estimator_ops_t ground_truth_estimator_ops;

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
} ground_truth_state_t;

static void gt_to_local(const ground_truth_state_t *st,
                        const sensor_gnss_t *gnss, double out[3]) {
    double lat_rad = gnss->latitude  * M_PI / 180.0;
    double lon_rad = gnss->longitude * M_PI / 180.0;
    out[0] = (lon_rad - st->origin_lon_rad) * GT_R_EARTH * cos(st->origin_lat_rad);
    out[1] = (lat_rad - st->origin_lat_rad) * GT_R_EARTH;
    out[2] = gnss->altitude - st->origin_alt;
}

static int gt_init(estimator_t *est) {
    ground_truth_state_t *st = calloc(1, sizeof(*st));
    if (!st) return -1;
    est->ops = &ground_truth_estimator_ops;
    est->ctx = st;
    return 0;
}

static void gt_ingest(estimator_t *est, const sensor_snapshot_t *snap,
                      double dt) {
    ground_truth_state_t *st = est->ctx;
    if (!st || !snap) return;
    (void)dt;

    if (!st->origin_set && snap->has_gnss) {
        st->origin_lat_rad = snap->gnss.latitude  * M_PI / 180.0;
        st->origin_lon_rad = snap->gnss.longitude * M_PI / 180.0;
        st->origin_alt     = snap->gnss.altitude;
        st->origin_set     = true;
    }

    if (snap->has_gnss) {
        gt_to_local(st, &snap->gnss, st->pos);
        memcpy(st->vel, snap->gnss.velocity, 3 * sizeof(double));
        st->timestamp_ns = snap->gnss.header.timestamp_ns;
    } else if (snap->has_baro && st->origin_set) {
        st->pos[2] = snap->baro.altitude_m - st->origin_alt;
        st->timestamp_ns = snap->baro.header.timestamp_ns;
    }

    if (snap->has_imu) {
        memcpy(st->quat, snap->imu.orientation, 4 * sizeof(double));
        memcpy(st->angvel, snap->imu.gyro, 3 * sizeof(double));
    }
}

static void gt_get_state(const estimator_t *est, estimated_state_t *out) {
    const ground_truth_state_t *st = est->ctx;
    if (!st || !out) return;

    memset(out, 0, sizeof(*out));
    memcpy(out->pos, st->pos, 3 * sizeof(double));
    memcpy(out->vel, st->vel, 3 * sizeof(double));
    memcpy(out->quat, st->quat, 4 * sizeof(double));
    memcpy(out->angvel, st->angvel, 3 * sizeof(double));
    out->timestamp_ns = st->timestamp_ns;
    out->valid = st->origin_set || st->timestamp_ns != 0;
}

static void gt_reset(estimator_t *est) {
    ground_truth_state_t *st = est->ctx;
    if (!st) return;
    memset(st, 0, sizeof(*st));
}

static void gt_destroy(estimator_t *est) {
    if (!est) return;
    free(est->ctx);
    est->ctx = NULL;
}

const estimator_ops_t ground_truth_estimator_ops = {
    .name      = "ground_truth",
    .init      = gt_init,
    .ingest    = gt_ingest,
    .get_state = gt_get_state,
    .reset     = gt_reset,
    .destroy   = gt_destroy,
};
