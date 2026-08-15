#include "vehicle/localization_ops.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

#define LOC_R_EARTH 6371000.0

extern const localization_ops_t ground_truth_localization_ops;

typedef struct {
    double origin_lat_rad;
    double origin_lon_rad;
    double origin_alt;
    bool   origin_set;
    double pos[3];
    uint64_t timestamp_ns;
    bool   valid;
} ground_truth_localization_state_t;

static void loc_to_local(const ground_truth_localization_state_t *st,
                         const sensor_gnss_t *gnss, double out[3]) {
    double lat_rad = gnss->latitude  * M_PI / 180.0;
    double lon_rad = gnss->longitude * M_PI / 180.0;
    out[0] = (lon_rad - st->origin_lon_rad) * LOC_R_EARTH * cos(st->origin_lat_rad);
    out[1] = (lat_rad - st->origin_lat_rad) * LOC_R_EARTH;
    out[2] = gnss->altitude - st->origin_alt;
}

static int gt_init(localization_t *loc) {
    ground_truth_localization_state_t *st = calloc(1, sizeof(*st));
    if (!st) return -1;
    loc->ops = &ground_truth_localization_ops;
    loc->ctx = st;
    return 0;
}

static void gt_ingest(localization_t *loc, const sensor_snapshot_t *snap) {
    ground_truth_localization_state_t *st = loc->ctx;
    if (!st || !snap) return;

    if (snap->has_gnss) {
        if (!st->origin_set) {
            st->origin_lat_rad = snap->gnss.latitude  * M_PI / 180.0;
            st->origin_lon_rad = snap->gnss.longitude * M_PI / 180.0;
            st->origin_alt     = snap->gnss.altitude;
            st->origin_set     = true;
        }
        loc_to_local(st, &snap->gnss, st->pos);
        st->timestamp_ns = snap->gnss.header.timestamp_ns;
        st->valid = true;
    } else if (snap->has_baro && st->valid) {
        st->pos[2] = snap->baro.altitude_m - st->origin_alt;
        st->timestamp_ns = snap->baro.header.timestamp_ns;
    }
}

static void gt_get_fix(const localization_t *loc, localization_fix_t *out) {
    const ground_truth_localization_state_t *st = loc->ctx;
    if (!st || !out) return;
    memcpy(out->pos, st->pos, 3 * sizeof(double));
    out->timestamp_ns = st->timestamp_ns;
    out->valid = st->valid;
}

static void gt_reset(localization_t *loc) {
    ground_truth_localization_state_t *st = loc->ctx;
    if (!st) return;
    memset(st, 0, sizeof(*st));
}

static void gt_destroy(localization_t *loc) {
    if (!loc) return;
    free(loc->ctx);
    loc->ctx = NULL;
}

const localization_ops_t ground_truth_localization_ops = {
    .name     = "ground_truth",
    .init     = gt_init,
    .ingest   = gt_ingest,
    .get_fix  = gt_get_fix,
    .reset    = gt_reset,
    .destroy  = gt_destroy,
};
