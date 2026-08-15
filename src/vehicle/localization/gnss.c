#include "vehicle/localization_ops.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

#define GNSS_R_EARTH 6371000.0

extern const localization_ops_t gnss_localization_ops;

typedef struct {
    double origin_lat_rad;
    double origin_lon_rad;
    double origin_alt;
    bool   origin_set;
    double pos[3];
    uint64_t timestamp_ns;
} gnss_localization_state_t;

static void gnss_to_local(const gnss_localization_state_t *st,
                          const sensor_gnss_t *gnss, double out[3]) {
    double lat_rad = gnss->latitude  * M_PI / 180.0;
    double lon_rad = gnss->longitude * M_PI / 180.0;
    out[0] = (lon_rad - st->origin_lon_rad) * GNSS_R_EARTH * cos(st->origin_lat_rad);
    out[1] = (lat_rad - st->origin_lat_rad) * GNSS_R_EARTH;
    out[2] = gnss->altitude - st->origin_alt;
}

static int gnss_init(localization_t *loc) {
    gnss_localization_state_t *st = calloc(1, sizeof(*st));
    if (!st) return -1;
    loc->ops = &gnss_localization_ops;
    loc->ctx = st;
    return 0;
}

static void gnss_ingest(localization_t *loc, const sensor_snapshot_t *snap) {
    gnss_localization_state_t *st = loc->ctx;
    if (!st || !snap) return;

    if (!snap->has_gnss) return;
    if (snap->gnss.fix_type < 2 || snap->gnss.num_satellites < 4) return;

    if (!st->origin_set) {
        st->origin_lat_rad = snap->gnss.latitude  * M_PI / 180.0;
        st->origin_lon_rad = snap->gnss.longitude * M_PI / 180.0;
        st->origin_alt     = snap->gnss.altitude;
        st->origin_set     = true;
    }
    gnss_to_local(st, &snap->gnss, st->pos);
    st->timestamp_ns = snap->gnss.header.timestamp_ns;
}

static void gnss_get_fix(const localization_t *loc, localization_fix_t *out) {
    const gnss_localization_state_t *st = loc->ctx;
    if (!st || !out) return;
    memcpy(out->pos, st->pos, 3 * sizeof(double));
    out->timestamp_ns = st->timestamp_ns;
    out->valid = st->origin_set && st->timestamp_ns != 0;
}

static void gnss_reset(localization_t *loc) {
    gnss_localization_state_t *st = loc->ctx;
    if (!st) return;
    memset(st, 0, sizeof(*st));
}

static void gnss_destroy(localization_t *loc) {
    if (!loc) return;
    free(loc->ctx);
    loc->ctx = NULL;
}

const localization_ops_t gnss_localization_ops = {
    .name     = "gnss",
    .init     = gnss_init,
    .ingest   = gnss_ingest,
    .get_fix  = gnss_get_fix,
    .reset    = gnss_reset,
    .destroy  = gnss_destroy,
};
