#include "vehicle/attitude_math.h"
#include "vehicle/estimator_ops.h"
#include "vehicle/kf_axis.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

#define KF_G 9.80665
#define KF_R_EARTH 6371000.0
enum {
    KF_HISTORY = 512,
    KF_MIN_SATS = 6,
};
#define KF_FIELD_MIN_UT 5.0
#define KF_ACC_GAIN 0.6
#define KF_MAG_GAIN 0.25
#define KF_ACC_TRUST_BAND 0.15
#define KF_GNSS_POS_SIGMA 1.0
#define KF_GNSS_VEL_SIGMA 0.15
#define KF_GNSS_VERTICAL_FACTOR 1.5
#define KF_BARO_SIGMA 0.6
#define KF_BARO_START_VAR 4.0
#define KF_ACCEL_LAG_S 0.6
#define KF_INIT_G_BAND 0.12
#define KF_INIT_TIMEOUT_S 0.25
#define KF_MIN_DT 1e-4
#define KF_MAX_DT 0.1
#define NS_PER_S 1e-9

extern const estimator_ops_t kf_ins_ops;

typedef struct {
    double stamp_ns;
    double pos[3];
    double vel[3];
} history_t;

typedef struct {
    double quat[4];
    double angvel[3];
    double mag_ref[3];
    int mag_ref_set;
    kf_axis_t axis[3];
    history_t history[KF_HISTORY];
    int history_head;
    int history_count;
    double lat0_rad;
    double lon0_rad;
    double alt0;
    int origin_set;
    int pos_init;
    int attitude_init;
    int yaw_init;
    uint64_t first_imu_ns;
    uint64_t last_imu_ns;
    double last_baro_alt;
    int baro_seen;
} kf_state_t;

static const history_t *history_at(const kf_state_t *state, double stamp_ns) {
    const history_t *best = NULL;
    double best_err = 1e30;
    for (int back = 0; back < state->history_count; back++) {
        int idx = (state->history_head - 1 - back + KF_HISTORY) % KF_HISTORY;
        double err = fabs(state->history[idx].stamp_ns - stamp_ns);
        if (err < best_err) {
            best_err = err;
            best = &state->history[idx];
        }
        if (state->history[idx].stamp_ns < stamp_ns) {
            break;
        }
    }
    return best;
}

static void history_push(kf_state_t *state, double stamp_ns) {
    history_t *slot = &state->history[state->history_head];
    slot->stamp_ns = stamp_ns;
    for (int idx = 0; idx < 3; idx++) {
        slot->pos[idx] = state->axis[idx].state[0];
        slot->vel[idx] = state->axis[idx].state[1];
    }
    state->history_head = (state->history_head + 1) % KF_HISTORY;
    if (state->history_count < KF_HISTORY) {
        state->history_count++;
    }
}

static void accel_correction(const sensor_imu_t *imu, const double accel_est[3], mat3_t rot,
                             double corr[3]) {
    double spec[3] = {imu->accel[0], imu->accel[1], imu->accel[2]};
    double spec_norm = norm3(spec);
    if (spec_norm <= 1e-3) {
        return;
    }
    double expected_world[3] = {accel_est[0], accel_est[1], accel_est[2] + KF_G};
    double expected_body[3];
    mat_t_vec(expected_body, rot, expected_world);
    double expected_norm = norm3(expected_body);
    double trust = expected_norm > 1e-3 &&
                           fabs(spec_norm - expected_norm) / KF_G < KF_ACC_TRUST_BAND
                       ? 1.0
                       : 0.0;
    double measured[3] = {spec[0] / spec_norm, spec[1] / spec_norm, spec[2] / spec_norm};
    double up_body[3] = {0.0, 0.0, 1.0};
    if (expected_norm > 1e-3) {
        for (int idx = 0; idx < 3; idx++) {
            up_body[idx] = expected_body[idx] / expected_norm;
        }
    }
    double err[3];
    cross3(err, measured, up_body);
    for (int idx = 0; idx < 3; idx++) {
        corr[idx] += KF_ACC_GAIN * trust * err[idx];
    }
}

static void mag_correction(kf_state_t *state, const sensor_imu_t *imu, mat3_t rot, double corr[3]) {
    double mag_norm = norm3(imu->mag);
    if (mag_norm <= KF_FIELD_MIN_UT) {
        return;
    }
    double mag_body[3] = {imu->mag[0], imu->mag[1], imu->mag[2]};
    if (!state->mag_ref_set) {
        mat_vec(state->mag_ref, rot, mag_body);
        state->mag_ref_set = 1;
        return;
    }
    double est_body[3];
    mat_t_vec(est_body, rot, state->mag_ref);
    double est_norm = norm3(est_body);
    if (est_norm <= 1e-6) {
        return;
    }
    double meas[3] = {mag_body[0] / mag_norm, mag_body[1] / mag_norm, mag_body[2] / mag_norm};
    double pred[3] = {est_body[0] / est_norm, est_body[1] / est_norm, est_body[2] / est_norm};
    double err[3];
    cross3(err, meas, pred);
    double err_world[3];
    mat_vec(err_world, rot, err);
    double yaw_only[3] = {0.0, 0.0, err_world[2]};
    double err_body[3];
    mat_t_vec(err_body, rot, yaw_only);
    for (int idx = 0; idx < 3; idx++) {
        corr[idx] += KF_MAG_GAIN * err_body[idx];
    }
}

static void attitude_update(kf_state_t *state, const sensor_imu_t *imu, const double accel_est[3],
                            double dt) {
    mat3_t rot;
    quat_to_matrix(state->quat, rot);
    double corr[3] = {0.0, 0.0, 0.0};
    accel_correction(imu, accel_est, rot, corr);
    mag_correction(state, imu, rot, corr);
    double rate[3];
    for (int idx = 0; idx < 3; idx++) {
        rate[idx] = imu->gyro[idx] + corr[idx];
    }
    double delta[4] = {1.0, 0.5 * dt * rate[0], 0.5 * dt * rate[1], 0.5 * dt * rate[2]};
    double next[4];
    quat_mul(next, state->quat, delta);
    memcpy(state->quat, next, sizeof(next));
    normalise_quat(state->quat);
    memcpy(state->angvel, imu->gyro, sizeof(state->angvel));
}

static void init_yaw_from_mag(kf_state_t *state, const sensor_imu_t *imu) {
    if (norm3(imu->mag) <= KF_FIELD_MIN_UT) {
        return;
    }
    mat3_t rot;
    quat_to_matrix(state->quat, rot);
    double mag_body[3] = {imu->mag[0], imu->mag[1], imu->mag[2]};
    double mag_world[3];
    mat_vec(mag_world, rot, mag_body);
    double psi = M_PI / 2.0 - atan2(mag_world[1], mag_world[0]);
    double yaw_quat[4] = {cos(0.5 * psi), 0.0, 0.0, sin(0.5 * psi)};
    double next[4];
    quat_mul(next, yaw_quat, state->quat);
    memcpy(state->quat, next, sizeof(next));
    normalise_quat(state->quat);
    state->yaw_init = 1;
}

static void init_attitude(kf_state_t *state, const sensor_imu_t *imu) {
    double spec_norm = norm3(imu->accel);
    int usable = fabs(spec_norm - KF_G) < KF_INIT_G_BAND * KF_G;
    double waited = (double)(imu->header.timestamp_ns - state->first_imu_ns) * NS_PER_S;
    if (!usable && waited < KF_INIT_TIMEOUT_S) {
        return;
    }
    double up[3] = {0.0, 0.0, 1.0};
    if (usable) {
        for (int idx = 0; idx < 3; idx++) {
            up[idx] = imu->accel[idx] / spec_norm;
        }
    }
    double roll = atan2(up[1], up[2]);
    double pitch = atan2(-up[0], sqrt(up[1] * up[1] + up[2] * up[2]));
    double cos_r = cos(0.5 * roll);
    double sin_r = sin(0.5 * roll);
    double cos_p = cos(0.5 * pitch);
    double sin_p = sin(0.5 * pitch);
    state->quat[0] = cos_r * cos_p;
    state->quat[1] = sin_r * cos_p;
    state->quat[2] = cos_r * sin_p;
    state->quat[3] = -sin_r * sin_p;
    normalise_quat(state->quat);
    state->attitude_init = 1;
}

static void gnss_to_local(const kf_state_t *state, const sensor_gnss_t *gnss, double pos[3],
                          double vel[3]) {
    pos[0] = (gnss->longitude * M_PI / 180.0 - state->lon0_rad) * KF_R_EARTH * cos(state->lat0_rad);
    pos[1] = (gnss->latitude * M_PI / 180.0 - state->lat0_rad) * KF_R_EARTH;
    pos[2] = gnss->altitude - state->alt0;
    memcpy(vel, gnss->velocity, 3 * sizeof(double));
}

static void fuse_gnss(kf_state_t *state, const sensor_gnss_t *gnss) {
    double pos[3];
    double vel[3];
    gnss_to_local(state, gnss, pos, vel);
    if (!state->pos_init) {
        for (int idx = 0; idx < 3; idx++) {
            kf_axis_init(&state->axis[idx], idx == 2 ? 4 : 3, pos[idx], vel[idx]);
        }
        if (state->baro_seen) {
            state->axis[2].state[3] = (state->last_baro_alt - state->alt0) - pos[2];
            state->axis[2].cov[3][3] = KF_BARO_START_VAR;
        }
        state->pos_init = 1;
        return;
    }
    const history_t *past = history_at(state, (double)gnss->header.timestamp_ns);
    double scale = gnss->hdop > 1.0 ? gnss->hdop : 1.0;
    for (int idx = 0; idx < 3; idx++) {
        kf_axis_t *axis = &state->axis[idx];
        const double pos_row[KF_AXIS_MAX_STATES] = {1.0, 0.0, 0.0, 0.0};
        const double vel_row[KF_AXIS_MAX_STATES] = {0.0, 1.0, 0.0, 0.0};
        double pos_ref = past != NULL ? past->pos[idx] : axis->state[0];
        double vel_ref = past != NULL ? past->vel[idx] : axis->state[1];
        double sigma = KF_GNSS_POS_SIGMA * scale * (idx == 2 ? KF_GNSS_VERTICAL_FACTOR : 1.0);
        kf_axis_update(axis, pos_row, pos[idx] - pos_ref, sigma * sigma);
        kf_axis_update(axis, vel_row, vel[idx] - vel_ref, KF_GNSS_VEL_SIGMA * KF_GNSS_VEL_SIGMA);
    }
}

static void fuse_baro(kf_state_t *state, const sensor_baro_t *baro) {
    state->last_baro_alt = baro->altitude_m;
    state->baro_seen = 1;
    if (!state->pos_init) {
        return;
    }
    kf_axis_t *axis = &state->axis[2];
    const double row[KF_AXIS_MAX_STATES] = {1.0, 0.0, 0.0, 1.0};
    double predicted = axis->state[0] + axis->state[3];
    kf_axis_update(axis, row, (baro->altitude_m - state->alt0) - predicted,
                   KF_BARO_SIGMA * KF_BARO_SIGMA);
}

static int kf_init(estimator_t *est) {
    kf_state_t *state = calloc(1, sizeof(*state));
    if (state == NULL) {
        return -1;
    }
    state->quat[0] = 1.0;
    est->ops = &kf_ins_ops;
    est->ctx = state;
    return 0;
}

static double imu_step(kf_state_t *state, const sensor_imu_t *imu, double fallback_dt) {
    double step = fallback_dt;
    if (state->last_imu_ns != 0 && imu->header.timestamp_ns > state->last_imu_ns) {
        step = (double)(imu->header.timestamp_ns - state->last_imu_ns) * NS_PER_S;
    }
    step = step < KF_MIN_DT ? KF_MIN_DT : step;
    step = step > KF_MAX_DT ? KF_MAX_DT : step;
    state->last_imu_ns = imu->header.timestamp_ns;
    if (state->first_imu_ns == 0) {
        state->first_imu_ns = imu->header.timestamp_ns > 0 ? imu->header.timestamp_ns : 1U;
    }
    return step;
}

static void estimate_world_accel(const kf_state_t *state, const sensor_imu_t *imu, double accel[3]) {
    accel[0] = 0.0;
    accel[1] = 0.0;
    accel[2] = 0.0;
    if (!state->pos_init) {
        return;
    }
    double now_ns = (double)imu->header.timestamp_ns;
    const history_t *old = history_at(state, now_ns - KF_ACCEL_LAG_S / NS_PER_S);
    if (old == NULL || now_ns - old->stamp_ns <= 0.5 * KF_ACCEL_LAG_S / NS_PER_S) {
        return;
    }
    double span = (now_ns - old->stamp_ns) * NS_PER_S;
    for (int idx = 0; idx < 3; idx++) {
        accel[idx] = (state->axis[idx].state[1] - old->vel[idx]) / span;
    }
}

static void process_imu(kf_state_t *state, const sensor_imu_t *imu, double fallback_dt) {
    double step = imu_step(state, imu, fallback_dt);
    if (!state->attitude_init) {
        init_attitude(state, imu);
    } else {
        if (!state->yaw_init) {
            init_yaw_from_mag(state, imu);
        }
        double accel_est[3];
        estimate_world_accel(state, imu, accel_est);
        attitude_update(state, imu, accel_est, step);
    }
    if (!state->pos_init || !state->attitude_init) {
        return;
    }
    mat3_t rot;
    quat_to_matrix(state->quat, rot);
    double accel_world[3];
    mat_vec(accel_world, rot, imu->accel);
    accel_world[2] -= KF_G;
    for (int idx = 0; idx < 3; idx++) {
        kf_axis_predict(&state->axis[idx], accel_world[idx], step);
    }
    history_push(state, (double)imu->header.timestamp_ns);
}

static void kf_ingest(estimator_t *est, const sensor_snapshot_t *snap, double dt) {
    kf_state_t *state = est->ctx;
    if (state == NULL || snap == NULL) {
        return;
    }
    if (!state->origin_set && snap->has_origin) {
        state->lat0_rad = snap->origin_lat_deg * M_PI / 180.0;
        state->lon0_rad = snap->origin_lon_deg * M_PI / 180.0;
        state->alt0 = snap->origin_alt_m;
        state->origin_set = 1;
    }
    if (!state->origin_set) {
        return;
    }
    if (snap->has_imu) {
        process_imu(state, &snap->imu, dt);
    }
    if (snap->has_gnss && snap->gnss.fix_type >= 3 && snap->gnss.num_satellites >= KF_MIN_SATS) {
        fuse_gnss(state, &snap->gnss);
    }
    if (snap->has_baro) {
        fuse_baro(state, &snap->baro);
    }
}

static void kf_get_state(const estimator_t *est, estimated_state_t *out) {
    const kf_state_t *state = est->ctx;
    if (state == NULL || out == NULL) {
        return;
    }
    memset(out, 0, sizeof(*out));
    for (int idx = 0; idx < 3; idx++) {
        out->pos[idx] = state->axis[idx].state[0];
        out->vel[idx] = state->axis[idx].state[1];
    }
    memcpy(out->quat, state->quat, sizeof(out->quat));
    memcpy(out->angvel, state->angvel, sizeof(out->angvel));
    out->timestamp_ns = state->last_imu_ns;
    out->valid = state->pos_init && state->attitude_init;
}

static void kf_reset(estimator_t *est) {
    kf_state_t *state = est->ctx;
    if (state == NULL) {
        return;
    }
    memset(state, 0, sizeof(*state));
    state->quat[0] = 1.0;
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
