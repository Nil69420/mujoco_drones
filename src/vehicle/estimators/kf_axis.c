#include "vehicle/kf_axis.h"

#include <string.h>

#define KF_ACCEL_SIGMA 0.10
#define KF_ACCEL_BIAS_WALK 0.004
#define KF_BARO_BIAS_WALK 0.03
#define KF_INIT_POS_VAR 4.0
#define KF_INIT_VEL_VAR 1.0
#define KF_INIT_BIAS_VAR 0.25
#define KF_INIT_BARO_VAR 100.0
#define KF_MIN_INNOVATION_VAR 1e-12

typedef double matrix_t[KF_AXIS_MAX_STATES][KF_AXIS_MAX_STATES];

void kf_axis_init(kf_axis_t *axis, int count, double position, double velocity) {
    memset(axis, 0, sizeof(*axis));
    axis->count = count;
    axis->state[0] = position;
    axis->state[1] = velocity;
    axis->cov[0][0] = KF_INIT_POS_VAR;
    axis->cov[1][1] = KF_INIT_VEL_VAR;
    axis->cov[2][2] = KF_INIT_BIAS_VAR;
    if (count > 3) {
        axis->cov[3][3] = KF_INIT_BARO_VAR;
    }
}

static void mat_mul(int size, matrix_t lhs, matrix_t rhs, matrix_t out) {
    for (int row = 0; row < size; row++) {
        for (int col = 0; col < size; col++) {
            double acc = 0.0;
            for (int mid = 0; mid < size; mid++) {
                acc += lhs[row][mid] * rhs[mid][col];
            }
            out[row][col] = acc;
        }
    }
}

static void mat_mul_transpose(int size, matrix_t lhs, matrix_t rhs, matrix_t out) {
    for (int row = 0; row < size; row++) {
        for (int col = 0; col < size; col++) {
            double acc = 0.0;
            for (int mid = 0; mid < size; mid++) {
                acc += lhs[row][mid] * rhs[col][mid];
            }
            out[row][col] = acc;
        }
    }
}

void kf_axis_predict(kf_axis_t *axis, double accel, double dt) {
    int size = axis->count;
    matrix_t trans = {{0}};
    for (int idx = 0; idx < size; idx++) {
        trans[idx][idx] = 1.0;
    }
    trans[0][1] = dt;
    trans[0][2] = -0.5 * dt * dt;
    trans[1][2] = -dt;
    double input = accel - axis->state[2];
    axis->state[0] += axis->state[1] * dt + 0.5 * input * dt * dt;
    axis->state[1] += input * dt;
    matrix_t scratch = {{0}};
    matrix_t next = {{0}};
    mat_mul(size, trans, axis->cov, scratch);
    mat_mul_transpose(size, scratch, trans, next);
    double gain[2] = {0.5 * dt * dt, dt};
    double accel_var = KF_ACCEL_SIGMA * KF_ACCEL_SIGMA;
    for (int row = 0; row < 2; row++) {
        for (int col = 0; col < 2; col++) {
            next[row][col] += gain[row] * gain[col] * accel_var;
        }
    }
    next[2][2] += KF_ACCEL_BIAS_WALK * KF_ACCEL_BIAS_WALK * dt;
    if (size > 3) {
        next[3][3] += KF_BARO_BIAS_WALK * KF_BARO_BIAS_WALK * dt;
    }
    memcpy(axis->cov, next, sizeof(next));
}

void kf_axis_update(kf_axis_t *axis, const double row[KF_AXIS_MAX_STATES], double innovation,
                    double variance) {
    int size = axis->count;
    double cov_row[KF_AXIS_MAX_STATES] = {0};
    for (int idx = 0; idx < size; idx++) {
        for (int col = 0; col < size; col++) {
            cov_row[idx] += axis->cov[idx][col] * row[col];
        }
    }
    double innov_var = variance;
    for (int idx = 0; idx < size; idx++) {
        innov_var += row[idx] * cov_row[idx];
    }
    if (innov_var < KF_MIN_INNOVATION_VAR) {
        return;
    }
    double gain[KF_AXIS_MAX_STATES];
    for (int idx = 0; idx < size; idx++) {
        gain[idx] = cov_row[idx] / innov_var;
        axis->state[idx] += gain[idx] * innovation;
    }
    matrix_t keep = {{0}};
    for (int idx = 0; idx < size; idx++) {
        for (int col = 0; col < size; col++) {
            keep[idx][col] = (idx == col ? 1.0 : 0.0) - gain[idx] * row[col];
        }
    }
    matrix_t scratch = {{0}};
    matrix_t next = {{0}};
    mat_mul(size, keep, axis->cov, scratch);
    mat_mul_transpose(size, scratch, keep, next);
    for (int idx = 0; idx < size; idx++) {
        for (int col = 0; col < size; col++) {
            next[idx][col] += gain[idx] * variance * gain[col];
        }
    }
    memcpy(axis->cov, next, sizeof(next));
}
