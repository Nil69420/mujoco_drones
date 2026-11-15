#ifndef MUJOCO_DRONES_VEHICLE_ATTITUDE_MATH_H
#define MUJOCO_DRONES_VEHICLE_ATTITUDE_MATH_H

#include <math.h>
#include <string.h>

typedef double mat3_t[3][3];

static inline void quat_mul(double out[4], const double lhs[4], const double rhs[4]) {
    out[0] = lhs[0] * rhs[0] - lhs[1] * rhs[1] - lhs[2] * rhs[2] - lhs[3] * rhs[3];
    out[1] = lhs[0] * rhs[1] + lhs[1] * rhs[0] + lhs[2] * rhs[3] - lhs[3] * rhs[2];
    out[2] = lhs[0] * rhs[2] - lhs[1] * rhs[3] + lhs[2] * rhs[0] + lhs[3] * rhs[1];
    out[3] = lhs[0] * rhs[3] + lhs[1] * rhs[2] - lhs[2] * rhs[1] + lhs[3] * rhs[0];
}

static inline void quat_to_matrix(const double quat[4], mat3_t rot) {
    double qw = quat[0];
    double qx = quat[1];
    double qy = quat[2];
    double qz = quat[3];
    rot[0][0] = 1.0 - 2.0 * (qy * qy + qz * qz);
    rot[0][1] = 2.0 * (qx * qy - qw * qz);
    rot[0][2] = 2.0 * (qx * qz + qw * qy);
    rot[1][0] = 2.0 * (qx * qy + qw * qz);
    rot[1][1] = 1.0 - 2.0 * (qx * qx + qz * qz);
    rot[1][2] = 2.0 * (qy * qz - qw * qx);
    rot[2][0] = 2.0 * (qx * qz - qw * qy);
    rot[2][1] = 2.0 * (qy * qz + qw * qx);
    rot[2][2] = 1.0 - 2.0 * (qx * qx + qy * qy);
}

static inline void mat_vec(double out[3], mat3_t rot, const double vec[3]) {
    for (int row = 0; row < 3; row++) {
        out[row] = rot[row][0] * vec[0] + rot[row][1] * vec[1] + rot[row][2] * vec[2];
    }
}

static inline void mat_t_vec(double out[3], mat3_t rot, const double vec[3]) {
    for (int col = 0; col < 3; col++) {
        out[col] = rot[0][col] * vec[0] + rot[1][col] * vec[1] + rot[2][col] * vec[2];
    }
}

static inline double norm3(const double vec[3]) {
    return sqrt(vec[0] * vec[0] + vec[1] * vec[1] + vec[2] * vec[2]);
}

static inline void cross3(double out[3], const double lhs[3], const double rhs[3]) {
    out[0] = lhs[1] * rhs[2] - lhs[2] * rhs[1];
    out[1] = lhs[2] * rhs[0] - lhs[0] * rhs[2];
    out[2] = lhs[0] * rhs[1] - lhs[1] * rhs[0];
}

static inline void normalise_quat(double quat[4]) {
    double mag = sqrt(quat[0] * quat[0] + quat[1] * quat[1] + quat[2] * quat[2] + quat[3] * quat[3]);
    if (mag < 1e-12) {
        quat[0] = 1.0;
        quat[1] = 0.0;
        quat[2] = 0.0;
        quat[3] = 0.0;
        return;
    }
    for (int idx = 0; idx < 4; idx++) {
        quat[idx] /= mag;
    }
}

#endif
