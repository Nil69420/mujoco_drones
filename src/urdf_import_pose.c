#include "urdf_import_pose.h"

#include <math.h>
#include <stddef.h>
#include <stdlib.h>

enum { POSE_VALUES = 6 };

static const double GIMBAL_LIMIT = 1.0 - 1e-12;
static const double IDENTITY_TOLERANCE = 1e-12;

imp_pose_t imp_pose_identity(void) {
    imp_pose_t pose = {{1, 0, 0, 0, 1, 0, 0, 0, 1}, {0, 0, 0}};
    return pose;
}

imp_pose_t imp_pose_from_xyzrpy(const double values[6]) {
    const double sr = sin(values[3]);
    const double cr = cos(values[3]);
    const double sp = sin(values[4]);
    const double cp = cos(values[4]);
    const double sy = sin(values[5]);
    const double cy = cos(values[5]);
    imp_pose_t pose = {{cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr, sy * cp,
                        sy * sp * sr + cy * cr, sy * sp * cr - cy * sr, -sp, cp * sr, cp * cr},
                       {values[0], values[1], values[2]}};
    return pose;
}

int imp_pose_parse(const char *text, imp_pose_t *out) {
    if (text == NULL || out == NULL) {
        return -1;
    }
    double values[POSE_VALUES];
    const char *cursor = text;
    for (int i = 0; i < POSE_VALUES; i++) {
        char *end = NULL;
        values[i] = strtod(cursor, &end);
        if (end == cursor || !isfinite(values[i])) {
            return -1;
        }
        cursor = end;
    }
    *out = imp_pose_from_xyzrpy(values);
    return 0;
}

imp_pose_t imp_pose_multiply(const imp_pose_t *left, const imp_pose_t *right) {
    imp_pose_t out;
    for (size_t row = 0; row < 3; row++) {
        for (size_t col = 0; col < 3; col++) {
            out.rotation[3 * row + col] = left->rotation[3 * row] * right->rotation[col] +
                                          left->rotation[3 * row + 1] * right->rotation[3 + col] +
                                          left->rotation[3 * row + 2] * right->rotation[6 + col];
        }
        out.translation[row] = left->rotation[3 * row] * right->translation[0] +
                               left->rotation[3 * row + 1] * right->translation[1] +
                               left->rotation[3 * row + 2] * right->translation[2] +
                               left->translation[row];
    }
    return out;
}

imp_pose_t imp_pose_inverse(const imp_pose_t *pose) {
    imp_pose_t out;
    for (size_t row = 0; row < 3; row++) {
        for (size_t col = 0; col < 3; col++) {
            out.rotation[3 * row + col] = pose->rotation[3 * col + row];
        }
    }
    for (size_t row = 0; row < 3; row++) {
        out.translation[row] = -(out.rotation[3 * row] * pose->translation[0] +
                                 out.rotation[3 * row + 1] * pose->translation[1] +
                                 out.rotation[3 * row + 2] * pose->translation[2]);
    }
    return out;
}

void imp_pose_to_xyzrpy(const imp_pose_t *pose, double out[6]) {
    const double *rot = pose->rotation;
    out[0] = pose->translation[0];
    out[1] = pose->translation[1];
    out[2] = pose->translation[2];
    const double sine_pitch = -rot[6];
    if (fabs(sine_pitch) < GIMBAL_LIMIT) {
        out[3] = atan2(rot[7], rot[8]);
        out[4] = asin(sine_pitch);
        out[5] = atan2(rot[3], rot[0]);
    } else {
        out[3] = atan2(-rot[5], rot[4]);
        out[4] = sine_pitch > 0.0 ? M_PI_2 : -M_PI_2;
        out[5] = 0.0;
    }
}

int imp_pose_is_identity(const imp_pose_t *pose) {
    const imp_pose_t identity = imp_pose_identity();
    for (int i = 0; i < 9; i++) {
        if (fabs(pose->rotation[i] - identity.rotation[i]) > IDENTITY_TOLERANCE) {
            return 0;
        }
    }
    for (int i = 0; i < 3; i++) {
        if (fabs(pose->translation[i]) > IDENTITY_TOLERANCE) {
            return 0;
        }
    }
    return 1;
}

void imp_pose_unrotate(const imp_pose_t *pose, const double vector[3], double out[3]) {
    for (size_t col = 0; col < 3; col++) {
        out[col] = pose->rotation[col] * vector[0] + pose->rotation[3 + col] * vector[1] +
                   pose->rotation[6 + col] * vector[2];
    }
}
