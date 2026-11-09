#include "bridge/ap_frames.h"

#include <math.h>
#include <string.h>

enum { RM_XX = 0, RM_XY, RM_XZ, RM_YX, RM_YY, RM_YZ, RM_ZX, RM_ZY, RM_ZZ };

void ap_rotation_to_quaternion(const double rotation[9], double quat_wxyz[4]) {
    double trace = rotation[RM_XX] + rotation[RM_YY] + rotation[RM_ZZ];
    double qw = 1.0;
    double qx = 0.0;
    double qy = 0.0;
    double qz = 0.0;
    if (trace > 0.0) {
        double scale = sqrt(trace + 1.0) * 2.0;
        qw = 0.25 * scale;
        qx = (rotation[RM_ZY] - rotation[RM_YZ]) / scale;
        qy = (rotation[RM_XZ] - rotation[RM_ZX]) / scale;
        qz = (rotation[RM_YX] - rotation[RM_XY]) / scale;
    } else if (rotation[RM_XX] > rotation[RM_YY] && rotation[RM_XX] > rotation[RM_ZZ]) {
        double scale = sqrt(1.0 + rotation[RM_XX] - rotation[RM_YY] - rotation[RM_ZZ]) * 2.0;
        qw = (rotation[RM_ZY] - rotation[RM_YZ]) / scale;
        qx = 0.25 * scale;
        qy = (rotation[RM_XY] + rotation[RM_YX]) / scale;
        qz = (rotation[RM_XZ] + rotation[RM_ZX]) / scale;
    } else if (rotation[RM_YY] > rotation[RM_ZZ]) {
        double scale = sqrt(1.0 + rotation[RM_YY] - rotation[RM_XX] - rotation[RM_ZZ]) * 2.0;
        qw = (rotation[RM_XZ] - rotation[RM_ZX]) / scale;
        qx = (rotation[RM_XY] + rotation[RM_YX]) / scale;
        qy = 0.25 * scale;
        qz = (rotation[RM_YZ] + rotation[RM_ZY]) / scale;
    } else {
        double scale = sqrt(1.0 + rotation[RM_ZZ] - rotation[RM_XX] - rotation[RM_YY]) * 2.0;
        qw = (rotation[RM_YX] - rotation[RM_XY]) / scale;
        qx = (rotation[RM_XZ] + rotation[RM_ZX]) / scale;
        qy = (rotation[RM_YZ] + rotation[RM_ZY]) / scale;
        qz = 0.25 * scale;
    }
    double norm = sqrt(qw * qw + qx * qx + qy * qy + qz * qz);
    if (qw < 0.0) {
        norm = -norm;
    }
    quat_wxyz[0] = qw / norm;
    quat_wxyz[1] = qx / norm;
    quat_wxyz[2] = qy / norm;
    quat_wxyz[3] = qz / norm;
}

static void enu_to_ned(const double enu[3], double ned[3]) {
    ned[0] = enu[1];
    ned[1] = enu[0];
    ned[2] = -enu[2];
}

static void flu_to_frd(const double flu[3], double frd[3]) {
    frd[0] = flu[0];
    frd[1] = -flu[1];
    frd[2] = -flu[2];
}

void ap_truth_to_state(const ap_truth_t *truth, ap_state_t *out) {
    double rel[3];
    for (int i = 0; i < 3; i++) {
        rel[i] = truth->position_enu_m[i] - truth->origin_enu_m[i];
    }
    out->timestamp_s = truth->time_s;
    enu_to_ned(rel, out->position_ned_m);
    enu_to_ned(truth->velocity_enu_mps, out->velocity_ned_mps);
    flu_to_frd(truth->gyro_flu_rps, out->gyro_rps);
    flu_to_frd(truth->accel_flu_mps2, out->accel_body_mps2);

    const double *rot = truth->rotation_body_to_world;
    double ned_from_enu[9] = {0, 1, 0, 1, 0, 0, 0, 0, -1};
    double frd_to_flu[3] = {1.0, -1.0, -1.0};
    double mid[9];
    double result[9];
    for (int row = 0; row < 3; row++) {
        for (int col = 0; col < 3; col++) {
            double sum = 0.0;
            for (int k = 0; k < 3; k++) {
                sum += ned_from_enu[row * 3 + k] * rot[k * 3 + col];
            }
            mid[row * 3 + col] = sum;
        }
    }
    for (int row = 0; row < 3; row++) {
        for (int col = 0; col < 3; col++) {
            result[row * 3 + col] = mid[row * 3 + col] * frd_to_flu[col];
        }
    }
    ap_rotation_to_quaternion(result, out->quaternion_wxyz);
}
