#include "bridge/bf_packets.h"

#include <math.h>
#include <string.h>

static const double EARTH_RADIUS_M = 6378137.0;
static const double DEG_PER_RAD = 57.29577951308232;

static void put_u64(uint8_t *out, uint64_t value) {
    for (int i = 0; i < 8; i++) {
        out[i] = (uint8_t)((value >> (8 * i)) & 0xFFU);
    }
}

static void put_f64(uint8_t *out, double value) {
    uint64_t bits = 0;
    memcpy(&bits, &value, sizeof(bits));
    put_u64(out, bits);
}

static float get_f32(const uint8_t *in) {
    uint32_t bits = (uint32_t)in[0] | ((uint32_t)in[1] << 8) | ((uint32_t)in[2] << 16) |
                    ((uint32_t)in[3] << 24);
    float value = 0.0F;
    memcpy(&value, &bits, sizeof(value));
    return value;
}

void bf_fdm_from_truth(const ap_truth_t *truth, const px4_home_t *home, bf_fdm_t *out) {
    memset(out, 0, sizeof(*out));
    double north = truth->position_enu_m[1] - truth->origin_enu_m[1];
    double east = truth->position_enu_m[0] - truth->origin_enu_m[0];
    double up = truth->position_enu_m[2] - truth->origin_enu_m[2];
    double lat = home->lat_deg + north / EARTH_RADIUS_M * DEG_PER_RAD;
    double lon = home->lon_deg +
                 east / (EARTH_RADIUS_M * cos(home->lat_deg / DEG_PER_RAD)) * DEG_PER_RAD;
    double rotated[9];
    static const double SIGN[3] = {1.0, -1.0, -1.0};
    out->timestamp_s = truth->time_s;
    for (int i = 0; i < 3; i++) {
        out->gyro_frd_rps[i] = SIGN[i] * truth->gyro_flu_rps[i];
        out->accel_frd_mps2[i] = SIGN[i] * truth->accel_flu_mps2[i];
        out->velocity_enu_mps[i] = truth->velocity_enu_mps[i];
    }
    for (int row = 0; row < 3; row++) {
        for (int col = 0; col < 3; col++) {
            rotated[row * 3 + col] =
                SIGN[row] * truth->rotation_body_to_world[row * 3 + col] * SIGN[col];
        }
    }
    ap_rotation_to_quaternion(rotated, out->quat_wxyz);
    out->lat_deg = 2.0 * home->lat_deg - lat;
    out->lon_deg = 2.0 * home->lon_deg - lon;
    out->alt_m = home->alt_m + up;
    out->pressure_pa = px4_isa_pressure_hpa(out->alt_m) * 100.0;
}

size_t bf_fdm_encode(const bf_fdm_t *fdm, uint8_t *out, size_t cap) {
    if (fdm == NULL || out == NULL || cap < BF_FDM_BYTES) {
        return 0;
    }
    size_t at = 0;
    put_f64(out + at, fdm->timestamp_s);
    at += 8;
    for (int i = 0; i < 3; i++, at += 8) {
        put_f64(out + at, fdm->gyro_frd_rps[i]);
    }
    for (int i = 0; i < 3; i++, at += 8) {
        put_f64(out + at, fdm->accel_frd_mps2[i]);
    }
    for (int i = 0; i < 4; i++, at += 8) {
        put_f64(out + at, fdm->quat_wxyz[i]);
    }
    for (int i = 0; i < 3; i++, at += 8) {
        put_f64(out + at, fdm->velocity_enu_mps[i]);
    }
    put_f64(out + at, fdm->lon_deg);
    at += 8;
    put_f64(out + at, fdm->lat_deg);
    at += 8;
    put_f64(out + at, fdm->alt_m);
    at += 8;
    put_f64(out + at, fdm->pressure_pa);
    return BF_FDM_BYTES;
}

size_t bf_rc_encode(double timestamp_s, const uint16_t channels[BF_RC_CHANNELS], uint8_t *out,
                    size_t cap) {
    if (channels == NULL || out == NULL || cap < BF_RC_BYTES) {
        return 0;
    }
    put_f64(out, timestamp_s);
    for (int i = 0; i < BF_RC_CHANNELS; i++) {
        out[8U + 2U * (size_t)i] = (uint8_t)(channels[i] & 0xFFU);
        out[9U + 2U * (size_t)i] = (uint8_t)(channels[i] >> 8);
    }
    return BF_RC_BYTES;
}

int bf_motors_decode(const uint8_t *data, size_t len, float motors[BF_MOTOR_COUNT]) {
    if (data == NULL || motors == NULL || len != BF_MOTOR_BYTES) {
        return -1;
    }
    for (int i = 0; i < BF_MOTOR_COUNT; i++) {
        motors[i] = get_f32(data + 4U * (size_t)i);
    }
    return 0;
}
