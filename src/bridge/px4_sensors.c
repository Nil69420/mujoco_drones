#include "bridge/px4_sensors.h"

#include <math.h>
#include <string.h>

enum { GPS_FIX_3D = 3, GPS_SATELLITES = 12, GPS_EPH = 80, GPS_EPV = 120 };

static const double EARTH_RADIUS_M = 6378137.0;
static const double DEG_PER_RAD = 57.29577951308232;
static const double ISA_SEA_LEVEL_PA = 101325.0;
static const double ISA_LAPSE_PER_M = 2.25577e-5;
static const double ISA_EXPONENT = 5.25588;
static const double ISA_TEMP_C = 15.0;
static const double ISA_LAPSE_K_PER_M = 0.0065;
static const double MAX_CDEG = 36000.0;
static const double MAX_I16 = 32767.0;
static const double MAX_U16 = 65535.0;
static const double PASCALS_PER_HPA = 100.0;
static const double MICROTESLA_PER_GAUSS = 100.0;

px4_home_t px4_home_default(void) {
    px4_home_t home;
    memset(&home, 0, sizeof(home));
    home.lat_deg = 47.397742;
    home.lon_deg = 8.545594;
    home.alt_m = 488.0;
    home.field_ned_gauss[0] = 0.21;
    home.field_ned_gauss[1] = 0.0;
    home.field_ned_gauss[2] = 0.43;
    return home;
}

double px4_isa_pressure_hpa(double altitude_m) {
    double base = 1.0 - ISA_LAPSE_PER_M * altitude_m;
    return ISA_SEA_LEVEL_PA * pow(fmax(base, 0.0), ISA_EXPONENT) / 100.0;
}

static void rotate_inverse(const double quat[4], const double in[3], double out[3]) {
    double qw = quat[0];
    double qx = -quat[1];
    double qy = -quat[2];
    double qz = -quat[3];
    double tx = 2.0 * (qy * in[2] - qz * in[1]);
    double ty = 2.0 * (qz * in[0] - qx * in[2]);
    double tz = 2.0 * (qx * in[1] - qy * in[0]);
    out[0] = in[0] + qw * tx + (qy * tz - qz * ty);
    out[1] = in[1] + qw * ty + (qz * tx - qx * tz);
    out[2] = in[2] + qw * tz + (qx * ty - qy * tx);
}

static int16_t clamp_i16(double value) {
    return (int16_t)lround(fmin(MAX_I16, fmax(-MAX_I16, value)));
}

static uint16_t clamp_u16(double value) {
    return (uint16_t)lround(fmin(MAX_U16, fmax(0.0, value)));
}

void px4_sensor_from_state(const ap_state_t *state, const px4_home_t *home, px4_hil_sensor_t *out) {
    memset(out, 0, sizeof(*out));
    double altitude = home->alt_m - state->position_ned_m[2];
    double field_body[3];
    rotate_inverse(state->quaternion_wxyz, home->field_ned_gauss, field_body);
    out->time_usec = (uint64_t)llround(state->timestamp_s * 1.0e6);
    for (int i = 0; i < 3; i++) {
        out->acc_mps2[i] = (float)state->accel_body_mps2[i];
        out->gyro_rps[i] = (float)state->gyro_rps[i];
        out->mag_gauss[i] = (float)field_body[i];
    }
    out->abs_pressure_hpa = (float)px4_isa_pressure_hpa(altitude);
    out->pressure_alt_m = (float)altitude;
    out->temperature_c = (float)(ISA_TEMP_C - ISA_LAPSE_K_PER_M * altitude);
    out->fields_updated = PX4_FIELDS_ALL;
}

void px4_gps_from_state(const ap_state_t *state, const px4_home_t *home, px4_hil_gps_t *out) {
    memset(out, 0, sizeof(*out));
    double lat_rad = home->lat_deg / DEG_PER_RAD;
    double dlat = state->position_ned_m[0] / EARTH_RADIUS_M * DEG_PER_RAD;
    double dlon = state->position_ned_m[1] / (EARTH_RADIUS_M * cos(lat_rad)) * DEG_PER_RAD;
    double vn = state->velocity_ned_mps[0];
    double ve = state->velocity_ned_mps[1];
    double cog = atan2(ve, vn) * DEG_PER_RAD;
    if (cog < 0.0) {
        cog += 360.0;
    }
    out->time_usec = (uint64_t)llround(state->timestamp_s * 1.0e6);
    out->lat_e7 = (int32_t)llround((home->lat_deg + dlat) * 1.0e7);
    out->lon_e7 = (int32_t)llround((home->lon_deg + dlon) * 1.0e7);
    out->alt_mm = (int32_t)llround((home->alt_m - state->position_ned_m[2]) * 1000.0);
    out->eph = GPS_EPH;
    out->epv = GPS_EPV;
    out->vel_cms = clamp_u16(hypot(vn, ve) * 100.0);
    out->vn_cms = clamp_i16(vn * 100.0);
    out->ve_cms = clamp_i16(ve * 100.0);
    out->vd_cms = clamp_i16(state->velocity_ned_mps[2] * 100.0);
    out->cog_cdeg = clamp_u16(fmin(cog * 100.0, MAX_CDEG - 1.0));
    out->fix_type = GPS_FIX_3D;
    out->satellites = GPS_SATELLITES;
}

void px4_sensor_apply_models(px4_hil_sensor_t *sensor, const px4_model_samples_t *samples) {
    uint32_t fields = 0;
    if (samples->imu != NULL && samples->imu_fresh) {
        static const double SIGN[3] = {1.0, -1.0, -1.0};
        for (int i = 0; i < 3; i++) {
            sensor->acc_mps2[i] = (float)(SIGN[i] * samples->imu->accel[i]);
            sensor->gyro_rps[i] = (float)(SIGN[i] * samples->imu->gyro[i]);
            if (samples->mag_modelled) {
                sensor->mag_gauss[i] = (float)(SIGN[i] * samples->imu->mag[i] / MICROTESLA_PER_GAUSS);
            }
        }
    }
    fields |= PX4_FIELDS_ACCEL | PX4_FIELDS_GYRO;
    if (!samples->mag_modelled || (samples->imu != NULL && samples->imu_fresh)) {
        fields |= PX4_FIELDS_MAG;
    }
    if (samples->baro != NULL) {
        if (samples->baro_fresh) {
            sensor->abs_pressure_hpa = (float)(samples->baro->pressure_pa / PASCALS_PER_HPA);
            sensor->temperature_c = (float)samples->baro->temperature_c;
            fields |= PX4_FIELDS_BARO;
        }
    } else {
        fields |= PX4_FIELDS_BARO;
    }
    sensor->fields_updated = fields;
}
