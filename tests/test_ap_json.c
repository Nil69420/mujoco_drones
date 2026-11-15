#include "bridge/ap_json.h"
#include "test_support.h"

#include <stdlib.h>
#include <string.h>

typedef struct {
    const char *section;
    const char *key;
    int count;
    bool required;
} key_t_;

static const key_t_ KEYS[] = {
    {"", "timestamp", 1, true},      {"imu", "gyro", 3, true},
    {"imu", "accel_body", 3, true},  {"", "position", 3, false},
    {"", "quaternion", 4, false},    {"", "velocity", 3, true},
    {"", "rng_1", 1, false},         {"", "rng_2", 1, false},
    {"", "velocity_wind", 3, false}, {"", "airspeed", 1, false},
    {"battery", "voltage", 1, false}, {"battery", "current", 1, false},
};

static int model_parse(const char *json, double vals[][4], bool seen[]) {
    for (size_t i = 0; i < sizeof(KEYS) / sizeof(KEYS[0]); i++) {
        const char *p = strstr(json, KEYS[i].section);
        if (p == NULL) {
            if (KEYS[i].required) {
                return -1;
            }
            continue;
        }
        p += strlen(KEYS[i].section) + 1;
        p = strstr(p, KEYS[i].key);
        if (p == NULL) {
            if (KEYS[i].required) {
                return -1;
            }
            continue;
        }
        seen[i] = true;
        p += strlen(KEYS[i].key) + 2;
        if (KEYS[i].count > 1) {
            if (*p != '[') {
                return -1;
            }
            p++;
        }
        for (int c = 0; c < KEYS[i].count; c++) {
            char *end = NULL;
            vals[i][c] = strtod(p, &end);
            if (end == p) {
                return -1;
            }
            p = end;
            if (c + 1 < KEYS[i].count) {
                if (*p != ',') {
                    return -1;
                }
                p++;
            }
        }
    }
    return 0;
}

static void le16(uint8_t *p, uint16_t v) {
    p[0] = (uint8_t)(v & 0xFFu);
    p[1] = (uint8_t)(v >> 8);
}

static void le32(uint8_t *p, uint32_t v) {
    le16(p, (uint16_t)(v & 0xFFFFu));
    le16(p + 2, (uint16_t)(v >> 16));
}

static void test_decode(void) {
    uint8_t buf[AP_JSON_PACKET_32_BYTES];
    ap_servo_frame_t f;
    memset(buf, 0, sizeof(buf));
    le16(buf, AP_JSON_MAGIC_16);
    le16(buf + 2, 400);
    le32(buf + 4, 123456u);
    for (int i = 0; i < 16; i++) {
        le16(buf + 8 + 2 * i, (uint16_t)(1000 + i * 10));
    }
    CHECK(ap_servo_decode(buf, AP_JSON_PACKET_16_BYTES, &f) == 0);
    CHECK(f.channels == 16 && f.frame_rate == 400 && f.frame_count == 123456u);
    CHECK(f.pwm[0] == 1000 && f.pwm[15] == 1150);
    CHECK(ap_servo_decode(buf, AP_JSON_PACKET_16_BYTES - 1, &f) < 0);
    CHECK(ap_servo_decode(buf, AP_JSON_PACKET_32_BYTES, &f) < 0);
    le16(buf, 1234);
    CHECK(ap_servo_decode(buf, AP_JSON_PACKET_16_BYTES, &f) < 0);
    le16(buf, AP_JSON_MAGIC_32);
    for (int i = 0; i < 32; i++) {
        le16(buf + 8 + 2 * i, (uint16_t)(1500 + i));
    }
    CHECK(ap_servo_decode(buf, AP_JSON_PACKET_32_BYTES, &f) == 0);
    CHECK(f.channels == 32 && f.pwm[31] == 1531);
    CHECK(ap_servo_decode(NULL, 40, &f) < 0);
    CHECK(ap_servo_decode(buf, 1, &f) < 0);
}

static ap_state_t sample_state(void) {
    ap_state_t s;
    memset(&s, 0, sizeof(s));
    s.timestamp_s = 12345.678901;
    s.gyro_rps[0] = 0.01;
    s.gyro_rps[1] = -0.02;
    s.gyro_rps[2] = 0.5;
    s.accel_body_mps2[2] = -9.80665;
    s.position_ned_m[0] = 1234.5678;
    s.position_ned_m[2] = -10.25;
    s.velocity_ned_mps[1] = 3.5;
    s.quaternion_wxyz[0] = 1.0;
    return s;
}

static void test_encode_required(void) {
    ap_state_t s = sample_state();
    char out[1024];
    size_t n = ap_state_encode(&s, out, sizeof(out));
    CHECK(n > 0 && out[0] == '\n' && out[n - 1] == '\n' && strlen(out) == n);
    double vals[16][4];
    bool seen[16];
    memset(vals, 0, sizeof(vals));
    memset(seen, 0, sizeof(seen));
    CHECK(model_parse(out + 1, vals, seen) == 0);
    CHECK_NEAR(vals[0][0], 12345.678901, 1e-6);
    CHECK_NEAR(vals[1][2], 0.5, 1e-9);
    CHECK_NEAR(vals[2][2], -9.80665, 1e-9);
    CHECK_NEAR(vals[3][0], 1234.5678, 1e-6);
    CHECK_NEAR(vals[3][2], -10.25, 1e-9);
    CHECK(seen[4]);
    CHECK_NEAR(vals[4][0], 1.0, 1e-12);
    CHECK_NEAR(vals[5][1], 3.5, 1e-9);
    CHECK(!seen[6] && !seen[8] && !seen[9] && !seen[10]);
}

static void test_encode_optional(void) {
    ap_state_t s = sample_state();
    s.has_rng[0] = true;
    s.rng_m[0] = 4.25;
    s.has_rng[1] = true;
    s.rng_m[1] = 12.5;
    s.has_wind = true;
    s.wind_ned_mps[0] = 3.2;
    s.wind_ned_mps[2] = -0.7;
    s.has_airspeed = true;
    s.airspeed_mps = 17.5;
    s.has_battery = true;
    s.battery_v = 50.39;
    s.battery_a = 64.01;
    s.velocity_ned_mps[0] = 1.5;
    char out[1024];
    CHECK(ap_state_encode(&s, out, sizeof(out)) > 0);
    double vals[16][4];
    bool seen[16];
    memset(vals, 0, sizeof(vals));
    memset(seen, 0, sizeof(seen));
    CHECK(model_parse(out + 1, vals, seen) == 0);
    CHECK_NEAR(vals[5][0], 1.5, 1e-9);
    CHECK_NEAR(vals[6][0], 4.25, 1e-9);
    CHECK_NEAR(vals[7][0], 12.5, 1e-9);
    CHECK_NEAR(vals[8][0], 3.2, 1e-9);
    CHECK_NEAR(vals[8][2], -0.7, 1e-9);
    CHECK_NEAR(vals[9][0], 17.5, 1e-9);
    CHECK_NEAR(vals[10][0], 50.39, 1e-9);
    CHECK_NEAR(vals[11][0], 64.01, 1e-9);
    CHECK(strstr(out, "\"velocity\"") < strstr(out, "\"velocity_wind\""));
}

static void test_encode_rejects_bad_input(void) {
    ap_state_t s = sample_state();
    char out[1024];
    char tiny[32];
    CHECK(ap_state_encode(&s, tiny, sizeof(tiny)) == 0 && tiny[0] == '\0');
    CHECK(ap_state_encode(NULL, out, sizeof(out)) == 0);
    CHECK(ap_state_encode(&s, NULL, 10) == 0);
    CHECK(ap_state_encode(&s, out, 0) == 0);
    s.gyro_rps[1] = NAN;
    CHECK(ap_state_encode(&s, out, sizeof(out)) == 0 && out[0] == '\0');
    s = sample_state();
    s.position_ned_m[0] = INFINITY;
    CHECK(ap_state_encode(&s, out, sizeof(out)) == 0);
    s = sample_state();
    s.timestamp_s = -1.0;
    CHECK(ap_state_encode(&s, out, sizeof(out)) == 0);
}

int main(void) {
    test_decode();
    test_encode_required();
    test_encode_optional();
    test_encode_rejects_bad_input();
    if (g_failures != 0) {
        fprintf(stderr, "%d failures\n", g_failures);
        return 1;
    }
    printf("ap_json tests passed\n");
    return 0;
}
