#include "bridge/ap_json.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

static uint16_t read_u16(const uint8_t *p) {
    return (uint16_t)((uint16_t)p[0] | (uint16_t)((uint16_t)p[1] << 8));
}

static uint32_t read_u32(const uint8_t *p) {
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) |
           ((uint32_t)p[3] << 24);
}

int ap_servo_decode(const uint8_t *buf, size_t len, ap_servo_frame_t *out) {
    if (buf == NULL || out == NULL || len < 2) {
        return -1;
    }
    int channels = 0;
    size_t expected = 0;
    uint16_t magic = read_u16(buf);
    if (magic == AP_JSON_MAGIC_16) {
        channels = 16;
        expected = AP_JSON_PACKET_16_BYTES;
    } else if (magic == AP_JSON_MAGIC_32) {
        channels = 32;
        expected = AP_JSON_PACKET_32_BYTES;
    } else {
        return -1;
    }
    if (len != expected) {
        return -1;
    }
    memset(out, 0, sizeof(*out));
    out->frame_rate = read_u16(buf + 2);
    out->frame_count = read_u32(buf + 4);
    out->channels = channels;
    for (size_t i = 0; i < (size_t)channels; i++) {
        out->pwm[i] = read_u16(buf + 8U + 2U * i);
    }
    return 0;
}

typedef struct {
    char *buf;
    size_t cap;
    size_t len;
    bool ok;
} writer_t;

static void put_text(writer_t *w, const char *text) {
    size_t n = strlen(text);
    if (!w->ok || w->len + n >= w->cap) {
        w->ok = false;
        return;
    }
    memcpy(w->buf + w->len, text, n + 1);
    w->len += n;
}

static void put_number(writer_t *w, double v) {
    char tmp[40];
    if (!isfinite(v)) {
        w->ok = false;
        return;
    }
    int n = snprintf(tmp, sizeof(tmp), "%.9g", v);
    if (n <= 0 || (size_t)n >= sizeof(tmp)) {
        w->ok = false;
        return;
    }
    put_text(w, tmp);
}

static void put_timestamp(writer_t *w, double v) {
    char tmp[340];
    if (!isfinite(v) || v < 0.0 || v > 1.0e9) {
        w->ok = false;
        return;
    }
    int n = snprintf(tmp, sizeof(tmp), "\"timestamp\":%.6f", v);
    if (n <= 0 || (size_t)n >= sizeof(tmp)) {
        w->ok = false;
        return;
    }
    put_text(w, tmp);
}

static void put_vec(writer_t *w, const char *key, const double *v, int n) {
    put_text(w, "\"");
    put_text(w, key);
    put_text(w, "\":[");
    for (int i = 0; i < n; i++) {
        if (i > 0) {
            put_text(w, ",");
        }
        put_number(w, v[i]);
    }
    put_text(w, "]");
}

static void put_scalar(writer_t *w, const char *key, double v) {
    put_text(w, "\"");
    put_text(w, key);
    put_text(w, "\":");
    put_number(w, v);
}

static void put_optional(writer_t *w, const ap_state_t *s) {
    for (int i = 0; i < AP_JSON_MAX_RANGEFINDERS; i++) {
        if (s->has_rng[i]) {
            char key[8];
            (void)snprintf(key, sizeof(key), "rng_%d", i + 1);
            put_text(w, ",");
            put_scalar(w, key, s->rng_m[i]);
        }
    }
    if (s->has_wind) {
        put_text(w, ",");
        put_vec(w, "velocity_wind", s->wind_ned_mps, 3);
    }
    if (s->has_airspeed) {
        put_text(w, ",");
        put_scalar(w, "airspeed", s->airspeed_mps);
    }
    if (s->has_battery) {
        put_text(w, ",\"battery\":{");
        put_scalar(w, "voltage", s->battery_v);
        put_text(w, ",");
        put_scalar(w, "current", s->battery_a);
        put_text(w, "}");
    }
}

size_t ap_state_encode(const ap_state_t *state, char *out, size_t cap) {
    if (state == NULL || out == NULL || cap == 0) {
        return 0;
    }
    writer_t w = {out, cap, 0, true};
    out[0] = '\0';
    put_text(&w, "\n{");
    put_timestamp(&w, state->timestamp_s);
    put_text(&w, ",\"imu\":{");
    put_vec(&w, "gyro", state->gyro_rps, 3);
    put_text(&w, ",");
    put_vec(&w, "accel_body", state->accel_body_mps2, 3);
    put_text(&w, "},");
    put_vec(&w, "position", state->position_ned_m, 3);
    put_text(&w, ",");
    put_vec(&w, "quaternion", state->quaternion_wxyz, 4);
    put_text(&w, ",");
    put_vec(&w, "velocity", state->velocity_ned_mps, 3);
    put_optional(&w, state);
    put_text(&w, "}\n");
    if (!w.ok) {
        out[0] = '\0';
        return 0;
    }
    return w.len;
}
