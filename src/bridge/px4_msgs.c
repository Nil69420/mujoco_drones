#include "bridge/px4_msgs.h"

#include <string.h>

enum {
    HEARTBEAT_BYTES = 9,
    HIL_SENSOR_BYTES = 65,
    HIL_GPS_BYTES = 39,
    ACTUATOR_BYTES = 81,
    MAV_TYPE_QUADROTOR = 2,
    MAV_AUTOPILOT_GENERIC_SIM = 12,
    MAVLINK_VERSION = 3
};

static void put_u16(uint8_t *out, uint16_t value) {
    out[0] = (uint8_t)(value & 0xFFU);
    out[1] = (uint8_t)(value >> 8);
}

static void put_u32(uint8_t *out, uint32_t value) {
    put_u16(out, (uint16_t)(value & 0xFFFFU));
    put_u16(out + 2, (uint16_t)(value >> 16));
}

static void put_u64(uint8_t *out, uint64_t value) {
    put_u32(out, (uint32_t)(value & 0xFFFFFFFFU));
    put_u32(out + 4, (uint32_t)(value >> 32));
}

static void put_f32(uint8_t *out, float value) {
    uint32_t bits = 0;
    memcpy(&bits, &value, sizeof(bits));
    put_u32(out, bits);
}

static uint16_t get_u16(const uint8_t *in) {
    return (uint16_t)((uint16_t)in[0] | (uint16_t)((uint16_t)in[1] << 8));
}

static uint32_t get_u32(const uint8_t *in) {
    return (uint32_t)get_u16(in) | ((uint32_t)get_u16(in + 2) << 16);
}

static uint64_t get_u64(const uint8_t *in) {
    return (uint64_t)get_u32(in) | ((uint64_t)get_u32(in + 4) << 32);
}

static float get_f32(const uint8_t *in) {
    uint32_t bits = get_u32(in);
    float value = 0.0F;
    memcpy(&value, &bits, sizeof(value));
    return value;
}

static size_t finish(uint8_t *out, size_t cap, mav2_frame_t *frame, uint8_t seq, uint8_t sysid,
                     uint8_t compid, uint32_t msgid, size_t len) {
    frame->seq = seq;
    frame->sysid = sysid;
    frame->compid = compid;
    frame->msgid = msgid;
    frame->len = (uint8_t)len;
    return mav2_encode(out, cap, frame);
}

size_t px4_encode_heartbeat(uint8_t *out, size_t cap, uint8_t seq, uint8_t sysid, uint8_t compid) {
    mav2_frame_t frame;
    memset(&frame, 0, sizeof(frame));
    frame.payload[4] = MAV_TYPE_QUADROTOR;
    frame.payload[5] = MAV_AUTOPILOT_GENERIC_SIM;
    frame.payload[6] = PX4_MODE_ARMED;
    frame.payload[8] = MAVLINK_VERSION;
    return finish(out, cap, &frame, seq, sysid, compid, MAV2_MSG_HEARTBEAT, HEARTBEAT_BYTES);
}

size_t px4_encode_hil_sensor(uint8_t *out, size_t cap, uint8_t seq, uint8_t sysid, uint8_t compid,
                             const px4_hil_sensor_t *msg) {
    mav2_frame_t frame;
    memset(&frame, 0, sizeof(frame));
    uint8_t *payload = frame.payload;
    put_u64(payload, msg->time_usec);
    for (size_t i = 0; i < 3; i++) {
        put_f32(payload + 8 + 4 * i, msg->acc_mps2[i]);
        put_f32(payload + 20 + 4 * i, msg->gyro_rps[i]);
        put_f32(payload + 32 + 4 * i, msg->mag_gauss[i]);
    }
    put_f32(payload + 44, msg->abs_pressure_hpa);
    put_f32(payload + 48, msg->diff_pressure_hpa);
    put_f32(payload + 52, msg->pressure_alt_m);
    put_f32(payload + 56, msg->temperature_c);
    put_u32(payload + 60, msg->fields_updated);
    payload[64] = msg->id;
    return finish(out, cap, &frame, seq, sysid, compid, MAV2_MSG_HIL_SENSOR, HIL_SENSOR_BYTES);
}

size_t px4_encode_hil_gps(uint8_t *out, size_t cap, uint8_t seq, uint8_t sysid, uint8_t compid,
                          const px4_hil_gps_t *msg) {
    mav2_frame_t frame;
    memset(&frame, 0, sizeof(frame));
    uint8_t *payload = frame.payload;
    put_u64(payload, msg->time_usec);
    put_u32(payload + 8, (uint32_t)msg->lat_e7);
    put_u32(payload + 12, (uint32_t)msg->lon_e7);
    put_u32(payload + 16, (uint32_t)msg->alt_mm);
    put_u16(payload + 20, msg->eph);
    put_u16(payload + 22, msg->epv);
    put_u16(payload + 24, msg->vel_cms);
    put_u16(payload + 26, (uint16_t)msg->vn_cms);
    put_u16(payload + 28, (uint16_t)msg->ve_cms);
    put_u16(payload + 30, (uint16_t)msg->vd_cms);
    put_u16(payload + 32, msg->cog_cdeg);
    payload[34] = msg->fix_type;
    payload[35] = msg->satellites;
    payload[36] = msg->id;
    put_u16(payload + 37, msg->yaw_cdeg);
    return finish(out, cap, &frame, seq, sysid, compid, MAV2_MSG_HIL_GPS, HIL_GPS_BYTES);
}

int px4_decode_actuator_controls(const mav2_frame_t *frame, px4_actuator_controls_t *out) {
    if (frame == NULL || out == NULL || frame->msgid != MAV2_MSG_HIL_ACTUATOR_CONTROLS) {
        return -1;
    }
    uint8_t padded[ACTUATOR_BYTES];
    memset(padded, 0, sizeof(padded));
    memcpy(padded, frame->payload, frame->len < ACTUATOR_BYTES ? frame->len : ACTUATOR_BYTES);
    out->time_usec = get_u64(padded);
    out->flags = get_u64(padded + 8);
    for (size_t i = 0; i < PX4_ACTUATOR_COUNT; i++) {
        out->controls[i] = get_f32(padded + 16 + 4 * i);
    }
    out->mode = padded[80];
    return 0;
}
