#include "bridge/mav2.h"
#include "bridge/mav_dialect.h"
#include "bridge/px4_msgs.h"
#include "test_support.h"

#include <string.h>

/* Reference frames produced by the MAVLink C library generated for PX4, system 1 component 1 */
static const uint8_t GOLDEN_HEARTBEAT[21] = {
    0xfd, 0x09, 0x00, 0x00, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x02, 0x0c, 0x80, 0x00, 0x03, 0xa9, 0x7d,
};

static const uint8_t GOLDEN_SENSOR[74] = {
    0xfd, 0x3e, 0x00, 0x00, 0x02, 0x01, 0x01, 0x6b, 0x00, 0x00, 0xa0, 0x0f,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3f, 0x00, 0x00,
    0x80, 0xbe, 0x0a, 0xe8, 0x1c, 0xc1, 0x0a, 0xd7, 0x23, 0x3c, 0x0a, 0xd7,
    0xa3, 0xbc, 0x8f, 0xc2, 0xf5, 0x3c, 0xcd, 0xcc, 0x4c, 0x3e, 0x00, 0x00,
    0x00, 0x00, 0xcd, 0xcc, 0xcc, 0x3e, 0x00, 0x50, 0x7d, 0x44, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x28, 0x41, 0x00, 0x00, 0xac, 0x41, 0xff, 0x1f,
    0x32, 0xe9,
};

static const uint8_t GOLDEN_GPS[51] = {
    0xfd, 0x27, 0x00, 0x00, 0x03, 0x01, 0x01, 0x71, 0x00, 0x00, 0xa0, 0x86,
    0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x4c, 0x52, 0x40, 0x1c, 0x44, 0xf4,
    0x17, 0x05, 0x40, 0x72, 0x07, 0x00, 0x50, 0x00, 0x78, 0x00, 0x96, 0x00,
    0x64, 0x00, 0xce, 0xff, 0x0a, 0x00, 0x78, 0x69, 0x03, 0x0c, 0x01, 0x34,
    0x12, 0x4d, 0x5f,
};

static const uint8_t GOLDEN_ACTUATORS[93] = {
    0xfd, 0x51, 0x00, 0x00, 0x04, 0x01, 0x01, 0x5d, 0x00, 0x00, 0x40, 0x1f,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x3f, 0x00, 0x00, 0x80, 0x3e, 0x00, 0x00,
    0x40, 0x3f, 0x00, 0x00, 0x80, 0x3f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x81, 0x8b, 0x9b,
};

static const uint8_t GOLDEN_ACTUATORS_SPARSE[32] = {
    0xfd, 0x14, 0x00, 0x00, 0x05, 0x01, 0x01, 0x5d, 0x00, 0x00, 0x28, 0x23,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x3f, 0x74, 0x34,
};

static int feed_all(mav2_parser_t *parser, const uint8_t *bytes, size_t len, mav2_frame_t *out) {
    int frames = 0;
    for (size_t i = 0; i < len; i++) {
        frames += mav2_parser_feed(parser, bytes[i], out);
    }
    return frames;
}

static void test_encoders_match_reference(void) {
    uint8_t buf[MAV2_MAX_FRAME_BYTES];
    size_t len = px4_encode_heartbeat(buf, sizeof(buf), 1, 1, 1);
    CHECK(len == sizeof(GOLDEN_HEARTBEAT) && memcmp(buf, GOLDEN_HEARTBEAT, len) == 0);

    px4_hil_sensor_t sensor;
    memset(&sensor, 0, sizeof(sensor));
    sensor.time_usec = 4000;
    sensor.acc_mps2[0] = 0.5F;
    sensor.acc_mps2[1] = -0.25F;
    sensor.acc_mps2[2] = -9.80665F;
    sensor.gyro_rps[0] = 0.01F;
    sensor.gyro_rps[1] = -0.02F;
    sensor.gyro_rps[2] = 0.03F;
    sensor.mag_gauss[0] = 0.2F;
    sensor.mag_gauss[2] = 0.4F;
    sensor.abs_pressure_hpa = 1013.25F;
    sensor.pressure_alt_m = 10.5F;
    sensor.temperature_c = 21.5F;
    sensor.fields_updated = PX4_FIELDS_ALL;
    len = px4_encode_hil_sensor(buf, sizeof(buf), 2, 1, 1, &sensor);
    CHECK(len == sizeof(GOLDEN_SENSOR) && memcmp(buf, GOLDEN_SENSOR, len) == 0);

    px4_hil_gps_t gps;
    memset(&gps, 0, sizeof(gps));
    gps.time_usec = 100000;
    gps.lat_e7 = 473977420;
    gps.lon_e7 = 85455940;
    gps.alt_mm = 488000;
    gps.eph = 80;
    gps.epv = 120;
    gps.vel_cms = 150;
    gps.vn_cms = 100;
    gps.ve_cms = -50;
    gps.vd_cms = 10;
    gps.cog_cdeg = 27000;
    gps.fix_type = 3;
    gps.satellites = 12;
    gps.id = 1;
    gps.yaw_cdeg = 0x1234;
    len = px4_encode_hil_gps(buf, sizeof(buf), 3, 1, 1, &gps);
    CHECK(len == sizeof(GOLDEN_GPS) && memcmp(buf, GOLDEN_GPS, len) == 0);
}

static void test_parse_reference_actuators(void) {
    mav2_parser_t parser;
    mav2_frame_t frame;
    px4_actuator_controls_t act;
    mav2_parser_reset(&parser);
    CHECK(feed_all(&parser, GOLDEN_ACTUATORS, sizeof(GOLDEN_ACTUATORS), &frame) == 1);
    CHECK(frame.msgid == MAV2_MSG_HIL_ACTUATOR_CONTROLS && frame.sysid == 1 && frame.seq == 4);
    CHECK(px4_decode_actuator_controls(&frame, &act) == 0);
    CHECK(act.time_usec == 8000 && act.flags == 1 && act.mode == 129);
    CHECK_NEAR(act.controls[0], 0.5, 1e-9);
    CHECK_NEAR(act.controls[1], 0.25, 1e-9);
    CHECK_NEAR(act.controls[2], 0.75, 1e-9);
    CHECK_NEAR(act.controls[3], 1.0, 1e-9);
    CHECK_NEAR(act.controls[4], 0.0, 1e-9);

    CHECK(feed_all(&parser, GOLDEN_ACTUATORS_SPARSE, sizeof(GOLDEN_ACTUATORS_SPARSE), &frame) == 1);
    CHECK(frame.len == 20 && px4_decode_actuator_controls(&frame, &act) == 0);
    CHECK(act.time_usec == 9000 && act.mode == 0);
    CHECK_NEAR(act.controls[0], 0.5, 1e-9);
    CHECK_NEAR(act.controls[15], 0.0, 1e-9);
}

static void test_parser_resync_and_rejects(void) {
    mav2_parser_t parser;
    mav2_frame_t frame;
    uint8_t stream[256];
    mav2_parser_reset(&parser);
    size_t pos = 0;
    stream[pos++] = 0x00;
    stream[pos++] = 0x55;
    memcpy(stream + pos, GOLDEN_HEARTBEAT, sizeof(GOLDEN_HEARTBEAT));
    pos += sizeof(GOLDEN_HEARTBEAT);
    memcpy(stream + pos, GOLDEN_ACTUATORS_SPARSE, sizeof(GOLDEN_ACTUATORS_SPARSE));
    pos += sizeof(GOLDEN_ACTUATORS_SPARSE);
    CHECK(feed_all(&parser, stream, pos, &frame) == 2);

    uint8_t bad[sizeof(GOLDEN_ACTUATORS_SPARSE)];
    memcpy(bad, GOLDEN_ACTUATORS_SPARSE, sizeof(bad));
    bad[12] ^= 0x01U;
    CHECK(feed_all(&parser, bad, sizeof(bad), &frame) == 0);
    CHECK(feed_all(&parser, GOLDEN_HEARTBEAT, sizeof(GOLDEN_HEARTBEAT), &frame) == 1);

    uint8_t unknown[13] = {MAV2_STX, 1, 0, 0, 0, 1, 1, 0xE8, 0x03, 0x00, 0x00, 0x00, 0x00};
    CHECK(feed_all(&parser, unknown, sizeof(unknown), &frame) == 0);
    CHECK(feed_all(&parser, GOLDEN_HEARTBEAT, sizeof(GOLDEN_HEARTBEAT), &frame) == 1);
}

static uint16_t test_crc(const uint8_t *data, size_t len, uint8_t extra) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i <= len; i++) {
        uint8_t byte = i < len ? data[i] : extra;
        uint8_t tmp = (uint8_t)(byte ^ (uint8_t)(crc & 0xFFU));
        tmp = (uint8_t)(tmp ^ (uint8_t)(tmp << 4));
        crc = (uint16_t)((crc >> 8) ^ ((uint16_t)tmp << 8) ^ ((uint16_t)tmp << 3) ^ ((uint16_t)tmp >> 4));
    }
    return crc;
}

static void test_signed_frame_is_consumed(void) {
    uint8_t frame_bytes[sizeof(GOLDEN_HEARTBEAT) + MAV2_SIGNATURE_BYTES];
    memcpy(frame_bytes, GOLDEN_HEARTBEAT, sizeof(GOLDEN_HEARTBEAT));
    frame_bytes[2] = 1;
    size_t body = sizeof(GOLDEN_HEARTBEAT) - 2;
    uint16_t crc = test_crc(frame_bytes + 1, body - 1, 50);
    frame_bytes[body] = (uint8_t)(crc & 0xFFU);
    frame_bytes[body + 1] = (uint8_t)(crc >> 8);
    memset(frame_bytes + sizeof(GOLDEN_HEARTBEAT), MAV2_STX, MAV2_SIGNATURE_BYTES);
    mav2_parser_t parser;
    mav2_frame_t frame;
    mav2_parser_reset(&parser);
    CHECK(feed_all(&parser, frame_bytes, sizeof(frame_bytes), &frame) == 1);
    CHECK(frame.msgid == MAV2_MSG_HEARTBEAT);
    CHECK(feed_all(&parser, GOLDEN_HEARTBEAT, sizeof(GOLDEN_HEARTBEAT), &frame) == 1);
}

static void test_encode_limits(void) {
    uint8_t small[8];
    CHECK(px4_encode_heartbeat(small, sizeof(small), 0, 1, 1) == 0);
    mav2_frame_t frame;
    memset(&frame, 0, sizeof(frame));
    frame.msgid = 9999;
    frame.len = 4;
    uint8_t buf[MAV2_MAX_FRAME_BYTES];
    CHECK(mav2_encode(buf, sizeof(buf), &frame) == 0);
    CHECK(mav2_encode(buf, sizeof(buf), NULL) == 0);
    CHECK(mav2_encode(NULL, sizeof(buf), &frame) == 0);
    CHECK(mav2_crc_extra(107) == 108 && mav2_crc_extra(9999) < 0);
    px4_actuator_controls_t act;
    frame.msgid = MAV2_MSG_HEARTBEAT;
    CHECK(px4_decode_actuator_controls(&frame, &act) < 0);
    CHECK(px4_decode_actuator_controls(NULL, &act) < 0);
}

static void test_dialect_table_matches_the_official_generator(void) {
    CHECK(MAV_DIALECT_COUNT == 289U);
    for (unsigned i = 1; i < MAV_DIALECT_COUNT; i++) {
        CHECK(MAV_DIALECT[i - 1].msgid < MAV_DIALECT[i].msgid);
    }
    static const struct {
        uint32_t id;
        uint8_t extra, base, max, ts, tc;
    } OFFICIAL[] = {
        {0, 50, 9, 9, 255, 255},     {69, 243, 11, 18, 10, 255},  {76, 152, 33, 33, 30, 31},
        {323, 78, 147, 147, 0, 1},   {400, 110, 254, 254, 4, 5},  {11002, 234, 179, 180, 4, 5},
        {12915, 94, 249, 249, 0, 1}, {131, 223, 255, 255, 255, 255},
    };
    for (size_t i = 0; i < sizeof(OFFICIAL) / sizeof(OFFICIAL[0]); i++) {
        const mav_message_info_t *info = mav_dialect_find(OFFICIAL[i].id);
        CHECK(info != NULL);
        if (info == NULL) {
            continue;
        }
        CHECK(info->crc_extra == OFFICIAL[i].extra && info->base_len == OFFICIAL[i].base);
        CHECK(info->max_len == OFFICIAL[i].max);
        CHECK(info->target_system == OFFICIAL[i].ts && info->target_component == OFFICIAL[i].tc);
    }
    CHECK(mav_dialect_find(9999) == NULL && mav_dialect_find(0xFFFFFFU) == NULL);
}

static void test_every_dialect_message_round_trips(void) {
    mav2_parser_t parser;
    mav2_parser_reset(&parser);
    for (unsigned i = 0; i < MAV_DIALECT_COUNT; i++) {
        const mav_message_info_t *info = &MAV_DIALECT[i];
        const uint8_t lengths[2] = {info->base_len, info->max_len};
        for (int k = 0; k < 2; k++) {
            mav2_frame_t in;
            memset(&in, 0, sizeof(in));
            in.seq = (uint8_t)i;
            in.sysid = 7;
            in.compid = 190;
            in.msgid = info->msgid;
            in.len = lengths[k];
            for (unsigned byte_index = 0; byte_index < in.len; byte_index++) {
                in.payload[byte_index] = (uint8_t)(((byte_index * 31U + info->msgid) & 0x7FU) | 1U);
            }
            uint8_t wire[MAV2_MAX_FRAME_BYTES];
            size_t n = mav2_encode(wire, sizeof(wire), &in);
            CHECK(n > 0);
            mav2_frame_t out;
            CHECK(feed_all(&parser, wire, n, &out) == 1);
            CHECK(out.msgid == in.msgid && out.len == in.len && out.sysid == 7 && out.compid == 190);
            CHECK(memcmp(out.payload, in.payload, in.len) == 0);
        }
    }
}

static void test_payload_longer_than_the_message_allows_is_refused(void) {
    mav2_frame_t frame;
    memset(&frame, 0, sizeof(frame));
    frame.msgid = MAV2_MSG_HEARTBEAT;
    frame.len = 10;
    memset(frame.payload, 1, sizeof(frame.payload));
    uint8_t wire[MAV2_MAX_FRAME_BYTES];
    CHECK(mav2_encode(wire, sizeof(wire), &frame) == 0);

    uint8_t raw[MAV2_HEADER_BYTES + 10 + 2] = {MAV2_STX, 10, 0, 0, 1, 1, 1, 0, 0, 0};
    memset(raw + MAV2_HEADER_BYTES, 1, 10);
    uint16_t crc = test_crc(raw + 1, (size_t)(MAV2_HEADER_BYTES - 1) + 10U, (uint8_t)mav2_crc_extra(0));
    raw[MAV2_HEADER_BYTES + 10] = (uint8_t)(crc & 0xFFU);
    raw[MAV2_HEADER_BYTES + 11] = (uint8_t)(crc >> 8);
    mav2_parser_t parser;
    mav2_parser_reset(&parser);
    mav2_frame_t out;
    CHECK(feed_all(&parser, raw, sizeof(raw), &out) == 0);
    raw[1] = 9;
    crc = test_crc(raw + 1, (size_t)(MAV2_HEADER_BYTES - 1) + 9U, (uint8_t)mav2_crc_extra(0));
    raw[MAV2_HEADER_BYTES + 9] = (uint8_t)(crc & 0xFFU);
    raw[MAV2_HEADER_BYTES + 10] = (uint8_t)(crc >> 8);
    CHECK(feed_all(&parser, raw, (size_t)MAV2_HEADER_BYTES + 9U + 2U, &out) == 1);
}

static void test_truncated_payload_reads_back_zero_extended(void) {
    mav2_frame_t frame;
    memset(&frame, 0, sizeof(frame));
    frame.msgid = 76;
    frame.len = 33;
    frame.payload[0] = 0x11;
    uint8_t wire[MAV2_MAX_FRAME_BYTES];
    size_t n = mav2_encode(wire, sizeof(wire), &frame);
    CHECK(n == (size_t)MAV2_HEADER_BYTES + 1U + 2U);
    mav2_parser_t parser;
    mav2_parser_reset(&parser);
    mav2_frame_t out;
    memset(&out, 0xAA, sizeof(out));
    CHECK(feed_all(&parser, wire, n, &out) == 1);
    CHECK(out.len == 1 && out.payload[0] == 0x11 && out.payload[1] == 0 && out.payload[32] == 0);
}

int main(void) {
    test_encoders_match_reference();
    test_parse_reference_actuators();
    test_parser_resync_and_rejects();
    test_signed_frame_is_consumed();
    test_encode_limits();
    test_dialect_table_matches_the_official_generator();
    test_every_dialect_message_round_trips();
    test_payload_longer_than_the_message_allows_is_refused();
    test_truncated_payload_reads_back_zero_extended();
    if (g_failures != 0) {
        fprintf(stderr, "%d failures\n", g_failures);
        return 1;
    }
    printf("mav2 tests passed\n");
    return 0;
}
