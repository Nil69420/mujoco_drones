#include "bridge/mav2.h"

#include "bridge/mav_dialect.h"

#include <string.h>

enum { CRC_SEED = 0xFFFF, INCOMPAT_SIGNED = 1, OFFSET_LEN = 1, OFFSET_INCOMPAT = 2, OFFSET_SEQ = 4 };

const mav_message_info_t *mav_dialect_find(uint32_t msgid) {
    unsigned low = 0;
    unsigned high = MAV_DIALECT_COUNT;
    while (low < high) {
        unsigned mid = low + (high - low) / 2U;
        if (MAV_DIALECT[mid].msgid == msgid) {
            return &MAV_DIALECT[mid];
        }
        if (MAV_DIALECT[mid].msgid < msgid) {
            low = mid + 1U;
        } else {
            high = mid;
        }
    }
    return NULL;
}

int mav2_crc_extra(uint32_t msgid) {
    const mav_message_info_t *info = mav_dialect_find(msgid);
    return info != NULL ? (int)info->crc_extra : -1;
}

static uint16_t crc_step(uint16_t crc, uint8_t byte) {
    uint8_t tmp = (uint8_t)(byte ^ (uint8_t)(crc & 0xFFU));
    tmp = (uint8_t)(tmp ^ (uint8_t)(tmp << 4));
    return (uint16_t)((crc >> 8) ^ ((uint16_t)tmp << 8) ^ ((uint16_t)tmp << 3) ^ ((uint16_t)tmp >> 4));
}

static uint16_t crc_buffer(const uint8_t *data, size_t len, uint8_t extra) {
    uint16_t crc = CRC_SEED;
    for (size_t i = 0; i < len; i++) {
        crc = crc_step(crc, data[i]);
    }
    return crc_step(crc, extra);
}

size_t mav2_encode(uint8_t *out, size_t cap, const mav2_frame_t *frame) {
    if (out == NULL || frame == NULL) {
        return 0;
    }
    const mav_message_info_t *info = mav_dialect_find(frame->msgid);
    if (info == NULL || frame->msgid > 0xFFFFFFU || frame->len > info->max_len) {
        return 0;
    }
    int extra = info->crc_extra;
    size_t len = frame->len;
    while (len > 1 && frame->payload[len - 1] == 0) {
        len--;
    }
    size_t total = (size_t)MAV2_HEADER_BYTES + len + 2U;
    if (cap < total) {
        return 0;
    }
    out[0] = MAV2_STX;
    out[OFFSET_LEN] = (uint8_t)len;
    out[OFFSET_INCOMPAT] = 0;
    out[3] = 0;
    out[OFFSET_SEQ] = frame->seq;
    out[5] = frame->sysid;
    out[6] = frame->compid;
    out[7] = (uint8_t)(frame->msgid & 0xFFU);
    out[8] = (uint8_t)((frame->msgid >> 8) & 0xFFU);
    out[9] = (uint8_t)((frame->msgid >> 16) & 0xFFU);
    memcpy(out + MAV2_HEADER_BYTES, frame->payload, len);
    uint16_t crc = crc_buffer(out + 1, (size_t)(MAV2_HEADER_BYTES - 1) + len, (uint8_t)extra);
    out[MAV2_HEADER_BYTES + len] = (uint8_t)(crc & 0xFFU);
    out[MAV2_HEADER_BYTES + len + 1] = (uint8_t)(crc >> 8);
    return total;
}

void mav2_parser_reset(mav2_parser_t *parser) {
    parser->have = 0;
    parser->need = 0;
}

static int finish_frame(const mav2_parser_t *parser, mav2_frame_t *out) {
    const uint8_t *buf = parser->buf;
    size_t len = buf[OFFSET_LEN];
    uint32_t msgid = (uint32_t)buf[7] | ((uint32_t)buf[8] << 8) | ((uint32_t)buf[9] << 16);
    const mav_message_info_t *info = mav_dialect_find(msgid);
    if (info == NULL || len > info->max_len) {
        return 0;
    }
    int extra = info->crc_extra;
    uint16_t crc = crc_buffer(buf + 1, (size_t)(MAV2_HEADER_BYTES - 1) + len, (uint8_t)extra);
    uint16_t wire = (uint16_t)((uint16_t)buf[MAV2_HEADER_BYTES + len] |
                               (uint16_t)((uint16_t)buf[MAV2_HEADER_BYTES + len + 1] << 8));
    if (crc != wire) {
        return 0;
    }
    memset(out, 0, sizeof(*out));
    out->seq = buf[OFFSET_SEQ];
    out->sysid = buf[5];
    out->compid = buf[6];
    out->msgid = msgid;
    out->len = (uint8_t)len;
    memcpy(out->payload, buf + MAV2_HEADER_BYTES, len);
    return 1;
}

int mav2_parser_feed(mav2_parser_t *parser, uint8_t byte, mav2_frame_t *out) {
    if (parser->have == 0) {
        if (byte != MAV2_STX) {
            return 0;
        }
        parser->buf[0] = byte;
        parser->have = 1;
        parser->need = MAV2_HEADER_BYTES;
        return 0;
    }
    parser->buf[parser->have++] = byte;
    if (parser->have == MAV2_HEADER_BYTES) {
        size_t signature = (parser->buf[OFFSET_INCOMPAT] & INCOMPAT_SIGNED) != 0 ? MAV2_SIGNATURE_BYTES : 0;
        parser->need = (size_t)MAV2_HEADER_BYTES + parser->buf[OFFSET_LEN] + 2U + signature;
    }
    if (parser->have < parser->need) {
        return 0;
    }
    int ok = finish_frame(parser, out);
    mav2_parser_reset(parser);
    return ok;
}
