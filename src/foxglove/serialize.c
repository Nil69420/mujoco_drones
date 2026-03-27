#include "foxglove/serialize.h"
#include "sensors/sensor_types.h"

#include <stdio.h>

int fg_serialize_imu(const void *data, char *buf, size_t sz) {
    const sensor_imu_t *m = data;
    return snprintf(buf, sz,
        "{\"timestamp_ns\":%lu,\"seq\":%u,"
        "\"accel\":[%.6f,%.6f,%.6f],"
        "\"gyro\":[%.6f,%.6f,%.6f],"
        "\"mag\":[%.6f,%.6f,%.6f],"
        "\"orientation\":[%.6f,%.6f,%.6f,%.6f]}",
        (unsigned long)m->header.timestamp_ns, m->header.sequence,
        m->accel[0], m->accel[1], m->accel[2],
        m->gyro[0],  m->gyro[1],  m->gyro[2],
        m->mag[0],   m->mag[1],   m->mag[2],
        m->orientation[0], m->orientation[1],
        m->orientation[2], m->orientation[3]);
}

int fg_serialize_gnss(const void *data, char *buf, size_t sz) {
    const sensor_gnss_t *m = data;
    return snprintf(buf, sz,
        "{\"timestamp_ns\":%lu,\"seq\":%u,"
        "\"latitude\":%.8f,\"longitude\":%.8f,\"altitude\":%.4f,"
        "\"velocity\":[%.6f,%.6f,%.6f],"
        "\"fix_type\":%u,\"num_satellites\":%u,\"hdop\":%.2f}",
        (unsigned long)m->header.timestamp_ns, m->header.sequence,
        m->latitude, m->longitude, m->altitude,
        m->velocity[0], m->velocity[1], m->velocity[2],
        m->fix_type, m->num_satellites, m->hdop);
}

int fg_serialize_baro(const void *data, char *buf, size_t sz) {
    const sensor_baro_t *m = data;
    return snprintf(buf, sz,
        "{\"timestamp_ns\":%lu,\"seq\":%u,"
        "\"pressure_pa\":%.2f,\"temperature_c\":%.2f,\"altitude_m\":%.4f}",
        (unsigned long)m->header.timestamp_ns, m->header.sequence,
        m->pressure_pa, m->temperature_c, m->altitude_m);
}

int fg_serialize_lidar(const void *data, char *buf, size_t sz) {
    const sensor_lidar_t *m = data;
    int off = snprintf(buf, sz,
        "{\"timestamp_ns\":%lu,\"seq\":%u,"
        "\"num_rays\":%u,\"angle_min\":%.4f,\"angle_max\":%.4f,"
        "\"range_min\":%.4f,\"range_max\":%.4f,\"ranges\":[",
        (unsigned long)m->header.timestamp_ns, m->header.sequence,
        m->num_rays, (double)m->angle_min, (double)m->angle_max,
        (double)m->range_min, (double)m->range_max);

    for (int i = 0; i < m->num_rays && (size_t)off < sz - 20; i++) {
        if (i > 0) buf[off++] = ',';
        off += snprintf(buf + off, sz - (size_t)off, "%.4f",
                        (double)m->ranges[i]);
    }
    off += snprintf(buf + off, sz - (size_t)off, "]}");
    return off;
}

int fg_serialize_infrared(const void *data, char *buf, size_t sz) {
    const sensor_infrared_t *m = data;
    return snprintf(buf, sz,
        "{\"timestamp_ns\":%lu,\"seq\":%u,"
        "\"range_m\":%.4f,\"range_min\":%.4f,"
        "\"range_max\":%.4f,\"beam_angle\":%.4f}",
        (unsigned long)m->header.timestamp_ns, m->header.sequence,
        (double)m->range_m, (double)m->range_min,
        (double)m->range_max, (double)m->beam_angle);
}

int fg_serialize_camera_meta(const void *data, char *buf, size_t sz) {
    const sensor_camera_meta_t *m = data;
    return snprintf(buf, sz,
        "{\"timestamp_ns\":%lu,\"seq\":%u,"
        "\"width\":%u,\"height\":%u,\"channels\":%u,"
        "\"fov_y\":%.4f,\"rgb_size\":%u,\"depth_size\":%u}",
        (unsigned long)m->header.timestamp_ns, m->header.sequence,
        m->width, m->height, m->channels,
        (double)m->fov_y, m->rgb_size, m->depth_size);
}

const fg_serialize_fn fg_serializers[] = {
    fg_serialize_imu,
    fg_serialize_gnss,
    fg_serialize_baro,
    fg_serialize_lidar,
    fg_serialize_infrared,
    fg_serialize_camera_meta,
};

const int fg_num_serializers =
    (int)(sizeof(fg_serializers) / sizeof(fg_serializers[0]));
