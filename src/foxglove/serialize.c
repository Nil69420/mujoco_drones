#include "foxglove/serialize.h"
#include "sensors/sensor_types.h"

#include <stdio.h>

int fg_serialize_imu(const void *data, char *buf, size_t sz) {
    const sensor_imu_t *msg = data;
    return snprintf(buf, sz,
        "{\"timestamp_ns\":%lu,\"seq\":%u,"
        "\"accel\":[%.6f,%.6f,%.6f],"
        "\"gyro\":[%.6f,%.6f,%.6f],"
        "\"mag\":[%.6f,%.6f,%.6f],"
        "\"orientation\":[%.6f,%.6f,%.6f,%.6f]}",
        (unsigned long)msg->header.timestamp_ns, msg->header.sequence,
        msg->accel[0], msg->accel[1], msg->accel[2],
        msg->gyro[0],  msg->gyro[1],  msg->gyro[2],
        msg->mag[0],   msg->mag[1],   msg->mag[2],
        msg->orientation[0], msg->orientation[1],
        msg->orientation[2], msg->orientation[3]);
}

int fg_serialize_gnss(const void *data, char *buf, size_t sz) {
    const sensor_gnss_t *msg = data;
    return snprintf(buf, sz,
        "{\"timestamp_ns\":%lu,\"seq\":%u,"
        "\"latitude\":%.8f,\"longitude\":%.8f,\"altitude\":%.4f,"
        "\"velocity\":[%.6f,%.6f,%.6f],"
        "\"fix_type\":%u,\"num_satellites\":%u,\"hdop\":%.2f}",
        (unsigned long)msg->header.timestamp_ns, msg->header.sequence,
        msg->latitude, msg->longitude, msg->altitude,
        msg->velocity[0], msg->velocity[1], msg->velocity[2],
        msg->fix_type, msg->num_satellites, msg->hdop);
}

int fg_serialize_baro(const void *data, char *buf, size_t sz) {
    const sensor_baro_t *msg = data;
    return snprintf(buf, sz,
        "{\"timestamp_ns\":%lu,\"seq\":%u,"
        "\"pressure_pa\":%.2f,\"temperature_c\":%.2f,\"altitude_m\":%.4f}",
        (unsigned long)msg->header.timestamp_ns, msg->header.sequence,
        msg->pressure_pa, msg->temperature_c, msg->altitude_m);
}

int fg_serialize_lidar(const void *data, char *buf, size_t sz) {
    const sensor_lidar_t *msg = data;
    int off = snprintf(buf, sz,
        "{\"timestamp_ns\":%lu,\"seq\":%u,"
        "\"num_rays\":%u,\"angle_min\":%.4f,\"angle_max\":%.4f,"
        "\"range_min\":%.4f,\"range_max\":%.4f,\"ranges\":[",
        (unsigned long)msg->header.timestamp_ns, msg->header.sequence,
        msg->num_rays, (double)msg->angle_min, (double)msg->angle_max,
        (double)msg->range_min, (double)msg->range_max);

    for (int i = 0; i < msg->num_rays && (size_t)off < sz - 20; i++) {
        if (i > 0) buf[off++] = ',';
        off += snprintf(buf + off, sz - (size_t)off, "%.4f",
                        (double)msg->ranges[i]);
    }
    off += snprintf(buf + off, sz - (size_t)off, "]}");
    return off;
}

int fg_serialize_infrared(const void *data, char *buf, size_t sz) {
    const sensor_infrared_t *msg = data;
    return snprintf(buf, sz,
        "{\"timestamp_ns\":%lu,\"seq\":%u,"
        "\"range_m\":%.4f,\"range_min\":%.4f,"
        "\"range_max\":%.4f,\"beam_angle\":%.4f}",
        (unsigned long)msg->header.timestamp_ns, msg->header.sequence,
        (double)msg->range_m, (double)msg->range_min,
        (double)msg->range_max, (double)msg->beam_angle);
}

int fg_serialize_camera_meta(const void *data, char *buf, size_t sz) {
    const sensor_camera_meta_t *msg = data;
    return snprintf(buf, sz,
        "{\"timestamp_ns\":%lu,\"seq\":%u,"
        "\"width\":%u,\"height\":%u,\"channels\":%u,"
        "\"fov_y\":%.4f,\"rgb_size\":%u,\"depth_size\":%u}",
        (unsigned long)msg->header.timestamp_ns, msg->header.sequence,
        msg->width, msg->height, msg->channels,
        (double)msg->fov_y, msg->rgb_size, msg->depth_size);
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
