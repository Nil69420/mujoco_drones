#ifndef MUJOCO_DRONES_FOXGLOVE_SERIALIZE_H
#define MUJOCO_DRONES_FOXGLOVE_SERIALIZE_H

#include <stddef.h>

typedef int (*fg_serialize_fn)(const void *data, char *buf, size_t sz);

int fg_serialize_imu(const void *data, char *buf, size_t sz);
int fg_serialize_gnss(const void *data, char *buf, size_t sz);
int fg_serialize_baro(const void *data, char *buf, size_t sz);
int fg_serialize_lidar(const void *data, char *buf, size_t sz);
int fg_serialize_infrared(const void *data, char *buf, size_t sz);
int fg_serialize_camera_meta(const void *data, char *buf, size_t sz);
int fg_serialize_camera_rgb(const void *data, char *buf, size_t sz);

extern const fg_serialize_fn fg_serializers[];
extern const int fg_num_serializers;

#endif
