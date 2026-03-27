#ifndef MUJOCO_DRONES_FOXGLOVE_SHA1_H
#define MUJOCO_DRONES_FOXGLOVE_SHA1_H

#include <stddef.h>
#include <stdint.h>

void fg_sha1(const uint8_t *data, size_t len, uint8_t out[20]);

#endif
