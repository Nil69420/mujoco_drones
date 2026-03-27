#ifndef MUJOCO_DRONES_FOXGLOVE_BASE64_H
#define MUJOCO_DRONES_FOXGLOVE_BASE64_H

#include <stddef.h>
#include <stdint.h>

size_t fg_base64_encode(const uint8_t *in, size_t len, char *out);

#endif
