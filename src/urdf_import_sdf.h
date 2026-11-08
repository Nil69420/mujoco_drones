#ifndef MUJOCO_DRONES_URDF_IMPORT_SDF_H
#define MUJOCO_DRONES_URDF_IMPORT_SDF_H

#include <stddef.h>

// Translate a Gazebo SDF model document (the scoped subset: links with
// inertial/visual/collision, joints revolute/fixed/continuous, poses) into
// a URDF document. Elements outside the subset print named warnings.
// Returns 0 on success (out_urdf malloc'd, NUL-terminated), -1 on failure.
int sdf_to_urdf(const char *sdf_text, size_t sdf_len, char **out_urdf, size_t *out_len);

#endif
