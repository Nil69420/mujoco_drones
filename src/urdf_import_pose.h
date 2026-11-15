#ifndef MUJOCO_DRONES_URDF_IMPORT_POSE_H
#define MUJOCO_DRONES_URDF_IMPORT_POSE_H

typedef struct {
    double rotation[9];
    double translation[3];
} imp_pose_t;

imp_pose_t imp_pose_identity(void);
imp_pose_t imp_pose_from_xyzrpy(const double values[6]);
int imp_pose_parse(const char *text, imp_pose_t *out);
imp_pose_t imp_pose_multiply(const imp_pose_t *left, const imp_pose_t *right);
imp_pose_t imp_pose_inverse(const imp_pose_t *pose);
void imp_pose_to_xyzrpy(const imp_pose_t *pose, double out[6]);
int imp_pose_is_identity(const imp_pose_t *pose);
void imp_pose_unrotate(const imp_pose_t *pose, const double vector[3], double out[3]);

#endif
