#ifndef MUJOCO_DRONES_VEHICLE_KF_AXIS_H
#define MUJOCO_DRONES_VEHICLE_KF_AXIS_H

enum { KF_AXIS_MAX_STATES = 4 };

typedef struct {
    int count;
    double state[KF_AXIS_MAX_STATES];
    double cov[KF_AXIS_MAX_STATES][KF_AXIS_MAX_STATES];
} kf_axis_t;

void kf_axis_init(kf_axis_t *axis, int count, double position, double velocity);
void kf_axis_predict(kf_axis_t *axis, double accel, double dt);
void kf_axis_update(kf_axis_t *axis, const double row[KF_AXIS_MAX_STATES], double innovation,
                    double variance);

#endif
