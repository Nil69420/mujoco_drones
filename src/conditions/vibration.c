#include "conditions/conditions.h"

#include <math.h>
#include <string.h>

#define TWO_PI 6.28318530717958647692
#define NYQUIST_GUARD 0.45
#define MAX_MOUNT_SUBSTEP 0.2
enum {
    MAX_SUBSTEPS = 64,
};
int vibration_init(vibration_t *vib, int rotors, int blades, double rated_rpm,
                   double imbalance_accel_ms2, double mount_hz, double mount_zeta,
                   uint64_t seed) {
    if (rotors < 1 || rotors > CONDITIONS_MAX_ROTORS || blades < 1 || rated_rpm <= 0.0 ||
        mount_hz <= 0.0) {
        return -1;
    }
    memset(vib, 0, sizeof(*vib));
    vib->rotors = rotors;
    vib->blades = blades;
    vib->rated_rpm = rated_rpm;
    vib->imbalance_accel_ms2 = imbalance_accel_ms2;
    vib->blade_pass_ratio = 0.3;
    vib->gyro_per_accel = 0.005;
    vib->mount_hz = mount_hz;
    vib->mount_zeta = mount_zeta;
    noise_rng_t rng;
    noise_seed(&rng, seed);
    for (int i = 0; i < rotors; i++) {
        vib->rotor_scale[i] = 0.6 + 0.8 * noise_uniform(&rng);
        vib->phase[i] = TWO_PI * noise_uniform(&rng);
    }
    return 0;
}

static void mount_step(vibration_t *vib, const double excitation[3], double dt) {
    double omega = TWO_PI * vib->mount_hz;
    int steps = (int)ceil(omega * dt / MAX_MOUNT_SUBSTEP);
    if (steps < 1) {
        steps = 1;
    }
    if (steps > MAX_SUBSTEPS) {
        steps = MAX_SUBSTEPS;
    }
    double h = dt / (double)steps;
    for (int step = 0; step < steps; step++) {
        for (int i = 0; i < 3; i++) {
            double acc = omega * omega * (excitation[i] - vib->mount_pos[i]) -
                         2.0 * vib->mount_zeta * omega * vib->mount_vel[i];
            vib->mount_vel[i] += acc * h;
            vib->mount_pos[i] += vib->mount_vel[i] * h;
        }
    }
}

void vibration_step(vibration_t *vib, const double *rotor_rpm, double dt, double accel_out[3],
                    double gyro_out[3]) {
    double excitation[3] = {0.0, 0.0, 0.0};
    for (int i = 0; i < vib->rotors; i++) {
        double rpm = rotor_rpm[i] > 0.0 ? rotor_rpm[i] : 0.0;
        double freq = rpm / 60.0;
        double ratio = rpm / vib->rated_rpm;
        double amp = vib->imbalance_accel_ms2 * ratio * ratio * vib->rotor_scale[i];
        vib->phase[i] = fmod(vib->phase[i] + TWO_PI * freq * dt, TWO_PI);
        if (freq * dt < NYQUIST_GUARD) {
            excitation[0] += amp * cos(vib->phase[i]);
            excitation[1] += amp * sin(vib->phase[i]);
        }
        if ((double)vib->blades * freq * dt < NYQUIST_GUARD) {
            excitation[2] += vib->blade_pass_ratio * amp *
                             cos((double)vib->blades * vib->phase[i]);
        }
    }
    mount_step(vib, excitation, dt);
    accel_out[0] = vib->mount_pos[0];
    accel_out[1] = vib->mount_pos[1];
    accel_out[2] = vib->mount_pos[2];
    gyro_out[0] = vib->gyro_per_accel * vib->mount_pos[1];
    gyro_out[1] = -vib->gyro_per_accel * vib->mount_pos[0];
    gyro_out[2] = 0.3 * vib->gyro_per_accel * vib->mount_pos[2];
}
