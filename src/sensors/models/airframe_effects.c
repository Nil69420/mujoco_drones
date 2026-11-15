#include "sensors/models/airframe_effects.h"

#include <math.h>
#include <string.h>

static int in_range(double value, double low, double high) {
    return isfinite(value) && value >= low && value <= high;
}

int airframe_effects_init(airframe_effects_t *fx, const airframe_effects_params_t *params,
                          uint64_t seed) {
    if (fx == NULL || params == NULL) {
        return -1;
    }
    if (!in_range(params->accel_thrust_deficit_mps2, 0.0, AIRFRAME_EFFECTS_MAX_DEFICIT_MPS2) ||
        !in_range(params->baro_lag_s, 0.0, AIRFRAME_EFFECTS_MAX_LAG_S) ||
        !in_range(params->camera_drop_probability, 0.0, 1.0) ||
        !in_range(params->camera_time_shift_s, -AIRFRAME_EFFECTS_MAX_SHIFT_S,
                  AIRFRAME_EFFECTS_MAX_SHIFT_S)) {
        return -1;
    }
    memset(fx, 0, sizeof(*fx));
    fx->params = *params;
    noise_seed(&fx->rng, seed);
    return 0;
}

double airframe_effects_accel_z_offset(const airframe_effects_t *fx, double thrust_fraction) {
    double fraction = isfinite(thrust_fraction) ? fmin(fmax(thrust_fraction, 0.0), 1.0) : 0.0;
    return fx->params.accel_thrust_deficit_mps2 * fraction;
}

double airframe_effects_baro(airframe_effects_t *fx, double pressure_pa, double dt) {
    if (!fx->baro_primed || fx->params.baro_lag_s <= 0.0) {
        fx->baro_state = pressure_pa;
        fx->baro_primed = 1;
        return pressure_pa;
    }
    if (!(dt > 0.0)) {
        return fx->baro_state;
    }
    double alpha = 1.0 - exp(-dt / fx->params.baro_lag_s);
    fx->baro_state += alpha * (pressure_pa - fx->baro_state);
    return fx->baro_state;
}

int airframe_effects_camera_dropped(airframe_effects_t *fx) {
    return noise_uniform(&fx->rng) < fx->params.camera_drop_probability;
}

double airframe_effects_camera_stamp(const airframe_effects_t *fx, double capture_time_s) {
    return capture_time_s + fx->params.camera_time_shift_s;
}
