#include "conditions/conditions.h"

#include <math.h>

void thermal_init(thermal_t *thermal, double start_temp_c, double time_constant_s,
                  double self_heating_k) {
    thermal->temp_c = start_temp_c;
    thermal->time_constant_s = time_constant_s > 1e-3 ? time_constant_s : 1e-3;
    thermal->self_heating_k = self_heating_k;
}

double thermal_step(thermal_t *thermal, double ambient_c, double dt) {
    double target = ambient_c + thermal->self_heating_k;
    double alpha = 1.0 - exp(-dt / thermal->time_constant_s);
    thermal->temp_c += (target - thermal->temp_c) * alpha;
    return thermal->temp_c;
}
