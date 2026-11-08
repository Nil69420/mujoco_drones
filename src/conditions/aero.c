#include "conditions/conditions.h"

#include <math.h>

void aero_drag_force(double air_density, double cd_area_m2, const double wind_world[3],
                     const double velocity_world[3], double force_out[3]) {
    double rel[3];
    for (int i = 0; i < 3; i++) {
        rel[i] = wind_world[i] - velocity_world[i];
    }
    double speed = sqrt(rel[0] * rel[0] + rel[1] * rel[1] + rel[2] * rel[2]);
    double scale = 0.5 * air_density * cd_area_m2 * speed;
    for (int i = 0; i < 3; i++) {
        force_out[i] = scale * rel[i];
    }
}

void wind_ned_to_enu(const double ned[3], double enu[3]) {
    enu[0] = ned[1];
    enu[1] = ned[0];
    enu[2] = -ned[2];
}
