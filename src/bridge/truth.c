#include "bridge/truth.h"

#include <string.h>

void bridge_read_truth(const vehicle_t *vehicle, const mjModel *model, mjData *data,
                       const double origin_enu[3], ap_truth_t *truth) {
    int body = vehicle->base_body_id;
    mjtNum velocity[6];
    memset(truth, 0, sizeof(*truth));
    mj_forward(model, data);
    mj_objectVelocity(model, data, mjOBJ_BODY, body, velocity, 0);
    truth->time_s = data->time;
    for (int i = 0; i < 3; i++) {
        truth->position_enu_m[i] = data->xpos[3 * body + i];
        truth->origin_enu_m[i] = origin_enu[i];
        truth->velocity_enu_mps[i] = velocity[3 + i];
        truth->gyro_flu_rps[i] = data->sensordata[vehicle->sensors.adr_gyro + i];
        truth->accel_flu_mps2[i] = data->sensordata[vehicle->sensors.adr_accel + i];
    }
    for (int i = 0; i < 9; i++) {
        truth->rotation_body_to_world[i] = data->xmat[9 * body + i];
    }
}

void bridge_apply_command(const vehicle_t *vehicle, const mjModel *model, mjData *data,
                          const actuator_command_t *command) {
    for (int i = 0; i < command->count; i++) {
        int index = vehicle->actuator_index[i];
        if (index >= 0 && index < model->nu) {
            data->ctrl[index] = command->value[i];
        }
    }
}
