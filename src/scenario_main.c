#include "scenario.h"
#include "vehicle/vehicle.h"
#include "vehicle/assembly_config.h"
#include "vehicle/airframe.h"
#include "vehicle/controller_ops.h"
#include "vehicle/estimator_ops.h"
#include "vehicle/localization_ops.h"
#include "environments/environment_ops.h"
#include "transport/transport_renoir.h"
#include "setpoint.h"

#include <mujoco/mujoco.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

static int compose_scene(scenario_t *scenario, mjModel **out_model,
                         mjSpec **out_scene, char *error, int error_sz) {
    const environment_ops_t *env =
        environment_registry_find(scenario->environment);
    if (!env) {
        snprintf(error, (size_t)error_sz,
                 "environment '%s' not registered", scenario->environment);
        return -1;
    }

    mjSpec *scene = mj_parseXML("model/environments/base_scene.xml",
                                NULL, error, error_sz);
    if (!scene) return -1;

    mjsBody *world = mjs_findBody(scene, "world");
    if (!world) {
        snprintf(error, (size_t)error_sz, "world body not found in base scene");
        mj_deleteSpec(scene);
        return -1;
    }

    if (env->generate(scene, scenario->environment_seed) != 0) {
        snprintf(error, (size_t)error_sz, "environment '%s' generation failed",
                 scenario->environment);
        mj_deleteSpec(scene);
        return -1;
    }

    for (int i = 0; i < scenario->vehicle_count; i++) {
        const vehicle_assembly_spec_t *as = &scenario->vehicles[i];
        const airframe_spec_t *af = airframe_registry_find(as->airframe);
        if (!af) {
            snprintf(error, (size_t)error_sz,
                     "vehicle %d: airframe '%s' not registered",
                     i, as->airframe);
            mj_deleteSpec(scene);
            return -1;
        }

        char path[256];
        snprintf(path, sizeof(path), "model/airframes/%s", af->mjcf_path);
        mjSpec *air = mj_parseXML(path, NULL, error, error_sz);
        if (!air) {
            mj_deleteSpec(scene);
            return -1;
        }

        mjsBody *root = mjs_findBody(air, af->base_body);
        if (!root) {
            snprintf(error, (size_t)error_sz,
                     "airframe '%s': body '%s' not found",
                     af->name, af->base_body);
            mj_deleteSpec(air);
            mj_deleteSpec(scene);
            return -1;
        }

        char prefix[VEHICLE_MAX_PREFIX_LEN];
        snprintf(prefix, sizeof(prefix), "%.*s_",
                       (int)(sizeof(prefix) - 2), as->id);

        mjsFrame *mount = mjs_addFrame(world, NULL);
        mount->pos[0] = as->spawn_pos[0];
        mount->pos[1] = as->spawn_pos[1];
        mount->pos[2] = as->spawn_pos[2];
        double half_yaw = 0.5 * as->spawn_yaw;
        mount->quat[0] = cos(half_yaw);
        mount->quat[3] = sin(half_yaw);

        mjsElement *att = mjs_attach(mount->element, root->element,
                                     prefix, "");
        if (!att) {
            snprintf(error, (size_t)error_sz,
                     "vehicle %d: mjs_attach failed: %s",
                     i, mjs_getError(scene));
            mj_deleteSpec(air);
            mj_deleteSpec(scene);
            return -1;
        }
        mj_deleteSpec(air);
    }

    mjModel *model = mj_compile(scene, NULL);
    if (!model) {
        snprintf(error, (size_t)error_sz, "scene compile failed: %s",
                 mjs_getError(scene));
        mj_deleteSpec(scene);
        return -1;
    }

    *out_model = model;
    *out_scene = scene;
    return 0;
}

static int build_vehicles(scenario_t *scenario, const mjModel *model,
                          transport_t *tp, vehicle_t *vehicles) {
    for (int i = 0; i < scenario->vehicle_count; i++) {
        const vehicle_assembly_spec_t *as = &scenario->vehicles[i];
        vehicle_t *veh = &vehicles[i];
        memset(veh, 0, sizeof(*veh));

        snprintf(veh->prefix, sizeof(veh->prefix), "%.*s_",
                          (int)(sizeof(veh->prefix) - 2), as->id);
        veh->airframe = airframe_registry_find(as->airframe);
        veh->controller.ops = controller_registry_find(as->controller);
        veh->estimator.ops = estimator_registry_find(as->estimator);
        veh->localization.ops = localization_registry_find(as->localization);

        if (!veh->airframe || !veh->controller.ops || !veh->estimator.ops ||
            !veh->localization.ops) {
            fprintf(stderr, "[scenario] vehicle %d: unresolved assembly "
                            "component (airframe=%s controller=%s "
                            "estimator=%s localization=%s)\n",
                    i, as->airframe, as->controller,
                    as->estimator, as->localization);
            return -1;
        }

        veh->target.x = as->spawn_pos[0];
        veh->target.y = as->spawn_pos[1];
        veh->target.z = as->spawn_pos[2];
        veh->target.yaw = as->spawn_yaw;

        if (vehicle_resolve(veh, model, tp) != 0) {
            fprintf(stderr, "[scenario] vehicle %d resolve failed\n", i);
            return -1;
        }
    }
    return 0;
}

static void destroy_vehicles(scenario_t *scenario, vehicle_t *vehicles) {
    for (int i = 0; i < scenario->vehicle_count; i++) {
        vehicle_cleanup(&vehicles[i]);
    }
}

static void print_vehicle_state(scenario_t *scenario, vehicle_t *vehicles,
                                const mjModel *model, mjData *data) {
    for (int i = 0; i < scenario->vehicle_count; i++) {
        vehicle_t *veh = &vehicles[i];
        char prefix_buf[VEHICLE_MAX_PREFIX_LEN];
        snprintf(prefix_buf, sizeof(prefix_buf), "%.*s",
                       (int)(sizeof(prefix_buf) - 1), veh->prefix);
        char body_name[64] = "";
        if (veh->airframe) {
            snprintf(body_name, sizeof(body_name), "%s%.*s",
                     prefix_buf,
                     (int)(sizeof(body_name) - sizeof(prefix_buf)),
                     veh->airframe->base_body);
        }
        int bid = mj_name2id(model, mjOBJ_BODY, body_name);
        const double *pos_ptr = bid >= 0
                                ? data->xpos + (ptrdiff_t)3 * bid : NULL;
        printf("t=%6.2f [%s] pos=(%7.2f %7.2f %7.2f) "
               "tgt=(%5.1f %5.1f %5.1f)\n",
               data->time, veh->prefix,
               pos_ptr ? pos_ptr[0] : 0.0,
               pos_ptr ? pos_ptr[1] : 0.0,
               pos_ptr ? pos_ptr[2] : 0.0,
               veh->target.x, veh->target.y, veh->target.z);
    }
}

static int run_scenario(scenario_t *scenario, vehicle_t *vehicles,
                        const mjModel *model, mjData *data, double dt) {
    int print_every = (int)(0.2 / dt);
    int step = 0;

    while (data->time < scenario->duration_s) {
        for (int i = 0; i < scenario->vehicle_count; i++) {
            sensor_update(&vehicles[i].sensors, model, data,
                          &vehicles[i].target);
        }
        mj_step(model, data);
        for (int i = 0; i < scenario->vehicle_count; i++) {
            vehicle_step(&vehicles[i], model, data, dt);
        }

        if (++step % print_every == 0) {
            print_vehicle_state(scenario, vehicles, model, data);
        }
    }

    printf("\n[scenario] final states:\n");
    for (int i = 0; i < scenario->vehicle_count; i++) {
        vehicle_t *veh = &vehicles[i];
        char prefix_buf[VEHICLE_MAX_PREFIX_LEN];
        snprintf(prefix_buf, sizeof(prefix_buf), "%.*s",
                       (int)(sizeof(prefix_buf) - 1), veh->prefix);
        char body_name[64] = "";
        if (veh->airframe) {
            snprintf(body_name, sizeof(body_name), "%s%.*s",
                     prefix_buf,
                     (int)(sizeof(body_name) - sizeof(prefix_buf)),
                     veh->airframe->base_body);
        }
        int bid = mj_name2id(model, mjOBJ_BODY, body_name);
        const double *pos_ptr = bid >= 0
                                ? data->xpos + (ptrdiff_t)3 * bid : NULL;
        printf("  [%s] pos=(%.3f %.3f %.3f)\n",
               veh->prefix,
               pos_ptr ? pos_ptr[0] : 0.0,
               pos_ptr ? pos_ptr[1] : 0.0,
               pos_ptr ? pos_ptr[2] : 0.0);
    }
    return 0;
}

int scenario_main_run(const char *path, bool headless, double duration_override) {
    (void)headless;

    scenario_t scenario;
    memset(&scenario, 0, sizeof(scenario));
    if (scenario_load(path, &scenario) != 0) return 1;
    if (duration_override > 0.0) {
        scenario.duration_s = duration_override;
    }

    mjModel *model = NULL;
    mjSpec *scene = NULL;
    char error[1024] = "";
    if (compose_scene(&scenario, &model, &scene, error, sizeof(error)) != 0) {
        fprintf(stderr, "[scenario] ERROR: %s\n", error);
        return 1;
    }

    mjData *data = mj_makeData(model);
    for (int i = 0; i < scenario.vehicle_count; i++) {
        if (scenario.vehicles[i].spawn_speed > 0.0) {
            data->qvel[(ptrdiff_t)6 * i] = scenario.vehicles[i].spawn_speed;
        }
    }
    mj_forward(model, data);

    transport_t tp;
    if (transport_renoir_create(&tp) != 0 || transport_init(&tp) != 0) {
        fprintf(stderr, "[scenario] ERROR: transport init failed\n");
        mj_deleteData(data);
        mj_deleteModel(model);
        mj_deleteSpec(scene);
        return 1;
    }

    vehicle_t vehicles[SCENARIO_MAX_VEHICLES];
    memset(vehicles, 0, sizeof(vehicles));
    if (build_vehicles(&scenario, model, &tp, vehicles) != 0) {
        transport_shutdown(&tp);
        mj_deleteData(data);
        mj_deleteModel(model);
        mj_deleteSpec(scene);
        return 1;
    }

    printf("[scenario] '%s': %d vehicle(s), environment=%s seed=%u, "
           "duration=%.1f s\n",
           path, scenario.vehicle_count, scenario.environment,
           scenario.environment_seed, scenario.duration_s);
    printf("  bodies=%ld actuators=%ld timestep=%.4f\n",
           model->nbody, model->nu, model->opt.timestep);

    int rc = run_scenario(&scenario, vehicles, model, data, model->opt.timestep);

    destroy_vehicles(&scenario, vehicles);
    transport_shutdown(&tp);
    mj_deleteData(data);
    mj_deleteModel(model);
    mj_deleteSpec(scene);
    return rc;
}
