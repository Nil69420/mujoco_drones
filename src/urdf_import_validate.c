#include "urdf_import_internal.h"

#include <dirent.h>
#include <errno.h>
#include <math.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>

int imp_run_validation(mjModel *model, int expected_meshes, int expected_mesh_geoms) {
    int failures = 0;

    for (int bidx = 1; bidx < model->nbody; bidx++) {
        if (model->body_mass[bidx] < 0.0) {
            fprintf(stderr, "urdf_import: validation FAIL: body %d has negative mass %g\n", bidx,
                    model->body_mass[bidx]);
            failures++;
        }
        const double *inertia = &model->body_inertia[(ptrdiff_t)bidx * 3];
        if (inertia[0] <= 0.0 || inertia[1] <= 0.0 || inertia[2] <= 0.0) {
            fprintf(stderr,
                    "urdf_import: validation FAIL: body %d has non-positive "
                    "principal inertia (%g %g %g)\n",
                    bidx, inertia[0], inertia[1], inertia[2]);
            failures++;
        }
        int hops = 0;
        int parent = model->body_parentid[bidx];
        while (parent > 0 && hops < model->nbody) {
            parent = model->body_parentid[parent];
            hops++;
        }
        if (hops >= model->nbody) {
            fprintf(stderr,
                    "urdf_import: validation FAIL: body %d has no path to the "
                    "world frame\n",
                    bidx);
            failures++;
        }
    }

    if (model->nmesh != expected_meshes) {
        fprintf(stderr,
                "urdf_import: validation FAIL: %ld meshes compiled, expected %d "
                "(silent mesh loss)\n",
                (long)model->nmesh, expected_meshes);
        failures++;
    }
    int mesh_geoms = 0;
    for (int geom = 0; geom < model->ngeom; geom++) {
        if (model->geom_type[geom] == mjGEOM_MESH) {
            mesh_geoms++;
        }
    }
    if (mesh_geoms != expected_mesh_geoms) {
        fprintf(stderr,
                "urdf_import: validation FAIL: %d mesh geoms compiled, expected "
                "%d\n",
                mesh_geoms, expected_mesh_geoms);
        failures++;
    }

    model->opt.enableflags |= mjENBL_ENERGY;
    mjData *data = mj_makeData(model);
    double settle_time = 2.0;
    int steps = (int)(settle_time / model->opt.timestep);
    double e0 = data->energy[0] + data->energy[1];
    double e_max = e0;
    double e_mid = e0;
    double e_end = e0;
    int nan_detected = 0;
    for (int i = 0; i < steps; i++) {
        mj_step(model, data);
        for (int qidx = 0; qidx < model->nq; qidx++) {
            if (!isfinite(data->qpos[qidx])) {
                nan_detected = 1;
            }
        }
        for (int vidx = 0; vidx < model->nv; vidx++) {
            if (!isfinite(data->qvel[vidx])) {
                nan_detected = 1;
            }
        }
        double energy = data->energy[0] + data->energy[1];
        if (energy > e_max) {
            e_max = energy;
        }
        if (i == steps / 2) {
            e_mid = energy;
        }
        e_end = energy;
    }
    if (nan_detected) {
        fprintf(stderr, "urdf_import: validation FAIL: NaN/Inf in qpos/qvel "
                        "during settle\n");
        failures++;
    }
    if (e_max > 1e6) {
        fprintf(stderr,
                "urdf_import: validation FAIL: energy blowup during settle "
                "(Emax=%g > 1e6)\n",
                e_max);
        failures++;
    }
    double drift = e_end - e_mid;
    double drift_budget = e_mid > 1.0 ? e_mid * 0.5 : 1.0;
    if (drift > drift_budget) {
        fprintf(stderr,
                "urdf_import: validation FAIL: energy drift in the second half "
                "of the settle (E_mid=%g E_end=%g)\n",
                e_mid, e_end);
        failures++;
    }
    mj_deleteData(data);

    if (failures == 0) {
        printf("  validation: PASS (%d steps settle, E0=%g Emax=%g E_end=%g)\n", steps, e0, e_max,
               e_end);
    } else {
        fprintf(stderr, "urdf_import: validation FAILED with %d finding(s)\n", failures);
    }
    return failures == 0 ? 0 : -1;
}
