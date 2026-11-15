#include "urdf_import_internal.h"

#include <dirent.h>
#include <errno.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>

static int add_joint_actuator(mjSpec *spec, const char *jname, int rotor, double kp,
                              const config_actuator_t *ov) {
    mjsActuator *act = mjs_addActuator(spec, NULL);
    if (!act) {
        fprintf(stderr, "urdf_import: warning: cannot create actuator for joint '%s'\n", jname);
        return 0;
    }
    const char *err =
        rotor ? mjs_setToMotor(act) : mjs_setToPosition(act, kp, NULL, NULL, NULL, 0.0);
    if (err && err[0] != '\0') {
        fprintf(stderr, "urdf_import: warning: %s (joint '%s')\n", err, jname);
        return 0;
    }
    act->trntype = mjTRN_JOINT;
    mjs_setString(act->target, jname);
    mjs_setString(mjs_getName(act->element), jname);
    if (ov && ov->has_torque) {
        act->gear[5] = ov->torque;
    }
    if (ov && ov->has_ctrlrange) {
        act->ctrllimited = 1;
        act->ctrlrange[0] = ov->ctrlrange[0];
        act->ctrlrange[1] = ov->ctrlrange[1];
    }
    return 1;
}

typedef struct {
    const config_actuator_t *override;
    const config_actuator_t *spin;
    int rotor;
    int bounded;
    int spin_only;
} joint_plan_t;

static void mark_matched(const config_actuator_t *entry) {
    if (entry) {
        ((config_actuator_t *)entry)->matched = 1;
    }
}

static joint_plan_t plan_joint(const actuator_config_t *cfg, const char *jname, const mjsJoint *jnt) {
    joint_plan_t plan;
    plan.override = cfg ? config_find(cfg, jname) : NULL;
    plan.spin = cfg ? config_find_spin(cfg, jname) : NULL;
    mark_matched(plan.override);
    mark_matched(plan.spin);
    plan.spin_only = plan.spin && !plan.override;
    plan.rotor = plan.override
                     ? plan.override->is_motor
                     : (strstr(jname, "rotor") != NULL || strstr(jname, "prop") != NULL);
    plan.bounded = plan.override ? !plan.override->is_motor : jnt->range[0] < jnt->range[1];
    return plan;
}

static int add_spin_actuator(mjSpec *spec, const char *jname, const config_actuator_t *spin) {
    char spin_name[256];
    snprintf(spin_name, sizeof(spin_name), "%.*s_spin", (int)(sizeof(spin_name) - 6), jname);
    mjsActuator *sact = mjs_addActuator(spec, NULL);
    if (!sact) {
        fprintf(stderr, "urdf_import: warning: cannot create spin actuator for joint '%s'\n", jname);
        return 0;
    }
    const char *serr = mjs_setToVelocity(sact, spin->kv);
    if (serr && serr[0] != '\0') {
        fprintf(stderr, "urdf_import: warning: %s (joint '%s')\n", serr, jname);
        return 0;
    }
    sact->trntype = mjTRN_JOINT;
    mjs_setString(sact->target, jname);
    mjs_setString(mjs_getName(sact->element), spin_name);
    if (spin->has_ctrlrange) {
        sact->ctrllimited = 1;
        sact->ctrlrange[0] = spin->ctrlrange[0];
        sact->ctrlrange[1] = spin->ctrlrange[1];
    }
    printf("  actuator: spin (kv=%g) -> '%s'\n", spin->kv, jname);
    return 1;
}

static double joint_kp(const joint_plan_t *plan) {
    if (plan->rotor) {
        return 0.0;
    }
    return plan->override ? plan->override->kp : 40.0;
}

static const char *joint_kp_note(const joint_plan_t *plan) {
    if (plan->rotor) {
        return "";
    }
    return plan->override ? " (config kp)" : " (heuristic kp=40.0)";
}

static int inject_for_joint(mjSpec *spec, const actuator_config_t *cfg, const char *jname,
                            const mjsJoint *jnt) {
    joint_plan_t plan = plan_joint(cfg, jname, jnt);
    if (!plan.rotor && !plan.bounded && !plan.spin_only) {
        fprintf(stderr,
                "urdf_import: note: joint '%s' left undriven (no rotor/prop tag, unbounded range)\n",
                jname);
        return 0;
    }
    int added = 0;
    double kp = joint_kp(&plan);
    if (!plan.spin_only && add_joint_actuator(spec, jname, plan.rotor, kp, plan.override)) {
        const char *note = joint_kp_note(&plan);
        printf("  actuator: %s -> '%s'%s\n", plan.rotor ? "motor" : "position", jname, note);
        added++;
    }
    if (plan.spin) {
        added += add_spin_actuator(spec, jname, plan.spin);
    }
    return added;
}

static void warn_unmatched(const actuator_config_t *cfg) {
    if (!cfg) {
        return;
    }
    for (int idx = 0; idx < cfg->count; idx++) {
        if (!cfg->entries[idx].matched) {
            fprintf(stderr, "urdf_import: warning: config joint '%s' not found in the model\n",
                    cfg->entries[idx].joint_name);
        }
    }
}

int imp_inject_actuators(mjSpec *spec, const actuator_config_t *cfg) {
    int count = 0;
    for (mjsElement *elem = mjs_firstElement(spec, mjOBJ_JOINT); elem;
         elem = mjs_nextElement(spec, elem)) {
        const char *jname = mjs_getString(mjs_getName(elem));
        const mjsJoint *jnt = mjs_asJoint(elem);
        if (jname && jnt) {
            count += inject_for_joint(spec, cfg, jname, jnt);
        }
    }
    warn_unmatched(cfg);
    return count;
}

void imp_convert_motors_to_sites(mjModel *model, mjSpec *spec) {
    for (mjsElement *el = mjs_firstElement(spec, mjOBJ_ACTUATOR); el;
         el = mjs_nextElement(spec, el)) {
        mjsActuator *act = mjs_asActuator(el);
        if (!act || act->gaintype != mjGAIN_FIXED || act->biastype != mjBIAS_NONE ||
            act->trntype != mjTRN_JOINT) {
            continue;
        }
        const char *target = mjs_getString(act->target);
        if (!target) {
            continue;
        }
        int jid = mj_name2id(model, mjOBJ_JOINT, target);
        if (jid < 0) {
            continue;
        }
        int bid = model->jnt_bodyid[jid];
        if (bid <= 0) {
            continue;
        }
        const char *body_name = mj_id2name(model, mjOBJ_BODY, bid);
        mjsBody *body = body_name ? mjs_findBody(spec, body_name) : NULL;
        if (!body) {
            continue;
        }
        char site_name[300];
        snprintf(site_name, sizeof(site_name), "%.*s_thrust_site", (int)(sizeof(site_name) - 14),
                 target);
        mjsSite *site = mjs_addSite(body, NULL);
        if (!site) {
            continue;
        }
        mjs_setString(mjs_getName(site->element), site_name);
        site->size[0] = 0.005;
        site->size[1] = 0.005;
        site->size[2] = 0.005;
        const double axis_body[3] = {model->jnt_axis[jid * 3 + 0], model->jnt_axis[jid * 3 + 1],
                                     model->jnt_axis[jid * 3 + 2]};
        mju_quatZ2Vec(site->quat, axis_body);
        act->trntype = mjTRN_SITE;
        mjs_setString(act->target, site_name);
        act->gear[0] = 0.0;
        act->gear[1] = 0.0;
        act->gear[2] = 1.0;
        printf("  actuator: motor '%s' -> site lift '%s' (gear 0 0 1 0 0 %g)\n", target, site_name,
               act->gear[5]);
    }
}

void imp_tune_position_gains(mjModel *model, mjSpec *spec, const actuator_config_t *cfg) {
    for (mjsElement *el = mjs_firstElement(spec, mjOBJ_ACTUATOR); el;
         el = mjs_nextElement(spec, el)) {
        mjsActuator *act = mjs_asActuator(el);
        if (!act || act->biastype != mjBIAS_AFFINE) {
            continue;
        }
        const char *target = mjs_getString(act->target);
        if (!target) {
            continue;
        }
        if (cfg && config_find(cfg, target)) {
            continue;
        }
        int jid = mj_name2id(model, mjOBJ_JOINT, target);
        if (jid < 0) {
            continue;
        }
        int bid = model->jnt_bodyid[jid];
        if (bid <= 0) {
            continue;
        }
        double i0 = model->body_inertia[bid * 3 + 0];
        double i1 = model->body_inertia[bid * 3 + 1];
        double i2 = model->body_inertia[bid * 3 + 2];
        double imin = fmin(fmin(i0, i1), fmin(i2, fmin(i0, i1)));
        if (imin <= 0.0) {
            continue;
        }
        double kp = imin * 2500.0;
        double kv = imin * 100.0;
        act->biasprm[0] = kp;
        act->biasprm[1] = kv;
        printf("  actuator: position '%s' tuned kp=%g kv=%g (I_min=%g)\n", target, kp, kv, imin);
    }
}
