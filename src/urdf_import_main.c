#include "urdf_import_config.h"
#include "urdf_import_sdf.h"
#include "urdf_import_xml.h"
#include <mujoco/mujoco.h>

#include <dirent.h>
#include <errno.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>

static void print_usage(const char *prog) {
    printf("Usage: %s <input-dir> <output-name> [OPTIONS]\n\n", prog);
    printf("Import a URDF (or SDF) airframe with its meshes into\n");
    printf("model/airframes/<output-name>/ for use as a Stage 1 airframe.\n\n");
    printf("Arguments:\n");
    printf("  <input-dir>    Directory containing exactly one .urdf/.sdf file\n");
    printf("                 and its mesh assets\n");
    printf("  <output-name>  Airframe name (used for the output directory and\n");
    printf("                 the airframe_spec_t registration)\n");
    printf("Options:\n");
    printf("  --model FILE   Explicit model file override (bypasses directory "
           "scan)\n");
    printf("  --config FILE  Optional actuator-override config file\n");
    printf("  --class NAME   Airframe class tag for the registry entry\n");
    printf("  -h, --help     Show this help\n");
}

static int find_model_file(const char *dir, char *out, size_t out_sz) {
    DIR *d = opendir(dir);
    if (!d) {
        fprintf(stderr, "urdf_import: cannot open input directory '%s'\n", dir);
        return -1;
    }

    int found = 0;
    struct dirent *ent = NULL;
    while ((ent = readdir(d)) != NULL) { // NOLINT(concurrency-mt-unsafe)
        if (ent->d_type == DT_DIR) {
            continue;
        }
        char full[1024];
        snprintf(full, sizeof(full), "%s/%s", dir, ent->d_name);
        struct stat st;
        if (stat(full, &st) != 0 || !S_ISREG(st.st_mode)) {
            continue;
        }
        size_t len = strlen(ent->d_name);
        const char *ext = len >= 5 ? ent->d_name + len - 5 : "";
        if (strcmp(ext, ".urdf") != 0 && strcmp(ext, ".sdf") != 0) {
            continue;
        }
        if (found) {
            fprintf(stderr,
                    "urdf_import: more than one .urdf/.sdf file in '%s' "
                    "(found '%s' and '%s') - pass --model explicitly\n",
                    dir, out, ent->d_name);
            closedir(d);
            return -1;
        }
        snprintf(out, out_sz, "%s", ent->d_name);
        found = 1;
    }
    closedir(d);

    if (!found) {
        fprintf(stderr, "urdf_import: no .urdf or .sdf file found in '%s'\n", dir);
        return -1;
    }
    return 0;
}

static int read_file(const char *path, char **out, size_t *out_len) {
    FILE *f = fopen(path, "rb");
    if (!f) {
        return -1;
    }
    if (fseek(f, 0, SEEK_END) != 0) {
        fclose(f);
        return -1;
    }
    long sz = ftell(f);
    if (sz < 0) {
        fclose(f);
        return -1;
    }
    rewind(f);
    char *buf = malloc((size_t)sz + 1);
    if (!buf) {
        fclose(f);
        return -1;
    }
    size_t got = fread(buf, 1, (size_t)sz, f);
    fclose(f);
    if (got != (size_t)sz) {
        free(buf);
        return -1;
    }
    buf[got] = '\0';
    *out = buf;
    *out_len = got;
    return 0;
}

static int write_file(const char *path, const char *data, size_t len) {
    FILE *f = fopen(path, "wb");
    if (!f) {
        return -1;
    }
    size_t wrote = fwrite(data, 1, len, f);
    fclose(f);
    return wrote == len ? 0 : -1;
}

static int ensure_dir(const char *path) {
    char tmp[1024];
    snprintf(tmp, sizeof(tmp), "%.*s", (int)(sizeof(tmp) - 1), path);
    for (char *pc = tmp + 1; *pc; pc++) {
        if (*pc == '/') {
            *pc = '\0';
            if (mkdir(tmp, 0755) != 0 && errno != EEXIST) {
                return -1;
            }
            *pc = '/';
        }
    }
    if (mkdir(tmp, 0755) != 0 && errno != EEXIST) {
        return -1;
    }
    struct stat st;
    return stat(tmp, &st) == 0 && S_ISDIR(st.st_mode) ? 0 : -1;
}

typedef struct {
    char *orig_ref;
    char *resolved;
    char *stored_name;
} mesh_entry_t;

typedef struct {
    char link[256];
    char meshname[256];
    double pos[3];
    double quat[4];
    double scale[3];
} visual_mesh_t;

typedef struct {
    mesh_entry_t entries[256];
    int count;
} mesh_table_t;

static const char *mesh_basename(const char *path) {
    const char *base = strrchr(path, '/');
    return base ? base + 1 : path;
}

static void mesh_table_free(mesh_table_t *t) {
    for (int i = 0; i < t->count; i++) {
        free(t->entries[i].orig_ref);
        free(t->entries[i].resolved);
        free(t->entries[i].stored_name);
    }
    t->count = 0;
}

static int mesh_table_lookup(const mesh_table_t *t, const char *stored) {
    for (int i = 0; i < t->count; i++) {
        if (strcmp(t->entries[i].stored_name, stored) == 0) {
            return i;
        }
    }
    return -1;
}

static int resolve_mesh_path(const char *input_dir, const char *ref, char *out, size_t out_sz) {
    const char *rest = NULL;
    if (strncmp(ref, "package://", 10) == 0) {
        rest = strchr(ref + 10, '/');
        if (!rest) {
            return -1;
        }
    } else if (ref[0] == '/') {
        snprintf(out, out_sz, "%s", ref);
        return 0;
    } else {
        rest = ref;
    }

    if (rest[0] == '/') {
        rest++;
    }
    const char *base = mesh_basename(rest);

    const char *candidates[4];
    char c1[1024], c2[1024], c3[1024], c4[1024];
    snprintf(c1, sizeof(c1), "%s/%s", input_dir, rest);
    snprintf(c2, sizeof(c2), "%s/%s", input_dir, base);
    snprintf(c3, sizeof(c3), "%s/meshes/%s", input_dir, rest);
    snprintf(c4, sizeof(c4), "%s/meshes/%s", input_dir, base);
    candidates[0] = c1;
    candidates[1] = c2;
    candidates[2] = c3;
    candidates[3] = c4;

    for (int i = 0; i < 4; i++) {
        struct stat st;
        if (stat(candidates[i], &st) == 0 && S_ISREG(st.st_mode)) {
            snprintf(out, out_sz, "%s", candidates[i]);
            return 0;
        }
    }
    return -1;
}

static int copy_file(const char *src, const char *dst) {
    FILE *in = fopen(src, "rb");
    if (!in) {
        return -1;
    }
    FILE *out = fopen(dst, "wb");
    if (!out) {
        fclose(in);
        return -1;
    }
    char buf[65536];
    size_t n = 0;
    int rc = 0;
    while ((n = fread(buf, 1, sizeof(buf), in)) > 0) {
        if (fwrite(buf, 1, n, out) != n) {
            rc = -1;
            break;
        }
    }
    fclose(in);
    fclose(out);
    return rc;
}

static int parse_vec3(const char *s, double out[3]) {
    const char *cursor = s;
    for (int i = 0; i < 3; i++) {
        char *end = NULL;
        out[i] = strtod(cursor, &end);
        if (end == cursor) {
            return 0;
        }
        cursor = end;
    }
    return 1;
}

static int mesh_table_add(mesh_table_t *table, const char *filename, const char *resolved,
                          const char *stored) {
    int existing = mesh_table_lookup(table, stored);
    if (existing >= 0) {
        if (strcmp(table->entries[existing].orig_ref, filename) != 0) {
            fprintf(stderr, "urdf_import: duplicate mesh basename for '%s'\n", filename);
            return -1;
        }
        return existing;
    }
    if (table->count >= 256) {
        fprintf(stderr, "urdf_import: too many mesh references\n");
        return -1;
    }
    mesh_entry_t *entry = &table->entries[table->count++];
    entry->orig_ref = strdup(filename);
    entry->resolved = strdup(resolved);
    entry->stored_name = strdup(stored);
    return table->count - 1;
}

static void record_visual_mesh(visual_mesh_t *visuals, int *nvisuals, int visuals_cap,
                               const char *cur_link, const char *meshname, int have_rpy,
                               const double origin_xyz[3], const double origin_rpy[3],
                               const char *scale_text) {
    if (*nvisuals >= visuals_cap) {
        fprintf(stderr, "urdf_import: warning: too many visual meshes; dropping '%s'\n", meshname);
        return;
    }
    visual_mesh_t *vis = &visuals[*nvisuals];
    snprintf(vis->link, sizeof(vis->link), "%.*s", (int)(sizeof(vis->link) - 1), cur_link);
    snprintf(vis->meshname, sizeof(vis->meshname), "%.*s", (int)(sizeof(vis->meshname) - 1),
             meshname);
    vis->pos[0] = origin_xyz[0];
    vis->pos[1] = origin_xyz[1];
    vis->pos[2] = origin_xyz[2];
    vis->quat[0] = 1.0;
    vis->quat[1] = 0.0;
    vis->quat[2] = 0.0;
    vis->quat[3] = 0.0;
    if (have_rpy) {
        mju_euler2Quat(vis->quat, origin_rpy, "XYZ");
    }
    vis->scale[0] = 1.0;
    vis->scale[1] = 1.0;
    vis->scale[2] = 1.0;
    if (scale_text && parse_vec3(scale_text, vis->scale)) {
        // scale parsed
    }
    (*nvisuals)++;
}

static int handle_mesh_element(xml_element_t *elem, const char *input_dir, mesh_table_t *table,
                               const char *cur_link, int in_visual, int have_rpy,
                               const double origin_xyz[3], const double origin_rpy[3],
                               visual_mesh_t *visuals, int *nvisuals, int visuals_cap) {
    const char *filename = xml_attr(elem, "filename");
    if (!filename) {
        return 0;
    }
    char resolved[1024];
    if (resolve_mesh_path(input_dir, filename, resolved, sizeof(resolved)) != 0) {
        fprintf(stderr, "urdf_import: cannot resolve mesh reference '%s'\n", filename);
        return -1;
    }
    struct stat st;
    if (stat(resolved, &st) != 0 || !S_ISREG(st.st_mode)) {
        fprintf(stderr,
                "urdf_import: mesh file not found for reference '%s' "
                "(looked for %s)\n",
                filename, resolved);
        return -1;
    }

    char stored[512];
    snprintf(stored, sizeof(stored), "%.*s", (int)(sizeof(stored) - 1), mesh_basename(resolved));
    int entry_idx = mesh_table_add(table, filename, resolved, stored);
    if (entry_idx < 0) {
        return -1;
    }

    if (in_visual) {
        record_visual_mesh(visuals, nvisuals, visuals_cap, cur_link,
                           table->entries[entry_idx].stored_name, have_rpy, origin_xyz, origin_rpy,
                           xml_attr(elem, "scale"));
    } else {
        fprintf(stderr,
                "urdf_import: warning: collision mesh '%s' dropped (native URDF "
                "import cannot load collision meshes)\n",
                filename);
    }
    return 0;
}

static int rewrite_meshes(const char *input_dir, char *urdf_text, size_t urdf_len,
                          const char *output_name, mesh_table_t *table, visual_mesh_t *visuals,
                          int *nvisuals, int visuals_cap, char *root_link_out, char *root_mass_out,
                          size_t root_mass_sz) {
    xml_reader_t reader;
    xml_reader_init(&reader, urdf_text, urdf_len);
    xml_element_t elem;

    char cur_link[256] = "";
    char link_names[256][256];
    int nlink_names = 0;
    char child_names[256][256];
    int nchild_names = 0;
    int saw_root_inertial = 0;
    char root_mass_text[64] = "";
    char root_inertia_text[256] = "";
    int in_root_inertial = 0;
    int have_root_mass = 0;
    int in_visual = 0;
    int have_rpy = 0;
    double origin_xyz[3] = {0.0, 0.0, 0.0};
    double origin_rpy[3] = {0.0, 0.0, 0.0};

    while (xml_next_element(&reader, &elem)) {
        if (strcmp(elem.name, "link") == 0) {
            const char *name = xml_attr(&elem, "name");
            if (name) {
                snprintf(cur_link, sizeof(cur_link), "%.*s", (int)(sizeof(cur_link) - 1), name);
                if (nlink_names < 256) {
                    snprintf(link_names[nlink_names], sizeof(link_names[nlink_names]), "%.*s",
                             (int)(sizeof(link_names[nlink_names]) - 1), name);
                    nlink_names++;
                }
            }
            in_visual = 0;
            have_rpy = 0;
        } else if (strcmp(elem.name, "child") == 0) {
            const char *child = xml_attr(&elem, "link");
            if (child && nchild_names < 256) {
                snprintf(child_names[nchild_names], sizeof(child_names[nchild_names]), "%.*s",
                         (int)(sizeof(child_names[nchild_names]) - 1), child);
                nchild_names++;
            }
        } else if (strcmp(elem.name, "inertial") == 0) {
            in_root_inertial = !saw_root_inertial && strcmp(cur_link, "import_root") != 0;
            saw_root_inertial = 1;
        } else if (in_root_inertial && strcmp(elem.name, "mass") == 0) {
            const char *mass = xml_attr(&elem, "value");
            if (mass && !have_root_mass) {
                snprintf(root_mass_text, sizeof(root_mass_text), "%.*s",
                         (int)(sizeof(root_mass_text) - 1), mass);
                have_root_mass = 1;
            }
        } else if (in_root_inertial && strcmp(elem.name, "inertia") == 0) {
            const char *ixx = xml_attr(&elem, "ixx");
            const char *iyy = xml_attr(&elem, "iyy");
            const char *izz = xml_attr(&elem, "izz");
            if (ixx && iyy && izz && root_inertia_text[0] == '\0') {
                snprintf(root_inertia_text, sizeof(root_inertia_text), "%.*s %.*s %.*s",
                         (int)(sizeof(root_inertia_text) / 3 - 1), ixx,
                         (int)(sizeof(root_inertia_text) / 3 - 1), iyy,
                         (int)(sizeof(root_inertia_text) / 3 - 1), izz);
            }
        } else if (in_root_inertial && strcmp(elem.name, "link") == 0) {
            in_root_inertial = 0;
        } else if (strcmp(elem.name, "visual") == 0) {
            in_visual = 1;
            have_rpy = 0;
        } else if (strcmp(elem.name, "collision") == 0) {
            in_visual = 0;
            have_rpy = 0;
        } else if (strcmp(elem.name, "origin") == 0) {
            const char *xyz = xml_attr(&elem, "xyz");
            const char *rpy = xml_attr(&elem, "rpy");
            if (xyz && parse_vec3(xyz, origin_xyz)) {
                // position parsed
            }
            if (rpy && parse_vec3(rpy, origin_rpy)) {
                have_rpy = 1;
            }
        } else if (strcmp(elem.name, "mesh") == 0) {
            if (handle_mesh_element(&elem, input_dir, table, cur_link, in_visual, have_rpy,
                                    origin_xyz, origin_rpy, visuals, nvisuals, visuals_cap) != 0) {
                xml_element_clear(&elem);
                mesh_table_free(table);
                return -1;
            }
        }
        xml_element_clear(&elem);
    }

    if (root_link_out) {
        const char *root = nlink_names > 0 ? link_names[0] : "";
        for (int i = 0; i < nlink_names; i++) {
            int is_child = 0;
            for (int k = 0; k < nchild_names; k++) {
                if (strcmp(link_names[i], child_names[k]) == 0) {
                    is_child = 1;
                    break;
                }
            }
            if (!is_child) {
                root = link_names[i];
                break;
            }
        }
        snprintf(root_link_out, 256, "%.*s", 255, root);
        if (root_mass_out && root_mass_sz > 0) {
            if (have_root_mass) {
                snprintf(root_mass_out, root_mass_sz, "mass=\"%s\" diaginertia=\"%s\"",
                         root_mass_text,
                         root_inertia_text[0] ? root_inertia_text : "1e-06 1e-06 1e-06");
            } else {
                snprintf(root_mass_out, root_mass_sz, "%.*s", (int)(root_mass_sz - 1), "");
            }
        }
    }

    char meshes_dir[512];
    snprintf(meshes_dir, sizeof(meshes_dir), "model/airframes/%.*s/meshes",
             (int)(sizeof(meshes_dir) - 30), output_name);
    if (ensure_dir(meshes_dir) != 0) {
        fprintf(stderr, "urdf_import: cannot create '%s'\n", meshes_dir);
        mesh_table_free(table);
        return -1;
    }

    for (int i = 0; i < table->count; i++) {
        char dst[1024];
        snprintf(dst, sizeof(dst), "%s/%.*s", meshes_dir,
                 (int)(sizeof(dst) - sizeof(meshes_dir) - 1), table->entries[i].stored_name);
        if (copy_file(table->entries[i].resolved, dst) != 0) {
            fprintf(stderr, "urdf_import: failed to copy mesh '%s' -> '%s'\n",
                    table->entries[i].resolved, dst);
            mesh_table_free(table);
            return -1;
        }
        printf("  mesh: %s -> %s\n", table->entries[i].resolved, dst);
    }

    for (int i = 0; i < table->count; i++) {
        char needle[1100];
        char repl[1100];
        snprintf(needle, sizeof(needle), "filename=\"%s\"", table->entries[i].orig_ref);
        snprintf(repl, sizeof(repl), "filename=\"meshes/%s\"", table->entries[i].stored_name);
        size_t needle_len = strlen(needle);
        size_t repl_len = strlen(repl);
        char *hit = strstr(urdf_text, needle);
        while (hit) {
            memmove(hit + repl_len, hit + needle_len, strlen(hit + needle_len) + 1);
            memcpy(hit, repl, repl_len);
            hit = strstr(hit + repl_len, needle);
        }
    }
    return 0;
}

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

static int inject_actuators(mjSpec *spec, actuator_config_t *cfg) {
    int count = 0;
    for (mjsElement *el = mjs_firstElement(spec, mjOBJ_JOINT); el; el = mjs_nextElement(spec, el)) {
        const char *jname = mjs_getString(mjs_getName(el));
        if (!jname) {
            continue;
        }
        mjsJoint *jnt = mjs_asJoint(el);
        if (!jnt) {
            continue;
        }

        const config_actuator_t *ov = cfg ? config_find(cfg, jname) : NULL;
        const config_actuator_t *spin = cfg ? config_find_spin(cfg, jname) : NULL;
        if (ov) {
            ((config_actuator_t *)ov)->matched = 1;
        }
        if (spin) {
            ((config_actuator_t *)spin)->matched = 1;
        }
        int spin_only = spin && !ov;
        int rotor =
            ov ? ov->is_motor : (strstr(jname, "rotor") != NULL || strstr(jname, "prop") != NULL);
        int bounded = ov ? !ov->is_motor : jnt->range[0] < jnt->range[1];
        if (!rotor && !bounded && !spin_only) {
            fprintf(stderr,
                    "urdf_import: note: joint '%s' left undriven "
                    "(no rotor/prop tag, unbounded range)\n",
                    jname);
            continue;
        }

        double kp = rotor ? 0.0 : (ov ? ov->kp : 40.0);
        if (!spin_only && add_joint_actuator(spec, jname, rotor, kp, ov)) {
            printf("  actuator: %s -> '%s'%s\n", rotor ? "motor" : "position", jname,
                   rotor ? "" : (ov ? " (config kp)" : " (heuristic kp=40.0)"));
            count++;
        }
        if (spin) {
            char spin_name[256];
            snprintf(spin_name, sizeof(spin_name), "%.*s_spin", (int)(sizeof(spin_name) - 6),
                     jname);
            mjsActuator *sact = mjs_addActuator(spec, NULL);
            if (!sact) {
                fprintf(stderr,
                        "urdf_import: warning: cannot create spin "
                        "actuator for joint '%s'\n",
                        jname);
            } else {
                const char *serr = mjs_setToVelocity(sact, spin->kv);
                if (serr && serr[0] != '\0') {
                    fprintf(stderr, "urdf_import: warning: %s (joint '%s')\n", serr, jname);
                } else {
                    sact->trntype = mjTRN_JOINT;
                    mjs_setString(sact->target, jname);
                    mjs_setString(mjs_getName(sact->element), spin_name);
                    if (spin->has_ctrlrange) {
                        sact->ctrllimited = 1;
                        sact->ctrlrange[0] = spin->ctrlrange[0];
                        sact->ctrlrange[1] = spin->ctrlrange[1];
                    }
                    printf("  actuator: spin (kv=%g) -> '%s'\n", spin->kv, jname);
                    count++;
                }
            }
        }
    }

    if (cfg) {
        for (int i = 0; i < cfg->count; i++) {
            if (!cfg->entries[i].matched) {
                fprintf(stderr,
                        "urdf_import: warning: config joint '%s' not found in "
                        "the model\n",
                        cfg->entries[i].joint_name);
            }
        }
    }
    return count;
}

static int synthesize_visual_meshes(mjSpec *spec, const visual_mesh_t *visuals, int n) {
    char mesh_names[256][256];
    int nmesh = 0;
    int added = 0;
    for (int i = 0; i < n; i++) {
        mjsBody *body = mjs_findBody(spec, visuals[i].link);
        if (!body) {
            body = mjs_findBody(spec, "world");
            fprintf(stderr,
                    "urdf_import: note: body '%s' not found for mesh '%s'; "
                    "attaching to the world body (root link merged into world "
                    "by the URDF import)\n",
                    visuals[i].link, visuals[i].meshname);
        }
        if (!body) {
            fprintf(stderr, "urdf_import: warning: no target body for mesh '%s'; dropped\n",
                    visuals[i].meshname);
            continue;
        }

        int existing = -1;
        for (int k = 0; k < nmesh; k++) {
            if (strcmp(mesh_names[k], visuals[i].meshname) == 0) {
                existing = k;
                break;
            }
        }
        if (existing < 0) {
            if (nmesh >= 256) {
                fprintf(stderr, "urdf_import: warning: too many meshes; dropping '%s'\n",
                        visuals[i].meshname);
                continue;
            }
            mjsMesh *mesh = mjs_addMesh(spec, NULL);
            if (!mesh) {
                fprintf(stderr, "urdf_import: warning: cannot create mesh for '%s'\n",
                        visuals[i].meshname);
                continue;
            }
            char mesh_path[1024];
            snprintf(mesh_path, sizeof(mesh_path), "meshes/%s", visuals[i].meshname);
            mjs_setString(mesh->file, mesh_path);
            mjs_setString(mjs_getName(mesh->element), visuals[i].meshname);
            mesh->scale[0] = visuals[i].scale[0];
            mesh->scale[1] = visuals[i].scale[1];
            mesh->scale[2] = visuals[i].scale[2];
            snprintf(mesh_names[nmesh], sizeof(mesh_names[nmesh]), "%.*s",
                     (int)(sizeof(mesh_names[nmesh]) - 1), visuals[i].meshname);
            nmesh++;
        }

        mjsGeom *geom = mjs_addGeom(body, NULL);
        if (!geom) {
            fprintf(stderr, "urdf_import: warning: cannot create geom for mesh '%s'\n",
                    visuals[i].meshname);
            continue;
        }
        geom->type = mjGEOM_MESH;
        mjs_setString(geom->meshname, visuals[i].meshname);
        geom->pos[0] = visuals[i].pos[0];
        geom->pos[1] = visuals[i].pos[1];
        geom->pos[2] = visuals[i].pos[2];
        geom->quat[0] = visuals[i].quat[0];
        geom->quat[1] = visuals[i].quat[1];
        geom->quat[2] = visuals[i].quat[2];
        geom->quat[3] = visuals[i].quat[3];
        geom->contype = 0;
        geom->conaffinity = 0;
        printf("  visual mesh: '%s' on body '%s' (scale %.3f %.3f %.3f)\n", visuals[i].meshname,
               visuals[i].link, visuals[i].scale[0], visuals[i].scale[1], visuals[i].scale[2]);
        added++;
    }
    return added;
}

static int add_named_site(mjsBody *body, const char *name, const double pos[3],
                          const double quat[4], const double size[3]) {
    mjsSite *site = mjs_addSite(body, NULL);
    if (!site) {
        fprintf(stderr, "urdf_import: warning: cannot create site '%s'\n", name);
        return -1;
    }
    mjs_setString(mjs_getName(site->element), name);
    site->pos[0] = pos[0];
    site->pos[1] = pos[1];
    site->pos[2] = pos[2];
    site->quat[0] = quat[0];
    site->quat[1] = quat[1];
    site->quat[2] = quat[2];
    site->quat[3] = quat[3];
    site->size[0] = size[0];
    site->size[1] = size[1];
    site->size[2] = size[2];
    return 0;
}

static int add_named_sensor(mjSpec *spec, mjtSensor type, const char *name) {
    mjsSensor *sensor = mjs_addSensor(spec);
    if (!sensor) {
        fprintf(stderr, "urdf_import: warning: cannot create sensor '%s'\n", name);
        return -1;
    }
    mjs_setString(mjs_getName(sensor->element), name);
    sensor->type = type;
    sensor->objtype = mjOBJ_SITE;
    mjs_setString(sensor->objname, "imu_site");
    return 0;
}

static int finalize_spec(mjSpec *spec) {
    mjsBody *base = mjs_findBody(spec, "world");
    if (!base) {
        fprintf(stderr, "urdf_import: warning: world body not found\n");
        return -1;
    }
    mjsBody *ref = mjs_addBody(base, NULL);
    if (!ref) {
        fprintf(stderr, "urdf_import: warning: cannot create base_link body\n");
        return -1;
    }
    mjs_setString(mjs_getName(ref->element), "base_link");
    ref->mass = 1e-6;
    ref->inertia[0] = 1e-9;
    ref->inertia[1] = 1e-9;
    ref->inertia[2] = 1e-9;

    const double pos_imu[3] = {0.0, 0.0, 0.0};
    const double pos_lidar[3] = {0.0, 0.0, -0.02};
    const double pos_infrared[3] = {0.0, 0.0, -0.06};
    const double quat_identity[4] = {1.0, 0.0, 0.0, 0.0};
    double quat_infrared[4] = {0.0, 0.0, 0.0, 0.0};
    const double size_site[3] = {0.005, 0.005, 0.005};
    mju_euler2Quat(quat_infrared, (double[]){3.14159265358979, 0.0, 0.0}, "XYZ");

    if (add_named_site(base, "imu_site", pos_imu, quat_identity, size_site) != 0 ||
        add_named_site(base, "lidar_site", pos_lidar, quat_identity, size_site) != 0 ||
        add_named_site(base, "infrared_site", pos_infrared, quat_infrared, size_site) != 0) {
        return -1;
    }

    if (add_named_sensor(spec, mjSENS_ACCELEROMETER, "imu_accel") != 0 ||
        add_named_sensor(spec, mjSENS_GYRO, "imu_gyro") != 0 ||
        add_named_sensor(spec, mjSENS_FRAMEQUAT, "orientation") != 0 ||
        add_named_sensor(spec, mjSENS_FRAMEPOS, "position") != 0 ||
        add_named_sensor(spec, mjSENS_FRAMEXAXIS, "heading") != 0 ||
        add_named_sensor(spec, mjSENS_VELOCIMETER, "linvel") != 0 ||
        add_named_sensor(spec, mjSENS_FRAMEYAXIS, "mag_y") != 0 ||
        add_named_sensor(spec, mjSENS_FRAMEZAXIS, "mag_z") != 0) {
        return -1;
    }
    return 0;
}

static int emit_airframe_spec(mjSpec *spec, const char *output_name, const char *airframe_class,
                              const actuator_config_t *cfg) {
    char path[1024];
    snprintf(path, sizeof(path), "src/vehicle/airframes/%s_spec.c", output_name);
    if (ensure_dir("src/vehicle/airframes") != 0) {
        fprintf(stderr, "urdf_import: cannot create 'src/vehicle/airframes'\n");
        return -1;
    }

    FILE *f = fopen(path, "wb");
    if (!f) {
        fprintf(stderr, "urdf_import: cannot write '%s'\n", path);
        return -1;
    }
    fprintf(f, "/* Generated by urdf_import - do not edit by hand.\n");
    fprintf(f, " * Airframe spec for '%s' (class '%s'). */\n", output_name, airframe_class);
    fprintf(f, "#include \"vehicle/airframe.h\"\n");
    fprintf(f, "#include <stddef.h>\n\n");
    fprintf(f, "const airframe_spec_t %s_SPEC = {\n", output_name);
    fprintf(f, "    .name           = \"%s\",\n", output_name);
    fprintf(f, "    .airframe_class = \"%s\",\n", airframe_class);
    fprintf(f, "    .mjcf_path      = \"%s/%s.xml\",\n", output_name, output_name);
    char all_names[CONFIG_MAX_ACTUATORS][CONFIG_NAME_LEN];
    int used[CONFIG_MAX_ACTUATORS] = {0};
    int n_all = 0;
    for (mjsElement *el = mjs_firstElement(spec, mjOBJ_ACTUATOR); el;
         el = mjs_nextElement(spec, el)) {
        const char *name = mjs_getString(mjs_getName(el));
        if (name && n_all < CONFIG_MAX_ACTUATORS) {
            snprintf(all_names[n_all], sizeof(all_names[n_all]), "%.*s",
                     (int)(sizeof(all_names[n_all]) - 1), name);
            n_all++;
        }
    }

    fprintf(f, "    .channel_names  = {\n");
    int count = 0;
    if (cfg) {
        for (int i = 0; i < cfg->channel_order_count; i++) {
            int found = -1;
            for (int k = 0; k < n_all; k++) {
                if (strcmp(all_names[k], cfg->channel_order[i]) == 0) {
                    found = k;
                    break;
                }
            }
            if (found < 0) {
                fprintf(stderr,
                        "urdf_import: warning: channel order references '%s' "
                        "which is not an actuator in the model\n",
                        cfg->channel_order[i]);
                continue;
            }
            fprintf(f, "        \"%s\",\n", all_names[found]);
            used[found] = 1;
            count++;
        }
    }
    for (int pass = 0; pass < 2; pass++) {
        for (int i = 0; i < n_all; i++) {
            if (used[i]) {
                continue;
            }
            size_t name_len = strlen(all_names[i]);
            int is_spin = name_len > 5 && strcmp(all_names[i] + name_len - 5, "_spin") == 0;
            if ((pass == 0) == is_spin) {
                continue;
            }
            fprintf(f, "        \"%s\",\n", all_names[i]);
            count++;
        }
    }
    fprintf(f, "    },\n");
    fprintf(f, "    .channel_count  = %d,\n", count);
    fprintf(f, "    .imu_site       = \"imu_site\",\n");
    fprintf(f, "    .lidar_site     = \"lidar_site\",\n");
    fprintf(f, "    .infrared_site  = \"infrared_site\",\n");
    fprintf(f, "    .camera_name    = NULL,\n");
    fprintf(f, "    .base_body      = \"base_link\",\n");
    fprintf(f, "};\n");
    fclose(f);

    printf("urdf_import: wrote '%s' (%d channels)\n", path, count);
    printf("urdf_import: register in src/vehicle/airframe_registry.c:\n");
    printf("    extern const airframe_spec_t %s_SPEC;\n", output_name);
    printf("    add &%s_SPEC to airframe_table()\n", output_name);
    printf("    add \"%s_spec.c\" to SOURCES in CMakeLists.txt\n", output_name);
    return 0;
}

static int wrap_worldbody_for_scene(const char *final_path, const char *root_mass) {
    char *text = NULL;
    size_t len = 0;
    if (read_file(final_path, &text, &len) != 0) {
        fprintf(stderr, "urdf_import: cannot read final MJCF '%s' for wrapping\n", final_path);
        return -1;
    }
    if (strstr(text, "<freejoint name=\"root\"") != NULL) {
        free(text);
        return 0;
    }

    const char *world_open = "<worldbody>";
    const char *world_close = "  </worldbody>";
    const char *ref_body = "    <body name=\"base_link\"/>\n";
    char wrapper_open[1024];
    if (root_mass && root_mass[0] != '\0') {
        snprintf(wrapper_open, sizeof(wrapper_open),
                 "\n    <body name=\"base_link\" pos=\"0 0 0\">\n"
                 "      <inertial pos=\"0 0 0\" %s/>\n"
                 "      <freejoint name=\"root\"/>",
                 root_mass);
    } else {
        snprintf(wrapper_open, sizeof(wrapper_open),
                 "\n    <body name=\"base_link\" pos=\"0 0 0\">\n"
                 "      <inertial pos=\"0 0 0\" mass=\"0.0001\" "
                 "diaginertia=\"1e-06 1e-06 1e-06\"/>\n"
                 "      <freejoint name=\"root\"/>");
    }
    const char *wrapper_close = "    </body>\n";

    char *tmp = malloc(len + 1024 + 1);
    if (!tmp) {
        free(text);
        return -1;
    }
    char *dst = tmp;
    char *p = text;
    char *found;
    while ((found = strstr(p, world_open)) != NULL) {
        size_t seg = (size_t)(found - p) + strlen(world_open);
        memcpy(dst, p, seg);
        dst += seg;
        memcpy(dst, wrapper_open, strlen(wrapper_open));
        dst += strlen(wrapper_open);
        p = found + strlen(world_open);
    }
    memcpy(dst, p, strlen(p) + 1);

    char *tmp2 = malloc(len + 1024 + 1);
    if (!tmp2) {
        free(tmp);
        free(text);
        return -1;
    }
    char *dst2 = tmp2;
    char *scan = tmp;
    while (*scan) {
        if (strncmp(scan, ref_body, strlen(ref_body)) == 0) {
            scan += strlen(ref_body);
        } else if (strncmp(scan, world_close, strlen(world_close)) == 0) {
            memcpy(dst2, wrapper_close, strlen(wrapper_close));
            dst2 += strlen(wrapper_close);
            memcpy(dst2, world_close, strlen(world_close));
            dst2 += strlen(world_close);
            scan += strlen(world_close);
        } else {
            *dst2++ = *scan++;
        }
    }
    *dst2 = '\0';

    FILE *f = fopen(final_path, "wb");
    if (!f) {
        free(tmp2);
        free(tmp);
        free(text);
        return -1;
    }
    fwrite(tmp2, 1, strlen(tmp2), f);
    fclose(f);
    free(tmp2);
    free(tmp);
    free(text);
    printf("urdf_import: wrapped worldbody under base_link with freejoint "
           "('root')\n");
    return 0;
}

static mjModel *finalize_and_compile(mjSpec *spec, const char *final_path, mesh_table_t *meshes,
                                     char **urdf_text) {
    mjModel *model = mj_compile(spec, NULL);
    if (!model) {
        fprintf(stderr, "urdf_import: mj_compile failed on the final spec: %s\n",
                mjs_getError(spec));
        mj_deleteSpec(spec);
        mesh_table_free(meshes);
        free(*urdf_text);
        return NULL;
    }
    char re_save_err[1024] = "";
    if (mj_saveXML(spec, final_path, re_save_err, sizeof(re_save_err)) != 0) {
        fprintf(stderr, "urdf_import: cannot re-save final MJCF '%s': %s\n", final_path,
                re_save_err);
        mj_deleteModel(model);
        mj_deleteSpec(spec);
        mesh_table_free(meshes);
        free(*urdf_text);
        return NULL;
    }
    return model;
}

static void convert_motors_to_sites(mjModel *model, mjSpec *spec) {
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
        double axis_body[3];
        const double axis_world[3] = {model->jnt_axis[jid * 3 + 0], model->jnt_axis[jid * 3 + 1],
                                      model->jnt_axis[jid * 3 + 2]};
        double body_quat[4] = {model->body_quat[bid * 4 + 0], model->body_quat[bid * 4 + 1],
                               model->body_quat[bid * 4 + 2], model->body_quat[bid * 4 + 3]};
        double rbody[9];
        mju_quat2Mat(rbody, body_quat);
        for (int r = 0; r < 3; r++) {
            axis_body[r] = rbody[r + 0] * axis_world[0] + rbody[r + 3] * axis_world[1] +
                           rbody[r + 6] * axis_world[2];
        }
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

static void tune_position_gains(mjModel *model, mjSpec *spec, const actuator_config_t *cfg) {
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

static int run_validation(mjModel *model, int expected_meshes, int expected_mesh_geoms) {
    int failures = 0;

    for (int bidx = 1; bidx < model->nbody; bidx++) {
        if (model->body_mass[bidx] < 0.0) {
            fprintf(stderr, "urdf_import: validation FAIL: body %d has negative mass %g\n", bidx,
                    model->body_mass[bidx]);
            failures++;
        }
        const double *inertia = &model->body_inertia[bidx * 3];
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
    for (int g = 0; g < model->ngeom; g++) {
        if (model->geom_type[g] == mjGEOM_MESH) {
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
        for (int q = 0; q < model->nq; q++) {
            if (!isfinite(data->qpos[q])) {
                nan_detected = 1;
            }
        }
        for (int v = 0; v < model->nv; v++) {
            if (!isfinite(data->qvel[v])) {
                nan_detected = 1;
            }
        }
        double e = data->energy[0] + data->energy[1];
        if (e > e_max) {
            e_max = e;
        }
        if (i == steps / 2) {
            e_mid = e;
        }
        e_end = e;
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

static int validate_output_name(const char *name) {
    if (!name || name[0] == '\0') {
        return -1;
    }
    for (const char *ch = name; *ch; ch++) {
        if (!((*ch >= 'a' && *ch <= 'z') || (*ch >= '0' && *ch <= '9') || *ch == '_')) {
            return -1;
        }
    }
    return 0;
}

static int parse_args(int argc, char **argv, const char **model_override, const char **config_path,
                      const char **airframe_class) {
    for (int i = 3; i < argc; i++) {
        if (!strcmp(argv[i], "--model") && i + 1 < argc) {
            *model_override = argv[++i];
        } else if (!strcmp(argv[i], "--config") && i + 1 < argc) {
            *config_path = argv[++i];
        } else if (!strcmp(argv[i], "--class") && i + 1 < argc) {
            *airframe_class = argv[++i];
        } else if (!strcmp(argv[i], "-h") || !strcmp(argv[i], "--help")) {
            print_usage(argv[0]);
            return 2;
        } else {
            fprintf(stderr, "urdf_import: unknown option '%s'\n", argv[i]);
            print_usage(argv[0]);
            return 1;
        }
    }
    return 0;
}

int main(int argc, char **argv) {
    if (argc < 3) {
        print_usage(argv[0]);
        return 1;
    }

    const char *input_dir = argv[1];
    const char *output_name = argv[2];
    const char *model_override = NULL;
    const char *config_path = NULL;
    const char *airframe_class = NULL;

    int arg_rc = parse_args(argc, argv, &model_override, &config_path, &airframe_class);
    if (arg_rc == 1) {
        return 1;
    }
    if (arg_rc == 2) {
        return 0;
    }

    if (validate_output_name(output_name) != 0) {
        fprintf(stderr,
                "urdf_import: invalid output name '%s' (lowercase "
                "alphanumeric and underscore only)\n",
                output_name);
        return 1;
    }

    char model_path[1024];
    if (model_override) {
        snprintf(model_path, sizeof(model_path), "%s", model_override);
    } else {
        char model_file[512];
        if (find_model_file(input_dir, model_file, sizeof(model_file)) != 0) {
            return 1;
        }
        snprintf(model_path, sizeof(model_path), "%s/%s", input_dir, model_file);
    }

    char *urdf_text = NULL;
    size_t urdf_len = 0;
    if (read_file(model_path, &urdf_text, &urdf_len) != 0) {
        fprintf(stderr, "urdf_import: cannot read '%s'\n", model_path);
        return 1;
    }

    xml_reader_t root_reader;
    xml_reader_init(&root_reader, urdf_text, urdf_len);
    xml_element_t root_elem;
    int root_rc = xml_next_element(&root_reader, &root_elem);
    if (root_rc && strcmp(root_elem.name, "sdf") == 0) {
        char *urdf_out = NULL;
        size_t urdf_out_len = 0;
        if (sdf_to_urdf(urdf_text, urdf_len, &urdf_out, &urdf_out_len) != 0) {
            fprintf(stderr, "urdf_import: SDF translation failed for '%s'\n", model_path);
            free(urdf_text);
            return 1;
        }
        printf("urdf_import: translated SDF '%s' to URDF (%zu bytes)\n", model_path, urdf_out_len);
        free(urdf_text);
        urdf_text = urdf_out;
        urdf_len = urdf_out_len;
    } else if (!root_rc || strcmp(root_elem.name, "robot") != 0) {
        fprintf(stderr,
                "urdf_import: '%s' is neither a URDF (<robot>) nor an SDF "
                "(<sdf>) document\n",
                model_path);
        free(urdf_text);
        return 1;
    }
    xml_element_clear(&root_elem);

    mesh_table_t meshes;
    memset(&meshes, 0, sizeof(meshes));
    visual_mesh_t visuals[256];
    int nvisuals = 0;
    char root_link[256] = "";
    char root_mass[1024] = "";
    if (rewrite_meshes(input_dir, urdf_text, urdf_len, output_name, &meshes, visuals, &nvisuals,
                       (int)(sizeof(visuals) / sizeof(visuals[0])), root_link, root_mass,
                       sizeof(root_mass)) != 0) {
        free(urdf_text);
        return 1;
    }
    urdf_len = strlen(urdf_text);

    char name_buf[64];
    snprintf(name_buf, sizeof(name_buf), "%.*s", (int)(sizeof(name_buf) - 1), output_name);
    char out_dir[512];
    snprintf(out_dir, sizeof(out_dir), "model/airframes/%s", name_buf);
    if (ensure_dir(out_dir) != 0) {
        fprintf(stderr, "urdf_import: cannot create '%s'\n", out_dir);
        mesh_table_free(&meshes);
        free(urdf_text);
        return 1;
    }
    char rewritten_path[1024];
    snprintf(rewritten_path, sizeof(rewritten_path), "%s/%s_source.urdf", out_dir, name_buf);
    if (write_file(rewritten_path, urdf_text, urdf_len) != 0) {
        fprintf(stderr, "urdf_import: cannot write '%s'\n", rewritten_path);
        mesh_table_free(&meshes);
        free(urdf_text);
        return 1;
    }
    printf("urdf_import: rewrote mesh references -> '%s' (%d meshes)\n", rewritten_path,
           meshes.count);

    actuator_config_t cfg;
    int have_cfg = 0;
    if (config_path) {
        char cfg_err[512] = "";
        if (config_parse(config_path, &cfg, cfg_err, sizeof(cfg_err)) != 0) {
            fprintf(stderr, "urdf_import: %s\n", cfg_err);
            mesh_table_free(&meshes);
            free(urdf_text);
            return 1;
        }
        have_cfg = 1;
    }

    char error[1024] = "";
    mjSpec *spec = mj_parseXML(rewritten_path, NULL, error, sizeof(error));
    if (!spec) {
        fprintf(stderr, "urdf_import: %s\n", error);
        mesh_table_free(&meshes);
        free(urdf_text);
        return 1;
    }

    int nact = inject_actuators(spec, have_cfg ? &cfg : NULL);

    mjModel *model = mj_compile(spec, NULL);
    if (!model) {
        fprintf(stderr, "urdf_import: mj_compile failed on the injected spec: %s\n",
                mjs_getError(spec));
        mj_deleteSpec(spec);
        mesh_table_free(&meshes);
        free(urdf_text);
        return 1;
    }
    printf("  compiled (pre-save): nbody=%ld njnt=%ld nu=%ld\n", (long)model->nbody,
           (long)model->njnt, (long)model->nu);
    convert_motors_to_sites(model, spec);
    tune_position_gains(model, spec, have_cfg ? &cfg : NULL);
    mj_deleteModel(model);
    model = mj_compile(spec, NULL);
    if (!model) {
        fprintf(stderr, "urdf_import: mj_compile failed after gain tuning: %s\n",
                mjs_getError(spec));
        mj_deleteSpec(spec);
        mesh_table_free(&meshes);
        free(urdf_text);
        return 1;
    }
    mj_deleteModel(model);

    char save_err[1024] = "";
    char final_path[1024];
    snprintf(final_path, sizeof(final_path), "%s/%s.xml", out_dir, name_buf);
    if (mj_saveXML(spec, final_path, save_err, sizeof(save_err)) != 0) {
        fprintf(stderr, "urdf_import: cannot save final MJCF '%s': %s\n", final_path, save_err);
        mj_deleteSpec(spec);
        mesh_table_free(&meshes);
        free(urdf_text);
        return 1;
    }
    printf("  saved final MJCF -> '%s'\n", final_path);

    char parse_err[1024] = "";
    mjSpec *final_spec = mj_parseXML(final_path, NULL, parse_err, sizeof(parse_err));
    if (!final_spec) {
        fprintf(stderr, "urdf_import: cannot reparse saved MJCF: %s\n", parse_err);
        mj_deleteSpec(spec);
        mesh_table_free(&meshes);
        free(urdf_text);
        return 1;
    }
    mj_deleteSpec(spec);

    int nmesh_geoms = synthesize_visual_meshes(final_spec, visuals, nvisuals);
    if (finalize_spec(final_spec) != 0) {
        fprintf(stderr, "urdf_import: finalize_spec failed\n");
        mj_deleteSpec(final_spec);
        mesh_table_free(&meshes);
        free(urdf_text);
        return 1;
    }
    mjModel *final_model = finalize_and_compile(final_spec, final_path, &meshes, &urdf_text);
    if (!final_model) {
        return 1;
    }
    mj_deleteModel(final_model);
    mj_deleteSpec(final_spec);

    if (wrap_worldbody_for_scene(final_path, root_mass) != 0) {
        mesh_table_free(&meshes);
        free(urdf_text);
        return 1;
    }
    char wrap_err[1024] = "";
    final_spec = mj_parseXML(final_path, NULL, wrap_err, sizeof(wrap_err));
    if (!final_spec) {
        fprintf(stderr, "urdf_import: cannot reparse wrapped MJCF: %s\n", wrap_err);
        mesh_table_free(&meshes);
        free(urdf_text);
        return 1;
    }
    final_model = mj_compile(final_spec, NULL);
    if (!final_model) {
        fprintf(stderr, "urdf_import: mj_compile failed on the wrapped spec: %s\n",
                mjs_getError(final_spec));
        mj_deleteSpec(final_spec);
        mesh_table_free(&meshes);
        free(urdf_text);
        return 1;
    }

    int compiled_mesh_geoms = 0;
    for (int gi = 0; gi < final_model->ngeom; gi++) {
        if (final_model->geom_type[gi] == mjGEOM_MESH) {
            compiled_mesh_geoms++;
        }
    }

    if (!airframe_class) {
        airframe_class = "rotor_multicopter";
    }

    printf("urdf_import: parsed '%s' OK (output name '%s'%s)\n", model_path, output_name,
           config_path ? ", config applied" : "");
    printf("  compiled: nbody=%ld njnt=%ld nu=%ld nmesh=%ld mesh_geoms=%d\n",
           (long)final_model->nbody, (long)final_model->njnt, (long)final_model->nu,
           (long)final_model->nmesh, compiled_mesh_geoms);
    printf("  actuators injected: %d, visual meshes synthesized: %d (of %d)\n", nact, nmesh_geoms,
           nvisuals);

    if (run_validation(final_model, meshes.count, compiled_mesh_geoms) != 0) {
        mj_deleteModel(final_model);
        mj_deleteSpec(final_spec);
        mesh_table_free(&meshes);
        free(urdf_text);
        return 1;
    }

    if (emit_airframe_spec(final_spec, output_name, airframe_class, have_cfg ? &cfg : NULL) != 0) {
        mj_deleteModel(final_model);
        mj_deleteSpec(final_spec);
        mesh_table_free(&meshes);
        free(urdf_text);
        return 1;
    }

    mj_deleteModel(final_model);
    mj_deleteSpec(final_spec);
    mesh_table_free(&meshes);
    free(urdf_text);
    return 0;
}
