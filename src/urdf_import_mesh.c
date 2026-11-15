#include "urdf_import_internal.h"

#include <dirent.h>
#include <errno.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>

static const char *mesh_basename(const char *path) {
    const char *base = strrchr(path, '/');
    return base ? base + 1 : path;
}

void imp_mesh_table_free(mesh_table_t *t) {
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
    if (scale_text && imp_parse_vec3(scale_text, vis->scale)) {
        // scale parsed
    }
    (*nvisuals)++;
}

static int handle_mesh_element(const xml_element_t *elem, const char *input_dir, mesh_table_t *table,
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

enum {
    MAX_NAMES = 256,
    NAME_BYTES = 256,
    MASS_TEXT_BYTES = 64,
    INERTIA_TEXT_BYTES = 256,
    ROOT_SPEC_BYTES = 512,
    PATH_BYTES = 1024,
    PATTERN_BYTES = 1100
};

typedef struct {
    char cur_link[NAME_BYTES];
    char link_names[MAX_NAMES][NAME_BYTES];
    int link_count;
    char child_names[MAX_NAMES][NAME_BYTES];
    int child_count;
    int saw_root_inertial;
    int in_root_inertial;
    int have_root_mass;
    char root_mass_text[MASS_TEXT_BYTES];
    char root_inertia_text[INERTIA_TEXT_BYTES];
    int in_visual;
    int have_rpy;
    double origin_xyz[3];
    double origin_rpy[3];
} scan_state_t;

static void copy_name(char *dst, size_t cap, const char *src) {
    size_t length = strnlen(src, cap - 1);
    memcpy(dst, src, length);
    dst[length] = '\0';
}

static void scan_link(scan_state_t *st, const xml_element_t *elem) {
    const char *name = xml_attr(elem, "name");
    if (name) {
        copy_name(st->cur_link, sizeof(st->cur_link), name);
        if (st->link_count < MAX_NAMES) {
            copy_name(st->link_names[st->link_count], NAME_BYTES, name);
            st->link_count++;
        }
    }
    st->in_visual = 0;
    st->have_rpy = 0;
}

static void scan_child(scan_state_t *st, const xml_element_t *elem) {
    const char *child = xml_attr(elem, "link");
    if (child && st->child_count < MAX_NAMES) {
        copy_name(st->child_names[st->child_count], NAME_BYTES, child);
        st->child_count++;
    }
}

static void scan_root_inertia(scan_state_t *st, const xml_element_t *elem) {
    const char *ixx = xml_attr(elem, "ixx");
    const char *iyy = xml_attr(elem, "iyy");
    const char *izz = xml_attr(elem, "izz");
    if (ixx && iyy && izz && st->root_inertia_text[0] == '\0') {
        enum { INERTIA_TOKEN_BYTES = INERTIA_TEXT_BYTES / 3 };
        char xx[INERTIA_TOKEN_BYTES];
        char yy[INERTIA_TOKEN_BYTES];
        char zz[INERTIA_TOKEN_BYTES];
        copy_name(xx, sizeof(xx), ixx);
        copy_name(yy, sizeof(yy), iyy);
        copy_name(zz, sizeof(zz), izz);
        snprintf(st->root_inertia_text, sizeof(st->root_inertia_text), "%s %s %s", xx, yy, zz);
    }
}

static void scan_origin(scan_state_t *st, const xml_element_t *elem) {
    const char *xyz = xml_attr(elem, "xyz");
    const char *rpy = xml_attr(elem, "rpy");
    if (xyz) {
        (void)imp_parse_vec3(xyz, st->origin_xyz);
    }
    if (rpy && imp_parse_vec3(rpy, st->origin_rpy)) {
        st->have_rpy = 1;
    }
}

static void scan_inertial_parts(scan_state_t *st, const xml_element_t *elem) {
    if (strcmp(elem->name, "mass") == 0) {
        const char *mass = xml_attr(elem, "value");
        if (mass && !st->have_root_mass) {
            copy_name(st->root_mass_text, sizeof(st->root_mass_text), mass);
            st->have_root_mass = 1;
        }
    } else if (strcmp(elem->name, "inertia") == 0) {
        scan_root_inertia(st, elem);
    } else if (strcmp(elem->name, "link") == 0) {
        st->in_root_inertial = 0;
    }
}

static int scan_element(scan_state_t *st, xml_element_t *elem, const char *input_dir,
                        mesh_table_t *table, visual_mesh_t *visuals, int *nvisuals,
                        int visuals_cap) {
    const char *tag = elem->name;
    if (strcmp(tag, "link") == 0) {
        scan_link(st, elem);
    } else if (strcmp(tag, "child") == 0) {
        scan_child(st, elem);
    } else if (strcmp(tag, "inertial") == 0) {
        st->in_root_inertial = !st->saw_root_inertial && strcmp(st->cur_link, "import_root") != 0;
        st->saw_root_inertial = 1;
    } else if (st->in_root_inertial &&
               (strcmp(tag, "mass") == 0 || strcmp(tag, "inertia") == 0)) {
        scan_inertial_parts(st, elem);
    } else if (strcmp(tag, "visual") == 0) {
        st->in_visual = 1;
        st->have_rpy = 0;
    } else if (strcmp(tag, "collision") == 0) {
        st->in_visual = 0;
        st->have_rpy = 0;
    } else if (strcmp(tag, "origin") == 0) {
        scan_origin(st, elem);
    } else if (strcmp(tag, "mesh") == 0) {
        return handle_mesh_element(elem, input_dir, table, st->cur_link, st->in_visual, st->have_rpy,
                                   st->origin_xyz, st->origin_rpy, visuals, nvisuals, visuals_cap);
    }
    return 0;
}

static const char *find_root_link(const scan_state_t *st) {
    const char *root = st->link_count > 0 ? st->link_names[0] : "";
    for (int idx = 0; idx < st->link_count; idx++) {
        int is_child = 0;
        for (int other = 0; other < st->child_count; other++) {
            if (strcmp(st->link_names[idx], st->child_names[other]) == 0) {
                is_child = 1;
                break;
            }
        }
        if (!is_child) {
            return st->link_names[idx];
        }
    }
    return root;
}

static void report_root(const scan_state_t *st, char *root_link_out, char *root_mass_out,
                        size_t root_mass_sz) {
    snprintf(root_link_out, NAME_BYTES, "%.*s", NAME_BYTES - 1, find_root_link(st));
    if (root_mass_out == NULL || root_mass_sz == 0) {
        return;
    }
    if (!st->have_root_mass) {
        root_mass_out[0] = '\0';
        return;
    }
    char spec[ROOT_SPEC_BYTES];
    snprintf(spec, sizeof(spec), "mass=\"%s\" diaginertia=\"%s\"", st->root_mass_text,
             st->root_inertia_text[0] ? st->root_inertia_text : "1e-06 1e-06 1e-06");
    size_t len = strlen(spec);
    len = len >= root_mass_sz ? root_mass_sz - 1 : len;
    memcpy(root_mass_out, spec, len);
    root_mass_out[len] = '\0';
}

static int copy_meshes(const mesh_table_t *table, const char *output_name) {
    char meshes_dir[512];
    snprintf(meshes_dir, sizeof(meshes_dir), "model/airframes/%.*s/meshes",
             (int)(sizeof(meshes_dir) - 30), output_name);
    if (imp_ensure_dir(meshes_dir) != 0) {
        fprintf(stderr, "urdf_import: cannot create '%s'\n", meshes_dir);
        return -1;
    }
    for (int idx = 0; idx < table->count; idx++) {
        char dst[PATH_BYTES];
        snprintf(dst, sizeof(dst), "%s/%.*s", meshes_dir,
                 (int)(sizeof(dst) - sizeof(meshes_dir) - 1), table->entries[idx].stored_name);
        if (imp_copy_file(table->entries[idx].resolved, dst) != 0) {
            fprintf(stderr, "urdf_import: failed to copy mesh '%s' -> '%s'\n",
                    table->entries[idx].resolved, dst);
            return -1;
        }
        printf("  mesh: %s -> %s\n", table->entries[idx].resolved, dst);
    }
    return 0;
}

static void patch_references(const mesh_table_t *table, char *urdf_text) {
    for (int idx = 0; idx < table->count; idx++) {
        char needle[PATTERN_BYTES];
        char repl[PATTERN_BYTES];
        snprintf(needle, sizeof(needle), "filename=\"%s\"", table->entries[idx].orig_ref);
        snprintf(repl, sizeof(repl), "filename=\"meshes/%s\"", table->entries[idx].stored_name);
        size_t needle_len = strlen(needle);
        size_t repl_len = strlen(repl);
        char *hit = strstr(urdf_text, needle);
        while (hit) {
            memmove(hit + repl_len, hit + needle_len, strlen(hit + needle_len) + 1);
            memcpy(hit, repl, repl_len);
            hit = strstr(hit + repl_len, needle);
        }
    }
}

int imp_rewrite_meshes(const char *input_dir, char *urdf_text, size_t urdf_len,
                       const char *output_name, mesh_table_t *table, visual_mesh_t *visuals,
                       int *nvisuals, int visuals_cap, char *root_link_out, char *root_mass_out,
                       size_t root_mass_sz) {
    xml_reader_t reader;
    xml_reader_init(&reader, urdf_text, urdf_len);
    xml_element_t elem;
    scan_state_t *st = calloc(1, sizeof(*st));
    if (st == NULL) {
        return -1;
    }
    while (xml_next_element(&reader, &elem)) {
        int status = scan_element(st, &elem, input_dir, table, visuals, nvisuals, visuals_cap);
        xml_element_clear(&elem);
        if (status != 0) {
            imp_mesh_table_free(table);
            free(st);
            return -1;
        }
    }
    if (root_link_out) {
        report_root(st, root_link_out, root_mass_out, root_mass_sz);
    }
    free(st);
    if (copy_meshes(table, output_name) != 0) {
        imp_mesh_table_free(table);
        return -1;
    }
    patch_references(table, urdf_text);
    return 0;
}

int imp_synthesize_visual_meshes(mjSpec *spec, const visual_mesh_t *visuals, int n) {
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
