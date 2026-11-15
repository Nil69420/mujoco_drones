#define _GNU_SOURCE
#include "urdf_import_sdf.h"
#include "urdf_import_xml.h"

#include <ctype.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define SDF_MAX_CHILDREN 32
#define SDF_MAX_ATTRS 8
#define SDF_MAX_DEPTH 64

typedef struct sdf_node {
    char *name;
    struct {
        char *name;
        char *value;
    } attrs[SDF_MAX_ATTRS];
    int nattrs;
    char *text;
    struct sdf_node *children[SDF_MAX_CHILDREN];
    int nchildren;
} sdf_node_t;

typedef struct {
    char *data;
    size_t len;
    size_t cap;
} sb_t;

static int sb_reserve(sb_t *sb, size_t extra) {
    if (sb->len + extra + 1 <= sb->cap) {
        return 0;
    }
    size_t new_cap = sb->cap ? sb->cap * 2 : 4096;
    while (new_cap < sb->len + extra + 1) {
        new_cap *= 2;
    }
    char *nd = realloc(sb->data, new_cap);
    if (!nd) {
        return -1;
    }
    sb->data = nd;
    sb->cap = new_cap;
    return 0;
}

static int sb_printf(sb_t *sb, const char *fmt, ...) {
    va_list args;
    va_start(args, fmt);
    va_list args_copy;
    va_copy(args_copy, args);
    int need = vsnprintf(NULL, 0, fmt, args_copy);  // NOLINT(clang-analyzer-valist.Uninitialized,clang-diagnostic-format-nonliteral)
    va_end(args_copy);
    if (need < 0) {
        va_end(args);
        return -1;
    }
    if (sb_reserve(sb, (size_t)need) != 0) {
        va_end(args);
        return -1;
    }
    vsnprintf(sb->data + sb->len, sb->cap - sb->len, fmt, args);  // NOLINT(clang-diagnostic-format-nonliteral)
    va_end(args);
    sb->len += (size_t)need;
    return 0;
}

static sdf_node_t *node_new(void) {
    sdf_node_t *n = calloc(1, sizeof(*n));
    return n;
}

static void node_free(sdf_node_t *n) { // NOLINT(misc-no-recursion)
    if (!n) {
        return;
    }
    free(n->name);
    for (int i = 0; i < n->nattrs; i++) {
        free(n->attrs[i].name);
        free(n->attrs[i].value);
    }
    free(n->text);
    for (int i = 0; i < n->nchildren; i++) {
        node_free(n->children[i]);
    }
    free(n);
}

static void warn_dropped(const char *name, const char *parent) {
    fprintf(stderr,
            "urdf_import: SDF: dropping <%s> under <%s> (outside the "
            "supported subset)\n",
            name, parent);
}

static const char *node_attr(const sdf_node_t *n, const char *name) {
    for (int i = 0; i < n->nattrs; i++) {
        if (strcmp(n->attrs[i].name, name) == 0) {
            return n->attrs[i].value;
        }
    }
    return NULL;
}

static const sdf_node_t *node_child(const sdf_node_t *n, const char *name) {
    for (int i = 0; i < n->nchildren; i++) {
        if (strcmp(n->children[i]->name, name) == 0) {
            return n->children[i];
        }
    }
    return NULL;
}

static int pose_is_identity(const char *pose) {
    if (!pose) {
        return 1;
    }
    const char *cursor = pose;
    for (int i = 0; i < 6; i++) {
        char *end = NULL;
        double value = strtod(cursor, &end);
        if (end == cursor || value != 0.0) {
            return 0;
        }
        cursor = end;
    }
    return 1;
}

static int node_text_double(const sdf_node_t *n, const char *child, double *out) {
    const sdf_node_t *kid = node_child(n, child);
    if (!kid || !kid->text) {
        return -1;
    }
    char *end = NULL;
    *out = strtod(kid->text, &end);
    return end != kid->text && *end == '\0' ? 0 : -1;
}

static int parse_pose(const char *text, double out[6]) {
    if (!text) {
        return -1;
    }
    const char *cursor = text;
    for (int i = 0; i < 6; i++) {
        char *end = NULL;
        out[i] = strtod(cursor, &end);
        if (end == cursor) {
            return -1;
        }
        cursor = end;
    }
    return 0;
}

static void emit_pose_origin(sb_t *sb, const char *pose_text) {
    double vals[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    double parsed[6];
    if (pose_text != NULL) {
        if (parse_pose(pose_text, parsed) == 0) {
            memcpy(vals, parsed, sizeof(vals));
        } else {
            fprintf(stderr,
                    "urdf_import: SDF: warning: unreadable pose '%s'; using "
                    "identity\n",
                    pose_text);
        }
    }
    sb_printf(sb, "<origin xyz=\"%g %g %g\" rpy=\"%g %g %g\"/>", vals[0], vals[1], vals[2], vals[3],
              vals[4], vals[5]);
}

static void emit_geometry(sb_t *sb, const sdf_node_t *geom, int is_collision) {
    if (!geom) {
        return;
    }
    const sdf_node_t *geom_child = node_child(geom, "box");
    if (geom_child) {
        const sdf_node_t *size = node_child(geom_child, "size");
        sb_printf(sb, "<geometry><box size=\"%s\"/></geometry>",
                  size && size->text ? size->text : "1 1 1");
        return;
    }
    geom_child = node_child(geom, "cylinder");
    if (geom_child) {
        const sdf_node_t *radius = node_child(geom_child, "radius");
        const sdf_node_t *length = node_child(geom_child, "length");
        sb_printf(sb, "<geometry><cylinder radius=\"%s\" length=\"%s\"/></geometry>",
                  radius && radius->text ? radius->text : "1",
                  length && length->text ? length->text : "1");
        return;
    }
    geom_child = node_child(geom, "sphere");
    if (geom_child) {
        const sdf_node_t *radius = node_child(geom_child, "radius");
        sb_printf(sb, "<geometry><sphere radius=\"%s\"/></geometry>",
                  radius && radius->text ? radius->text : "1");
        return;
    }
    geom_child = node_child(geom, "mesh");
    if (geom_child) {
        const sdf_node_t *uri = node_child(geom_child, "uri");
        const sdf_node_t *scale = node_child(geom_child, "scale");
        const char *uri_text = uri && uri->text ? uri->text : "";
        const char *base = strrchr(uri_text, '/');
        base = base ? base + 1 : uri_text;
        char mesh_file[512];
        const char *dot = strrchr(base, '.');
        int is_dae = dot && strcmp(dot, ".dae") == 0;
        size_t keep_len = is_dae ? (size_t)(dot - base) : strlen(base);
        if (keep_len >= sizeof(mesh_file) - 4) {
            keep_len = sizeof(mesh_file) - 4;
        }
        memcpy(mesh_file, base, keep_len);
        if (is_dae) {
            mesh_file[keep_len] = '.';
            memcpy(mesh_file + keep_len + 1, "obj", 3);
            keep_len += 4;
        }
        mesh_file[keep_len] = '\0';
        if (is_collision) {
            fprintf(stderr,
                    "urdf_import: SDF: warning: collision mesh '%s' dropped (native "
                    "URDF import cannot load collision meshes)\n",
                    mesh_file);
            return;
        }
        if (scale && scale->text) {
            sb_printf(sb, "<geometry><mesh filename=\"%s\" scale=\"%s\"/></geometry>", mesh_file,
                      scale->text);
        } else {
            sb_printf(sb, "<geometry><mesh filename=\"%s\"/></geometry>", mesh_file);
        }
        return;
    }
    fprintf(stderr,
            "urdf_import: SDF: warning: unsupported geometry in <%s>; "
            "emitting a unit box\n",
            is_collision ? "collision" : "visual");
    sb_printf(sb, "<geometry><box size=\"1 1 1\"/></geometry>");
}

static void emit_link(sb_t *sb, const sdf_node_t *link) {
    const char *name = node_attr(link, "name");
    if (!name) {
        name = "unnamed_link";
    }
    sb_printf(sb, "  <link name=\"%s\">\n", name);

    const sdf_node_t *inertial = node_child(link, "inertial");
    const sdf_node_t *ipose = inertial ? node_child(inertial, "pose") : NULL;
    sb_printf(sb, "    <inertial>");
    emit_pose_origin(sb, ipose && ipose->text ? ipose->text : NULL);
    double mass = 1e-6;
    if (inertial && node_text_double(inertial, "mass", &mass) != 0) {
        fprintf(stderr,
                "urdf_import: SDF: warning: link '%s' mass unreadable; "
                "using %g\n",
                name, mass);
    }
    double ixx = 0.0, ixy = 0.0, ixz = 0.0, iyy = 0.0, iyz = 0.0, izz = 0.0;
    if (inertial) {
        const sdf_node_t *in = node_child(inertial, "inertia");
        if (in) {
            node_text_double(in, "ixx", &ixx);
            node_text_double(in, "ixy", &ixy);
            node_text_double(in, "ixz", &ixz);
            node_text_double(in, "iyy", &iyy);
            node_text_double(in, "iyz", &iyz);
            node_text_double(in, "izz", &izz);
        } else {
            fprintf(stderr,
                    "urdf_import: SDF: warning: link '%s' has no <inertia>; "
                    "using zeros\n",
                    name);
        }
    } else {
        fprintf(stderr,
                "urdf_import: SDF: warning: link '%s' has no <inertial>; "
                "using defaults\n",
                name);
    }
    sb_printf(sb,
              "<mass value=\"%g\"/><inertia ixx=\"%g\" ixy=\"%g\" "
              "ixz=\"%g\" iyy=\"%g\" iyz=\"%g\" izz=\"%g\"/></inertial>\n",
              mass, ixx, ixy, ixz, iyy, iyz, izz);

    for (int i = 0; i < link->nchildren; i++) {
        const sdf_node_t *child = link->children[i];
        if (strcmp(child->name, "visual") == 0 || strcmp(child->name, "collision") == 0) {
            int is_collision = strcmp(child->name, "collision") == 0;
            const sdf_node_t *pose = node_child(child, "pose");
            const char *pose_text = pose && pose->text ? pose->text : "0 0 0 0 0 0";
            sb_printf(sb, "    <%s name=\"%s\">", child->name,
                      node_attr(child, "name") ? node_attr(child, "name") : "");
            emit_pose_origin(sb, pose_text);
            emit_geometry(sb, node_child(child, "geometry"), is_collision);
            sb_printf(sb, "</%s>\n", child->name);
        }
    }
    sb_printf(sb, "  </link>\n");
}

static int collect_link_names(const sdf_node_t *model, const char **out, int cap) {
    int count = 0;
    for (int i = 0; i < model->nchildren; i++) {
        const sdf_node_t *child = model->children[i];
        if (strcmp(child->name, "link") == 0 && count < cap) {
            const char *link_name = node_attr(child, "name");
            if (link_name) {
                out[count++] = link_name;
            }
        }
    }
    return count;
}

static int link_exists(const char *name, const char *const *links, int nlinks) {
    for (int i = 0; i < nlinks; i++) {
        if (strcmp(links[i], name) == 0) {
            return 1;
        }
    }
    return 0;
}

static void emit_joint(sb_t *sb, const sdf_node_t *joint, const char *const *links, int nlinks) {
    const char *name = node_attr(joint, "name");
    const char *type = node_attr(joint, "type");
    if (!name) {
        name = "unnamed_joint";
    }
    if (!type) {
        fprintf(stderr,
                "urdf_import: SDF: warning: joint '%s' has no type; "
                "emitting fixed\n",
                name);
        type = "fixed";
    } else if (strcmp(type, "revolute") != 0 && strcmp(type, "continuous") != 0 &&
               strcmp(type, "fixed") != 0 && strcmp(type, "prismatic") != 0) {
        fprintf(stderr,
                "urdf_import: SDF: warning: joint '%s' type '%s' unsupported; "
                "emitting fixed (tree stays connected)\n",
                name, type);
        type = "fixed";
    }

    const sdf_node_t *parent = node_child(joint, "parent");
    const sdf_node_t *child = node_child(joint, "child");
    const char *parent_name = parent && parent->text ? parent->text : "";
    const char *child_name = child && child->text ? child->text : "";
    if (!link_exists(parent_name, links, nlinks) || !link_exists(child_name, links, nlinks)) {
        fprintf(stderr,
                "urdf_import: SDF: warning: joint '%s' dropped (link '%s' or '%s' "
                "not a model link - comes from an external include)\n",
                name, parent_name, child_name);
        return;
    }
    sb_printf(sb, "  <joint name=\"%s\" type=\"%s\">\n", name, type);
    const sdf_node_t *pose = node_child(joint, "pose");
    const char *pose_text = pose && pose->text ? pose->text : "0 0 0 0 0 0";
    sb_printf(sb, "    ");
    emit_pose_origin(sb, pose_text);
    sb_printf(sb, "\n");
    sb_printf(sb, "    <parent link=\"%s\"/>\n", parent_name);
    sb_printf(sb, "    <child link=\"%s\"/>\n", child_name);
    const sdf_node_t *axis = node_child(joint, "axis");
    if (axis) {
        const sdf_node_t *xyz = node_child(axis, "xyz");
        if (xyz && xyz->text) {
            sb_printf(sb, "    <axis xyz=\"%s\"/>\n", xyz->text);
        }
        const sdf_node_t *limit = node_child(axis, "limit");
        if (limit) {
            const sdf_node_t *lower = node_child(limit, "lower");
            const sdf_node_t *upper = node_child(limit, "upper");
            sb_printf(sb,
                      "    <limit lower=\"%s\" upper=\"%s\" effort=\"1000\" "
                      "velocity=\"100\"/>\n",
                      lower && lower->text ? lower->text : "0",
                      upper && upper->text ? upper->text : "0");
        }
    }
    sb_printf(sb, "  </joint>\n");
}

static void emit_dropped_children(const sdf_node_t *parent) {
    static const char *kept[] = {
        "link", "joint", "pose",  "inertial", "mass",     "inertia",   "ixx",      "ixy",
        "ixz",  "iyy",   "iyz",   "izz",      "visual",   "collision", "geometry", "mesh",
        "uri",  "scale", "box",   "size",     "cylinder", "radius",    "length",   "sphere",
        "axis", "xyz",   "limit", "lower",    "upper",    "parent",    "child"};
    for (int i = 0; i < parent->nchildren; i++) {
        const char *name = parent->children[i]->name;
        int found = 0;
        for (size_t k = 0; k < sizeof(kept) / sizeof(kept[0]); k++) {
            if (strcmp(name, kept[k]) == 0) {
                found = 1;
                break;
            }
        }
        if (!found) {
            warn_dropped(name, parent->name);
        }
    }
}

int sdf_to_urdf(const char *sdf_text, size_t sdf_len, char **out_urdf, size_t *out_len) {
    xml_reader_t reader;
    xml_reader_init(&reader, (char *)sdf_text, sdf_len);
    xml_reader_set_report_closes(&reader, 1);
    xml_reader_set_capture_text(&reader, 1);

    sdf_node_t *root = NULL;
    sdf_node_t *stack[SDF_MAX_DEPTH];
    int depth = 0;
    xml_element_t elem;

    while (xml_next_element(&reader, &elem)) {
        if (elem.is_close) {
            if (depth > 0) {
                depth--;
            }
            xml_element_clear(&elem);
            continue;
        }
        sdf_node_t *node = node_new();
        if (!node) {
            node_free(root);
            xml_element_clear(&elem);
            return -1;
        }
        node->name = elem.name;
        elem.name = NULL;
        for (int i = 0; i < elem.num_attrs && node->nattrs < SDF_MAX_ATTRS; i++) {
            node->attrs[node->nattrs].name = elem.attrs[i].name;
            node->attrs[node->nattrs].value = elem.attrs[i].value;
            elem.attrs[i].name = NULL;
            elem.attrs[i].value = NULL;
            node->nattrs++;
        }
        node->text = elem.content;
        elem.content = NULL;
        int self_closing = elem.is_self_closing;
        xml_element_clear(&elem);

        if (!root) {
            root = node;
        } else if (depth > 0) {
            sdf_node_t *parent = stack[depth - 1];
            if (parent->nchildren >= SDF_MAX_CHILDREN) {
                fprintf(stderr,
                        "urdf_import: SDF: too many children under <%s>; "
                        "dropping <%s>\n",
                        parent->name, node->name);
                node_free(node);
                xml_element_clear(&elem);
                continue;
            }
            parent->children[parent->nchildren++] = node;
        } else {
            node_free(node);
            xml_element_clear(&elem);
            continue;
        }
        if (!self_closing) {
            if (depth >= SDF_MAX_DEPTH) {
                fprintf(stderr, "urdf_import: SDF: document nested too deeply\n");
                node_free(root);
                return -1;
            }
            stack[depth++] = node;
        }
    }

    const sdf_node_t *model = NULL;
    const sdf_node_t *sdf_root = root;
    if (sdf_root && strcmp(sdf_root->name, "sdf") == 0) {
        model = node_child(sdf_root, "model");
    } else if (sdf_root && strcmp(sdf_root->name, "model") == 0) {
        model = sdf_root;
    }
    if (!model) {
        fprintf(stderr, "urdf_import: SDF: no <model> element found\n");
        node_free(root);
        return -1;
    }

    const sdf_node_t *model_pose = node_child(model, "pose");
    if (model_pose && model_pose->text && !pose_is_identity(model_pose->text)) {
        fprintf(stderr,
                "urdf_import: SDF: warning: model pose '%s' ignored (vehicle "
                "placement is scenario-controlled)\n",
                model_pose->text);
    }

    const char *link_names[SDF_MAX_CHILDREN];
    int nlink_names = collect_link_names(model, link_names, SDF_MAX_CHILDREN);

    sb_t sb;
    memset(&sb, 0, sizeof(sb));
    const char *model_name = node_attr(model, "name");
    sb_printf(&sb, "<?xml version=\"1.0\"?>\n<robot name=\"%s\">\n",
              model_name ? model_name : "model");

    for (int i = 0; i < model->nchildren; i++) {
        const sdf_node_t *child = model->children[i];
        if (strcmp(child->name, "link") == 0) {
            emit_link(&sb, child);
        } else if (strcmp(child->name, "joint") == 0) {
            emit_joint(&sb, child, link_names, nlink_names);
        }
    }
    emit_dropped_children(model);
    sb_printf(&sb, "</robot>\n");

    node_free(root);
    if (!sb.data) {
        return -1;
    }
    *out_urdf = sb.data;
    *out_len = sb.len;
    return 0;
}
