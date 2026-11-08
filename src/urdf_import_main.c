#include "urdf_import_internal.h"

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

static int load_model_text(const char *model_path, char **text_out, size_t *len_out) {
    char *urdf_text = NULL;
    size_t urdf_len = 0;
    if (imp_read_file(model_path, &urdf_text, &urdf_len) != 0) {
        fprintf(stderr, "urdf_import: cannot read '%s'\n", model_path);
        return -1;
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
            return -1;
        }
        printf("urdf_import: translated SDF '%s' to URDF (%zu bytes)\n", model_path, urdf_out_len);
        free(urdf_text);
        urdf_text = urdf_out;
        urdf_len = urdf_out_len;
    } else if (!root_rc || strcmp(root_elem.name, "robot") != 0) {
        fprintf(stderr,
                "urdf_import: '%s' is neither a URDF (<robot>) nor an SDF (<sdf>) document\n",
                model_path);
        free(urdf_text);
        return -1;
    }
    xml_element_clear(&root_elem);
    *text_out = urdf_text;
    *len_out = urdf_len;
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
        if (imp_find_model_file(input_dir, model_file, sizeof(model_file)) != 0) {
            return 1;
        }
        snprintf(model_path, sizeof(model_path), "%s/%s", input_dir, model_file);
    }

    char *urdf_text = NULL;
    size_t urdf_len = 0;
    if (load_model_text(model_path, &urdf_text, &urdf_len) != 0) {
        return 1;
    }

    mesh_table_t meshes;
    memset(&meshes, 0, sizeof(meshes));
    visual_mesh_t visuals[256];
    int nvisuals = 0;
    char root_link[256] = "";
    char root_mass[1024] = "";
    if (imp_rewrite_meshes(input_dir, urdf_text, urdf_len, output_name, &meshes, visuals, &nvisuals,
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
    if (imp_ensure_dir(out_dir) != 0) {
        fprintf(stderr, "urdf_import: cannot create '%s'\n", out_dir);
        imp_mesh_table_free(&meshes);
        free(urdf_text);
        return 1;
    }
    char rewritten_path[1024];
    snprintf(rewritten_path, sizeof(rewritten_path), "%s/%s_source.urdf", out_dir, name_buf);
    if (imp_write_file(rewritten_path, urdf_text, urdf_len) != 0) {
        fprintf(stderr, "urdf_import: cannot write '%s'\n", rewritten_path);
        imp_mesh_table_free(&meshes);
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
            imp_mesh_table_free(&meshes);
            free(urdf_text);
            return 1;
        }
        have_cfg = 1;
    }

    char error[1024] = "";
    mjSpec *spec = mj_parseXML(rewritten_path, NULL, error, sizeof(error));
    if (!spec) {
        fprintf(stderr, "urdf_import: %s\n", error);
        imp_mesh_table_free(&meshes);
        free(urdf_text);
        return 1;
    }

    int nact = imp_inject_actuators(spec, have_cfg ? &cfg : NULL);

    mjModel *model = mj_compile(spec, NULL);
    if (!model) {
        fprintf(stderr, "urdf_import: mj_compile failed on the injected spec: %s\n",
                mjs_getError(spec));
        mj_deleteSpec(spec);
        imp_mesh_table_free(&meshes);
        free(urdf_text);
        return 1;
    }
    printf("  compiled (pre-save): nbody=%ld njnt=%ld nu=%ld\n", (long)model->nbody,
           (long)model->njnt, (long)model->nu);
    imp_convert_motors_to_sites(model, spec);
    imp_tune_position_gains(model, spec, have_cfg ? &cfg : NULL);
    mj_deleteModel(model);
    model = mj_compile(spec, NULL);
    if (!model) {
        fprintf(stderr, "urdf_import: mj_compile failed after gain tuning: %s\n",
                mjs_getError(spec));
        mj_deleteSpec(spec);
        imp_mesh_table_free(&meshes);
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
        imp_mesh_table_free(&meshes);
        free(urdf_text);
        return 1;
    }
    printf("  saved final MJCF -> '%s'\n", final_path);

    char parse_err[1024] = "";
    mjSpec *final_spec = mj_parseXML(final_path, NULL, parse_err, sizeof(parse_err));
    if (!final_spec) {
        fprintf(stderr, "urdf_import: cannot reparse saved MJCF: %s\n", parse_err);
        mj_deleteSpec(spec);
        imp_mesh_table_free(&meshes);
        free(urdf_text);
        return 1;
    }
    mj_deleteSpec(spec);

    int nmesh_geoms = imp_synthesize_visual_meshes(final_spec, visuals, nvisuals);
    if (imp_finalize_spec(final_spec) != 0) {
        fprintf(stderr, "urdf_import: finalize_spec failed\n");
        mj_deleteSpec(final_spec);
        imp_mesh_table_free(&meshes);
        free(urdf_text);
        return 1;
    }
    mjModel *final_model = imp_finalize_and_compile(final_spec, final_path, &meshes, &urdf_text);
    if (!final_model) {
        return 1;
    }
    mj_deleteModel(final_model);
    mj_deleteSpec(final_spec);

    if (imp_wrap_worldbody_for_scene(final_path, root_mass) != 0) {
        imp_mesh_table_free(&meshes);
        free(urdf_text);
        return 1;
    }
    char wrap_err[1024] = "";
    final_spec = mj_parseXML(final_path, NULL, wrap_err, sizeof(wrap_err));
    if (!final_spec) {
        fprintf(stderr, "urdf_import: cannot reparse wrapped MJCF: %s\n", wrap_err);
        imp_mesh_table_free(&meshes);
        free(urdf_text);
        return 1;
    }
    final_model = mj_compile(final_spec, NULL);
    if (!final_model) {
        fprintf(stderr, "urdf_import: mj_compile failed on the wrapped spec: %s\n",
                mjs_getError(final_spec));
        mj_deleteSpec(final_spec);
        imp_mesh_table_free(&meshes);
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

    if (imp_run_validation(final_model, meshes.count, compiled_mesh_geoms) != 0) {
        mj_deleteModel(final_model);
        mj_deleteSpec(final_spec);
        imp_mesh_table_free(&meshes);
        free(urdf_text);
        return 1;
    }

    if (imp_emit_airframe_spec(final_spec, output_name, airframe_class, have_cfg ? &cfg : NULL) != 0) {
        mj_deleteModel(final_model);
        mj_deleteSpec(final_spec);
        imp_mesh_table_free(&meshes);
        free(urdf_text);
        return 1;
    }

    mj_deleteModel(final_model);
    mj_deleteSpec(final_spec);
    imp_mesh_table_free(&meshes);
    free(urdf_text);
    return 0;
}
