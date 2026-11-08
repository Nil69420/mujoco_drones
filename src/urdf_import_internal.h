#ifndef MUJOCO_DRONES_URDF_IMPORT_INTERNAL_H
#define MUJOCO_DRONES_URDF_IMPORT_INTERNAL_H

#include "urdf_import_config.h"
#include "urdf_import_sdf.h"
#include "urdf_import_xml.h"

#include <mujoco/mujoco.h>
#include <stddef.h>

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

void imp_convert_motors_to_sites(mjModel *model, mjSpec *spec);
int imp_copy_file(const char *src, const char *dst);
int imp_emit_airframe_spec(mjSpec *spec, const char *output_name, const char *airframe_class,
                              const actuator_config_t *cfg);
int imp_ensure_dir(const char *path);
mjModel *imp_finalize_and_compile(mjSpec *spec, const char *final_path, mesh_table_t *meshes,
                                     char **urdf_text);
int imp_finalize_spec(mjSpec *spec);
int imp_find_model_file(const char *dir, char *out, size_t out_sz);
int imp_inject_actuators(mjSpec *spec, actuator_config_t *cfg);
void imp_mesh_table_free(mesh_table_t *t);
int imp_parse_vec3(const char *s, double out[3]);
int imp_read_file(const char *path, char **out, size_t *out_len);
int imp_rewrite_meshes(const char *input_dir, char *urdf_text, size_t urdf_len,
                          const char *output_name, mesh_table_t *table, visual_mesh_t *visuals,
                          int *nvisuals, int visuals_cap, char *root_link_out, char *root_mass_out,
                          size_t root_mass_sz);
int imp_run_validation(mjModel *model, int expected_meshes, int expected_mesh_geoms);
int imp_synthesize_visual_meshes(mjSpec *spec, const visual_mesh_t *visuals, int n);
void imp_tune_position_gains(mjModel *model, mjSpec *spec, const actuator_config_t *cfg);
int imp_wrap_worldbody_for_scene(const char *final_path, const char *root_mass);
int imp_write_file(const char *path, const char *data, size_t len);

#endif
