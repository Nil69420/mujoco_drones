#ifndef MUJOCO_DRONES_URDF_IMPORT_XML_H
#define MUJOCO_DRONES_URDF_IMPORT_XML_H

#include <stddef.h>

#define XML_MAX_ATTRS 32

typedef struct {
    char *name;
    char *value;
} xml_attr_t;

typedef struct {
    char *name;
    xml_attr_t attrs[XML_MAX_ATTRS];
    int num_attrs;
    int is_self_closing;
    int is_close;
    char *content;
} xml_element_t;

typedef struct {
    char *data;
    size_t len;
    size_t pos;
    int report_closes;
    int capture_text;
} xml_reader_t;

void xml_reader_init(xml_reader_t *r, char *data, size_t len);
void xml_reader_set_report_closes(xml_reader_t *r, int report_closes);
void xml_reader_set_capture_text(xml_reader_t *r, int capture_text);
int xml_next_element(xml_reader_t *r, xml_element_t *out);
void xml_element_clear(xml_element_t *e);
const char *xml_attr(const xml_element_t *e, const char *name);

#endif
