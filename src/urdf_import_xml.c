#define _GNU_SOURCE
#include "urdf_import_xml.h"

#include <ctype.h>
#include <stdlib.h>
#include <string.h>

void xml_reader_init(xml_reader_t *r, char *data, size_t len) {
    r->data = data;
    r->len = len;
    r->pos = 0;
    r->report_closes = 0;
    r->capture_text = 0;
}

void xml_reader_set_report_closes(xml_reader_t *r, int report_closes) {
    r->report_closes = report_closes;
}

void xml_reader_set_capture_text(xml_reader_t *r, int capture_text) {
    r->capture_text = capture_text;
}

void xml_element_clear(xml_element_t *e) {
    free(e->name);
    for (int i = 0; i < e->num_attrs; i++) {
        free(e->attrs[i].name);
        free(e->attrs[i].value);
    }
    free(e->content);
    memset(e, 0, sizeof(*e));
}

const char *xml_attr(const xml_element_t *e, const char *name) {
    for (int i = 0; i < e->num_attrs; i++) {
        if (strcmp(e->attrs[i].name, name) == 0) {
            return e->attrs[i].value;
        }
    }
    return NULL;
}

static char *decode_entities(const char *s, size_t len) {
    char *out = malloc(len + 1);
    if (!out) {
        return NULL;
    }
    size_t j = 0;
    for (size_t i = 0; i < len; i++) {
        if (s[i] == '&' && i + 3 < len && s[i + 1] == 'a' && s[i + 2] == 'm' && s[i + 3] == 'p' &&
            s[i + 4] == ';') {
            out[j++] = '&';
            i += 4;
        } else if (s[i] == '&' && i + 3 < len && s[i + 1] == 'l' && s[i + 2] == 't' &&
                   s[i + 3] == ';') {
            out[j++] = '<';
            i += 3;
        } else if (s[i] == '&' && i + 3 < len && s[i + 1] == 'g' && s[i + 2] == 't' &&
                   s[i + 3] == ';') {
            out[j++] = '>';
            i += 3;
        } else if (s[i] == '&' && i + 4 < len && s[i + 1] == 'q' && s[i + 2] == 'u' &&
                   s[i + 3] == 'o' && s[i + 4] == 't' && s[i + 5] == ';') {
            out[j++] = '"';
            i += 5;
        } else if (s[i] == '&' && i + 4 < len && s[i + 1] == 'a' && s[i + 2] == 'p' &&
                   s[i + 3] == 'o' && s[i + 4] == 's' && s[i + 5] == ';') {
            out[j++] = '\'';
            i += 5;
        } else {
            out[j++] = s[i];
        }
    }
    out[j] = '\0';
    return out;
}

static int skip_until(const char *d, size_t len, size_t *pos, const char *needle,
                      size_t needle_len) {
    const void *found = memmem(d + *pos, len - *pos, needle, needle_len);
    if (!found) {
        return -1;
    }
    *pos = (size_t)((const char *)found - d) + needle_len;
    return 0;
}

static int advance_past_non_tag(xml_reader_t *r, const char *d) {
    size_t len = r->len;
    while (r->pos < len) {
        if (d[r->pos] != '<') {
            r->pos++;
            continue;
        }
        if (r->pos + 4 <= len && strncmp(d + r->pos, "<!--", 4) == 0) {
            if (skip_until(d, len, &r->pos, "-->", 3) != 0) {
                return -1;
            }
            continue;
        }
        if (r->pos + 9 <= len && strncmp(d + r->pos, "<![CDATA[", 9) == 0) {
            if (skip_until(d, len, &r->pos, "]]>", 3) != 0) {
                return -1;
            }
            continue;
        }
        if (r->pos + 2 <= len && strncmp(d + r->pos, "<?", 2) == 0) {
            if (skip_until(d, len, &r->pos, "?>", 2) != 0) {
                return -1;
            }
            continue;
        }
        if (r->pos + 2 <= len && strncmp(d + r->pos, "</", 2) == 0) {
            if (r->report_closes) {
                return 0;
            }
            if (skip_until(d, len, &r->pos, ">", 1) != 0) {
                return -1;
            }
            continue;
        }
        return 0;
    }
    return -1;
}

// Parse one attribute at *pos (which points at the attribute name start).
// Returns 0 with *pos advanced past the attribute, 1 if the tag ended,
// -1 on malformed input.
static int parse_one_attr(xml_reader_t *r, size_t *pos, xml_element_t *out) {
    const char *d = r->data;
    size_t len = r->len;
    size_t j = *pos;
    while (j < len && isspace((unsigned char)d[j])) {
        j++;
    }
    if (j >= len || d[j] == '>') {
        *pos = j + (d[j] == '>' ? 1 : 0);
        return 1;
    }
    if (d[j] == '/' && j + 1 < len && d[j + 1] == '>') {
        out->is_self_closing = 1;
        *pos = j + 2;
        return 1;
    }

    size_t nstart = j;
    while (j < len && d[j] != '=' && !isspace((unsigned char)d[j]) && d[j] != '>') {
        j++;
    }
    if (j >= len || d[j] != '=') {
        while (j < len && d[j] != '>' && d[j] != '/') {
            j++;
        }
        *pos = j;
        return 0;
    }
    char *aname = malloc(j - nstart + 1);
    if (!aname) {
        return -1;
    }
    memcpy(aname, d + nstart, j - nstart);
    aname[j - nstart] = '\0';

    j++;
    while (j < len && isspace((unsigned char)d[j])) {
        j++;
    }
    if (j >= len) {
        free(aname);
        return -1;
    }
    char quote = d[j];
    size_t vstart = ++j;
    while (j < len && d[j] != quote) {
        j++;
    }
    if (j >= len) {
        free(aname);
        return -1;
    }
    char *avalue = decode_entities(d + vstart, j - vstart);
    if (!avalue) {
        free(aname);
        return -1;
    }
    j++;

    if (out->num_attrs < XML_MAX_ATTRS) {
        out->attrs[out->num_attrs].name = aname;
        out->attrs[out->num_attrs].value = avalue;
        out->num_attrs++;
    } else {
        free(aname);
        free(avalue);
    }
    *pos = j;
    return 0;
}

static void capture_text_after_tag(xml_reader_t *r, xml_element_t *out) {
    if (!r->capture_text || out->is_self_closing) {
        return;
    }
    const char *d = r->data;
    size_t pos = r->pos;
    if (pos >= r->len || d[pos] == '<') {
        return;
    }
    size_t tend = pos;
    while (tend < r->len && d[tend] != '<') {
        tend++;
    }
    size_t first = pos;
    size_t last = tend;
    while (first < last && isspace((unsigned char)d[first])) {
        first++;
    }
    while (last > first && isspace((unsigned char)d[last - 1])) {
        last--;
    }
    if (last > first) {
        out->content = decode_entities(d + first, last - first);
    }
}

int xml_next_element(xml_reader_t *r, xml_element_t *out) {
    memset(out, 0, sizeof(*out));
    const char *d = r->data;

    if (advance_past_non_tag(r, d) != 0) {
        return 0;
    }

    if (r->report_closes && r->pos + 2 <= r->len && strncmp(d + r->pos, "</", 2) == 0) {
        size_t cstart = r->pos + 2;
        size_t i = cstart;
        while (i < r->len && d[i] != '>' && !isspace((unsigned char)d[i])) {
            i++;
        }
        if (i >= r->len) {
            return 0;
        }
        char *cname = malloc(i - cstart + 1);
        if (!cname) {
            return 0;
        }
        memcpy(cname, d + cstart, i - cstart);
        cname[i - cstart] = '\0';
        out->name = cname;
        out->is_close = 1;
        r->pos = i + 1;
        return 1;
    }

    size_t start = r->pos + 1;
    size_t i = start;
    while (i < r->len && !isspace((unsigned char)d[i]) && d[i] != '>' && d[i] != '/') {
        i++;
    }
    if (i == start || i >= r->len) {
        return 0;
    }

    out->name = malloc(i - start + 1);
    if (!out->name) {
        return 0;
    }
    memcpy(out->name, d + start, i - start);
    out->name[i - start] = '\0';

    size_t pos = i;
    while (pos < r->len) {
        int rc = parse_one_attr(r, &pos, out);
        if (rc < 0) {
            xml_element_clear(out);
            return 0;
        }
        if (rc == 1) {
            r->pos = pos;
            capture_text_after_tag(r, out);
            return 1;
        }
    }
    return 0;
}
