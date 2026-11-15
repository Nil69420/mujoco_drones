#include "urdf_import_internal.h"

#include <dirent.h>
#include <errno.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>

static int has_suffix(const char *name, const char *suffix) {
    size_t name_len = strlen(name);
    size_t suffix_len = strlen(suffix);
    return name_len >= suffix_len && strcmp(name + name_len - suffix_len, suffix) == 0;
}

int imp_find_model_file(const char *dir, char *out, size_t out_sz) {
    DIR *d = opendir(dir);
    if (!d) {
        fprintf(stderr, "urdf_import: cannot open input directory '%s'\n", dir);
        return -1;
    }

    int found = 0;
    const struct dirent *ent = NULL;
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
        if (!has_suffix(ent->d_name, ".urdf") && !has_suffix(ent->d_name, ".sdf")) {
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

int imp_read_file(const char *path, char **out, size_t *out_len) {
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
    if (fseek(f, 0, SEEK_SET) != 0) {
        fclose(f);
        return -1;
    }
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

int imp_write_file(const char *path, const char *data, size_t len) {
    FILE *f = fopen(path, "wb");
    if (!f) {
        return -1;
    }
    size_t wrote = fwrite(data, 1, len, f);
    fclose(f);
    return wrote == len ? 0 : -1;
}

int imp_ensure_dir(const char *path) {
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

int imp_copy_file(const char *src, const char *dst) {
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

int imp_parse_vec3(const char *s, double out[3]) {
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
