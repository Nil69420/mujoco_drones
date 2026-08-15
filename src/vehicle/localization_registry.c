#include "vehicle/localization_ops.h"

#include <stddef.h>
#include <string.h>

extern const localization_ops_t ground_truth_localization_ops;
extern const localization_ops_t gnss_localization_ops;

static const localization_ops_t *const *localization_table(int *count) {
    static const localization_ops_t *const table[] = {
        &ground_truth_localization_ops,
        &gnss_localization_ops,
    };
    *count = (int)(sizeof(table) / sizeof(table[0]));
    return table;
}

const localization_ops_t *localization_registry_find(const char *name) {
    if (!name) return NULL;
    int count = 0;
    const localization_ops_t *const *table = localization_table(&count);
    for (int i = 0; i < count; i++) {
        if (table[i]->name && strcmp(table[i]->name, name) == 0) {
            return table[i];
        }
    }
    return NULL;
}

int localization_registry_count(void) {
    int count = 0;
    localization_table(&count);
    return count;
}

const localization_ops_t *localization_registry_at(int index) {
    int count = 0;
    const localization_ops_t *const *table = localization_table(&count);
    if (index < 0 || index >= count) return NULL;
    return table[index];
}
