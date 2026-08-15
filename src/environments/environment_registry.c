#include "environments/environment_ops.h"

#include <stddef.h>
#include <string.h>

extern const environment_ops_t desert_environment_ops;
extern const environment_ops_t forest_environment_ops;

static const environment_ops_t *const *environment_table(int *count) {
    static const environment_ops_t *const table[] = {
        &desert_environment_ops,
        &forest_environment_ops,
    };
    *count = (int)(sizeof(table) / sizeof(table[0]));
    return table;
}

const environment_ops_t *environment_registry_find(const char *name) {
    if (!name) return NULL;
    int count = 0;
    const environment_ops_t *const *table = environment_table(&count);
    for (int i = 0; i < count; i++) {
        if (table[i]->name && strcmp(table[i]->name, name) == 0) {
            return table[i];
        }
    }
    return NULL;
}

int environment_registry_count(void) {
    int count = 0;
    environment_table(&count);
    return count;
}

const environment_ops_t *environment_registry_at(int index) {
    int count = 0;
    const environment_ops_t *const *table = environment_table(&count);
    if (index < 0 || index >= count) return NULL;
    return table[index];
}
