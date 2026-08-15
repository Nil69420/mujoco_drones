#include "vehicle/controller_ops.h"

#include <stddef.h>
#include <string.h>

extern const controller_ops_t pd_multicopter_ops;
extern const controller_ops_t fixedwing_basic_ops;

static const controller_ops_t *const *controller_table(int *count) {
    static const controller_ops_t *const table[] = {
        &pd_multicopter_ops,
        &fixedwing_basic_ops,
    };
    *count = (int)(sizeof(table) / sizeof(table[0]));
    return table;
}

const controller_ops_t *controller_registry_find(const char *name) {
    if (!name) return NULL;
    int count = 0;
    const controller_ops_t *const *table = controller_table(&count);
    for (int i = 0; i < count; i++) {
        if (table[i]->name && strcmp(table[i]->name, name) == 0) {
            return table[i];
        }
    }
    return NULL;
}

int controller_registry_count(void) {
    int count = 0;
    controller_table(&count);
    return count;
}

const controller_ops_t *controller_registry_at(int index) {
    int count = 0;
    const controller_ops_t *const *table = controller_table(&count);
    if (index < 0 || index >= count) return NULL;
    return table[index];
}
