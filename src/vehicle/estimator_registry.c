#include "vehicle/estimator_ops.h"

#include <stddef.h>
#include <string.h>

extern const estimator_ops_t ground_truth_estimator_ops;
extern const estimator_ops_t complementary_filter_ops;

static const estimator_ops_t *const *estimator_table(int *count) {
    static const estimator_ops_t *const table[] = {
        &ground_truth_estimator_ops,
        &complementary_filter_ops,
    };
    *count = (int)(sizeof(table) / sizeof(table[0]));
    return table;
}

const estimator_ops_t *estimator_registry_find(const char *name) {
    if (!name) return NULL;
    int count = 0;
    const estimator_ops_t *const *table = estimator_table(&count);
    for (int i = 0; i < count; i++) {
        if (table[i]->name && strcmp(table[i]->name, name) == 0) {
            return table[i];
        }
    }
    return NULL;
}

int estimator_registry_count(void) {
    int count = 0;
    estimator_table(&count);
    return count;
}

const estimator_ops_t *estimator_registry_at(int index) {
    int count = 0;
    const estimator_ops_t *const *table = estimator_table(&count);
    if (index < 0 || index >= count) return NULL;
    return table[index];
}
