#include "environments/biome.h"
#include "environments/environment_ops.h"

#include <stddef.h>
#include <string.h>

enum { MAX_ENVIRONMENTS = 64 };

static int generate_biome(const environment_ops_t *self, mjSpec *spec, unsigned seed) {
    return biome_generate(spec, self->biome, seed);
}

static const environment_ops_t *table(int *count) {
    static environment_ops_t ops[MAX_ENVIRONMENTS];
    static int filled;
    if (filled == 0) {
        for (size_t idx = 0; idx < BIOME_PROFILE_COUNT && idx < MAX_ENVIRONMENTS; idx++) {
            ops[idx].name = BIOME_PROFILES[idx].name;
            ops[idx].description = BIOME_PROFILES[idx].description;
            ops[idx].biome = &BIOME_PROFILES[idx];
            ops[idx].generate = generate_biome;
            filled++;
        }
    }
    *count = filled;
    return ops;
}

const environment_ops_t *environment_registry_find(const char *name) {
    if (!name) {
        return NULL;
    }
    int count = 0;
    const environment_ops_t *ops = table(&count);
    for (int idx = 0; idx < count; idx++) {
        if (strcmp(ops[idx].name, name) == 0) {
            return &ops[idx];
        }
    }
    return NULL;
}

int environment_registry_count(void) {
    int count = 0;
    (void)table(&count);
    return count;
}

const environment_ops_t *environment_registry_at(int index) {
    int count = 0;
    const environment_ops_t *ops = table(&count);
    if (index < 0 || index >= count) {
        return NULL;
    }
    return &ops[index];
}
