#include "environments/biome.h"
#include "environments/environment_ops.h"

#include <stddef.h>
#include <string.h>

static int generate_biome(const environment_ops_t *self, mjSpec *spec, unsigned seed) {
    return biome_generate(spec, self->biome, seed);
}

static environment_ops_t g_ops[64];
static int g_count;

static void ensure_table(void) {
    if (g_count > 0) {
        return;
    }
    for (size_t i = 0; i < BIOME_PROFILE_COUNT && i < sizeof(g_ops) / sizeof(g_ops[0]); i++) {
        g_ops[i].name = BIOME_PROFILES[i].name;
        g_ops[i].description = BIOME_PROFILES[i].description;
        g_ops[i].biome = &BIOME_PROFILES[i];
        g_ops[i].generate = generate_biome;
        g_count++;
    }
}

const environment_ops_t *environment_registry_find(const char *name) {
    if (!name) return NULL;
    ensure_table();
    for (int i = 0; i < g_count; i++) {
        if (strcmp(g_ops[i].name, name) == 0) {
            return &g_ops[i];
        }
    }
    return NULL;
}

int environment_registry_count(void) {
    ensure_table();
    return g_count;
}

const environment_ops_t *environment_registry_at(int index) {
    ensure_table();
    if (index < 0 || index >= g_count) return NULL;
    return &g_ops[index];
}
