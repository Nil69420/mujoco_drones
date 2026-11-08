#include "conditions/conditions.h"

#include <string.h>

const conditions_profile_t CONDITIONS_PRESETS[] = {
    {"clear_calm", 15.0, 101325.0, 0.0, 0.0, 0.0, 1.0 / 7.0},
    {"breezy", 20.0, 101325.0, 4.0, 0.8, 3.0, 1.0 / 7.0},
    {"windy_gusty", 12.0, 100800.0, 9.0, 2.2, 7.7, 1.0 / 7.0},
    {"storm", 10.0, 99000.0, 15.0, 3.6, 15.4, 1.0 / 7.0},
    {"hot_desert", 45.0, 100500.0, 6.0, 0.4, 6.0, 1.0 / 7.0},
    {"cold_arctic", -25.0, 102000.0, 8.0, 5.0, 8.0, 1.0 / 7.0},
};

const size_t CONDITIONS_PRESET_COUNT = sizeof(CONDITIONS_PRESETS) / sizeof(CONDITIONS_PRESETS[0]);

const conditions_profile_t *conditions_find(const char *name) {
    if (name == NULL) {
        return NULL;
    }
    for (size_t i = 0; i < CONDITIONS_PRESET_COUNT; i++) {
        if (strcmp(CONDITIONS_PRESETS[i].name, name) == 0) {
            return &CONDITIONS_PRESETS[i];
        }
    }
    return NULL;
}
