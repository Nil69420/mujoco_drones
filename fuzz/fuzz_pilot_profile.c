#include "pilot/pilot.h"

#include <stdint.h>
#include <stdlib.h>
#include <string.h>

int LLVMFuzzerTestOneInput(const uint8_t *data, size_t size);

int LLVMFuzzerTestOneInput(const uint8_t *data, size_t size) {
    char *text = malloc(size + 1U);
    if (text == NULL) {
        return 0;
    }
    memcpy(text, data, size);
    text[size] = '\0';
    pilot_profile_t profile;
    char error[96];
    if (pilot_profile_parse(text, &profile, error, sizeof(error)) == 0) {
        char formatted[2048];
        char again_text[2048];
        pilot_profile_t again;
        if (pilot_profile_format(&profile, formatted, sizeof(formatted)) > 0) {
            if (pilot_profile_parse(formatted, &again, error, sizeof(error)) != 0) {
                abort();
            }
            if (pilot_profile_format(&again, again_text, sizeof(again_text)) < 0) {
                abort();
            }
        }
    }
    free(text);
    return 0;
}
