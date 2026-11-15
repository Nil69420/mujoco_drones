#ifndef MUJOCO_DRONES_PROVENANCE_H
#define MUJOCO_DRONES_PROVENANCE_H

typedef enum {
    PROVENANCE_DATASHEET = 0,
    PROVENANCE_SECONDARY = 1,
    PROVENANCE_ASSUMED = 2
} provenance_t;

static inline const char *provenance_name(provenance_t p) {
    switch (p) {
    case PROVENANCE_DATASHEET:
        return "datasheet";
    case PROVENANCE_SECONDARY:
        return "secondary";
    case PROVENANCE_ASSUMED:
        return "assumed";
    }
    return "unknown";
}

#endif
