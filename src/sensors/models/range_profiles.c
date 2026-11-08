#include "sensors/models/range_model.h"

const range_profile_t RANGE_PROFILES[] = {
    {
        .name = "ideal",
        .grade = "reference",
        .source = "zero-error reference",
        .accuracy_provenance = PROVENANCE_ASSUMED,
        .other_provenance = PROVENANCE_ASSUMED,
        .min_range_m = 0.0,
        .max_range_90_m = 1000.0,
        .max_range_10_m = 1000.0,
        .sigma_near_m = 0.0,
        .knee_m = 1e9,
        .sigma_far_rel = 0.0,
        .resolution_m = 0.0,
        .dropout_probability = 0.0,
        .max_rate_hz = 10000.0,
    },
    {
        .name = "tfmini_s",
        .grade = "consumer",
        .source = "Benewake TFmini-S datasheet (secondary): 0.1..12 m at 90% reflectivity, "
                  "0.1..7 m at 10%, accuracy +-6 cm (0.1..6 m) and +-1% (6..12 m), 100 Hz "
                  "default, 1..1000 Hz; sigma taken as half the quoted bound, dropout assumed",
        .accuracy_provenance = PROVENANCE_SECONDARY,
        .other_provenance = PROVENANCE_ASSUMED,
        .min_range_m = 0.1,
        .max_range_90_m = 12.0,
        .max_range_10_m = 7.0,
        .sigma_near_m = 0.03,
        .knee_m = 6.0,
        .sigma_far_rel = 0.005,
        .resolution_m = 0.01,
        .dropout_probability = 0.5,
        .max_rate_hz = 1000.0,
    },
};

const size_t RANGE_PROFILE_COUNT = sizeof(RANGE_PROFILES) / sizeof(RANGE_PROFILES[0]);
