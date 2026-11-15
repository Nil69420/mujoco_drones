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
    {
        .name = "tf02_pro",
        .grade = "industrial",
        .source = "Benewake TF02-Pro datasheet A02 (secondary): 40 m at 90% and 13.5 m at 10% "
                  "reflectivity, +-5 cm (0.1..5 m) and +-1% (5..40 m), 100 Hz default up to 1000 Hz; "
                  "sigma taken as half the quoted bound, dropout assumed",
        .accuracy_provenance = PROVENANCE_SECONDARY,
        .other_provenance = PROVENANCE_ASSUMED,
        .min_range_m = 0.1,
        .max_range_90_m = 40.0,
        .max_range_10_m = 13.5,
        .sigma_near_m = 0.025,
        .knee_m = 5.0,
        .sigma_far_rel = 0.005,
        .resolution_m = 0.01,
        .dropout_probability = 0.5,
        .max_rate_hz = 1000.0,
    },
    {
        .name = "lidar_lite_v3",
        .grade = "consumer",
        .source = "Garmin LIDAR-Lite v3 manual (secondary): 5 cm..40 m, +-2.5 cm below 5 m and +-10 cm "
                  "beyond, 1 cm resolution, 270 Hz typical; 10% reflectivity range and dropout assumed",
        .accuracy_provenance = PROVENANCE_SECONDARY,
        .other_provenance = PROVENANCE_ASSUMED,
        .min_range_m = 0.05,
        .max_range_90_m = 40.0,
        .max_range_10_m = 14.0,
        .sigma_near_m = 0.0125,
        .knee_m = 5.0,
        .sigma_far_rel = 0.01,
        .resolution_m = 0.01,
        .dropout_probability = 0.4,
        .max_rate_hz = 270.0,
    },
    {
        .name = "sf11_c",
        .grade = "professional",
        .source = "LightWare SF11/C (secondary, retail listings and manufacturer table): 0.2..100 m, "
                  "+-0.1 m on a 70% target, 1 cm resolution, 20 readings/s; sigma taken as half the "
                  "bound, 10% reflectivity range and dropout assumed",
        .accuracy_provenance = PROVENANCE_SECONDARY,
        .other_provenance = PROVENANCE_ASSUMED,
        .min_range_m = 0.2,
        .max_range_90_m = 100.0,
        .max_range_10_m = 35.0,
        .sigma_near_m = 0.05,
        .knee_m = 100.0,
        .sigma_far_rel = 0.0005,
        .resolution_m = 0.01,
        .dropout_probability = 0.3,
        .max_rate_hz = 20.0,
    },
};

const size_t RANGE_PROFILE_COUNT = sizeof(RANGE_PROFILES) / sizeof(RANGE_PROFILES[0]);
