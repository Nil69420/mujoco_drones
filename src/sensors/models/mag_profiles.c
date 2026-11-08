#include "sensors/models/mag_model.h"

const mag_profile_t MAG_PROFILES[] = {
    {
        .name = "ideal",
        .grade = "reference",
        .source = "zero-error reference",
        .noise_provenance = PROVENANCE_ASSUMED,
        .other_provenance = PROVENANCE_ASSUMED,
        .noise_nt = 0.0,
        .hard_iron_sigma_nt = 0.0,
        .soft_iron_sigma = 0.0,
        .motor_nt_per_a = 0.0,
        .tempco_nt_per_k = 0.0,
        .resolution_nt = 0.0,
        .range_nt = 1e9,
        .max_rate_hz = 1000.0,
    },
    {
        .name = "hmc5883l",
        .grade = "consumer",
        .source = "Honeywell HMC5883L datasheet text (secondary): 2 mG (200 nT) noise floor at "
                  "gain 0, standard deviation over 100 samples; hard and soft iron, motor "
                  "interference, tempco, rate assumed",
        .noise_provenance = PROVENANCE_SECONDARY,
        .other_provenance = PROVENANCE_ASSUMED,
        .noise_nt = 200.0,
        .hard_iron_sigma_nt = 3000.0,
        .soft_iron_sigma = 0.03,
        .motor_nt_per_a = 400.0,
        .tempco_nt_per_k = 20.0,
        .resolution_nt = 92.0,
        .range_nt = 130000.0,
        .max_rate_hz = 75.0,
    },
    {
        .name = "rm3100",
        .grade = "industrial",
        .source = "PNI RM3100 (secondary): noise 15 nT at the highest cycle count; hard and soft "
                  "iron, motor interference, tempco, rate and range assumed",
        .noise_provenance = PROVENANCE_SECONDARY,
        .other_provenance = PROVENANCE_ASSUMED,
        .noise_nt = 15.0,
        .hard_iron_sigma_nt = 1500.0,
        .soft_iron_sigma = 0.01,
        .motor_nt_per_a = 400.0,
        .tempco_nt_per_k = 2.0,
        .resolution_nt = 13.0,
        .range_nt = 800000.0,
        .max_rate_hz = 200.0,
    },
};

const size_t MAG_PROFILE_COUNT = sizeof(MAG_PROFILES) / sizeof(MAG_PROFILES[0]);
