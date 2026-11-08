#include "sensors/models/baro_model.h"

const baro_profile_t BARO_PROFILES[] = {
    {
        .name = "ideal",
        .grade = "reference",
        .source = "zero-error reference",
        .noise_provenance = PROVENANCE_ASSUMED,
        .other_provenance = PROVENANCE_ASSUMED,
        .noise_pa = 0.0,
        .drift_sigma_pa = 0.0,
        .offset_sigma_pa = 0.0,
        .tempco_pa_per_k = 0.0,
        .propwash_pa = 0.0,
        .resolution_pa = 0.0,
        .max_rate_hz = 1000.0,
    },
    {
        .name = "ms5611",
        .grade = "consumer",
        .source = "TE MS5611-01BA03 datasheet copies (secondary): RMS resolution 0.012 mbar at OSR "
                  "4096, conversion 8.22 ms; drift, offset, tempco and propwash assumed",
        .noise_provenance = PROVENANCE_SECONDARY,
        .other_provenance = PROVENANCE_ASSUMED,
        .noise_pa = 1.2,
        .drift_sigma_pa = 5.0,
        .offset_sigma_pa = 75.0,
        .tempco_pa_per_k = 1.0,
        .propwash_pa = 8.0,
        .resolution_pa = 0.1,
        .max_rate_hz = 100.0,
    },
    {
        .name = "bmp388",
        .grade = "industrial",
        .source = "Bosch BMP388 datasheet (primary): RMS noise 0.9 Pa (high resolution, IIR 3), "
                  "relative accuracy +-0.08 hPa, absolute +-0.5 hPa, TCO +-0.75 Pa/K, ODR up to "
                  "200 Hz; sigma interpretation of the +- figures and propwash assumed",
        .noise_provenance = PROVENANCE_DATASHEET,
        .other_provenance = PROVENANCE_DATASHEET,
        .noise_pa = 0.9,
        .drift_sigma_pa = 4.0,
        .offset_sigma_pa = 25.0,
        .tempco_pa_per_k = 0.75,
        .propwash_pa = 8.0,
        .resolution_pa = 0.01,
        .max_rate_hz = 200.0,
    },
    {
        .name = "dps310",
        .grade = "industrial",
        .source = "Infineon DPS310 datasheet (primary): precision 0.2 Pa RMS (high precision), "
                  "relative accuracy +-6 Pa, absolute +-100 Pa, temperature sensitivity 0.5 Pa/K, "
                  "1..128 Hz; sigma interpretation of the +- figures and propwash assumed",
        .noise_provenance = PROVENANCE_DATASHEET,
        .other_provenance = PROVENANCE_DATASHEET,
        .noise_pa = 0.2,
        .drift_sigma_pa = 3.0,
        .offset_sigma_pa = 50.0,
        .tempco_pa_per_k = 0.5,
        .propwash_pa = 8.0,
        .resolution_pa = 0.06,
        .max_rate_hz = 128.0,
    },
};

const size_t BARO_PROFILE_COUNT = sizeof(BARO_PROFILES) / sizeof(BARO_PROFILES[0]);
