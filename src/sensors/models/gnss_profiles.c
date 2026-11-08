#include "sensors/models/gnss_model.h"

const gnss_profile_t GNSS_PROFILES[] = {
    {
        .name = "ideal",
        .grade = "reference",
        .source = "zero-error reference receiver for debugging",
        .accuracy_provenance = PROVENANCE_ASSUMED,
        .other_provenance = PROVENANCE_ASSUMED,
        .horizontal_cep_m = 1e-6,
        .vertical_to_horizontal = 1.0,
        .velocity_sigma_ms = 0.0,
        .max_rate_hz = 1000.0,
        .corr_time_s = 100.0,
        .white_fraction = 0.5,
        .latency_s = 0.0,
        .ttff_cold_s = 0.0,
        .rtk_capable = 0,
        .rtk_cep_m = 0.0,
        .rtk_converge_s = 0.0,
    },
    {
        .name = "neo_m8n",
        .grade = "consumer",
        .source = "u-blox NEO-M8 product summary (secondary): 2.0 m CEP, up to 18 Hz single GNSS "
                  "and 10 Hz concurrent; velocity sigma 0.1 m/s from a Holybro M8 module sheet "
                  "(secondary); correlation, latency, cold start, vertical ratio assumed",
        .accuracy_provenance = PROVENANCE_SECONDARY,
        .other_provenance = PROVENANCE_ASSUMED,
        .horizontal_cep_m = 2.0,
        .vertical_to_horizontal = 1.5,
        .velocity_sigma_ms = 0.1,
        .max_rate_hz = 18.0,
        .corr_time_s = 120.0,
        .white_fraction = 0.2,
        .latency_s = 0.1,
        .ttff_cold_s = 26.0,
        .rtk_capable = 0,
        .rtk_cep_m = 0.0,
        .rtk_converge_s = 0.0,
    },
    {
        .name = "zed_f9p",
        .grade = "survey (RTK)",
        .source = "u-blox ZED-F9P product summary (secondary): RTK 0.01 m + 1 ppm CEP, RTK up to "
                  "20 Hz, cold start 24 s; standalone accuracy 1.5 m CEP, velocity, latency, "
                  "correlation and RTK convergence assumed",
        .accuracy_provenance = PROVENANCE_ASSUMED,
        .other_provenance = PROVENANCE_ASSUMED,
        .horizontal_cep_m = 1.5,
        .vertical_to_horizontal = 1.5,
        .velocity_sigma_ms = 0.05,
        .max_rate_hz = 20.0,
        .corr_time_s = 120.0,
        .white_fraction = 0.2,
        .latency_s = 0.05,
        .ttff_cold_s = 24.0,
        .rtk_capable = 1,
        .rtk_cep_m = 0.01,
        .rtk_converge_s = 15.0,
    },
};

const size_t GNSS_PROFILE_COUNT = sizeof(GNSS_PROFILES) / sizeof(GNSS_PROFILES[0]);
