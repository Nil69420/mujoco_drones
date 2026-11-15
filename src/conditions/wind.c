#include "conditions/conditions.h"

#include <math.h>

#define FT_PER_M 3.28084
#define M_PER_FT 0.3048
#define WIND_REF_HEIGHT_M 6.096
#define MIN_HEIGHT_M 1.0
#define MIN_AIRSPEED_MS 1.0

void wind_init(wind_t *wind, const conditions_profile_t *profile, uint64_t seed) {
    wind->profile = profile;
    wind->gust[0] = 0.0;
    wind->gust[1] = 0.0;
    wind->gust[2] = 0.0;
    noise_seed(&wind->rng, seed);
}

void wind_mean_at(const wind_t *wind, double height_m, double out_ned[3]) {
    const conditions_profile_t *prof = wind->profile;
    double h = height_m < MIN_HEIGHT_M ? MIN_HEIGHT_M : height_m;
    double speed = prof->wind_speed_20ft_ms * pow(h / WIND_REF_HEIGHT_M, prof->shear_exponent);
    out_ned[0] = speed * cos(prof->wind_toward_rad);
    out_ned[1] = speed * sin(prof->wind_toward_rad);
    out_ned[2] = 0.0;
}

void wind_step(wind_t *wind, double height_m, double airspeed_ms, double dt, double out_ned[3]) {
    const conditions_profile_t *prof = wind->profile;
    double h_ft = (height_m < MIN_HEIGHT_M ? MIN_HEIGHT_M : height_m) * FT_PER_M;
    double scale = 0.177 + 0.000823 * h_ft;
    double len_w = h_ft * M_PER_FT;
    double len_uv = (h_ft / pow(scale, 1.2)) * M_PER_FT;
    double sigma_w = 0.1 * prof->turbulence_w20_ms;
    double sigma_uv = sigma_w / pow(scale, 0.4);
    double speed = airspeed_ms < MIN_AIRSPEED_MS ? MIN_AIRSPEED_MS : airspeed_ms;
    const double lengths[3] = {len_uv, len_uv, len_w};
    const double sigmas[3] = {sigma_uv, sigma_uv, sigma_w};

    for (int i = 0; i < 3; i++) {
        double tau = lengths[i] / speed;
        double decay = exp(-dt / (tau > 1e-3 ? tau : 1e-3));
        wind->gust[i] = decay * wind->gust[i] +
                        noise_gaussian(&wind->rng, sigmas[i] * sqrt(1.0 - decay * decay));
    }
    wind_mean_at(wind, height_m, out_ned);
    for (int i = 0; i < 3; i++) {
        out_ned[i] += wind->gust[i];
    }
}
