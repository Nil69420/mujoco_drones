#include "sensors/models/camera_model.h"
#include "test_support.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

enum { W = 64, H = 48, FRAMES = 400 };

static camera_profile_t plain_profile(void) {
    camera_profile_t p = *camera_profile_find("ar0234");
    p.vignette_k = 0.0;
    p.prnu_sigma = 0.0;
    p.fpn_e = 0.0;
    p.hot_pixel_fraction = 0.0;
    p.defocus_px_per_k = 0.0;
    return p;
}

static void fill_gray(uint8_t *rgb, uint8_t v) {
    memset(rgb, v, (size_t)W * H * 3);
}

static void test_photon_transfer(void) {
    camera_profile_t p = plain_profile();
    camera_model_t m;
    CHECK(camera_model_init(&m, &p, W, H, 11) == 0);
    uint8_t rgb[W * H * 3];
    uint8_t out[W * H];
    double levels[] = {40.0, 128.0, 220.0};
    for (size_t k = 0; k < 3; k++) {
        fill_gray(rgb, (uint8_t)levels[k]);
        double sum = 0.0;
        double sq = 0.0;
        double n = 0.0;
        for (int f = 0; f < FRAMES; f++) {
            camera_model_apply(&m, rgb, out, 25.0, 20.0, 0.004);
            double px = out[5 * W + 7];
            sum += px;
            sq += px * px;
            n += 1.0;
        }
        double mean = sum / n;
        double var = sq / n - mean * mean;
        double lum = floor((77.0 * levels[k] + 150.0 * levels[k] + 29.0 * levels[k]) / 256.0);
        double signal_e = lum * 0.9 * p.full_well_e / 255.0;
        double dark_e = camera_dark_electrons(&p, 25.0, 0.004);
        double expected_mean = (signal_e + dark_e) * 255.0 / p.full_well_e;
        double expected_var = (signal_e + dark_e + p.read_noise_e * p.read_noise_e) *
                              (255.0 / p.full_well_e) * (255.0 / p.full_well_e) + 1.0 / 12.0;
        if (fabs(mean - expected_mean) > 0.6 || fabs(var / expected_var - 1.0) > 0.30) {
            fprintf(stderr, "level %.0f: mean %.2f (want %.2f) var %.3f (want %.3f)\n", levels[k],
                    mean, expected_mean, var, expected_var);
            g_failures++;
        }
    }
    camera_model_free(&m);
}

static void test_dark_and_defocus(void) {
    camera_profile_t p = plain_profile();
    double d25 = camera_dark_electrons(&p, 25.0, 1.0);
    double d51 = camera_dark_electrons(&p, 51.0, 1.0);
    CHECK(fabs(d51 / d25 - 16.0) < 1e-9);
    CHECK(fabs(camera_dark_electrons(&p, 25.0, 0.5) * 2.0 - d25) < 1e-9);

    p.defocus_px_per_k = 0.05;
    camera_model_t m;
    CHECK(camera_model_init(&m, &p, W, H, 3) == 0);
    uint8_t rgb[W * H * 3];
    uint8_t out[W * H];
    for (int y = 0; y < H; y++) {
        for (int x = 0; x < W; x++) {
            memset(&rgb[((size_t)y * W + (size_t)x) * 3], x < W / 2 ? 20 : 235, 3);
        }
    }
    camera_model_apply(&m, rgb, out, 25.0, 20.0, 0.004);
    int sharp_mid = abs((int)out[10 * W + W / 2] - (int)out[10 * W + W / 2 - 1]);
    camera_model_apply(&m, rgb, out, 25.0, 70.0, 0.004);
    int soft_mid = abs((int)out[10 * W + W / 2] - (int)out[10 * W + W / 2 - 1]);
    CHECK(sharp_mid > 150);
    CHECK(soft_mid < sharp_mid - 40);
    camera_model_free(&m);
}

static void test_geometry(void) {
    camera_profile_t p = *camera_profile_find("ar0234");
    p.vignette_k = 0.4;
    p.prnu_sigma = 0.0;
    p.fpn_e = 0.0;
    p.hot_pixel_fraction = 0.0;
    camera_model_t m;
    CHECK(camera_model_init(&m, &p, W, H, 5) == 0);
    uint8_t rgb[W * H * 3];
    uint8_t out[W * H];
    fill_gray(rgb, 200);
    double centre = 0.0;
    double corner = 0.0;
    for (int f = 0; f < 100; f++) {
        camera_model_apply(&m, rgb, out, 25.0, 20.0, 0.004);
        centre += out[(H / 2) * W + W / 2];
        corner += out[0] + out[W - 1] + out[(H - 1) * W] + out[H * W - 1];
    }
    centre /= 100.0;
    corner /= 400.0;
    double want = 1.0 / ((1.0 + 0.4) * (1.0 + 0.4));
    CHECK(fabs(corner / centre - want) < 0.05);

    memset(rgb, 0, sizeof(rgb));
    for (int x = 0; x < W; x++) {
        rgb[(size_t)(H - 1) * W * 3 + (size_t)x * 3] = 255;
        rgb[(size_t)(H - 1) * W * 3 + (size_t)x * 3 + 1] = 255;
        rgb[(size_t)(H - 1) * W * 3 + (size_t)x * 3 + 2] = 255;
    }
    camera_model_apply(&m, rgb, out, 25.0, 20.0, 0.004);
    CHECK(out[W / 2] > 100);
    CHECK(out[(H / 2) * W + W / 2] < 20);
    camera_model_free(&m);
}

static void test_misc(void) {
    camera_model_t m;
    const camera_profile_t *ar = camera_profile_find("ar0234");
    CHECK(ar != NULL && camera_profile_find("nope") == NULL);
    CHECK(camera_model_init(&m, ar, 1921, 1200, 1) != 0);
    CHECK(camera_model_init(&m, ar, 0, 10, 1) != 0);
    CHECK(camera_model_init(&m, NULL, 10, 10, 1) != 0);
    camera_model_t a;
    camera_model_t b;
    CHECK(camera_model_init(&a, ar, W, H, 9) == 0 && camera_model_init(&b, ar, W, H, 9) == 0);
    uint8_t rgb[W * H * 3];
    uint8_t oa[W * H];
    uint8_t ob[W * H];
    fill_gray(rgb, 100);
    camera_model_apply(&a, rgb, oa, 30.0, 25.0, 0.004);
    camera_model_apply(&b, rgb, ob, 30.0, 25.0, 0.004);
    CHECK(memcmp(oa, ob, sizeof(oa)) == 0);
    camera_model_free(&a);
    camera_model_free(&b);

    camera_model_t big;
    CHECK(camera_model_init(&big, ar, 1280, 720, 4) == 0);
    CHECK(big.hot_count == 184);
    camera_model_free(&big);
    for (size_t i = 0; i < CAMERA_PROFILE_COUNT; i++) {
        CHECK(CAMERA_PROFILES[i].max_fps >= 100.0 && CAMERA_PROFILES[i].full_well_e > 0.0);
    }
}

int main(void) {
    test_photon_transfer();
    test_dark_and_defocus();
    test_geometry();
    test_misc();
    if (g_failures != 0) {
        fprintf(stderr, "%d check(s) failed\n", g_failures);
        return 1;
    }
    puts("test_camera_model OK");
    return 0;
}
