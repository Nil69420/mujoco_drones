#include "sensors/sensor_types.h"
#include "transport/transport_renoir.h"
#include "renoir.h"
#include "transport/frame_out.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

enum { FRAME_RING_HEADER_BYTES = 8 };
enum { CAM_MAX_BYTES = sizeof(sensor_camera_rgb_hdr_t) + CAM_BUF_MAX_W * CAM_BUF_MAX_H * 3 };

typedef struct {
    double n, sum[9], sq[9];
} acc_t;

static void usage(const char *prog) {
    printf("Usage: %s [PREFIX] [SECONDS]\n", prog);
    printf("Prints rates and statistics of the IMU, GNSS and barometer topics.\n");
}

static double now_s(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (double)ts.tv_sec + (double)ts.tv_nsec * 1e-9;
}

int main(int argc, char **argv) {
    if (argc > 1 && (!strcmp(argv[1], "-h") || !strcmp(argv[1], "--help"))) {
        usage(argv[0]);
        return 0;
    }
    const char *prefix = argc > 1 ? argv[1] : "/drone";
    double seconds = argc > 2 ? strtod(argv[2], NULL) : 3.0;
    char t_imu[64], t_gnss[64], t_baro[64];
    snprintf(t_imu, sizeof(t_imu), "%s/imu", prefix);
    snprintf(t_gnss, sizeof(t_gnss), "%s/gnss", prefix);
    snprintf(t_baro, sizeof(t_baro), "%s/baro", prefix);

    transport_t tp;
    if (transport_renoir_create(&tp) != 0 || transport_init(&tp) != 0) {
        fprintf(stderr, "ERROR: transport init failed\n");
        return 1;
    }
    transport_sub_t s_imu, s_gnss, s_baro, s_ir;
    char t_ir[64];
    snprintf(t_ir, sizeof(t_ir), "%s/infrared", prefix);
    unsigned long ir_n = 0, ir_valid = 0;
    double ir_sum = 0.0, ir_sq = 0.0;
    float ir_rmin = 0.0F, ir_rmax = 0.0F;
    char t_cam[96];
    frame_out_name(t_cam, sizeof(t_cam), prefix, "_camera_frame");
    const char *pgm_path = argc > 3 ? argv[3] : NULL;
    RenoirFrameHandle cam_h = NULL;
    int have_cam = 0;
    {
        static RenoirRingInfo infos[128];
        uintptr_t total = 0;
        if (renoir_discover(infos, 128, &total) == RenoirErrorCode_Success) {
            for (uintptr_t i = 0; i < total && i < 128 && !have_cam; i++) {
                if (strcmp(infos[i].name, t_cam) == 0 && infos[i].msg_size >= sizeof(sensor_camera_rgb_hdr_t) &&
                    infos[i].msg_size <= CAM_MAX_BYTES + FRAME_RING_HEADER_BYTES) {
                    have_cam = renoir_frame_open(t_cam, infos[i].msg_size - FRAME_RING_HEADER_BYTES, infos[i].slot_count,
                                                 RENOIR_LEAN_CONSUMER, &cam_h) == RenoirErrorCode_Success;
                    if (!have_cam) fprintf(stderr, "found ring %s (msg %u, %u slots) but could not open it\n", t_cam, infos[i].msg_size, infos[i].slot_count);
                }
            }
        }
    }
    if (!have_cam) { fprintf(stderr, "camera ring not available\n"); }
    unsigned long cam_n = 0, cam_seq0 = 0, cam_seq1 = 0, cam_gaps = 0;
    unsigned long long cam_t0 = 0, cam_t1 = 0;
    double cam_mean = 0.0;
    unsigned cam_w = 0, cam_h_px = 0, cam_ch = 0;
    unsigned long cam_torn = 0;
    if (transport_subscribe(&tp, t_imu, sizeof(sensor_imu_t), &s_imu) != 0 ||
        transport_subscribe(&tp, t_gnss, sizeof(sensor_gnss_t), &s_gnss) != 0 ||
        transport_subscribe(&tp, t_baro, sizeof(sensor_baro_t), &s_baro) != 0 ||
        transport_subscribe(&tp, t_ir, sizeof(sensor_infrared_t), &s_ir) != 0) {
        fprintf(stderr, "ERROR: subscribe failed\n");
        transport_shutdown(&tp);
        return 1;
    }
    acc_t imu = {0}, baro = {0};
    unsigned long gnss_n = 0;
    unsigned long imu_seq_first = 0, imu_seq_last = 0, gnss_seq_first = 0, gnss_seq_last = 0, baro_seq_first = 0, baro_seq_last = 0;
    int imu_seq_set = 0;
    unsigned long long ts_imu0 = 0, ts_imu1 = 0, ts_g0 = 0, ts_g1 = 0, ts_b0 = 0, ts_b1 = 0;
    int fix = -1;
    int sats = 0;
    double hdop = 0.0, z_first = 0.0, z_last = 0.0;
    double end = now_s() + seconds;
    while (now_s() < end) {
        int got = 0;
        sensor_imu_t m;
        size_t len = 0;
        for (int burst = 0; burst < 64 && transport_read_next(&s_imu, &m, sizeof(m), &len) == 0 && len == sizeof(m); burst++) {
            double v[9] = {m.accel[0], m.accel[1], m.accel[2], m.gyro[0], m.gyro[1], m.gyro[2], m.mag[0], m.mag[1], m.mag[2]};
            for (int i = 0; i < 9; i++) { imu.sum[i] += v[i]; imu.sq[i] += v[i] * v[i]; }
            imu.n += 1.0;
            if (!imu_seq_set) { imu_seq_first = m.header.sequence; imu_seq_set = 1; }
            imu_seq_last = m.header.sequence;
            if (ts_imu0 == 0) ts_imu0 = m.header.timestamp_ns;
            ts_imu1 = m.header.timestamp_ns;
            got = 1;
        }
        sensor_gnss_t g;
        for (int burst = 0; burst < 64 && transport_read_next(&s_gnss, &g, sizeof(g), &len) == 0 && len == sizeof(g); burst++) {
            if (gnss_n == 0) z_first = g.altitude;
            z_last = g.altitude;
            fix = g.fix_type; sats = g.num_satellites; hdop = g.hdop;
            if (ts_g0 == 0) gnss_seq_first = g.header.sequence;
            gnss_seq_last = g.header.sequence;
            if (ts_g0 == 0) ts_g0 = g.header.timestamp_ns;
            ts_g1 = g.header.timestamp_ns;
            gnss_n++;
            got = 1;
        }
        sensor_baro_t b;
        for (int burst = 0; burst < 64 && transport_read_next(&s_baro, &b, sizeof(b), &len) == 0 && len == sizeof(b); burst++) {
            baro.sum[0] += b.pressure_pa; baro.sq[0] += b.pressure_pa * b.pressure_pa;
            baro.sum[1] += b.altitude_m; baro.sq[1] += b.altitude_m * b.altitude_m;
            if (ts_b0 == 0) baro_seq_first = b.header.sequence;
            baro_seq_last = b.header.sequence;
            if (ts_b0 == 0) ts_b0 = b.header.timestamp_ns;
            ts_b1 = b.header.timestamp_ns;
            baro.n += 1.0;
            got = 1;
        }
        sensor_infrared_t ir;
        for (int burst = 0; burst < 64 && transport_read_next(&s_ir, &ir, sizeof(ir), &len) == 0 && len == sizeof(ir); burst++) {
            ir_n++;
            ir_rmin = ir.range_min;
            ir_rmax = ir.range_max;
            if (ir.range_m >= 0.0F) {
                ir_valid++;
                ir_sum += (double)ir.range_m;
                ir_sq += (double)ir.range_m * (double)ir.range_m;
            }
            got = 1;
        }
        for (int burst = 0; burst < 4 && have_cam; burst++) {
            const uint8_t *cam_buf = NULL;
            size_t clen = 0;
            if (renoir_frame_view(cam_h, &cam_buf, &clen) != RenoirErrorCode_Success) break;
            if (clen < sizeof(sensor_camera_rgb_hdr_t)) { renoir_frame_release(cam_h); break; }
            const sensor_camera_rgb_hdr_t *ch = (const sensor_camera_rgb_hdr_t *)cam_buf;
            unsigned long long seq = ch->header.sequence;
            unsigned long long stamp = ch->header.timestamp_ns;
            unsigned fw = ch->width, fh = ch->height, fc = ch->channels;
            size_t px = (size_t)fw * fh * fc;
            double sum = 0.0;
            int valid = clen >= sizeof(*ch) + px && px > 0;
            if (valid) {
                const unsigned char *d = cam_buf + sizeof(*ch);
                for (size_t i = 0; i < px; i += 97) sum += d[i];
                if (pgm_path != NULL && fc == 1) {
                    FILE *f = fopen(pgm_path, "wb");
                    if (f != NULL) {
                        fprintf(f, "P5\n%u %u\n255\n", fw, fh);
                        fwrite(d, 1, px, f);
                        fclose(f);
                    }
                }
            }
            int intact = renoir_frame_view_intact(cam_h);
            renoir_frame_release(cam_h);
            if (!valid || !intact) { cam_torn++; continue; }
            if (cam_n == 0) { cam_seq0 = seq; cam_t0 = stamp; }
            else if (seq != cam_seq1 + 1) cam_gaps++;
            cam_seq1 = seq; cam_t1 = stamp;
            cam_w = fw; cam_h_px = fh; cam_ch = fc;
            cam_mean = sum / (double)((px + 96) / 97);
            cam_n++;
            got = 1;
        }
        if (!got) {
            struct timespec nap = {0, 200000};
            nanosleep(&nap, NULL);
        }
    }
    printf("imu: %.0f msgs (%.0f Hz), sequence span %lu (%.0f%% received)\n", imu.n, imu.n / seconds,
           imu_seq_last - imu_seq_first + 1, imu.n * 100.0 / (double)(imu_seq_last - imu_seq_first + 1));
    if (imu.n > 1.0) {
        for (int i = 0; i < 9; i++) {
            double mean = imu.sum[i] / imu.n;
            double var = imu.sq[i] / imu.n - mean * mean;
            printf("  %s%d mean %+9.5f std %.5f\n", i < 3 ? "accel" : (i < 6 ? "gyro" : "mag"), i % 3, mean, sqrt(var > 0 ? var : 0));
        }
    }
    printf("gnss: %lu msgs (%.1f Hz) fix=%d sats=%d hdop=%.2f alt first %.2f last %.2f\n", gnss_n,
           (double)gnss_n / seconds, fix, sats, hdop, z_first, z_last);
    printf("baro: %.0f msgs (%.1f Hz)\n", baro.n, baro.n / seconds);
    if (baro.n > 1.0) {
        for (int i = 0; i < 2; i++) {
            double mean = baro.sum[i] / baro.n;
            double var = baro.sq[i] / baro.n - mean * mean;
            printf("  %s mean %.3f std %.4f\n", i == 0 ? "pressure_pa" : "altitude_m", mean, sqrt(var > 0 ? var : 0));
        }
    }
    printf("last sequence numbers: imu %lu gnss %lu baro %lu\n", imu_seq_last, gnss_seq_last, baro_seq_last);
    printf("simulated-time rates: imu %.1f Hz, gnss %.1f Hz, baro %.1f Hz\n",
           ts_imu1 > ts_imu0 ? (double)(imu_seq_last - imu_seq_first) * 1e9 / (double)(ts_imu1 - ts_imu0) : 0.0,
           ts_g1 > ts_g0 ? (double)(gnss_seq_last - gnss_seq_first) * 1e9 / (double)(ts_g1 - ts_g0) : 0.0,
           ts_b1 > ts_b0 ? (double)(baro_seq_last - baro_seq_first) * 1e9 / (double)(ts_b1 - ts_b0) : 0.0);
    if (cam_n > 0) {
        printf("camera: %lu frames received (%lu torn), %ux%u x%u, mean level %.1f, sequence gaps %lu, "
               "simulated-time rate %.1f fps\n", cam_n, cam_torn, cam_w, cam_h_px, cam_ch, cam_mean, cam_gaps,
               cam_t1 > cam_t0 ? (double)(cam_seq1 - cam_seq0) * 1e9 / (double)(cam_t1 - cam_t0) : 0.0);
    }
    if (ir_n > 0) {
        double mean = ir_valid > 0 ? ir_sum / (double)ir_valid : 0.0;
        double var = ir_valid > 0 ? ir_sq / (double)ir_valid - mean * mean : 0.0;
        printf("infrared: %lu readings, %lu valid, mean %.3f m std %.4f m, limits %.2f..%.1f m\n", ir_n, ir_valid, mean,
               sqrt(var > 0.0 ? var : 0.0), (double)ir_rmin, (double)ir_rmax);
    }
    transport_close_sub(&s_ir);
    if (have_cam) renoir_frame_close(cam_h);
    transport_close_sub(&s_imu);
    transport_close_sub(&s_gnss);
    transport_close_sub(&s_baro);
    transport_shutdown(&tp);
    return 0;
}
