#include "sensors/sensor_types.h"
#include "transport/transport_renoir.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

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
    transport_sub_t s_imu, s_gnss, s_baro;
    if (transport_subscribe(&tp, t_imu, sizeof(sensor_imu_t), &s_imu) != 0 ||
        transport_subscribe(&tp, t_gnss, sizeof(sensor_gnss_t), &s_gnss) != 0 ||
        transport_subscribe(&tp, t_baro, sizeof(sensor_baro_t), &s_baro) != 0) {
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
        while (transport_read_next(&s_imu, &m, sizeof(m), &len) == 0 && len == sizeof(m)) {
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
        while (transport_read_next(&s_gnss, &g, sizeof(g), &len) == 0 && len == sizeof(g)) {
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
        while (transport_read_next(&s_baro, &b, sizeof(b), &len) == 0 && len == sizeof(b)) {
            baro.sum[0] += b.pressure_pa; baro.sq[0] += b.pressure_pa * b.pressure_pa;
            baro.sum[1] += b.altitude_m; baro.sq[1] += b.altitude_m * b.altitude_m;
            if (ts_b0 == 0) baro_seq_first = b.header.sequence;
            baro_seq_last = b.header.sequence;
            if (ts_b0 == 0) ts_b0 = b.header.timestamp_ns;
            ts_b1 = b.header.timestamp_ns;
            baro.n += 1.0;
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
    transport_shutdown(&tp);
    return 0;
}
