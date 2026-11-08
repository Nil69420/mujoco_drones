#include "renoir.h"
#include "sensors/sensor_types.h"
#include "transport/frame_out.h"
#include "transport/transport_renoir.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

enum {
    BURST = 64,
    CAM_BURST = 4,
    FRAME_RING_HEADER_BYTES = 8,
    CAM_PIXELS = CAM_BUF_MAX_W * CAM_BUF_MAX_H * 3,
    DISCOVER_CAPACITY = 128,
    SAMPLE_STRIDE = 97,
    IMU_CHANNELS = 9
};

typedef struct {
    unsigned long count;
    unsigned long first_seq;
    unsigned long last_seq;
    unsigned long long first_ns;
    unsigned long long last_ns;
} stream_t;

typedef struct {
    stream_t stream;
    double sum[IMU_CHANNELS];
    double sum_sq[IMU_CHANNELS];
} imu_stats_t;

typedef struct {
    stream_t stream;
    int fix;
    int satellites;
    double hdop;
    double altitude;
} gnss_stats_t;

typedef struct {
    stream_t stream;
    double sum[2];
    double sum_sq[2];
} baro_stats_t;

typedef struct {
    unsigned long total;
    unsigned long valid;
    double sum;
    double sum_sq;
    float range_min;
    float range_max;
} ir_stats_t;

typedef struct {
    stream_t stream;
    unsigned long gaps;
    unsigned long torn;
    double mean;
    unsigned width;
    unsigned height;
    unsigned channels;
} cam_stats_t;

static double now_s(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (double)ts.tv_sec + (double)ts.tv_nsec * 1e-9;
}

static void stream_add(stream_t *stream, unsigned long seq, unsigned long long stamp_ns) {
    if (stream->count == 0) {
        stream->first_seq = seq;
        stream->first_ns = stamp_ns;
    }
    stream->last_seq = seq;
    stream->last_ns = stamp_ns;
    stream->count++;
}

static double stream_rate(const stream_t *stream) {
    if (stream->last_ns <= stream->first_ns) {
        return 0.0;
    }
    return (double)(stream->last_seq - stream->first_seq) * 1e9 /
           (double)(stream->last_ns - stream->first_ns);
}

static double spread(double sum, double sum_sq, double count) {
    double mean = sum / count;
    double var = sum_sq / count - mean * mean;
    return sqrt(var > 0.0 ? var : 0.0);
}

static int drain_imu(transport_sub_t *sub, imu_stats_t *stats) {
    sensor_imu_t msg;
    size_t len = 0;
    int got = 0;
    for (int burst = 0; burst < BURST && transport_read_next(sub, &msg, sizeof(msg), &len) == 0 &&
                        len == sizeof(msg);
         burst++) {
        const double values[IMU_CHANNELS] = {msg.accel[0], msg.accel[1], msg.accel[2],
                                             msg.gyro[0],  msg.gyro[1],  msg.gyro[2],
                                             msg.mag[0],   msg.mag[1],   msg.mag[2]};
        for (int idx = 0; idx < IMU_CHANNELS; idx++) {
            stats->sum[idx] += values[idx];
            stats->sum_sq[idx] += values[idx] * values[idx];
        }
        stream_add(&stats->stream, msg.header.sequence, msg.header.timestamp_ns);
        got = 1;
    }
    return got;
}

static int drain_gnss(transport_sub_t *sub, gnss_stats_t *stats) {
    sensor_gnss_t msg;
    size_t len = 0;
    int got = 0;
    for (int burst = 0; burst < BURST && transport_read_next(sub, &msg, sizeof(msg), &len) == 0 &&
                        len == sizeof(msg);
         burst++) {
        stats->fix = msg.fix_type;
        stats->satellites = msg.num_satellites;
        stats->hdop = msg.hdop;
        stats->altitude = msg.altitude;
        stream_add(&stats->stream, msg.header.sequence, msg.header.timestamp_ns);
        got = 1;
    }
    return got;
}

static int drain_baro(transport_sub_t *sub, baro_stats_t *stats) {
    sensor_baro_t msg;
    size_t len = 0;
    int got = 0;
    for (int burst = 0; burst < BURST && transport_read_next(sub, &msg, sizeof(msg), &len) == 0 &&
                        len == sizeof(msg);
         burst++) {
        stats->sum[0] += msg.pressure_pa;
        stats->sum_sq[0] += msg.pressure_pa * msg.pressure_pa;
        stats->sum[1] += msg.altitude_m;
        stats->sum_sq[1] += msg.altitude_m * msg.altitude_m;
        stream_add(&stats->stream, msg.header.sequence, msg.header.timestamp_ns);
        got = 1;
    }
    return got;
}

static int drain_ir(transport_sub_t *sub, ir_stats_t *stats) {
    sensor_infrared_t msg;
    size_t len = 0;
    int got = 0;
    for (int burst = 0; burst < BURST && transport_read_next(sub, &msg, sizeof(msg), &len) == 0 &&
                        len == sizeof(msg);
         burst++) {
        stats->total++;
        stats->range_min = msg.range_min;
        stats->range_max = msg.range_max;
        if (msg.range_m >= 0.0F) {
            stats->valid++;
            stats->sum += (double)msg.range_m;
            stats->sum_sq += (double)msg.range_m * (double)msg.range_m;
        }
        got = 1;
    }
    return got;
}

static void dump_pgm(const char *path, const unsigned char *pixels, unsigned width, unsigned height) {
    FILE *file = fopen(path, "wb");
    if (file == NULL) {
        return;
    }
    fprintf(file, "P5\n%u %u\n255\n", width, height);
    fwrite(pixels, 1, (size_t)width * height, file);
    fclose(file);
}

static int read_frame(RenoirFrameHandle handle, cam_stats_t *stats, const char *pgm_path) {
    const uint8_t *data = NULL;
    size_t len = 0;
    if (renoir_frame_view(handle, &data, &len) != RenoirErrorCode_Success) {
        return 0;
    }
    if (len < sizeof(sensor_camera_rgb_hdr_t)) {
        renoir_frame_release(handle);
        return 0;
    }
    const sensor_camera_rgb_hdr_t *hdr = (const sensor_camera_rgb_hdr_t *)data;
    unsigned width = hdr->width;
    unsigned height = hdr->height;
    unsigned channels = hdr->channels;
    unsigned long seq = hdr->header.sequence;
    unsigned long long stamp = hdr->header.timestamp_ns;
    size_t pixels = (size_t)width * height * channels;
    int valid = len >= sizeof(*hdr) + pixels && pixels > 0;
    double sum = 0.0;
    if (valid) {
        const unsigned char *body = data + sizeof(*hdr);
        for (size_t idx = 0; idx < pixels; idx += SAMPLE_STRIDE) {
            sum += body[idx];
        }
        if (pgm_path != NULL && channels == 1) {
            dump_pgm(pgm_path, body, width, height);
        }
    }
    int intact = renoir_frame_view_intact(handle);
    renoir_frame_release(handle);
    if (!valid || !intact) {
        stats->torn++;
        return 1;
    }
    if (stats->stream.count > 0 && seq != stats->stream.last_seq + 1) {
        stats->gaps++;
    }
    stream_add(&stats->stream, seq, stamp);
    stats->width = width;
    stats->height = height;
    stats->channels = channels;
    stats->mean = sum / ceil((double)pixels / SAMPLE_STRIDE);
    return 1;
}

static int drain_camera(RenoirFrameHandle handle, cam_stats_t *stats, const char *pgm_path) {
    int got = 0;
    for (int burst = 0; burst < CAM_BURST && handle != NULL; burst++) {
        if (!read_frame(handle, stats, pgm_path)) {
            break;
        }
        got = 1;
    }
    return got;
}

static RenoirFrameHandle open_camera(const char *prefix) {
    char name[96];
    frame_out_name(name, sizeof(name), prefix, "_camera_frame");
    static RenoirRingInfo infos[DISCOVER_CAPACITY];
    uintptr_t total = 0;
    if (renoir_discover(infos, DISCOVER_CAPACITY, &total) != RenoirErrorCode_Success) {
        return NULL;
    }
    for (uintptr_t idx = 0; idx < total && idx < DISCOVER_CAPACITY; idx++) {
        const RenoirRingInfo *info = &infos[idx];
        if (strcmp(info->name, name) != 0 || info->msg_size < sizeof(sensor_camera_rgb_hdr_t) ||
            info->msg_size > sizeof(sensor_camera_rgb_hdr_t) + CAM_PIXELS + FRAME_RING_HEADER_BYTES) {
            continue;
        }
        RenoirFrameHandle handle = NULL;
        if (renoir_frame_open(name, info->msg_size - FRAME_RING_HEADER_BYTES, info->slot_count,
                              RENOIR_LEAN_CONSUMER, &handle) == RenoirErrorCode_Success) {
            return handle;
        }
        fprintf(stderr, "found ring %s but could not open it\n", name);
    }
    fprintf(stderr, "camera ring not available\n");
    return NULL;
}

static void report_imu(const imu_stats_t *stats, double seconds) {
    const stream_t *str = &stats->stream;
    unsigned long span = str->last_seq - str->first_seq + 1;
    printf("imu: %lu msgs (%.0f Hz), sequence span %lu (%.0f%% received)\n", str->count,
           (double)str->count / seconds, span, (double)str->count * 100.0 / (double)span);
    if (str->count > 1) {
        static const char *const NAMES[IMU_CHANNELS] = {"accel0", "accel1", "accel2",
                                                        "gyro0",  "gyro1",  "gyro2",
                                                        "mag0",   "mag1",   "mag2"};
        for (int idx = 0; idx < IMU_CHANNELS; idx++) {
            double count = (double)str->count;
            printf("  %s mean %+9.5f std %.5f\n", NAMES[idx], stats->sum[idx] / count,
                   spread(stats->sum[idx], stats->sum_sq[idx], count));
        }
    }
}

static void report_gnss_baro(const gnss_stats_t *gnss, const baro_stats_t *baro, double seconds) {
    printf("gnss: %lu msgs (%.1f Hz) fix=%d sats=%d hdop=%.2f alt last %.2f\n", gnss->stream.count,
           (double)gnss->stream.count / seconds, gnss->fix, gnss->satellites, gnss->hdop,
           gnss->altitude);
    printf("baro: %lu msgs (%.1f Hz)\n", baro->stream.count, (double)baro->stream.count / seconds);
    if (baro->stream.count > 1) {
        double count = (double)baro->stream.count;
        printf("  pressure_pa mean %.3f std %.4f\n", baro->sum[0] / count,
               spread(baro->sum[0], baro->sum_sq[0], count));
        printf("  altitude_m mean %.3f std %.4f\n", baro->sum[1] / count,
               spread(baro->sum[1], baro->sum_sq[1], count));
    }
    printf("last sequence numbers: imu - gnss %lu baro %lu\n", gnss->stream.last_seq,
           baro->stream.last_seq);
}

static void report_rates(const imu_stats_t *imu, const gnss_stats_t *gnss, const baro_stats_t *baro) {
    printf("simulated-time rates: imu %.1f Hz, gnss %.1f Hz, baro %.1f Hz\n",
           stream_rate(&imu->stream), stream_rate(&gnss->stream), stream_rate(&baro->stream));
}

static void report_ir(const ir_stats_t *stats) {
    if (stats->total == 0) {
        return;
    }
    double mean = stats->valid > 0 ? stats->sum / (double)stats->valid : 0.0;
    double deviation = stats->valid > 0 ? spread(stats->sum, stats->sum_sq, (double)stats->valid) : 0.0;
    printf("infrared: %lu readings, %lu valid, mean %.3f m std %.4f m, limits %.2f..%.1f m\n",
           stats->total, stats->valid, mean, deviation, (double)stats->range_min,
           (double)stats->range_max);
}

static void report_camera(const cam_stats_t *stats) {
    if (stats->stream.count == 0) {
        return;
    }
    printf("camera: %lu frames received (%lu torn), %ux%u x%u, mean level %.1f, sequence gaps %lu, "
           "simulated-time rate %.1f fps\n",
           stats->stream.count, stats->torn, stats->width, stats->height, stats->channels,
           stats->mean, stats->gaps, stream_rate(&stats->stream));
}

typedef struct {
    transport_sub_t imu;
    transport_sub_t gnss;
    transport_sub_t baro;
    transport_sub_t ir;
} subs_t;

static int subscribe_all(transport_t *tp, const char *prefix, subs_t *subs) {
    char topic[96];
    snprintf(topic, sizeof(topic), "%s/imu", prefix);
    int bad = transport_subscribe(tp, topic, sizeof(sensor_imu_t), &subs->imu) != 0;
    snprintf(topic, sizeof(topic), "%s/gnss", prefix);
    bad |= transport_subscribe(tp, topic, sizeof(sensor_gnss_t), &subs->gnss) != 0;
    snprintf(topic, sizeof(topic), "%s/baro", prefix);
    bad |= transport_subscribe(tp, topic, sizeof(sensor_baro_t), &subs->baro) != 0;
    snprintf(topic, sizeof(topic), "%s/infrared", prefix);
    bad |= transport_subscribe(tp, topic, sizeof(sensor_infrared_t), &subs->ir) != 0;
    return bad ? -1 : 0;
}

int main(int argc, char **argv) {
    if (argc > 1 && (strcmp(argv[1], "-h") == 0 || strcmp(argv[1], "--help") == 0)) {
        printf("Usage: %s [PREFIX] [SECONDS] [PGM_PATH]\n", argv[0]);
        printf("Prints rates and statistics of the IMU, GNSS, barometer, infrared and camera topics.\n");
        return 0;
    }
    const char *prefix = argc > 1 ? argv[1] : "/drone";
    double seconds = argc > 2 ? strtod(argv[2], NULL) : 3.0;
    const char *pgm_path = argc > 3 ? argv[3] : NULL;

    transport_t tp;
    if (transport_renoir_create(&tp) != 0 || transport_init(&tp) != 0) {
        fprintf(stderr, "ERROR: transport init failed\n");
        return 1;
    }
    subs_t subs;
    if (subscribe_all(&tp, prefix, &subs) != 0) {
        fprintf(stderr, "ERROR: subscribe failed\n");
        transport_shutdown(&tp);
        return 1;
    }
    RenoirFrameHandle camera = open_camera(prefix);
    imu_stats_t imu = {0};
    gnss_stats_t gnss = {0};
    baro_stats_t baro = {0};
    ir_stats_t ir = {0};
    cam_stats_t cam = {0};
    double end = now_s() + seconds;
    while (now_s() < end) {
        int got = drain_imu(&subs.imu, &imu);
        got |= drain_gnss(&subs.gnss, &gnss);
        got |= drain_baro(&subs.baro, &baro);
        got |= drain_ir(&subs.ir, &ir);
        got |= drain_camera(camera, &cam, pgm_path);
        if (!got) {
            struct timespec nap = {0, 200000};
            nanosleep(&nap, NULL);
        }
    }
    report_imu(&imu, seconds);
    report_gnss_baro(&gnss, &baro, seconds);
    report_rates(&imu, &gnss, &baro);
    report_ir(&ir);
    report_camera(&cam);
    if (camera != NULL) {
        renoir_frame_close(camera);
    }
    transport_close_sub(&subs.imu);
    transport_close_sub(&subs.gnss);
    transport_close_sub(&subs.baro);
    transport_close_sub(&subs.ir);
    transport_shutdown(&tp);
    return 0;
}
