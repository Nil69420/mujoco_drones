#ifndef MUJOCO_DRONES_BRIDGE_PX4_MSGS_H
#define MUJOCO_DRONES_BRIDGE_PX4_MSGS_H

#include "bridge/mav2.h"

enum {
    PX4_FIELDS_ACCEL = 0x7,
    PX4_FIELDS_GYRO = 0x38,
    PX4_FIELDS_MAG = 0x1C0,
    PX4_FIELDS_BARO = 0x1A00,
    PX4_FIELDS_ALL = 0x1FFF,
    PX4_ACTUATOR_COUNT = 16,
    PX4_MODE_ARMED = 128
};

typedef struct {
    uint64_t time_usec;
    float acc_mps2[3];
    float gyro_rps[3];
    float mag_gauss[3];
    float abs_pressure_hpa;
    float diff_pressure_hpa;
    float pressure_alt_m;
    float temperature_c;
    uint32_t fields_updated;
    uint8_t id;
} px4_hil_sensor_t;

typedef struct {
    uint64_t time_usec;
    int32_t lat_e7;
    int32_t lon_e7;
    int32_t alt_mm;
    uint16_t eph;
    uint16_t epv;
    uint16_t vel_cms;
    int16_t vn_cms;
    int16_t ve_cms;
    int16_t vd_cms;
    uint16_t cog_cdeg;
    uint8_t fix_type;
    uint8_t satellites;
    uint8_t id;
    uint16_t yaw_cdeg;
} px4_hil_gps_t;

typedef struct {
    uint64_t time_usec;
    uint64_t flags;
    float controls[PX4_ACTUATOR_COUNT];
    uint8_t mode;
} px4_actuator_controls_t;

size_t px4_encode_heartbeat(uint8_t *out, size_t cap, uint8_t seq, uint8_t sysid, uint8_t compid);

size_t px4_encode_hil_sensor(uint8_t *out, size_t cap, uint8_t seq, uint8_t sysid, uint8_t compid,
                             const px4_hil_sensor_t *msg);

size_t px4_encode_hil_gps(uint8_t *out, size_t cap, uint8_t seq, uint8_t sysid, uint8_t compid,
                          const px4_hil_gps_t *msg);

int px4_decode_actuator_controls(const mav2_frame_t *frame, px4_actuator_controls_t *out);

#endif
