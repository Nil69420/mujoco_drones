#include "sensors/sensors_internal.h"
#include "conditions/conditions.h"
#include "sensors/noise.h"

#include <math.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

void sensor_render_camera(sensor_mgr_t *mgr, const mjModel *model,
                          mjData *data) {
    if (!mgr->config.enable.camera || !mgr->cam_due) {
        return;
    }
    mgr->cam_due = false;

    int w = mgr->config.camera_width;
    int h = mgr->config.camera_height;

    if (!mgr->cam_initialized) {
        mjv_defaultScene(&mgr->cam_scene);
        mjv_makeScene(model, &mgr->cam_scene, 1000);
        mjModel *writable = (mjModel *)model;
        if (writable->vis.global.offwidth < w) {
            writable->vis.global.offwidth = w;
        }
        if (writable->vis.global.offheight < h) {
            writable->vis.global.offheight = h;
        }
        mjr_defaultContext(&mgr->cam_context);
        mgr->cam_context.offWidth  = w;
        mgr->cam_context.offHeight = h;
        mjr_makeContext(model, &mgr->cam_context, mjFONTSCALE_100);

        mjv_defaultCamera(&mgr->cam_view);
        mgr->cam_view.type       = mjCAMERA_FIXED;
        mgr->cam_view.fixedcamid = mgr->camera_id;

        mjv_defaultOption(&mgr->cam_opt);

        mgr->cam_initialized = true;
    }

    mjrRect vp = { 0, 0, w, h };
    mjv_updateScene(model, data, &mgr->cam_opt, NULL,
                    &mgr->cam_view, mjCAT_ALL, &mgr->cam_scene);
    mjr_setBuffer(mjFB_OFFSCREEN, &mgr->cam_context);
    mjr_render(vp, &mgr->cam_scene, &mgr->cam_context);

    uint8_t *pixels = mgr->rgb_buf + sizeof(sensor_camera_rgb_hdr_t);
    mjr_readPixels(pixels, mgr->depth_buf, vp, &mgr->cam_context);

    if (mgr->use_cam_model) {
        uint64_t t_mono = (uint64_t)(data->time * 1e9);
        size_t mono_sz = (size_t)(unsigned)w * (unsigned)h;
        uint8_t *frame = frame_out_loan(&mgr->cam_out, sizeof(sensor_camera_rgb_hdr_t) + mono_sz);
        if (frame == NULL) {
            return;
        }
        uint8_t *mono = frame + sizeof(sensor_camera_rgb_hdr_t);
        camera_model_apply(&mgr->cam_model, pixels, mono, mgr->sensor_temp_c,
                           mgr->config.ambient_temp_c, mgr->config.camera_exposure_s);
        sensor_camera_rgb_hdr_t *mh = (sensor_camera_rgb_hdr_t *)frame;
        memset(mh, 0, sizeof(*mh));
        mh->header.timestamp_ns = t_mono;
        mh->header.sequence     = mgr->seq_camera;
        mh->header.sensor_id    = SENSOR_ID_CAMERA;
        mh->width    = (uint16_t)w;
        mh->height   = (uint16_t)h;
        mh->channels = 1;
        sensor_camera_meta_t mm = {0};
        mm.header.timestamp_ns = t_mono;
        mm.header.sequence     = mgr->seq_camera++;
        mm.header.sensor_id    = SENSOR_ID_CAMERA;
        mm.width    = (uint16_t)w;
        mm.height   = (uint16_t)h;
        mm.channels = 1;
        mm.fov_y    = mgr->config.camera_fov;
        mm.rgb_size = (uint32_t)mono_sz;
        transport_publish(&mgr->pub_camera_meta, &mm, sizeof(mm));
        if (mgr->config.camera_legacy_topic) {
            transport_publish(&mgr->pub_camera_rgb, frame, sizeof(sensor_camera_rgb_hdr_t) + mono_sz);
        }
        (void)frame_out_commit(&mgr->cam_out);
        return;
    }

    size_t row_bytes = (size_t)(unsigned)w * 3U;
    uint8_t *temp_row = malloc(row_bytes);
    if (temp_row) {
        size_t half_rows = (size_t)(unsigned)h / 2U;
        size_t h_sz = (size_t)(unsigned)h;
        for (size_t row = 0; row < half_rows; row++) {
            uint8_t *top = pixels + row * row_bytes;
            uint8_t *bot = pixels + (h_sz - 1U - row) * row_bytes;
            memcpy(temp_row, top, row_bytes);
            memcpy(top, bot, row_bytes);
            memcpy(bot, temp_row, row_bytes);
        }
        free(temp_row);
    }

    uint64_t t_ns = (uint64_t)(data->time * 1e9);
    size_t rgb_sz   = (size_t)(unsigned)w * (unsigned)h * 3;
    size_t depth_sz = (size_t)(unsigned)w * (unsigned)h * sizeof(float);

    /* Fill camera RGB header so downstream consumers (bridge) have metadata */
    sensor_camera_rgb_hdr_t *rgb_hdr = (sensor_camera_rgb_hdr_t *)mgr->rgb_buf;
    rgb_hdr->header.timestamp_ns = t_ns;
    rgb_hdr->header.sequence     = mgr->seq_camera;
    rgb_hdr->header.sensor_id    = SENSOR_ID_CAMERA;
    rgb_hdr->width    = (uint16_t)w;
    rgb_hdr->height   = (uint16_t)h;
    rgb_hdr->channels = 3;

    sensor_camera_meta_t meta = {0};
    meta.header.timestamp_ns = t_ns;
    meta.header.sequence     = mgr->seq_camera++;
    meta.header.sensor_id    = SENSOR_ID_CAMERA;
    meta.width       = (uint16_t)w;
    meta.height      = (uint16_t)h;
    meta.channels    = 3;
    meta.fov_y       = mgr->config.camera_fov;
    meta.rgb_size    = (uint32_t)rgb_sz;
    meta.depth_size  = (uint32_t)depth_sz;
    meta.depth_scale = 1.0F;

    transport_publish(&mgr->pub_camera_meta, &meta, sizeof(meta));
    if (mgr->config.camera_legacy_topic) {
        transport_publish(&mgr->pub_camera_rgb, mgr->rgb_buf,
                          sizeof(sensor_camera_rgb_hdr_t) + rgb_sz);
    }
    uint8_t *frame = frame_out_loan(&mgr->cam_out, sizeof(sensor_camera_rgb_hdr_t) + rgb_sz);
    if (frame != NULL) {
        memcpy(frame, mgr->rgb_buf, sizeof(sensor_camera_rgb_hdr_t) + rgb_sz);
        (void)frame_out_commit(&mgr->cam_out);
    }
}
