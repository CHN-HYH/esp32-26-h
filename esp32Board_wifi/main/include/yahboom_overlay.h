#pragma once

#include <stdint.h>

#include "esp_camera.h"

#ifdef __cplusplus
extern "C" {
#endif

void yahboom_overlay_show_wait(uint32_t duration_ms);
void yahboom_overlay_show_start(uint32_t duration_ms);
void yahboom_overlay_draw_status(camera_fb_t *frame);
void yahboom_overlay_draw_fps(camera_fb_t *frame);

#ifdef __cplusplus
}
#endif
