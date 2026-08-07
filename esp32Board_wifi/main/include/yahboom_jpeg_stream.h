#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_camera.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

typedef struct
{
    const uint8_t *data;
    size_t length;
    int64_t ready_time_us;
    int64_t timestamp_sec;
    int32_t timestamp_usec;
    uint32_t encode_time_us;
} yahboom_jpeg_frame_t;

#ifdef __cplusplus
extern "C" {
#endif

void yahboom_jpeg_stream_init(QueueHandle_t capture_queue);
void yahboom_jpeg_stream_start(void);
void yahboom_jpeg_stream_stop(void);
bool yahboom_jpeg_stream_submit_frame(camera_fb_t *frame);
bool yahboom_jpeg_stream_wait(yahboom_jpeg_frame_t *frame, TickType_t ticks_to_wait);
void yahboom_jpeg_stream_release(void);
void yahboom_capture_start(void);
void yahboom_capture_stop(void);
bool yahboom_capture_get_frame(camera_fb_t **frame, TickType_t ticks_to_wait);
bool yahboom_capture_publish_frame(camera_fb_t *frame, bool *queue_replaced);

#ifdef __cplusplus
}
#endif
