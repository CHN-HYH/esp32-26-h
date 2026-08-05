#include "yahboom_performance.h"

#include <string.h>

#include "esp_log.h"
#include "esp_timer.h"

#define PERFORMANCE_WINDOW_US 2000000

static const char *TAG = "yahboom_perf";

typedef struct
{
    int64_t window_start_us;
    uint64_t capture_total_us;
    uint64_t detection_total_us;
    uint64_t overlay_total_us;
    uint32_t capture_max_us;
    uint32_t detection_max_us;
    uint32_t overlay_max_us;
    uint32_t frame_count;
    uint32_t queue_replace_count;
} camera_performance_t;

typedef struct
{
    int64_t window_start_us;
    uint64_t queue_wait_total_us;
    uint64_t encode_total_us;
    uint64_t send_total_us;
    uint64_t jpeg_total_size;
    uint32_t queue_wait_max_us;
    uint32_t encode_max_us;
    uint32_t send_max_us;
    uint32_t frame_count;
    uint32_t fallback_count;
} stream_performance_t;

static camera_performance_t camera_performance;
static stream_performance_t stream_performance;

static void update_maximum(uint32_t value, uint32_t *maximum)
{
    if (value > *maximum)
        *maximum = value;
}

void yahboom_performance_record_camera(uint32_t capture_us,
                                       uint32_t detection_us,
                                       uint32_t overlay_us,
                                       bool queue_replaced)
{
    const int64_t now_us = esp_timer_get_time();
    if (camera_performance.window_start_us == 0)
        camera_performance.window_start_us = now_us;

    camera_performance.capture_total_us += capture_us;
    camera_performance.detection_total_us += detection_us;
    camera_performance.overlay_total_us += overlay_us;
    camera_performance.frame_count++;
    if (queue_replaced)
        camera_performance.queue_replace_count++;

    update_maximum(capture_us, &camera_performance.capture_max_us);
    update_maximum(detection_us, &camera_performance.detection_max_us);
    update_maximum(overlay_us, &camera_performance.overlay_max_us);

    if (now_us - camera_performance.window_start_us < PERFORMANCE_WINDOW_US)
        return;

    const uint32_t frames = camera_performance.frame_count;
    ESP_LOGI(TAG,
             "PERF_CAMERA frames=%u capture_avg=%u max=%u detect_avg=%u max=%u "
             "overlay_avg=%u max=%u queue_replace=%u",
             (unsigned)frames,
             (unsigned)(camera_performance.capture_total_us / frames),
             (unsigned)camera_performance.capture_max_us,
             (unsigned)(camera_performance.detection_total_us / frames),
             (unsigned)camera_performance.detection_max_us,
             (unsigned)(camera_performance.overlay_total_us / frames),
             (unsigned)camera_performance.overlay_max_us,
             (unsigned)camera_performance.queue_replace_count);

    memset(&camera_performance, 0, sizeof(camera_performance));
    camera_performance.window_start_us = now_us;
}

void yahboom_performance_record_stream(uint32_t queue_wait_us,
                                       uint32_t encode_us,
                                       uint32_t send_us,
                                       size_t jpeg_size,
                                       bool fallback_encoder_used)
{
    const int64_t now_us = esp_timer_get_time();
    if (stream_performance.window_start_us == 0)
        stream_performance.window_start_us = now_us;

    stream_performance.queue_wait_total_us += queue_wait_us;
    stream_performance.encode_total_us += encode_us;
    stream_performance.send_total_us += send_us;
    stream_performance.jpeg_total_size += jpeg_size;
    stream_performance.frame_count++;
    if (fallback_encoder_used)
        stream_performance.fallback_count++;

    update_maximum(queue_wait_us, &stream_performance.queue_wait_max_us);
    update_maximum(encode_us, &stream_performance.encode_max_us);
    update_maximum(send_us, &stream_performance.send_max_us);

    if (now_us - stream_performance.window_start_us < PERFORMANCE_WINDOW_US)
        return;

    const uint32_t frames = stream_performance.frame_count;
    ESP_LOGI(TAG,
             "PERF_STREAM frames=%u wait_avg=%u max=%u encode_avg=%u max=%u "
             "send_avg=%u max=%u jpeg_avg=%u fallback=%u",
             (unsigned)frames,
             (unsigned)(stream_performance.queue_wait_total_us / frames),
             (unsigned)stream_performance.queue_wait_max_us,
             (unsigned)(stream_performance.encode_total_us / frames),
             (unsigned)stream_performance.encode_max_us,
             (unsigned)(stream_performance.send_total_us / frames),
             (unsigned)stream_performance.send_max_us,
             (unsigned)(stream_performance.jpeg_total_size / frames),
             (unsigned)stream_performance.fallback_count);

    memset(&stream_performance, 0, sizeof(stream_performance));
    stream_performance.window_start_us = now_us;
}
