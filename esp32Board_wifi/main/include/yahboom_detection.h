#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_camera.h"
#include "freertos/FreeRTOS.h"

// 固定水管识别区域。按实际水管在画面中的位置调整。
#define DETECTION_ROI_X                              0
#define DETECTION_ROI_Y                            108
#define DETECTION_ROI_WIDTH                        320
#define DETECTION_ROI_HEIGHT                        26
#define DETECTION_EDGE_MARGIN                        0
#define DETECTION_ROI_BORDER_THICKNESS_PIXELS        2

// 钢珠亮芯和暗环的几何参数，单位为像素。
#define DETECTION_RING_CENTER_RADIUS                 1
#define DETECTION_RING_RADIUS                        4
#define DETECTION_RING_DIAGONAL_OFFSET               3
#define DETECTION_RING_CENTERLINE_HALF_HEIGHT        5
#define DETECTION_RING_SCAN_STEP                     2
#define DETECTION_RING_TRACK_SEARCH_HALF_WIDTH      48

// 亮芯与暗环的亮度判定阈值，YUV422 中只使用 Y 亮度通道。
#define DETECTION_RING_MIN_CENTER_BRIGHTNESS       120
#define DETECTION_RING_MIN_SCORE                    32
#define DETECTION_RING_MIN_DARK_SAMPLE_CONTRAST     18
#define DETECTION_RING_MIN_DARK_SAMPLES              6

// 连续帧状态与图传标记参数。
#define DETECTION_EVERY_N_FRAMES                     2
#define DETECTION_MARKER_HALF_SIZE_PIXELS            6
#define DETECTION_MARKER_THICKNESS_PIXELS            2
#define DETECTION_LOST_COUNT                         3
#define DETECTION_TRACK_MAX_JUMP_PIXELS             30
#define DETECTION_REACQUIRE_CONFIRM_COUNT            2
#define DETECTION_REACQUIRE_MAX_DRIFT_PIXELS        40
#define DETECTION_LOG_INTERVAL_MS                  500

typedef struct
{
    uint8_t y;
    uint8_t cb;
    uint8_t cr;
} yahboom_yuv422_color_t;

typedef struct
{
    bool position_valid;
    int center_x;
    int center_y;
    uint8_t missing_count;
    uint16_t width;
    bool reacquire_valid;
    int reacquire_center_x;
    int reacquire_center_y;
    uint8_t reacquire_count;
    bool reported_target_found;
    uint8_t reported_missing_count;
} yahboom_detection_tracking_t;

typedef struct
{
    uint8_t detect_frame_count;
    bool invalid_roi_reported;
    TickType_t last_detection_log_tick;
    yahboom_detection_tracking_t tracking;
} yahboom_detection_context_t;

#ifdef __cplusplus
extern "C" {
#endif

// 初始化亮芯暗环识别状态。
void yahboom_detection_init(yahboom_detection_context_t *context);

// 在原始 YUV422 帧上完成识别并叠加 ROI 与目标标记。
void yahboom_detection_process(yahboom_detection_context_t *context, camera_fb_t *frame);

#ifdef __cplusplus
}
#endif
