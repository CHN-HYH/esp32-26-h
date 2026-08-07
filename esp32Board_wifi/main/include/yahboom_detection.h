#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_camera.h"
#include "freertos/FreeRTOS.h"

// 水管内部的固定识别区域，参数以相机原始 QVGA（320x240）像素为准。
#define DETECTION_ROI_X                                 0
#define DETECTION_ROI_Y                                 107
#define DETECTION_ROI_WIDTH                             320
#define DETECTION_ROI_HEIGHT                            26
#define DETECTION_EDGE_MARGIN                           0

// 钢珠在当前 QVGA 画面中约为 8 像素直径：中心亮斑周围有一圈较暗的边缘。
#define DETECTION_RING_CENTER_RADIUS                    1
#define DETECTION_RING_RADIUS                           4
#define DETECTION_RING_DIAGONAL_OFFSET                  3
#define DETECTION_RING_CENTERLINE_HALF_HEIGHT           5
#define DETECTION_RING_SCAN_STEP                        2
#define DETECTION_RING_TRACK_SEARCH_HALF_WIDTH         48
#define DETECTION_RING_MIN_CENTER_BRIGHTNESS            120
#define DETECTION_RING_MIN_SCORE                        32
#define DETECTION_RING_MIN_DARK_SAMPLE_CONTRAST         18
#define DETECTION_RING_MIN_DARK_SAMPLES                 6
#define DETECTION_RING_MIN_ACTIVE_COLUMNS               1
#define DETECTION_RING_MAX_COLUMN_GAP                   1

#define DETECTION_EVERY_N_FRAMES                        2
#define DETECTION_MARKER_HALF_SIZE_PIXELS               6
#define DETECTION_LOST_COUNT                            3
#define DETECTION_TRACK_MAX_JUMP_PIXELS                 30
#define DETECTION_REACQUIRE_CONFIRM_COUNT               2
#define DETECTION_REACQUIRE_MAX_DRIFT_PIXELS            40
#define DETECTION_LOG_INTERVAL_MS                       500

typedef struct
{
    // YUV422 中单个像素的亮度和共享色度值。
    uint8_t y;
    uint8_t cb;
    uint8_t cr;
} yahboom_yuv422_color_t;

typedef struct
{
    // 一段连续亮芯暗环响应的候选信息。
    bool valid;
    int center_x;
    int center_y;
    int distance_x;
    uint8_t score;
} yahboom_detection_candidate_t;

typedef struct
{
    // 已确认目标的位置、丢失计数和重新捕获状态。
    bool position_valid;
    int center_x;
    int center_y;
    uint8_t missing_count;
    uint16_t width;
    bool reacquire_valid;
    int reacquire_center_x;
    uint8_t reacquire_count;
    bool reported_target_found;
    uint8_t reported_missing_count;
} yahboom_detection_tracking_t;

typedef struct
{
    // 亮芯暗环检测的运行时状态；不保存背景图，也不依赖空场景采集。
    uint8_t detect_frame_count;
    bool invalid_input_reported;
    TickType_t last_detection_log_tick;
    yahboom_detection_tracking_t tracking;
} yahboom_detection_context_t;

#ifdef __cplusplus
extern "C" {
#endif

// 初始化亮芯暗环检测上下文。
void yahboom_detection_init(yahboom_detection_context_t *context);

// 处理一帧并在原始 YUV422 图像上叠加 ROI 与目标标记。
void yahboom_detection_process(yahboom_detection_context_t *context, camera_fb_t *frame);

#ifdef __cplusplus
}
#endif
