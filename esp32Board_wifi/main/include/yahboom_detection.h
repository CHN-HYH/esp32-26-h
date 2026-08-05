#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "esp_camera.h"

// 背景差分和一维 X 投影识别参数。
#define BACKGROUND_START_DELAY_MS               10000   // 上电后等待空场景稳定的时间，单位：ms
#define BACKGROUND_WARMUP_FRAMES                20      // 等待结束后丢弃的相机稳定帧数
#define BACKGROUND_CAPTURE_FRAMES               8       // 用于平均生成空场景背景的帧数
#define DETECTION_ROI_X                         0       // 识别区域左上角 X 坐标
#define DETECTION_ROI_Y                         95      // 识别区域左上角 Y 坐标
#define DETECTION_ROI_WIDTH                     320     // 识别区域宽度，单位：像素
#define DETECTION_ROI_HEIGHT                    50      // 识别区域高度，单位：像素
#define DETECTION_SCAN_STEP                     2       // 差分扫描步长
#define DETECTION_EDGE_MARGIN                   0       // 画面安全边缘宽度，单位：像素
#define DETECTION_BRIGHT_THRESHOLD              35      // 补偿后比背景更亮时的最小亮度差
#define DETECTION_DARK_THRESHOLD                15      // 补偿后比背景更暗时的最小亮度差
#define DETECTION_MAX_FOREGROUND_PERCENT        25      // 前景超过 ROI 的该比例时，判定为光照或遮挡扰动
#define DETECTION_X_MIN_COLUMN_SAMPLES          2       // 有效投影列的最少前景采样点数
#define DETECTION_X_MIN_WIDTH                   4       // 有效投影区间的最少连续采样列数
#define DETECTION_X_MIN_SAMPLES                 10      // 有效投影区间的最少前景采样点总数
#define DETECTION_MARKER_HALF_WIDTH_PIXELS      12      // 红色十字到左右白色竖线的距离，单位：像素
#define DETECTION_LOST_COUNT                    3       // 连续丢失多少个检测周期后判定目标离开
#define DETECTION_TRACK_MAX_JUMP_PIXELS         30      // 跟踪有效时允许的最大 X 偏移，单位：像素
#define DETECTION_REACQUIRE_CONFIRM_COUNT       2       // 远处单候选连续确认次数
#define DETECTION_REACQUIRE_MAX_DRIFT_PIXELS   40       // 远处单候选相邻确认允许的最大 X 偏移，单位：像素
#define DETECTION_LOG_INTERVAL_MS               500     // 坐标日志的最小输出间隔，单位：ms
#define STATUS_BANNER_DURATION_MS               3000    // 背景采集完成后 START 提示的显示时间，单位：ms

enum { YAHBOOM_DETECTION_HISTOGRAM_SIZE = 511 };

typedef struct
{
    // YUV422 中单个像素的亮度和共享色度值。
    uint8_t y;
    uint8_t cb;
    uint8_t cr;
} yahboom_yuv422_color_t;

typedef struct
{
    // 一段连续 X 投影区间及其加权中心。
    bool valid;
    int start_x;
    int end_x;
    int center_x;
    uint32_t samples;
    uint32_t weighted_x;
    int distance_x;
} yahboom_detection_candidate_t;

typedef struct
{
    // 当前目标的确认位置、丢失计数和重捕获状态。
    bool position_valid;
    int center_x;
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
    // 背景缓存和检测过程的全部运行时状态。
    uint8_t *background_y;
    size_t background_pixel_count;
    uint16_t background_width;
    uint16_t background_height;
    TickType_t background_delay_start_tick;
    bool background_delay_started;
    uint8_t background_warmup_count;
    uint8_t background_capture_count;
    bool background_ready;
    uint16_t light_difference_histogram[YAHBOOM_DETECTION_HISTOGRAM_SIZE];

    uint8_t detect_frame_count;
    bool invalid_roi_reported;
    TickType_t last_detection_log_tick;
    yahboom_detection_tracking_t tracking;
} yahboom_detection_context_t;

#ifdef __cplusplus
extern "C" {
#endif

// 初始化检测上下文；背景缓存会在首次处理帧时按实际尺寸分配。
void yahboom_detection_init(yahboom_detection_context_t *context);

// 处理一帧并在原始 YUV422 图像上叠加 ROI、目标标记和状态提示。
void yahboom_detection_process(yahboom_detection_context_t *context, camera_fb_t *frame);

#ifdef __cplusplus
}
#endif
