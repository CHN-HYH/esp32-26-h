#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_camera.h"
#include "freertos/FreeRTOS.h"

// 水管识别区域的默认配置。网页端可以在运行时调整 Y 起点和高度。
#define DETECTION_ROI_X                              0  // ROI 左上角 X 坐标。
#define DETECTION_ROI_Y_DEFAULT                     70  // ROI 左上角 Y 坐标默认值。
#define DETECTION_ROI_WIDTH                        320  // ROI 宽度。
#define DETECTION_ROI_HEIGHT_DEFAULT                25  // ROI 高度默认值。
#define DETECTION_EDGE_MARGIN                        0  // 距离图像边缘的安全留白。
#define DETECTION_ROI_BORDER_THICKNESS_PIXELS        2  // 图传中 ROI 蓝色边框的内缩线宽。

#ifdef __cplusplus
extern "C" {
#endif

// 运行时 ROI 参数。网页通过 /control 接口修改，重启后恢复默认值。
extern volatile int DETECTION_ROI_Y;
extern volatile int DETECTION_ROI_HEIGHT;

// 设置 ROI 参数；Y、高度还必须满足 Y + 高度 <= 240。
bool yahboom_detection_set_roi_y(int value);
bool yahboom_detection_set_roi_height(int value);

// 钢珠亮芯和暗环的几何参数，单位为像素。
#define DETECTION_RING_CENTER_RADIUS                 1  // 亮芯均值采样的上下左右偏移。
#define DETECTION_RING_RADIUS                        4  // 暗环上下左右采样点到候选中心的距离。
#define DETECTION_RING_DIAGONAL_OFFSET               3  // 暗环四个对角采样点的横纵偏移。
#define DETECTION_RING_CENTERLINE_HALF_HEIGHT        5  // 只在 ROI 中轴线正负该范围内搜索。
#define DETECTION_RING_SCAN_STEP                     2  // 候选扫描步长，越小越精细但计算量越大。
#define DETECTION_RING_TRACK_SEARCH_HALF_WIDTH      48  // 跟踪有效时相对上次 X 坐标的搜索半宽。

// 亮芯与暗环的亮度判定阈值，YUV422 中只使用 Y 亮度通道。
#define DETECTION_RING_MIN_CENTER_BRIGHTNESS       120  // 亮芯平均亮度下限。
#define DETECTION_RING_MIN_SCORE                    32  // 亮芯均值减暗环均值的最小对比度。
#define DETECTION_RING_MIN_DARK_SAMPLE_CONTRAST     18  // 单个暗环点相对亮芯的最小亮度差。
#define DETECTION_RING_MIN_DARK_SAMPLES              6  // 八个暗环采样点中必须满足对比度的最少数量。

// 连续帧状态与图传标记参数。
#define DETECTION_EVERY_N_FRAMES                     1  // 每隔多少相机帧执行一次识别。
#define DETECTION_MARKER_HALF_SIZE_PIXELS            6  // 红色目标方框的半边长。
#define DETECTION_MARKER_THICKNESS_PIXELS            2  // 红色目标方框的内缩线宽。
#define DETECTION_HOLD_MISSED_FRAMES                 1  // 连续漏检后仍保留最后一次实测坐标的检测周期数。
#define DETECTION_LOST_COUNT                         3  // 连续多少个检测周期未命中后清除轨迹。
#define DETECTION_TRACK_MAX_JUMP_PIXELS             30  // 跟踪状态下相邻检测允许的最大 X 跳变。
#define DETECTION_REACQUIRE_CONFIRM_COUNT            2  // 丢失后连续命中多少次才重新确认目标。
#define DETECTION_REACQUIRE_MAX_DRIFT_PIXELS        40  // 重捕获确认期间相邻候选允许的最大 X 漂移。
#define DETECTION_LOG_INTERVAL_MS                  500  // 识别日志的最小输出间隔，单位为毫秒。

typedef struct
{
    uint8_t y;   // YUV422 的亮度分量。
    uint8_t cb;  // YUV422 的蓝色色度分量。
    uint8_t cr;  // YUV422 的红色色度分量。
} yahboom_yuv422_color_t;

typedef struct
{
    bool position_valid;             // 最近一次确认的目标坐标是否有效。
    int center_x;                    // 最近一次确认的目标中心 X 坐标。
    int center_y;                    // 最近一次确认的目标中心 Y 坐标。
    uint8_t missing_count;           // 确认目标后的连续漏检次数。
    uint16_t width;                  // 发送给 MSP 的目标特征宽度。
    bool reacquire_valid;            // 是否已有待确认的重捕获候选。
    int reacquire_center_x;          // 待确认重捕获候选的 X 坐标。
    int reacquire_center_y;          // 待确认重捕获候选的 Y 坐标。
    uint8_t reacquire_count;         // 当前候选连续通过重捕获检查的次数。
    bool reported_target_found;      // 是否已经输出过 RING_FOUND 日志。
    uint8_t reported_missing_count;  // 用于控制 RING_LOST 日志的连续漏检计数。
} yahboom_detection_tracking_t;

typedef struct
{
    uint8_t detect_frame_count;              // 距离上一次实际识别的帧计数。
    bool invalid_roi_reported;               // 非法图像或 ROI 错误是否已输出日志。
    TickType_t last_detection_log_tick;      // 上一次坐标或未命中日志的系统节拍。
    yahboom_detection_tracking_t tracking;   // 目标跟踪与重捕获状态。
} yahboom_detection_context_t;

// 初始化亮芯暗环识别状态。
void yahboom_detection_init(yahboom_detection_context_t *context);

// 在原始 YUV422 帧上完成识别并叠加 ROI 与目标标记。
void yahboom_detection_process(yahboom_detection_context_t *context, camera_fb_t *frame);

#ifdef __cplusplus
}
#endif
