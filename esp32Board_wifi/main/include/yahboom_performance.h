#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// 记录摄像头任务各阶段耗时，内部每两秒输出一次统计结果。
void yahboom_performance_record_camera(uint32_t capture_us,
                                       uint32_t detection_us,
                                       uint32_t overlay_us,
                                       bool queue_replaced);

// 记录图传任务各阶段耗时，并统计是否退回通用 JPEG 编码器。
void yahboom_performance_record_stream(uint32_t queue_wait_us,
                                       uint32_t encode_us,
                                       uint32_t send_us,
                                       size_t jpeg_size,
                                       bool fallback_encoder_used);

#ifdef __cplusplus
}
#endif
