#include "yahboom_detection.h"

#include <string.h>

#include "esp_heap_caps.h"
#include "esp_log.h"
#include "freertos/task.h"
#include "yahboom_msp_uart.h"
#include "yahboom_overlay.h"

static const char *TAG = "yahboom_camera";
static const yahboom_yuv422_color_t kDetectionBoxColor = {235, 128, 128};
static const yahboom_yuv422_color_t kDetectionCrossColor = {76, 85, 255};
static const yahboom_yuv422_color_t kDetectionRoiColor = {150, 255, 100};
static const int kDetectionCrossRadius = 10;

static inline void draw_yuv422_pixel(camera_fb_t *frame, int x, int y,
                                     const yahboom_yuv422_color_t *color)
{
    if (x < 0 || x >= frame->width || y < 0 || y >= frame->height)
        return;

    size_t pixel_offset = (y * frame->width + x) * 2;
    size_t pair_offset = (y * frame->width + (x & ~1)) * 2;
    frame->buf[pixel_offset] = color->y;
    frame->buf[pair_offset + 1] = color->cb;
    frame->buf[pair_offset + 3] = color->cr;
}

static bool get_detection_roi(const camera_fb_t *frame, int *left, int *top,
                              int *right, int *bottom)
{
    const int minimum_x = DETECTION_EDGE_MARGIN;
    const int minimum_y = DETECTION_EDGE_MARGIN;
    const int maximum_x = frame->width - DETECTION_EDGE_MARGIN;
    const int maximum_y = frame->height - DETECTION_EDGE_MARGIN;

    if (maximum_x <= minimum_x || maximum_y <= minimum_y)
        return false;

    *left = DETECTION_ROI_X < minimum_x ? minimum_x : DETECTION_ROI_X;
    *top = DETECTION_ROI_Y < minimum_y ? minimum_y : DETECTION_ROI_Y;
    *right = DETECTION_ROI_X + DETECTION_ROI_WIDTH;
    *bottom = DETECTION_ROI_Y + DETECTION_ROI_HEIGHT;

    if (*right > maximum_x)
        *right = maximum_x;
    if (*bottom > maximum_y)
        *bottom = maximum_y;

    return *right > *left && *bottom > *top;
}

static void draw_detection_roi(camera_fb_t *frame)
{
    int left;
    int top;
    int right;
    int bottom;
    if (!get_detection_roi(frame, &left, &top, &right, &bottom))
        return;

    for (int x = left; x < right; x++)
    {
        draw_yuv422_pixel(frame, x, top, &kDetectionRoiColor);
        draw_yuv422_pixel(frame, x, bottom - 1, &kDetectionRoiColor);
    }

    for (int y = top; y < bottom; y++)
    {
        draw_yuv422_pixel(frame, left, y, &kDetectionRoiColor);
        draw_yuv422_pixel(frame, right - 1, y, &kDetectionRoiColor);
    }
}

static void draw_detection_marker(camera_fb_t *frame, int center_x,
                                  int roi_top, int roi_bottom)
{
    const int left_x = center_x - DETECTION_MARKER_HALF_WIDTH_PIXELS;
    const int right_x = center_x + DETECTION_MARKER_HALF_WIDTH_PIXELS;
    for (int y = roi_top; y < roi_bottom; y++)
    {
        draw_yuv422_pixel(frame, left_x, y, &kDetectionBoxColor);
        draw_yuv422_pixel(frame, right_x, y, &kDetectionBoxColor);
    }

    const int center_y = (roi_top + roi_bottom) / 2;
    for (int offset = -kDetectionCrossRadius; offset <= kDetectionCrossRadius; offset++)
    {
        draw_yuv422_pixel(frame, center_x + offset, center_y, &kDetectionCrossColor);
        draw_yuv422_pixel(frame, center_x, center_y + offset, &kDetectionCrossColor);
    }
}

static void draw_detection_overlays(camera_fb_t *frame)
{
    draw_detection_roi(frame);
    yahboom_overlay_draw_status(frame);
}

static bool capture_background(yahboom_detection_context_t *context,
                               const camera_fb_t *frame)
{
    const size_t pixel_count = frame->width * frame->height;
    if (frame->format != PIXFORMAT_YUV422 || frame->len < pixel_count * 2)
        return false;

    if (context->background_ready)
        return true;

    if (!context->background_delay_started)
    {
        context->background_delay_started = true;
        context->background_delay_start_tick = xTaskGetTickCount();
        yahboom_overlay_show_wait(BACKGROUND_START_DELAY_MS);
        ESP_LOGI(TAG, "BACKGROUND: preview active, waiting %d ms before capture",
                 BACKGROUND_START_DELAY_MS);
        return false;
    }

    if (xTaskGetTickCount() - context->background_delay_start_tick <
        pdMS_TO_TICKS(BACKGROUND_START_DELAY_MS))
        return false;

    if (context->background_warmup_count < BACKGROUND_WARMUP_FRAMES)
    {
        if (context->background_warmup_count == 0)
            ESP_LOGI(TAG, "BACKGROUND: delay complete, camera stabilizing before capture");
        context->background_warmup_count++;
        return false;
    }

    if (context->background_y == NULL)
    {
        context->background_y = (uint8_t *)heap_caps_malloc(
            pixel_count, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (context->background_y == NULL)
        {
            ESP_LOGE(TAG, "BACKGROUND: PSRAM allocation failed");
            return false;
        }
        context->background_pixel_count = pixel_count;
        context->background_width = frame->width;
        context->background_height = frame->height;
        ESP_LOGI(TAG, "BACKGROUND: capturing %d empty-scene frames",
                 BACKGROUND_CAPTURE_FRAMES);
    }

    if (context->background_width != frame->width ||
        context->background_height != frame->height ||
        context->background_pixel_count != pixel_count)
        return false;

    if (context->background_capture_count < BACKGROUND_CAPTURE_FRAMES)
    {
        for (size_t index = 0; index < pixel_count; index++)
        {
            uint8_t brightness = frame->buf[index * 2];
            if (context->background_capture_count == 0)
                context->background_y[index] = brightness;
            else
                context->background_y[index] =
                    ((uint16_t)context->background_y[index] * context->background_capture_count +
                     brightness) /
                    (context->background_capture_count + 1);
        }

        context->background_capture_count++;
        if (context->background_capture_count == BACKGROUND_CAPTURE_FRAMES)
        {
            context->background_ready = true;
            yahboom_overlay_show_start(STATUS_BANNER_DURATION_MS);
            ESP_LOGI(TAG, "BACKGROUND_READY: start recognition");
        }
    }

    return context->background_ready;
}

static void update_detection_state(yahboom_detection_context_t *context,
                                   bool target_found)
{
    yahboom_detection_tracking_t *tracking = &context->tracking;

    if (target_found)
    {
        if (!tracking->reported_target_found)
            ESP_LOGI(TAG, "DIFF_X_FOUND");
        tracking->reported_target_found = true;
        tracking->reported_missing_count = 0;
        return;
    }

    if (!tracking->reported_target_found)
        return;

    if (++tracking->reported_missing_count >= DETECTION_LOST_COUNT)
    {
        tracking->reported_target_found = false;
        tracking->reported_missing_count = 0;
        ESP_LOGI(TAG, "DIFF_X_LOST");
    }
}

static void send_msp_tracking_status(const yahboom_detection_context_t *context)
{
    const yahboom_detection_tracking_t *tracking = &context->tracking;
    const bool valid = !context->invalid_roi_reported && tracking->position_valid &&
                       tracking->missing_count == 0;
    yahboom_msp_uart_send(valid, valid ? (uint16_t)tracking->center_x : 0,
                          valid ? tracking->width : 0);
}

static int calculate_roi_light_offset(yahboom_detection_context_t *context,
                                      const camera_fb_t *frame, int left, int top,
                                      int right, int bottom)
{
    uint32_t sample_count = 0;
    uint32_t cumulative_count = 0;
    // 全局亮度补偿只需要估计中值，不需要和 X 投影使用相同密度。
    const int offset_scan_step = DETECTION_SCAN_STEP * 2;
    memset(context->light_difference_histogram, 0,
           sizeof(context->light_difference_histogram));

    for (int y = top; y < bottom; y += offset_scan_step)
    {
        for (int x = left; x < right; x += offset_scan_step)
        {
            size_t pixel_index = y * frame->width + x;
            int difference = (int)frame->buf[pixel_index * 2] -
                             context->background_y[pixel_index];
            context->light_difference_histogram[difference + 255]++;
            sample_count++;
        }
    }

    if (sample_count == 0)
        return 0;

    const uint32_t median_index = sample_count / 2;
    for (int index = 0; index < YAHBOOM_DETECTION_HISTOGRAM_SIZE; index++)
    {
        cumulative_count += context->light_difference_histogram[index];
        if (cumulative_count > median_index)
            return index - 255;
    }

    return 0;
}

static void detect_x_projection(yahboom_detection_context_t *context,
                                camera_fb_t *frame)
{
    int roi_left;
    int roi_top;
    int roi_right;
    int roi_bottom;
    if (!get_detection_roi(frame, &roi_left, &roi_top, &roi_right, &roi_bottom))
    {
        if (!context->invalid_roi_reported)
        {
            ESP_LOGE(TAG, "DIFF_X: invalid detection ROI");
            context->invalid_roi_reported = true;
        }
        update_detection_state(context, false);
        yahboom_overlay_draw_status(frame);
        send_msp_tracking_status(context);
        return;
    }
    context->invalid_roi_reported = false;

    const int sample_width = (roi_right - roi_left + DETECTION_SCAN_STEP - 1) /
                             DETECTION_SCAN_STEP;
    const int sample_height = (roi_bottom - roi_top + DETECTION_SCAN_STEP - 1) /
                              DETECTION_SCAN_STEP;
    const size_t sample_count = (size_t)sample_width * sample_height;

    if (!capture_background(context, frame))
    {
        draw_detection_overlays(frame);
        send_msp_tracking_status(context);
        return;
    }

    // 保留当前每两帧检测一次的节奏，避免识别再次挤占图传。
    if (++context->detect_frame_count < 2)
    {
        draw_detection_overlays(frame);
        send_msp_tracking_status(context);
        return;
    }
    context->detect_frame_count = 0;

    const int light_offset = calculate_roi_light_offset(context, frame, roi_left, roi_top,
                                                         roi_right, roi_bottom);
    uint16_t x_projection[sample_width];
    memset(x_projection, 0, sizeof(x_projection));

    size_t foreground_sample_count = 0;
    for (int sample_y = 0; sample_y < sample_height; sample_y++)
    {
        int y = roi_top + sample_y * DETECTION_SCAN_STEP;
        for (int sample_x = 0; sample_x < sample_width; sample_x++)
        {
            int x = roi_left + sample_x * DETECTION_SCAN_STEP;
            size_t pixel_index = y * frame->width + x;
            int signed_difference = (int)frame->buf[pixel_index * 2] -
                                    context->background_y[pixel_index] - light_offset;
            int difference = signed_difference < 0 ? -signed_difference : signed_difference;
            bool is_foreground = difference > DETECTION_BRIGHT_THRESHOLD ||
                                 signed_difference < -DETECTION_DARK_THRESHOLD;
            if (is_foreground)
            {
                x_projection[sample_x]++;
                foreground_sample_count++;
            }
        }
    }

    // 钢珠只应覆盖 ROI 的小部分；大面积变化通常是光照或遮挡扰动。
    if (foreground_sample_count * 100 > sample_count * DETECTION_MAX_FOREGROUND_PERCENT)
    {
        TickType_t now = xTaskGetTickCount();
        if (context->last_detection_log_tick == 0 ||
            now - context->last_detection_log_tick >= pdMS_TO_TICKS(DETECTION_LOG_INTERVAL_MS))
        {
            context->last_detection_log_tick = now;
            ESP_LOGI(TAG, "DIFF_LIGHT_DISTURBANCE changed=%u total=%u offset=%d",
                     (unsigned)foreground_sample_count, (unsigned)sample_count, light_offset);
        }

        if (++context->tracking.missing_count >= DETECTION_LOST_COUNT)
        {
            context->tracking.position_valid = false;
            context->tracking.width = 0;
            context->tracking.missing_count = 0;
        }
        context->tracking.reacquire_valid = false;
        context->tracking.reacquire_count = 0;
        update_detection_state(context, false);
        draw_detection_overlays(frame);
        send_msp_tracking_status(context);
        return;
    }

    yahboom_detection_candidate_t strongest = {0};
    yahboom_detection_candidate_t nearest = {0};
    yahboom_detection_candidate_t candidate = {0};
    uint8_t candidate_count = 0;
    int candidate_start_x = -1;
    uint32_t candidate_samples = 0;
    uint32_t candidate_weighted_x = 0;

    // 一段连续有效列就是一个候选，分别保留最强候选和最近候选。
    for (int sample_x = 0; sample_x <= sample_width; sample_x++)
    {
        bool column_is_active = sample_x < sample_width &&
                                x_projection[sample_x] >= DETECTION_X_MIN_COLUMN_SAMPLES;
        if (column_is_active)
        {
            if (candidate_start_x < 0)
            {
                candidate_start_x = sample_x;
                candidate_samples = 0;
                candidate_weighted_x = 0;
            }
            int pixel_x = roi_left + sample_x * DETECTION_SCAN_STEP;
            candidate_samples += x_projection[sample_x];
            candidate_weighted_x += pixel_x * x_projection[sample_x];
            continue;
        }

        if (candidate_start_x < 0)
            continue;

        int candidate_width = sample_x - candidate_start_x;
        if (candidate_width >= DETECTION_X_MIN_WIDTH &&
            candidate_samples >= DETECTION_X_MIN_SAMPLES)
        {
            candidate.valid = true;
            candidate.start_x = roi_left + candidate_start_x * DETECTION_SCAN_STEP;
            candidate.end_x = roi_left + (sample_x - 1) * DETECTION_SCAN_STEP;
            candidate.samples = candidate_samples;
            candidate.weighted_x = candidate_weighted_x;
            candidate.center_x = candidate_weighted_x / candidate_samples;
            candidate.distance_x = candidate.center_x - context->tracking.center_x;
            if (candidate.distance_x < 0)
                candidate.distance_x = -candidate.distance_x;
            candidate_count++;

            if (!strongest.valid || candidate.samples > strongest.samples)
                strongest = candidate;
            if (candidate.distance_x <= DETECTION_TRACK_MAX_JUMP_PIXELS &&
                (!nearest.valid || candidate.distance_x < nearest.distance_x ||
                 (candidate.distance_x == nearest.distance_x && candidate.samples > nearest.samples)))
                nearest = candidate;
        }
        candidate_start_x = -1;
    }

    yahboom_detection_tracking_t *tracking = &context->tracking;
    bool target_found = false;
    yahboom_detection_candidate_t selected = {0};
    if (tracking->position_valid && nearest.valid)
    {
        selected = nearest;
        target_found = true;
        tracking->center_x = selected.center_x;
        tracking->width = selected.end_x - selected.start_x + DETECTION_SCAN_STEP;
        tracking->missing_count = 0;
        tracking->reacquire_valid = false;
        tracking->reacquire_count = 0;
    }
    else if (candidate_count == 1 && strongest.valid)
    {
        int drift_x = strongest.center_x - tracking->reacquire_center_x;
        if (drift_x < 0)
            drift_x = -drift_x;

        if (!tracking->reacquire_valid || drift_x > DETECTION_REACQUIRE_MAX_DRIFT_PIXELS)
        {
            tracking->reacquire_valid = true;
            tracking->reacquire_center_x = strongest.center_x;
            tracking->reacquire_count = 1;
        }
        else
        {
            tracking->reacquire_center_x = strongest.center_x;
            tracking->reacquire_count++;
        }

        if (tracking->reacquire_count >= DETECTION_REACQUIRE_CONFIRM_COUNT)
        {
            selected = strongest;
            target_found = true;
            tracking->position_valid = true;
            tracking->center_x = selected.center_x;
            tracking->width = selected.end_x - selected.start_x + DETECTION_SCAN_STEP;
            tracking->missing_count = 0;
            tracking->reacquire_valid = false;
            tracking->reacquire_count = 0;
            ESP_LOGI(TAG, "DIFF_X_REACQUIRED x=%d", selected.center_x);
        }
    }
    else
    {
        tracking->reacquire_valid = false;
        tracking->reacquire_count = 0;
        if (candidate_count > 1)
        {
            TickType_t now = xTaskGetTickCount();
            if (context->last_detection_log_tick == 0 ||
                now - context->last_detection_log_tick >= pdMS_TO_TICKS(DETECTION_LOG_INTERVAL_MS))
            {
                context->last_detection_log_tick = now;
                ESP_LOGI(TAG, "DIFF_X_MULTIPLE_FAR candidates=%u previous_x=%d",
                         (unsigned)candidate_count, tracking->center_x);
            }
        }
    }

    if (!target_found && ++tracking->missing_count >= DETECTION_LOST_COUNT)
    {
        tracking->position_valid = false;
        tracking->width = 0;
        tracking->missing_count = 0;
        tracking->reacquire_valid = false;
        tracking->reacquire_count = 0;
    }

    if (target_found)
    {
        draw_detection_marker(frame, selected.center_x, roi_top, roi_bottom);
        TickType_t now = xTaskGetTickCount();
        if (context->last_detection_log_tick == 0 ||
            now - context->last_detection_log_tick >= pdMS_TO_TICKS(DETECTION_LOG_INTERVAL_MS))
        {
            context->last_detection_log_tick = now;
            int width = selected.end_x - selected.start_x + DETECTION_SCAN_STEP;
            ESP_LOGI(TAG, "DIFF_X x=%d width=%d samples=%u candidates=%u offset=%d",
                     selected.center_x, width, (unsigned)selected.samples,
                     (unsigned)candidate_count, light_offset);
        }
    }

    update_detection_state(context, target_found);
    draw_detection_overlays(frame);
    send_msp_tracking_status(context);
}

void yahboom_detection_init(yahboom_detection_context_t *context)
{
    if (context == NULL)
        return;

    memset(context, 0, sizeof(*context));
}

void yahboom_detection_process(yahboom_detection_context_t *context, camera_fb_t *frame)
{
    if (context == NULL || frame == NULL)
        return;

    detect_x_projection(context, frame);
}
