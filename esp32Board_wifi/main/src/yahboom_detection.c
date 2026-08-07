#include "yahboom_detection.h"

#include <string.h>

#include "esp_log.h"
#include "freertos/task.h"
#include "yahboom_msp_uart.h"
#include "yahboom_overlay.h"

static const char *TAG = "yahboom_camera";
static const yahboom_yuv422_color_t kDetectionBoxColor = {235, 128, 128};
static const yahboom_yuv422_color_t kDetectionCrossColor = {76, 85, 255};
static const yahboom_yuv422_color_t kDetectionRoiColor = {150, 255, 100};

typedef struct
{
    uint8_t center_luma;
    uint8_t score;
    uint8_t dark_sample_count;
} ring_measurement_t;

static inline void draw_yuv422_pixel(camera_fb_t *frame, int x, int y,
                                     const yahboom_yuv422_color_t *color)
{
    if (x < 0 || x >= frame->width || y < 0 || y >= frame->height)
        return;

    const size_t pixel_offset = ((size_t)y * frame->width + x) * 2;
    const size_t pair_offset = ((size_t)y * frame->width + (x & ~1)) * 2;
    frame->buf[pixel_offset] = color->y;
    frame->buf[pair_offset + 1] = color->cb;
    frame->buf[pair_offset + 3] = color->cr;
}

static inline uint8_t get_yuv422_luma(const camera_fb_t *frame, int x, int y)
{
    return frame->buf[((size_t)y * frame->width + x) * 2];
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

static void draw_detection_marker(camera_fb_t *frame, int center_x, int center_y)
{
    const int half_size = DETECTION_MARKER_HALF_SIZE_PIXELS;
    const int left = center_x - half_size;
    const int right = center_x + half_size;
    const int top = center_y - half_size;
    const int bottom = center_y + half_size;

    for (int x = left; x <= right; x++)
    {
        draw_yuv422_pixel(frame, x, top, &kDetectionBoxColor);
        draw_yuv422_pixel(frame, x, bottom, &kDetectionBoxColor);
    }

    for (int y = top; y <= bottom; y++)
    {
        draw_yuv422_pixel(frame, left, y, &kDetectionBoxColor);
        draw_yuv422_pixel(frame, right, y, &kDetectionBoxColor);
    }

    for (int offset = -DETECTION_RING_RADIUS; offset <= DETECTION_RING_RADIUS; offset++)
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

static void update_detection_state(yahboom_detection_context_t *context,
                                   bool target_found)
{
    yahboom_detection_tracking_t *tracking = &context->tracking;

    if (target_found)
    {
        if (!tracking->reported_target_found)
            ESP_LOGI(TAG, "RING_FOUND");
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
        ESP_LOGI(TAG, "RING_LOST");
    }
}

static void send_msp_tracking_status(const yahboom_detection_context_t *context)
{
    const yahboom_detection_tracking_t *tracking = &context->tracking;
    const bool valid = !context->invalid_input_reported && tracking->position_valid &&
                       tracking->missing_count == 0;
    yahboom_msp_uart_send(valid, valid ? (uint16_t)tracking->center_x : 0,
                          valid ? tracking->width : 0);
}

static ring_measurement_t calculate_ring_measurement(const camera_fb_t *frame,
                                                     int center_x, int center_y)
{
    static const int kRingOffsets[8][2] = {
        {0, -DETECTION_RING_RADIUS},
        {DETECTION_RING_DIAGONAL_OFFSET, -DETECTION_RING_DIAGONAL_OFFSET},
        {DETECTION_RING_RADIUS, 0},
        {DETECTION_RING_DIAGONAL_OFFSET, DETECTION_RING_DIAGONAL_OFFSET},
        {0, DETECTION_RING_RADIUS},
        {-DETECTION_RING_DIAGONAL_OFFSET, DETECTION_RING_DIAGONAL_OFFSET},
        {-DETECTION_RING_RADIUS, 0},
        {-DETECTION_RING_DIAGONAL_OFFSET, -DETECTION_RING_DIAGONAL_OFFSET},
    };

    ring_measurement_t measurement = {0};
    uint16_t center_sum = 0;
    uint8_t center_count = 0;
    for (int offset_y = -DETECTION_RING_CENTER_RADIUS;
         offset_y <= DETECTION_RING_CENTER_RADIUS; offset_y++)
    {
        for (int offset_x = -DETECTION_RING_CENTER_RADIUS;
             offset_x <= DETECTION_RING_CENTER_RADIUS; offset_x++)
        {
            if (offset_x * offset_x + offset_y * offset_y >
                DETECTION_RING_CENTER_RADIUS * DETECTION_RING_CENTER_RADIUS)
                continue;
            center_sum += get_yuv422_luma(frame, center_x + offset_x, center_y + offset_y);
            center_count++;
        }
    }

    const int center_luma = center_sum / center_count;
    measurement.center_luma = (uint8_t)center_luma;

    uint16_t ring_sum = 0;
    uint8_t dark_sample_count = 0;
    for (int index = 0; index < 8; index++)
    {
        const int ring_luma = get_yuv422_luma(frame,
                                               center_x + kRingOffsets[index][0],
                                               center_y + kRingOffsets[index][1]);
        ring_sum += ring_luma;
        if (center_luma - ring_luma >= DETECTION_RING_MIN_DARK_SAMPLE_CONTRAST)
            dark_sample_count++;
    }

    const int score = center_luma - ring_sum / 8;
    if (score > 0)
        measurement.score = score > UINT8_MAX ? UINT8_MAX : (uint8_t)score;
    measurement.dark_sample_count = dark_sample_count;

    return measurement;
}

static bool ring_measurement_matches(const ring_measurement_t *measurement)
{
    return measurement->center_luma >= DETECTION_RING_MIN_CENTER_BRIGHTNESS &&
           measurement->score >= DETECTION_RING_MIN_SCORE &&
           measurement->dark_sample_count >= DETECTION_RING_MIN_DARK_SAMPLES;
}

static void update_tracking_after_miss(yahboom_detection_tracking_t *tracking)
{
    if (++tracking->missing_count < DETECTION_LOST_COUNT)
        return;

    tracking->position_valid = false;
    tracking->width = 0;
    tracking->missing_count = 0;
    tracking->reacquire_valid = false;
    tracking->reacquire_count = 0;
}

static void detect_bright_ring(yahboom_detection_context_t *context, camera_fb_t *frame)
{
    int roi_left;
    int roi_top;
    int roi_right;
    int roi_bottom;
    const size_t pixel_count = (size_t)frame->width * frame->height;
    if (frame->format != PIXFORMAT_YUV422 || frame->len < pixel_count * 2 ||
        !get_detection_roi(frame, &roi_left, &roi_top, &roi_right, &roi_bottom))
    {
        if (!context->invalid_input_reported)
        {
            ESP_LOGE(TAG, "RING: invalid YUV422 frame or detection ROI");
            context->invalid_input_reported = true;
        }
        context->tracking.position_valid = false;
        context->tracking.width = 0;
        draw_detection_overlays(frame);
        send_msp_tracking_status(context);
        return;
    }
    context->invalid_input_reported = false;

    if (++context->detect_frame_count < DETECTION_EVERY_N_FRAMES)
    {
        if (context->tracking.position_valid)
            draw_detection_marker(frame, context->tracking.center_x, context->tracking.center_y);
        draw_detection_overlays(frame);
        send_msp_tracking_status(context);
        return;
    }
    context->detect_frame_count = 0;

    int scan_left = roi_left + DETECTION_RING_RADIUS;
    int scan_right = roi_right - DETECTION_RING_RADIUS;
    if (context->tracking.position_valid)
    {
        const int tracked_left = context->tracking.center_x -
                                 DETECTION_RING_TRACK_SEARCH_HALF_WIDTH;
        const int tracked_right = context->tracking.center_x +
                                  DETECTION_RING_TRACK_SEARCH_HALF_WIDTH;
        if (tracked_left > scan_left)
            scan_left = tracked_left;
        if (tracked_right < scan_right)
            scan_right = tracked_right;
    }
    const int centerline_y = (roi_top + roi_bottom - 1) / 2;
    int scan_top = centerline_y - DETECTION_RING_CENTERLINE_HALF_HEIGHT;
    int scan_bottom = centerline_y + DETECTION_RING_CENTERLINE_HALF_HEIGHT;
    if (scan_top < roi_top + DETECTION_RING_RADIUS)
        scan_top = roi_top + DETECTION_RING_RADIUS;
    if (scan_bottom > roi_bottom - DETECTION_RING_RADIUS - 1)
        scan_bottom = roi_bottom - DETECTION_RING_RADIUS - 1;

    if (scan_right <= scan_left || scan_bottom < scan_top)
    {
        if (!context->invalid_input_reported)
        {
            ESP_LOGE(TAG, "RING: detection ROI is too small for ring template");
            context->invalid_input_reported = true;
        }
        context->tracking.position_valid = false;
        context->tracking.width = 0;
        draw_detection_overlays(frame);
        send_msp_tracking_status(context);
        return;
    }

    const int roi_width = roi_right - roi_left;
    uint8_t column_scores[roi_width];
    uint8_t column_y_offsets[roi_width];
    memset(column_scores, 0, sizeof(column_scores));
    memset(column_y_offsets, 0, sizeof(column_y_offsets));
    ring_measurement_t best_raw_measurement = {0};

    // 识别只取隔点样本；已锁定目标时只在上一位置附近搜索，减少 PSRAM 随机读取。
    for (int x = scan_left; x < scan_right; x += DETECTION_RING_SCAN_STEP)
    {
        const int column_index = x - roi_left;
        for (int y = scan_top; y <= scan_bottom; y += DETECTION_RING_SCAN_STEP)
        {
            const ring_measurement_t measurement = calculate_ring_measurement(frame, x, y);
            if (measurement.score > best_raw_measurement.score ||
                (measurement.score == best_raw_measurement.score &&
                 measurement.dark_sample_count > best_raw_measurement.dark_sample_count))
                best_raw_measurement = measurement;

            if (!ring_measurement_matches(&measurement))
                continue;

            const uint8_t score = measurement.score;
            if (score <= column_scores[column_index])
                continue;

            column_scores[column_index] = score;
            column_y_offsets[column_index] = (uint8_t)(y - roi_top);
        }
    }

    yahboom_detection_candidate_t strongest = {0};
    yahboom_detection_candidate_t nearest = {0};
    uint8_t candidate_count = 0;
    int candidate_start = -1;
    int last_active = -1;
    uint32_t candidate_weight = 0;
    uint32_t candidate_weighted_x = 0;
    uint8_t candidate_peak_score = 0;
    int candidate_peak_y = centerline_y;

    for (int column_index = 0; column_index <= roi_width; column_index++)
    {
        const bool is_active = column_index < roi_width &&
                               column_scores[column_index] >= DETECTION_RING_MIN_SCORE;
        if (is_active)
        {
            if (candidate_start < 0)
            {
                candidate_start = column_index;
                candidate_weight = 0;
                candidate_weighted_x = 0;
                candidate_peak_score = 0;
            }

            const uint8_t score = column_scores[column_index];
            const int x = roi_left + column_index;
            candidate_weight += score;
            candidate_weighted_x += (uint32_t)x * score;
            if (score > candidate_peak_score)
            {
                candidate_peak_score = score;
                candidate_peak_y = roi_top + column_y_offsets[column_index];
            }
            last_active = column_index;
            continue;
        }

        if (candidate_start < 0 || column_index - last_active <= DETECTION_RING_MAX_COLUMN_GAP)
            continue;

        const int candidate_columns = last_active - candidate_start + 1;
        if (candidate_columns >= DETECTION_RING_MIN_ACTIVE_COLUMNS && candidate_weight != 0)
        {
            yahboom_detection_candidate_t candidate = {
                .valid = true,
                .center_x = (int)(candidate_weighted_x / candidate_weight),
                .center_y = candidate_peak_y,
                .score = candidate_peak_score,
            };
            candidate.distance_x = candidate.center_x - context->tracking.center_x;
            if (candidate.distance_x < 0)
                candidate.distance_x = -candidate.distance_x;
            candidate_count++;

            if (!strongest.valid || candidate.score > strongest.score)
                strongest = candidate;
            if (candidate.distance_x <= DETECTION_TRACK_MAX_JUMP_PIXELS &&
                (!nearest.valid || candidate.distance_x < nearest.distance_x ||
                 (candidate.distance_x == nearest.distance_x && candidate.score > nearest.score)))
                nearest = candidate;
        }

        candidate_start = -1;
        last_active = -1;
    }

    yahboom_detection_tracking_t *tracking = &context->tracking;
    yahboom_detection_candidate_t selected = {0};
    bool target_found = false;
    if (tracking->position_valid && nearest.valid)
    {
        selected = nearest;
        target_found = true;
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
            tracking->reacquire_valid = false;
            tracking->reacquire_count = 0;
            ESP_LOGI(TAG, "RING_REACQUIRED x=%d y=%d", selected.center_x, selected.center_y);
        }
    }
    else
    {
        tracking->reacquire_valid = false;
        tracking->reacquire_count = 0;
        const TickType_t now = xTaskGetTickCount();
        if (candidate_count > 1)
        {
            if (context->last_detection_log_tick == 0 ||
                now - context->last_detection_log_tick >= pdMS_TO_TICKS(DETECTION_LOG_INTERVAL_MS))
            {
                context->last_detection_log_tick = now;
                ESP_LOGI(TAG, "RING_MULTIPLE_FAR candidates=%u previous_x=%d",
                         (unsigned)candidate_count, tracking->center_x);
            }
        }
        else if (candidate_count == 0 &&
                 (context->last_detection_log_tick == 0 ||
                  now - context->last_detection_log_tick >= pdMS_TO_TICKS(DETECTION_LOG_INTERVAL_MS)))
        {
            context->last_detection_log_tick = now;
            ESP_LOGI(TAG, "RING_NO_MATCH center=%u contrast=%u dark=%u",
                     (unsigned)best_raw_measurement.center_luma,
                     (unsigned)best_raw_measurement.score,
                     (unsigned)best_raw_measurement.dark_sample_count);
        }
    }

    if (target_found)
    {
        tracking->position_valid = true;
        tracking->center_x = selected.center_x;
        tracking->center_y = selected.center_y;
        tracking->width = DETECTION_RING_RADIUS * 2;
        tracking->missing_count = 0;

        const TickType_t now = xTaskGetTickCount();
        if (context->last_detection_log_tick == 0 ||
            now - context->last_detection_log_tick >= pdMS_TO_TICKS(DETECTION_LOG_INTERVAL_MS))
        {
            context->last_detection_log_tick = now;
            ESP_LOGI(TAG, "RING x=%d y=%d score=%u candidates=%u",
                     selected.center_x, selected.center_y, (unsigned)selected.score,
                     (unsigned)candidate_count);
        }
        draw_detection_marker(frame, tracking->center_x, tracking->center_y);
    }
    else
    {
        update_tracking_after_miss(tracking);
        if (tracking->position_valid)
            draw_detection_marker(frame, tracking->center_x, tracking->center_y);
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

    detect_bright_ring(context, frame);
}
