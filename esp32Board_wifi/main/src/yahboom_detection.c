#include "yahboom_detection.h"

#include <string.h>

#include "esp_log.h"
#include "freertos/task.h"
#include "yahboom_msp_uart.h"
#include "yahboom_overlay.h"

static const char *TAG = "yahboom_camera";
static const yahboom_yuv422_color_t kDetectionBoxColor = {76, 85, 255};
static const yahboom_yuv422_color_t kDetectionRoiColor = {150, 255, 100};

// 这两个值由网页端动态调整，识别任务只读取，不保存到 Flash。
volatile int DETECTION_ROI_Y = DETECTION_ROI_Y_DEFAULT;
volatile int DETECTION_ROI_HEIGHT = DETECTION_ROI_HEIGHT_DEFAULT;

bool yahboom_detection_set_roi_y(int value)
{
    if (value < 0 || value + DETECTION_ROI_HEIGHT > 240)
        return false;

    DETECTION_ROI_Y = value;
    return true;
}

bool yahboom_detection_set_roi_height(int value)
{
    if (value < DETECTION_RING_RADIUS * 2 + 1 ||
        DETECTION_ROI_Y + value > 240)
        return false;

    DETECTION_ROI_HEIGHT = value;
    return true;
}

typedef struct
{
    bool valid;
    int center_x;
    int center_y;
    int center_brightness;
    int ring_brightness;
    int score;
    int dark_sample_count;
} ring_match_t;

static inline void draw_yuv422_pixel(camera_fb_t *frame, int x, int y,
                                     const yahboom_yuv422_color_t *color)
{
    if (x < 0 || x >= frame->width || y < 0 || y >= frame->height)
        return;

    const size_t pixel_offset = (y * frame->width + x) * 2;
    const size_t pair_offset = (y * frame->width + (x & ~1)) * 2;
    frame->buf[pixel_offset] = color->y;
    frame->buf[pair_offset + 1] = color->cb;
    frame->buf[pair_offset + 3] = color->cr;
}

static inline uint8_t get_yuv422_brightness(const camera_fb_t *frame, int x, int y)
{
    return frame->buf[(y * frame->width + x) * 2];
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
    const int roi_y = DETECTION_ROI_Y;
    const int roi_height = DETECTION_ROI_HEIGHT;
    *top = roi_y < minimum_y ? minimum_y : roi_y;
    *right = DETECTION_ROI_X + DETECTION_ROI_WIDTH;
    *bottom = roi_y + roi_height;

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

    for (int layer = 0; layer < DETECTION_ROI_BORDER_THICKNESS_PIXELS; layer++)
    {
        const int layer_left = left + layer;
        const int layer_right = right - 1 - layer;
        const int layer_top = top + layer;
        const int layer_bottom = bottom - 1 - layer;

        if (layer_left > layer_right || layer_top > layer_bottom)
            break;

        for (int x = layer_left; x <= layer_right; x++)
        {
            draw_yuv422_pixel(frame, x, layer_top, &kDetectionRoiColor);
            draw_yuv422_pixel(frame, x, layer_bottom, &kDetectionRoiColor);
        }
        for (int y = layer_top; y <= layer_bottom; y++)
        {
            draw_yuv422_pixel(frame, layer_left, y, &kDetectionRoiColor);
            draw_yuv422_pixel(frame, layer_right, y, &kDetectionRoiColor);
        }
    }
}

static void draw_detection_marker(camera_fb_t *frame, int center_x, int center_y)
{
    const int left = center_x - DETECTION_MARKER_HALF_SIZE_PIXELS;
    const int right = center_x + DETECTION_MARKER_HALF_SIZE_PIXELS;
    const int top = center_y - DETECTION_MARKER_HALF_SIZE_PIXELS;
    const int bottom = center_y + DETECTION_MARKER_HALF_SIZE_PIXELS;

    for (int layer = 0; layer < DETECTION_MARKER_THICKNESS_PIXELS; layer++)
    {
        const int layer_left = left + layer;
        const int layer_right = right - layer;
        const int layer_top = top + layer;
        const int layer_bottom = bottom - layer;

        for (int x = layer_left; x <= layer_right; x++)
        {
            draw_yuv422_pixel(frame, x, layer_top, &kDetectionBoxColor);
            draw_yuv422_pixel(frame, x, layer_bottom, &kDetectionBoxColor);
        }
        for (int y = layer_top; y <= layer_bottom; y++)
        {
            draw_yuv422_pixel(frame, layer_left, y, &kDetectionBoxColor);
            draw_yuv422_pixel(frame, layer_right, y, &kDetectionBoxColor);
        }
    }
}

static bool tracking_has_output(const yahboom_detection_tracking_t *tracking)
{
    return tracking->position_valid &&
           tracking->missing_count <= DETECTION_HOLD_MISSED_FRAMES;
}

static void draw_detection_overlays(camera_fb_t *frame,
                                    const yahboom_detection_context_t *context)
{
    draw_detection_roi(frame);
    if (tracking_has_output(&context->tracking))
    {
        draw_detection_marker(frame, context->tracking.center_x,
                              context->tracking.center_y);
    }
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
    const bool valid = !context->invalid_roi_reported && tracking_has_output(tracking);
    yahboom_msp_uart_send(valid, valid ? (uint16_t)tracking->center_x : 0,
                          valid ? tracking->width : 0);
}

static int calculate_center_brightness(const camera_fb_t *frame, int x, int y)
{
    return (get_yuv422_brightness(frame, x, y) +
            get_yuv422_brightness(frame, x - DETECTION_RING_CENTER_RADIUS, y) +
            get_yuv422_brightness(frame, x + DETECTION_RING_CENTER_RADIUS, y) +
            get_yuv422_brightness(frame, x, y - DETECTION_RING_CENTER_RADIUS) +
            get_yuv422_brightness(frame, x, y + DETECTION_RING_CENTER_RADIUS)) /
           5;
}

static ring_match_t measure_ring(const camera_fb_t *frame, int x, int y)
{
    const int ring_radius = DETECTION_RING_RADIUS;
    const int diagonal_offset = DETECTION_RING_DIAGONAL_OFFSET;
    const int ring_values[8] = {
        get_yuv422_brightness(frame, x - ring_radius, y),
        get_yuv422_brightness(frame, x + ring_radius, y),
        get_yuv422_brightness(frame, x, y - ring_radius),
        get_yuv422_brightness(frame, x, y + ring_radius),
        get_yuv422_brightness(frame, x - diagonal_offset, y - diagonal_offset),
        get_yuv422_brightness(frame, x + diagonal_offset, y - diagonal_offset),
        get_yuv422_brightness(frame, x - diagonal_offset, y + diagonal_offset),
        get_yuv422_brightness(frame, x + diagonal_offset, y + diagonal_offset),
    };

    ring_match_t match = {
        .center_x = x,
        .center_y = y,
        .center_brightness = calculate_center_brightness(frame, x, y),
    };
    for (int index = 0; index < 8; index++)
        match.ring_brightness += ring_values[index];
    match.ring_brightness /= 8;
    match.score = match.center_brightness - match.ring_brightness;

    for (int index = 0; index < 8; index++)
    {
        if (match.center_brightness - ring_values[index] >=
            DETECTION_RING_MIN_DARK_SAMPLE_CONTRAST)
        {
            match.dark_sample_count++;
        }
    }

    match.valid = match.center_brightness >= DETECTION_RING_MIN_CENTER_BRIGHTNESS &&
                  match.score >= DETECTION_RING_MIN_SCORE &&
                  match.dark_sample_count >= DETECTION_RING_MIN_DARK_SAMPLES;
    return match;
}

static void log_no_match(yahboom_detection_context_t *context,
                         const ring_match_t *best_measurement)
{
    const TickType_t now = xTaskGetTickCount();
    if (context->last_detection_log_tick != 0 &&
        now - context->last_detection_log_tick < pdMS_TO_TICKS(DETECTION_LOG_INTERVAL_MS))
    {
        return;
    }

    context->last_detection_log_tick = now;
    ESP_LOGI(TAG, "RING_NO_MATCH center=%d contrast=%d dark=%d",
             best_measurement->center_brightness, best_measurement->score,
             best_measurement->dark_sample_count);
}

static void log_match(yahboom_detection_context_t *context, const ring_match_t *match)
{
    const TickType_t now = xTaskGetTickCount();
    if (context->last_detection_log_tick != 0 &&
        now - context->last_detection_log_tick < pdMS_TO_TICKS(DETECTION_LOG_INTERVAL_MS))
    {
        return;
    }

    context->last_detection_log_tick = now;
    ESP_LOGI(TAG, "RING x=%d y=%d width=%d center=%d contrast=%d dark=%d",
             match->center_x, match->center_y, DETECTION_RING_RADIUS * 2,
             match->center_brightness, match->score, match->dark_sample_count);
}

static ring_match_t find_best_ring(const yahboom_detection_context_t *context,
                                   const camera_fb_t *frame, int roi_left, int roi_top,
                                   int roi_right, int roi_bottom)
{
    const int ring_radius = DETECTION_RING_RADIUS;
    const int centerline_y = (roi_top + roi_bottom) / 2;
    int scan_left = roi_left + ring_radius;
    int scan_right = roi_right - ring_radius;

    if (context->tracking.position_valid)
    {
        const int tracked_left = context->tracking.center_x -
                                 DETECTION_RING_TRACK_SEARCH_HALF_WIDTH;
        const int tracked_right = context->tracking.center_x +
                                  DETECTION_RING_TRACK_SEARCH_HALF_WIDTH + 1;
        if (scan_left < tracked_left)
            scan_left = tracked_left;
        if (scan_right > tracked_right)
            scan_right = tracked_right;
    }

    ring_match_t best_match = {0};
    ring_match_t best_measurement = {0};
    bool has_measurement = false;
    const int scan_top = centerline_y - DETECTION_RING_CENTERLINE_HALF_HEIGHT;
    const int scan_bottom = centerline_y + DETECTION_RING_CENTERLINE_HALF_HEIGHT;
    for (int y = scan_top; y <= scan_bottom; y += DETECTION_RING_SCAN_STEP)
    {
        if (y - ring_radius < roi_top || y + ring_radius >= roi_bottom)
            continue;

        for (int x = scan_left; x < scan_right; x += DETECTION_RING_SCAN_STEP)
        {
            ring_match_t measurement = measure_ring(frame, x, y);
            if (!has_measurement || measurement.score > best_measurement.score)
            {
                best_measurement = measurement;
                has_measurement = true;
            }
            if (measurement.valid &&
                (!best_match.valid || measurement.score > best_match.score))
            {
                best_match = measurement;
            }
        }
    }

    if (!best_match.valid && has_measurement)
        best_match = best_measurement;
    return best_match;
}

static void clear_tracking(yahboom_detection_tracking_t *tracking)
{
    tracking->position_valid = false;
    tracking->missing_count = 0;
    tracking->width = 0;
    tracking->reacquire_valid = false;
    tracking->reacquire_count = 0;
}

static bool update_tracking(yahboom_detection_context_t *context,
                            const ring_match_t *best_match)
{
    yahboom_detection_tracking_t *tracking = &context->tracking;
    if (best_match->valid && tracking->position_valid)
    {
        int jump_x = best_match->center_x - tracking->center_x;
        if (jump_x < 0)
            jump_x = -jump_x;

        if (jump_x <= DETECTION_TRACK_MAX_JUMP_PIXELS)
        {
            tracking->center_x = best_match->center_x;
            tracking->center_y = best_match->center_y;
            tracking->width = DETECTION_RING_RADIUS * 2;
            tracking->missing_count = 0;
            tracking->reacquire_valid = false;
            tracking->reacquire_count = 0;
            return true;
        }
    }

    if (best_match->valid && !tracking->position_valid)
    {
        int drift_x = best_match->center_x - tracking->reacquire_center_x;
        if (drift_x < 0)
            drift_x = -drift_x;
        int drift_y = best_match->center_y - tracking->reacquire_center_y;
        if (drift_y < 0)
            drift_y = -drift_y;

        if (!tracking->reacquire_valid ||
            drift_x > DETECTION_REACQUIRE_MAX_DRIFT_PIXELS ||
            drift_y > DETECTION_RING_CENTERLINE_HALF_HEIGHT)
        {
            tracking->reacquire_valid = true;
            tracking->reacquire_center_x = best_match->center_x;
            tracking->reacquire_center_y = best_match->center_y;
            tracking->reacquire_count = 1;
        }
        else
        {
            tracking->reacquire_center_x = best_match->center_x;
            tracking->reacquire_center_y = best_match->center_y;
            tracking->reacquire_count++;
        }

        if (tracking->reacquire_count >= DETECTION_REACQUIRE_CONFIRM_COUNT)
        {
            tracking->position_valid = true;
            tracking->center_x = best_match->center_x;
            tracking->center_y = best_match->center_y;
            tracking->width = DETECTION_RING_RADIUS * 2;
            tracking->missing_count = 0;
            tracking->reacquire_valid = false;
            tracking->reacquire_count = 0;
            ESP_LOGI(TAG, "RING_REACQUIRED x=%d y=%d", best_match->center_x,
                     best_match->center_y);
            return true;
        }
    }
    else
    {
        tracking->reacquire_valid = false;
        tracking->reacquire_count = 0;
    }

    if (++tracking->missing_count >= DETECTION_LOST_COUNT)
        clear_tracking(tracking);
    return false;
}

static void detect_bright_center_dark_ring(yahboom_detection_context_t *context,
                                           camera_fb_t *frame)
{
    const size_t expected_length = (size_t)frame->width * frame->height * 2;
    int roi_left;
    int roi_top;
    int roi_right;
    int roi_bottom;
    if (frame->format != PIXFORMAT_YUV422 || frame->len < expected_length ||
        !get_detection_roi(frame, &roi_left, &roi_top, &roi_right, &roi_bottom) ||
        roi_right - roi_left <= DETECTION_RING_RADIUS * 2 ||
        roi_bottom - roi_top <= DETECTION_RING_RADIUS * 2)
    {
        if (!context->invalid_roi_reported)
        {
            ESP_LOGE(TAG, "RING: invalid YUV422 frame or detection ROI");
            context->invalid_roi_reported = true;
        }
        clear_tracking(&context->tracking);
        update_detection_state(context, false);
        draw_detection_overlays(frame, context);
        send_msp_tracking_status(context);
        return;
    }
    context->invalid_roi_reported = false;

    if (++context->detect_frame_count >= DETECTION_EVERY_N_FRAMES)
    {
        context->detect_frame_count = 0;
        const ring_match_t best_match = find_best_ring(context, frame, roi_left, roi_top,
                                                       roi_right, roi_bottom);
        const bool target_found = update_tracking(context, &best_match);
        if (target_found)
            log_match(context, &best_match);
        else if (!best_match.valid)
            log_no_match(context, &best_match);
        update_detection_state(context, target_found);
    }

    draw_detection_overlays(frame, context);
    send_msp_tracking_status(context);
}

void yahboom_detection_init(yahboom_detection_context_t *context)
{
    if (context != NULL)
        memset(context, 0, sizeof(*context));
}

void yahboom_detection_process(yahboom_detection_context_t *context, camera_fb_t *frame)
{
    if (context == NULL || frame == NULL)
        return;

    detect_bright_center_dark_ring(context, frame);
}
