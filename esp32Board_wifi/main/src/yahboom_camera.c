#include "yahboom_camera.h"
#include "yahboom_overlay.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include <string.h>

static const char *TAG = "yahboom_camera";
static QueueHandle_t xQueueFrameO = NULL;

typedef struct
{
    uint8_t y;
    uint8_t cb;
    uint8_t cr;
} yuv422_color_t;

static const yuv422_color_t kDetectionBoxColor = {235, 128, 128};
static const yuv422_color_t kDetectionCrossColor = {76, 85, 255};
static const yuv422_color_t kDetectionRoiColor = {150, 255, 100};
static const int kDetectionCrossRadius = 10;
enum { LIGHT_DIFFERENCE_HISTOGRAM_SIZE = 511 };

static uint8_t *background_y = NULL;
static size_t background_pixel_count = 0;
static uint16_t background_width = 0;
static uint16_t background_height = 0;
static TickType_t background_delay_start_tick = 0;
static bool background_delay_started = false;
static uint8_t background_warmup_count = 0;
static uint8_t background_capture_count = 0;
static bool background_ready = false;
static uint8_t *difference_mask = NULL;
static uint16_t *difference_queue = NULL;
static size_t difference_mask_capacity = 0;
static uint16_t light_difference_histogram[LIGHT_DIFFERENCE_HISTOGRAM_SIZE];

static void draw_yuv422_pixel(camera_fb_t *frame, int x, int y, const yuv422_color_t *color)
{
    if (x < 0 || x >= frame->width || y < 0 || y >= frame->height)
        return;

    // YUV422 的相邻两个像素共用一组 Cb、Cr。
    size_t pixel_offset = (y * frame->width + x) * 2;
    size_t pair_offset = (y * frame->width + (x & ~1)) * 2;
    frame->buf[pixel_offset] = color->y;
    frame->buf[pair_offset + 1] = color->cb;
    frame->buf[pair_offset + 3] = color->cr;
}

static void draw_detection_marker(camera_fb_t *frame, int min_x, int min_y, int max_x, int max_y,
                                  int center_x, int center_y)
{
    for (int x = min_x; x <= max_x; x++)
    {
        draw_yuv422_pixel(frame, x, min_y, &kDetectionBoxColor);
        draw_yuv422_pixel(frame, x, max_y, &kDetectionBoxColor);
    }

    for (int y = min_y; y <= max_y; y++)
    {
        draw_yuv422_pixel(frame, min_x, y, &kDetectionBoxColor);
        draw_yuv422_pixel(frame, max_x, y, &kDetectionBoxColor);
    }

    for (int offset = -kDetectionCrossRadius; offset <= kDetectionCrossRadius; offset++)
    {
        draw_yuv422_pixel(frame, center_x + offset, center_y, &kDetectionCrossColor);
        draw_yuv422_pixel(frame, center_x, center_y + offset, &kDetectionCrossColor);
    }
}

static bool get_detection_roi(const camera_fb_t *frame, int *left, int *top, int *right, int *bottom)
{
    const int minimum_x = DIFFERENCE_CIRCLE_EDGE_MARGIN;
    const int minimum_y = DIFFERENCE_CIRCLE_EDGE_MARGIN;
    const int maximum_x = frame->width - DIFFERENCE_CIRCLE_EDGE_MARGIN;
    const int maximum_y = frame->height - DIFFERENCE_CIRCLE_EDGE_MARGIN;

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

static bool capture_background(const camera_fb_t *frame)
{
    size_t pixel_count = frame->width * frame->height;

    if (frame->format != PIXFORMAT_YUV422 || frame->len < pixel_count * 2)
        return false;

    if (background_ready)
        return true;

    if (!background_delay_started)
    {
        background_delay_started = true;
        background_delay_start_tick = xTaskGetTickCount();
        yahboom_overlay_show_wait(BACKGROUND_START_DELAY_MS);
        ESP_LOGI(TAG, "BACKGROUND: preview active, waiting %d ms before capture", BACKGROUND_START_DELAY_MS);
        return false;
    }

    if (xTaskGetTickCount() - background_delay_start_tick < pdMS_TO_TICKS(BACKGROUND_START_DELAY_MS))
        return false;

    if (background_warmup_count < BACKGROUND_WARMUP_FRAMES)
    {
        if (background_warmup_count == 0)
            ESP_LOGI(TAG, "BACKGROUND: delay complete, camera stabilizing before capture");
        background_warmup_count++;
        return false;
    }

    if (background_y == NULL)
    {
        background_y = (uint8_t *)heap_caps_malloc(pixel_count, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (background_y == NULL)
        {
            ESP_LOGE(TAG, "BACKGROUND: PSRAM allocation failed");
            return false;
        }
        background_pixel_count = pixel_count;
        background_width = frame->width;
        background_height = frame->height;
        ESP_LOGI(TAG, "BACKGROUND: capturing %d empty-scene frames", BACKGROUND_CAPTURE_FRAMES);
    }

    if (background_width != frame->width || background_height != frame->height ||
        background_pixel_count != pixel_count)
        return false;

    if (background_capture_count < BACKGROUND_CAPTURE_FRAMES)
    {
        for (size_t index = 0; index < pixel_count; index++)
        {
            uint8_t brightness = frame->buf[index * 2];
            if (background_capture_count == 0)
                background_y[index] = brightness;
            else
                background_y[index] = ((uint16_t)background_y[index] * background_capture_count + brightness) /
                                      (background_capture_count + 1);
        }

        background_capture_count++;
        if (background_capture_count == BACKGROUND_CAPTURE_FRAMES)
        {
            background_ready = true;
            yahboom_overlay_show_start(STATUS_BANNER_DURATION_MS);
            ESP_LOGI(TAG, "BACKGROUND_READY: start recognition");
        }
    }

    return background_ready;
}

static void update_circle_detection_state(bool circle_found)
{
    static bool circle_was_found = false;
    static uint8_t missing_count = 0;

    if (circle_found)
    {
        if (!circle_was_found)
            ESP_LOGI(TAG, "DIFF_CIRCLE_FOUND");
        circle_was_found = true;
        missing_count = 0;
        return;
    }

    if (!circle_was_found)
        return;

    if (++missing_count >= DIFFERENCE_CIRCLE_LOST_COUNT)
    {
        circle_was_found = false;
        missing_count = 0;
        ESP_LOGI(TAG, "DIFF_CIRCLE_LOST");
    }
}

static int calculate_roi_light_offset(const camera_fb_t *frame, int left, int top, int right, int bottom)
{
    uint32_t sample_count = 0;
    uint32_t cumulative_count = 0;

    memset(light_difference_histogram, 0, sizeof(light_difference_histogram));

    for (int y = top; y < bottom; y += DIFFERENCE_CIRCLE_SCAN_STEP)
    {
        for (int x = left; x < right; x += DIFFERENCE_CIRCLE_SCAN_STEP)
        {
            size_t pixel_index = y * frame->width + x;
            int difference = (int)frame->buf[pixel_index * 2] - background_y[pixel_index];
            light_difference_histogram[difference + 255]++;
            sample_count++;
        }
    }

    if (sample_count == 0)
        return 0;

    const uint32_t median_index = sample_count / 2;
    for (int index = 0; index < LIGHT_DIFFERENCE_HISTOGRAM_SIZE; index++)
    {
        cumulative_count += light_difference_histogram[index];
        if (cumulative_count > median_index)
            return index - 255;
    }

    return 0;
}

static void detect_difference_circle(camera_fb_t *frame)
{
    static uint8_t detect_frame_count = 0;
    static bool invalid_roi_reported = false;
    static TickType_t last_detection_log_tick = 0;
    static bool tracked_position_valid = false;
    static int tracked_center_x = 0;
    static int tracked_center_y = 0;
    static uint8_t tracking_missing_count = 0;
    int roi_left;
    int roi_top;
    int roi_right;
    int roi_bottom;
    if (!get_detection_roi(frame, &roi_left, &roi_top, &roi_right, &roi_bottom))
    {
        if (!invalid_roi_reported)
        {
            ESP_LOGE(TAG, "DIFF_CIRCLE: invalid detection ROI");
            invalid_roi_reported = true;
        }
        update_circle_detection_state(false);
        yahboom_overlay_draw_status(frame);
        return;
    }
    invalid_roi_reported = false;

    const int sample_width = (roi_right - roi_left +
                              DIFFERENCE_CIRCLE_SCAN_STEP - 1) /
                             DIFFERENCE_CIRCLE_SCAN_STEP;
    const int sample_height = (roi_bottom - roi_top +
                               DIFFERENCE_CIRCLE_SCAN_STEP - 1) /
                              DIFFERENCE_CIRCLE_SCAN_STEP;
    const size_t sample_count = (size_t)sample_width * sample_height;

    if (!capture_background(frame))
    {
        draw_detection_roi(frame);
        yahboom_overlay_draw_status(frame);
        return;
    }

    // 每两帧检测一次，避免本次可行性测试影响当前图传帧率。
    if (++detect_frame_count < 2)
    {
        draw_detection_roi(frame);
        yahboom_overlay_draw_status(frame);
        return;
    }
    detect_frame_count = 0;

    // 钢珠只占 ROI 的小部分，亮度差的中值可近似表示整帧光照偏移。
    const int light_offset = calculate_roi_light_offset(frame, roi_left, roi_top, roi_right, roi_bottom);

    if (difference_mask_capacity < sample_count)
    {
        uint8_t *new_mask = (uint8_t *)heap_caps_malloc(sample_count, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        uint16_t *new_queue = (uint16_t *)heap_caps_malloc(sample_count * sizeof(uint16_t),
                                                            MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (new_mask == NULL || new_queue == NULL)
        {
            if (new_mask != NULL)
                heap_caps_free(new_mask);
            if (new_queue != NULL)
                heap_caps_free(new_queue);
            ESP_LOGE(TAG, "DIFF_CIRCLE: component buffer allocation failed");
            return;
        }
        if (difference_mask != NULL)
            heap_caps_free(difference_mask);
        if (difference_queue != NULL)
            heap_caps_free(difference_queue);
        difference_mask = new_mask;
        difference_queue = new_queue;
        difference_mask_capacity = sample_count;
    }

    size_t foreground_sample_count = 0;
    for (int sample_y = 0; sample_y < sample_height; sample_y++)
    {
        int y = roi_top + sample_y * DIFFERENCE_CIRCLE_SCAN_STEP;
        for (int sample_x = 0; sample_x < sample_width; sample_x++)
        {
            int x = roi_left + sample_x * DIFFERENCE_CIRCLE_SCAN_STEP;
            size_t pixel_index = y * frame->width + x;
            int signed_difference = (int)frame->buf[pixel_index * 2] - background_y[pixel_index] - light_offset;
            int difference = signed_difference < 0 ? -signed_difference : signed_difference;
            bool is_foreground = difference > DIFFERENCE_CIRCLE_BRIGHT_THRESHOLD ||
                                 signed_difference < -DIFFERENCE_CIRCLE_DARK_THRESHOLD;
            difference_mask[sample_y * sample_width + sample_x] = is_foreground ? 1 : 0;
            if (is_foreground)
                foreground_sample_count++;
        }
    }

    // 钢珠只应覆盖 ROI 的小部分；大面积变化通常是光照或遮挡扰动。
    if (foreground_sample_count * 100 > sample_count * DETECTION_MAX_FOREGROUND_PERCENT)
    {
        TickType_t now = xTaskGetTickCount();
        if (last_detection_log_tick == 0 ||
            now - last_detection_log_tick >= pdMS_TO_TICKS(DETECTION_LOG_INTERVAL_MS))
        {
            last_detection_log_tick = now;
            ESP_LOGI(TAG, "DIFF_LIGHT_DISTURBANCE changed=%u total=%u offset=%d",
                     (unsigned)foreground_sample_count, (unsigned)sample_count, light_offset);
        }

        if (++tracking_missing_count >= DIFFERENCE_CIRCLE_LOST_COUNT)
        {
            tracked_position_valid = false;
            tracking_missing_count = 0;
        }
        update_circle_detection_state(false);
        draw_detection_roi(frame);
        yahboom_overlay_draw_status(frame);
        return;
    }

    bool candidate_found = false;
    uint32_t best_area = 0;
    int best_min_x = 0;
    int best_min_y = 0;
    int best_max_x = 0;
    int best_max_y = 0;
    int best_center_x = 0;
    int best_center_y = 0;

    for (int start_y = 0; start_y < sample_height; start_y++)
    {
        for (int start_x = 0; start_x < sample_width; start_x++)
        {
            size_t start_index = start_y * sample_width + start_x;
            if (difference_mask[start_index] == 0)
                continue;

            uint16_t queue_size = 0;
            uint32_t component_count = 0;
            uint32_t sum_x = 0;
            uint32_t sum_y = 0;
            int min_x = frame->width;
            int min_y = frame->height;
            int max_x = -1;
            int max_y = -1;
            difference_queue[queue_size++] = (uint16_t)start_index;
            difference_mask[start_index] = 2;

            while (queue_size > 0)
            {
                uint16_t current_index = difference_queue[--queue_size];
                int current_x = current_index % sample_width;
                int current_y = current_index / sample_width;
                int pixel_x = roi_left + current_x * DIFFERENCE_CIRCLE_SCAN_STEP;
                int pixel_y = roi_top + current_y * DIFFERENCE_CIRCLE_SCAN_STEP;
                component_count++;
                sum_x += pixel_x;
                sum_y += pixel_y;
                if (pixel_x < min_x) min_x = pixel_x;
                if (pixel_x > max_x) max_x = pixel_x;
                if (pixel_y < min_y) min_y = pixel_y;
                if (pixel_y > max_y) max_y = pixel_y;

                const int neighbor_x[8] = {current_x - 1, current_x + 1, current_x, current_x,
                                           current_x - 1, current_x + 1, current_x - 1, current_x + 1};
                const int neighbor_y[8] = {current_y, current_y, current_y - 1, current_y + 1,
                                           current_y - 1, current_y - 1, current_y + 1, current_y + 1};
                for (int neighbor = 0; neighbor < 8; neighbor++)
                {
                    if (neighbor_x[neighbor] < 0 || neighbor_x[neighbor] >= sample_width ||
                        neighbor_y[neighbor] < 0 || neighbor_y[neighbor] >= sample_height)
                        continue;
                    size_t neighbor_index = neighbor_y[neighbor] * sample_width + neighbor_x[neighbor];
                    if (difference_mask[neighbor_index] == 1 && queue_size < sample_count)
                    {
                        difference_mask[neighbor_index] = 2;
                        difference_queue[queue_size++] = (uint16_t)neighbor_index;
                    }
                }
            }

            uint32_t estimated_area = component_count * DIFFERENCE_CIRCLE_SCAN_STEP * DIFFERENCE_CIRCLE_SCAN_STEP;
            if (estimated_area < DIFFERENCE_CIRCLE_MIN_AREA)
                continue;

            int width = max_x - min_x + DIFFERENCE_CIRCLE_SCAN_STEP;
            int height = max_y - min_y + DIFFERENCE_CIRCLE_SCAN_STEP;
            uint32_t box_area = width * height;
            uint32_t fill_percent = estimated_area * 100 / box_area;
            if (width * 100 < height * 55 || width * 100 > height * 145 ||
                fill_percent < DIFFERENCE_CIRCLE_MIN_FILL_PERCENT ||
                estimated_area <= best_area)
                continue;

            best_area = estimated_area;
            best_min_x = min_x;
            best_min_y = min_y;
            best_max_x = max_x;
            best_max_y = max_y;
            best_center_x = sum_x / component_count;
            best_center_y = sum_y / component_count;
            candidate_found = true;
        }
    }

    bool circle_found = false;
    if (candidate_found)
    {
        int delta_x = best_center_x - tracked_center_x;
        int delta_y = best_center_y - tracked_center_y;
        int distance_squared = delta_x * delta_x + delta_y * delta_y;
        int maximum_jump_squared = DETECTION_TRACK_MAX_JUMP_PIXELS * DETECTION_TRACK_MAX_JUMP_PIXELS;

        if (!tracked_position_valid || distance_squared <= maximum_jump_squared)
        {
            circle_found = true;
            tracked_position_valid = true;
            tracked_center_x = best_center_x;
            tracked_center_y = best_center_y;
            tracking_missing_count = 0;
        }
        else
        {
            TickType_t now = xTaskGetTickCount();
            if (last_detection_log_tick == 0 ||
                now - last_detection_log_tick >= pdMS_TO_TICKS(DETECTION_LOG_INTERVAL_MS))
            {
                last_detection_log_tick = now;
                ESP_LOGI(TAG, "DIFF_CIRCLE_JUMP_REJECT x=%d y=%d previous_x=%d previous_y=%d",
                         best_center_x, best_center_y, tracked_center_x, tracked_center_y);
            }
        }
    }

    if (!circle_found && ++tracking_missing_count >= DIFFERENCE_CIRCLE_LOST_COUNT)
    {
        tracked_position_valid = false;
        tracking_missing_count = 0;
    }

    if (circle_found)
    {
        int width = best_max_x - best_min_x + DIFFERENCE_CIRCLE_SCAN_STEP;
        int height = best_max_y - best_min_y + DIFFERENCE_CIRCLE_SCAN_STEP;
        uint32_t box_area = width * height;
        uint32_t fill_percent = best_area * 100 / box_area;
        draw_detection_marker(frame, best_min_x, best_min_y,
                              best_max_x + DIFFERENCE_CIRCLE_SCAN_STEP - 1,
                              best_max_y + DIFFERENCE_CIRCLE_SCAN_STEP - 1,
                              best_center_x, best_center_y);
        TickType_t now = xTaskGetTickCount();
        if (last_detection_log_tick == 0 ||
            now - last_detection_log_tick >= pdMS_TO_TICKS(DETECTION_LOG_INTERVAL_MS))
        {
            last_detection_log_tick = now;
            ESP_LOGI(TAG, "DIFF_CIRCLE x=%d y=%d w=%d h=%d area=%lu fill=%lu%% offset=%d",
                     best_center_x, best_center_y, width, height,
                     (unsigned long)best_area, (unsigned long)fill_percent, light_offset);
        }
    }

    update_circle_detection_state(circle_found);
    draw_detection_roi(frame);
    yahboom_overlay_draw_status(frame);
}

static void task_process_handler(void *arg)
{
    while (true)
    {
        camera_fb_t *frame = esp_camera_fb_get();
        if (!frame)
            continue;

        detect_difference_circle(frame);
        yahboom_overlay_draw_fps(frame);

        if (xQueueSend(xQueueFrameO, &frame, 0) == pdTRUE)
            continue;

        // 队列只保留最新帧，旧帧必须先归还摄像头驱动。
        camera_fb_t *stale_frame = NULL;
        if (xQueueReceive(xQueueFrameO, &stale_frame, 0) == pdTRUE)
            esp_camera_fb_return(stale_frame);

        if (xQueueSend(xQueueFrameO, &frame, 0) != pdTRUE)
            esp_camera_fb_return(frame);
    }
}

void my_register_camera(const pixformat_t pixel_fromat,
                     const framesize_t frame_size,
                     const uint8_t fb_count,
                     const QueueHandle_t frame_o)
{
    ESP_LOGI(TAG, "Camera module is %s", CAMERA_MODULE_NAME);

#if CONFIG_CAMERA_MODULE_ESP_EYE || CONFIG_CAMERA_MODULE_ESP32_CAM_BOARD
    /* IO13, IO14 is designed for JTAG by default,
     * to use it as generalized input,
     * firstly declair it as pullup input */
    gpio_config_t conf;
    conf.mode = GPIO_MODE_INPUT;
    conf.pull_up_en = GPIO_PULLUP_ENABLE;
    conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    conf.intr_type = GPIO_INTR_DISABLE;
    conf.pin_bit_mask = 1LL << 13;
    gpio_config(&conf);
    conf.pin_bit_mask = 1LL << 14;
    gpio_config(&conf);
#endif

    camera_config_t config;
    config.ledc_channel = LEDC_CHANNEL_0;
    config.ledc_timer = LEDC_TIMER_0;
    config.pin_d0 = CAMERA_PIN_D0;
    config.pin_d1 = CAMERA_PIN_D1;
    config.pin_d2 = CAMERA_PIN_D2;
    config.pin_d3 = CAMERA_PIN_D3;
    config.pin_d4 = CAMERA_PIN_D4;
    config.pin_d5 = CAMERA_PIN_D5;
    config.pin_d6 = CAMERA_PIN_D6;
    config.pin_d7 = CAMERA_PIN_D7;
    config.pin_xclk = CAMERA_PIN_XCLK;
    config.pin_pclk = CAMERA_PIN_PCLK;
    config.pin_vsync = CAMERA_PIN_VSYNC;
    config.pin_href = CAMERA_PIN_HREF;
    config.pin_sscb_sda = CAMERA_PIN_SIOD;
    config.pin_sscb_scl = CAMERA_PIN_SIOC;
    config.pin_pwdn = CAMERA_PIN_PWDN;
    config.pin_reset = CAMERA_PIN_RESET;
    config.xclk_freq_hz = XCLK_FREQ_HZ;
    config.pixel_format = pixel_fromat;
    config.frame_size = frame_size;
    config.jpeg_quality = 12;
    config.fb_count = fb_count;
    config.fb_location = CAMERA_FB_IN_PSRAM;
    config.grab_mode = CAMERA_GRAB_LATEST;

    // camera init
    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Camera init failed with error 0x%x", err);
        return;
    }

    sensor_t *s = esp_camera_sensor_get();
    if (s->id.PID == OV3660_PID || s->id.PID == OV2640_PID) {
        s->set_vflip(s, 1); //flip it back    
    } else if (s->id.PID == GC0308_PID) {
        s->set_hmirror(s, 0);
    } else if (s->id.PID == GC032A_PID) {
        s->set_vflip(s, 1);
    }

    //initial sensors are flipped vertically and colors are a bit saturated
    if (s->id.PID == OV3660_PID)
    {
        s->set_brightness(s, 1);  //up the blightness just a bit
        s->set_saturation(s, -2); //lower the saturation
    }

    if(s->id.PID == GC2145_PID)
    {
        //垂直反转 -way1
        // s->set_reg(s, 0xfe, 0xFF, 0);
        // s->set_reg(s, 0x17, 0x03, 2);

        // //水平镜像
        // s->set_reg(s, 0xfe, 0xFF, 0);
        // s->set_reg(s, 0x17, 0x03, 1);

        // //垂直反转+水平镜像
        // s->set_reg(s, 0xfe, 0xFF, 0);
        // s->set_reg(s, 0x17, 0x03, 3);


        //way2-该办法不好
        //s->set_vflip(s, 1); //垂直反转  
        //中间要延迟
        // s->set_hmirror(s, 1); //水平镜像

        int temp;
        //关闭白平衡
        s->set_reg(s, 0xfe, 0xFF, 0);
        //temp = s->get_reg(s,0x42,0xFF);
        s->set_reg(s, 0x42, 0xFF, 0xfd);


    }

    xQueueFrameO = frame_o;
    xTaskCreatePinnedToCore(task_process_handler, TAG, 3 * 1024, NULL, 5, NULL, 1);
}
