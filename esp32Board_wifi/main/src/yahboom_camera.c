#include "yahboom_camera.h"
#include "my_usart.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_heap_caps.h"

static const char *TAG = "yahboom_camera";
static QueueHandle_t xQueueFrameO = NULL;

// 桌面背景差分测试参数：上电后自动采集空场景。
#define BACKGROUND_START_DELAY_MS               10000       // 上电后等待空场景稳定的时间，单位：ms
#define BACKGROUND_WARMUP_FRAMES                20          // 等待结束后丢弃的相机稳定帧数
#define BACKGROUND_CAPTURE_FRAMES               8           // 用于平均生成空场景背景的帧数
#define DIFFERENCE_CIRCLE_SCAN_STEP             2           // 差分扫描步长，2 表示每隔 2 个像素取一个样本
#define DIFFERENCE_CIRCLE_EDGE_MARGIN           8           // 不参与识别的画面边缘宽度，单位：像素
#define DIFFERENCE_CIRCLE_BRIGHT_THRESHOLD      35          // 当前像素比背景更亮时的最小亮度差
#define DIFFERENCE_CIRCLE_DARK_THRESHOLD        15          // 当前像素比背景更暗时的最小亮度差，适应钢珠暗部   
#define DIFFERENCE_CIRCLE_MIN_AREA              40          // 有效圆形候选的最小估算面积，单位：像素
#define DIFFERENCE_CIRCLE_MIN_FILL_PERCENT      35          // 候选区域在外接框中的最小填充率，单位：%
#define DIFFERENCE_CIRCLE_MAX_FILL_PERCENT      100         // 候选区域在外接框中的最大填充率，单位：%
#define DIFFERENCE_CIRCLE_LOST_COUNT            3           // 连续丢失多少个检测周期后判定目标离开
#define DETECTION_BOX_Y                         235         // 识别外接框颜色的 YUV 亮度 Y，接近白色
#define DETECTION_BOX_CB                        128         // 识别外接框颜色的 YUV 蓝色色度 Cb，中性
#define DETECTION_BOX_CR                        128         // 识别外接框颜色的 YUV 红色色度 Cr，中性
#define DETECTION_CROSS_Y                       76          // 中心十字颜色的 YUV 亮度 Y
#define DETECTION_CROSS_CB                      85          // 中心十字颜色的 YUV 蓝色色度 Cb
#define DETECTION_CROSS_CR                      255         // 中心十字颜色的 YU  V 红色色度 Cr，组合后为红色
#define DETECTION_CROSS_RADIUS                  10          // 中心十字从中心向四个方向延伸的长度，单位：像素
#define START_BANNER_DURATION_MS                3000        // 背景采集完成后 START 提示的显示时间，单位：ms
#define START_BANNER_Y                          16          // WAIT/START 提示底色的 YUV 亮度 Y，接近黑色
#define START_BANNER_CB                         128         // WAIT/START 提示底色的 YUV 蓝色色度 Cb，中性
#define START_BANNER_CR                         128         // WAIT/START 提示底色的 YUV 红色色度 Cr，中性
#define START_TEXT_Y                            235         // WAIT/START 文字颜色的 YUV 亮度 Y，接近白色
#define START_TEXT_CB                           128         // WAIT/START 文字颜色的 YUV 蓝色色度 Cb，中性
#define START_TEXT_CR                           128         // WAIT/START 文字颜色的 YUV 红色色度 Cr，中性

static uint8_t *background_y = NULL;
static size_t background_pixel_count = 0;
static uint16_t background_width = 0;
static uint16_t background_height = 0;
static TickType_t background_delay_start_tick = 0;
static TickType_t recognition_start_tick = 0;
static bool background_delay_started = false;
static uint8_t background_warmup_count = 0;
static uint8_t background_capture_count = 0;
static bool background_ready = false;
static uint8_t *difference_mask = NULL;
static uint16_t *difference_queue = NULL;
static size_t difference_mask_capacity = 0;

static void draw_yuv422_pixel(camera_fb_t *frame, int x, int y,
                               uint8_t brightness, uint8_t chroma_blue, uint8_t chroma_red)
{
    if (x < 0 || x >= frame->width || y < 0 || y >= frame->height)
        return;

    // YUV422 的相邻两个像素共用一组 Cb、Cr。
    size_t pixel_offset = (y * frame->width + x) * 2;
    size_t pair_offset = (y * frame->width + (x & ~1)) * 2;
    frame->buf[pixel_offset] = brightness;
    frame->buf[pair_offset + 1] = chroma_blue;
    frame->buf[pair_offset + 3] = chroma_red;
}

static void draw_detection_marker(camera_fb_t *frame, int min_x, int min_y, int max_x, int max_y,
                                  int center_x, int center_y)
{
    for (int x = min_x; x <= max_x; x++)
    {
        draw_yuv422_pixel(frame, x, min_y, DETECTION_BOX_Y, DETECTION_BOX_CB, DETECTION_BOX_CR);
        draw_yuv422_pixel(frame, x, max_y, DETECTION_BOX_Y, DETECTION_BOX_CB, DETECTION_BOX_CR);
    }

    for (int y = min_y; y <= max_y; y++)
    {
        draw_yuv422_pixel(frame, min_x, y, DETECTION_BOX_Y, DETECTION_BOX_CB, DETECTION_BOX_CR);
        draw_yuv422_pixel(frame, max_x, y, DETECTION_BOX_Y, DETECTION_BOX_CB, DETECTION_BOX_CR);
    }

    for (int offset = -DETECTION_CROSS_RADIUS; offset <= DETECTION_CROSS_RADIUS; offset++)
    {
        draw_yuv422_pixel(frame, center_x + offset, center_y,
                           DETECTION_CROSS_Y, DETECTION_CROSS_CB, DETECTION_CROSS_CR);
        draw_yuv422_pixel(frame, center_x, center_y + offset,
                           DETECTION_CROSS_Y, DETECTION_CROSS_CB, DETECTION_CROSS_CR);
    }
}

static void draw_status_banner(camera_fb_t *frame)
{
    // 5x7 点阵字体。
    static const uint8_t start_font[5][7] = {
        {0x0f, 0x10, 0x0e, 0x01, 0x1e, 0x10, 0x0f},
        {0x1f, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04},
        {0x0e, 0x11, 0x11, 0x1f, 0x11, 0x11, 0x11},
        {0x1e, 0x11, 0x11, 0x1e, 0x14, 0x12, 0x11},
        {0x1f, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04},
    };
    static const uint8_t wait_font[4][7] = {
        {0x11, 0x11, 0x11, 0x11, 0x15, 0x15, 0x0a},
        {0x0e, 0x11, 0x11, 0x11, 0x1f, 0x11, 0x11},
        {0x1f, 0x04, 0x04, 0x04, 0x04, 0x04, 0x1f},
        {0x1f, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04},
    };
    const uint8_t (*font)[7] = NULL;
    int character_count = 0;
    const int scale = frame->width >= 200 ? 3 : 2;
    const int origin_x = 8;
    const int origin_y = 8;
    const int glyph_width = 5 * scale;
    const int glyph_height = 7 * scale;

    if (background_delay_started && !background_ready &&
        xTaskGetTickCount() - background_delay_start_tick < pdMS_TO_TICKS(BACKGROUND_START_DELAY_MS))
    {
        font = wait_font;
        character_count = 4;
    }
    else if (recognition_start_tick != 0 &&
             xTaskGetTickCount() - recognition_start_tick < pdMS_TO_TICKS(START_BANNER_DURATION_MS))
    {
        font = start_font;
        character_count = 5;
    }
    else
    {
        return;
    }

    const int text_width = glyph_width * character_count + scale * (character_count - 1);
    const int banner_width = text_width + scale * 4;
    const int banner_height = glyph_height + scale * 4;

    for (int y = origin_y; y < origin_y + banner_height; y++)
    {
        for (int x = origin_x; x < origin_x + banner_width; x++)
            draw_yuv422_pixel(frame, x, y, START_BANNER_Y, START_BANNER_CB, START_BANNER_CR);
    }

    for (int character = 0; character < character_count; character++)
    {
        int glyph_x = origin_x + scale * 2 + character * (glyph_width + scale);
        int glyph_y = origin_y + scale * 2;
        for (int row = 0; row < 7; row++)
        {
            for (int column = 0; column < 5; column++)
            {
                if ((font[character][row] & (1 << (4 - column))) == 0)
                    continue;

                for (int offset_y = 0; offset_y < scale; offset_y++)
                {
                    for (int offset_x = 0; offset_x < scale; offset_x++)
                        draw_yuv422_pixel(frame, glyph_x + column * scale + offset_x,
                                          glyph_y + row * scale + offset_y,
                                          START_TEXT_Y, START_TEXT_CB, START_TEXT_CR);
                }
            }
        }
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
            recognition_start_tick = xTaskGetTickCount();
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

static void detect_difference_circle(camera_fb_t *frame)
{
    static uint8_t detect_frame_count = 0;
    const int sample_width = (frame->width - 2 * DIFFERENCE_CIRCLE_EDGE_MARGIN +
                              DIFFERENCE_CIRCLE_SCAN_STEP - 1) /
                             DIFFERENCE_CIRCLE_SCAN_STEP;
    const int sample_height = (frame->height - 2 * DIFFERENCE_CIRCLE_EDGE_MARGIN +
                               DIFFERENCE_CIRCLE_SCAN_STEP - 1) /
                              DIFFERENCE_CIRCLE_SCAN_STEP;
    const size_t sample_count = (size_t)sample_width * sample_height;

    if (!capture_background(frame))
    {
        draw_status_banner(frame);
        return;
    }

    // 每三帧检测一次，避免本次可行性测试影响当前图传帧率。
    if (++detect_frame_count < 3)
    {
        draw_status_banner(frame);
        return;
    }
    detect_frame_count = 0;

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

    for (int sample_y = 0; sample_y < sample_height; sample_y++)
    {
        int y = DIFFERENCE_CIRCLE_EDGE_MARGIN + sample_y * DIFFERENCE_CIRCLE_SCAN_STEP;
        for (int sample_x = 0; sample_x < sample_width; sample_x++)
        {
            int x = DIFFERENCE_CIRCLE_EDGE_MARGIN + sample_x * DIFFERENCE_CIRCLE_SCAN_STEP;
            size_t pixel_index = y * frame->width + x;
            int signed_difference = (int)frame->buf[pixel_index * 2] - background_y[pixel_index];
            int difference = signed_difference < 0 ? -signed_difference : signed_difference;
            difference_mask[sample_y * sample_width + sample_x] =
                (difference > DIFFERENCE_CIRCLE_BRIGHT_THRESHOLD ||
                 signed_difference < -DIFFERENCE_CIRCLE_DARK_THRESHOLD) ? 1 : 0;
        }
    }

    bool circle_found = false;
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
                int pixel_x = DIFFERENCE_CIRCLE_EDGE_MARGIN + current_x * DIFFERENCE_CIRCLE_SCAN_STEP;
                int pixel_y = DIFFERENCE_CIRCLE_EDGE_MARGIN + current_y * DIFFERENCE_CIRCLE_SCAN_STEP;
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
                fill_percent > DIFFERENCE_CIRCLE_MAX_FILL_PERCENT ||
                estimated_area <= best_area)
                continue;

            best_area = estimated_area;
            best_min_x = min_x;
            best_min_y = min_y;
            best_max_x = max_x;
            best_max_y = max_y;
            best_center_x = sum_x / component_count;
            best_center_y = sum_y / component_count;
            circle_found = true;
        }
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
        ESP_LOGI(TAG, "DIFF_CIRCLE x=%d y=%d w=%d h=%d area=%lu fill=%lu%%",
                 best_center_x, best_center_y, width, height,
                 (unsigned long)best_area, (unsigned long)fill_percent);
    }

    update_circle_detection_state(circle_found);
    draw_status_banner(frame);
}

static void task_process_handler(void *arg)
{
    while (true)
    {
        camera_fb_t *frame = esp_camera_fb_get();
        if (!frame)
            continue;

        detect_difference_circle(frame);

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
