#include "yahboom_camera.h"
#include "my_usart.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_heap_caps.h"

static const char *TAG = "yahboom_camera";
static QueueHandle_t xQueueFrameO = NULL;

// 桌面背景差分测试参数：上电后自动采集空场景。
#define BACKGROUND_START_DELAY_MS      5000
#define BACKGROUND_WARMUP_FRAMES          20
#define BACKGROUND_CAPTURE_FRAMES          8
#define DIFFERENCE_CIRCLE_THRESHOLD       35
#define DIFFERENCE_CIRCLE_SCAN_STEP        2
#define DIFFERENCE_CIRCLE_EDGE_MARGIN      8
#define DIFFERENCE_CIRCLE_MIN_AREA         80
#define DIFFERENCE_CIRCLE_MIN_FILL_PERCENT 45
#define DIFFERENCE_CIRCLE_MAX_FILL_PERCENT 95
#define DIFFERENCE_CIRCLE_LOST_COUNT        3
#define DETECTION_BOX_Y                    235
#define DETECTION_BOX_CB                   128
#define DETECTION_BOX_CR                   128
#define DETECTION_CROSS_Y                   76
#define DETECTION_CROSS_CB                  85
#define DETECTION_CROSS_CR                 255
#define DETECTION_CROSS_RADIUS              10

static uint8_t *background_y = NULL;
static size_t background_pixel_count = 0;
static uint16_t background_width = 0;
static uint16_t background_height = 0;
static TickType_t background_delay_start_tick = 0;
static bool background_delay_started = false;
static uint8_t background_warmup_count = 0;
static uint8_t background_capture_count = 0;
static bool background_ready = false;

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
    uint32_t changed_pixel_count = 0;
    uint32_t sum_x = 0;
    uint32_t sum_y = 0;
    int min_x;
    int min_y;
    int max_x;
    int max_y;

    if (!capture_background(frame))
        return;

    // 每三帧检测一次，避免本次可行性测试影响当前图传帧率。
    if (++detect_frame_count < 3)
        return;
    detect_frame_count = 0;

    min_x = frame->width;
    min_y = frame->height;
    max_x = -1;
    max_y = -1;

    for (int y = DIFFERENCE_CIRCLE_EDGE_MARGIN; y < frame->height - DIFFERENCE_CIRCLE_EDGE_MARGIN; y += DIFFERENCE_CIRCLE_SCAN_STEP)
    {
        for (int x = DIFFERENCE_CIRCLE_EDGE_MARGIN; x < frame->width - DIFFERENCE_CIRCLE_EDGE_MARGIN; x += DIFFERENCE_CIRCLE_SCAN_STEP)
        {
            size_t pixel_index = y * frame->width + x;
            int difference = (int)frame->buf[pixel_index * 2] - background_y[pixel_index];
            if (difference < 0)
                difference = -difference;
            if (difference <= DIFFERENCE_CIRCLE_THRESHOLD)
                continue;

            changed_pixel_count++;
            sum_x += x;
            sum_y += y;
            if (x < min_x) min_x = x;
            if (x > max_x) max_x = x;
            if (y < min_y) min_y = y;
            if (y > max_y) max_y = y;
        }
    }

    bool circle_found = false;
    if (changed_pixel_count * DIFFERENCE_CIRCLE_SCAN_STEP * DIFFERENCE_CIRCLE_SCAN_STEP >= DIFFERENCE_CIRCLE_MIN_AREA)
    {
        int width = max_x - min_x + DIFFERENCE_CIRCLE_SCAN_STEP;
        int height = max_y - min_y + DIFFERENCE_CIRCLE_SCAN_STEP;
        uint32_t box_area = width * height;
        uint32_t fill_percent = changed_pixel_count * DIFFERENCE_CIRCLE_SCAN_STEP * DIFFERENCE_CIRCLE_SCAN_STEP * 100 / box_area;

        // 单个圆的外接矩形近似正方形，圆在外接矩形中的填充率约为 78%。
        if (width * 100 >= height * 65 && width * 100 <= height * 135 &&
            fill_percent >= DIFFERENCE_CIRCLE_MIN_FILL_PERCENT && fill_percent <= DIFFERENCE_CIRCLE_MAX_FILL_PERCENT)
        {
            int center_x = sum_x / changed_pixel_count;
            int center_y = sum_y / changed_pixel_count;
            uint32_t estimated_area = changed_pixel_count * DIFFERENCE_CIRCLE_SCAN_STEP * DIFFERENCE_CIRCLE_SCAN_STEP;
            circle_found = true;
            draw_detection_marker(frame, min_x, min_y,
                                  max_x + DIFFERENCE_CIRCLE_SCAN_STEP - 1,
                                  max_y + DIFFERENCE_CIRCLE_SCAN_STEP - 1,
                                  center_x, center_y);
            ESP_LOGI(TAG, "DIFF_CIRCLE x=%d y=%d w=%d h=%d area=%lu fill=%lu%%",
                     center_x, center_y, width, height,
                     (unsigned long)estimated_area, (unsigned long)fill_percent);
        }
    }

    update_circle_detection_state(circle_found);
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
