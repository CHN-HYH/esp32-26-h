#include "yahboom_camera.h"
#include "yahboom_detection.h"
#include "yahboom_overlay.h"

#include "esp_log.h"

static const char *TAG = "yahboom_camera";
static QueueHandle_t xQueueFrameO = NULL;
static yahboom_detection_context_t detection_context;

static void task_process_handler(void *arg)
{
    (void)arg;

    while (true)
    {
        camera_fb_t *frame = esp_camera_fb_get();
        if (frame == NULL)
            continue;

        yahboom_detection_process(&detection_context, frame);
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

void my_register_camera(const pixformat_t pixel_format,
                        const framesize_t frame_size,
                        const uint8_t fb_count,
                        const QueueHandle_t frame_queue)
{
    ESP_LOGI(TAG, "Camera module is %s", CAMERA_MODULE_NAME);

#if CONFIG_CAMERA_MODULE_ESP_EYE || CONFIG_CAMERA_MODULE_ESP32_CAM_BOARD
    // IO13、IO14 默认用于 JTAG，这两个引脚需要作为普通输入时打开上拉。
    gpio_config_t gpio_conf = {
        .pin_bit_mask = (1ULL << 13) | (1ULL << 14),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&gpio_conf);
#endif

    camera_config_t camera_config = {
        .ledc_channel = LEDC_CHANNEL_0,
        .ledc_timer = LEDC_TIMER_0,
        .pin_d0 = CAMERA_PIN_D0,
        .pin_d1 = CAMERA_PIN_D1,
        .pin_d2 = CAMERA_PIN_D2,
        .pin_d3 = CAMERA_PIN_D3,
        .pin_d4 = CAMERA_PIN_D4,
        .pin_d5 = CAMERA_PIN_D5,
        .pin_d6 = CAMERA_PIN_D6,
        .pin_d7 = CAMERA_PIN_D7,
        .pin_xclk = CAMERA_PIN_XCLK,
        .pin_pclk = CAMERA_PIN_PCLK,
        .pin_vsync = CAMERA_PIN_VSYNC,
        .pin_href = CAMERA_PIN_HREF,
        .pin_sscb_sda = CAMERA_PIN_SIOD,
        .pin_sscb_scl = CAMERA_PIN_SIOC,
        .pin_pwdn = CAMERA_PIN_PWDN,
        .pin_reset = CAMERA_PIN_RESET,
        .xclk_freq_hz = XCLK_FREQ_HZ,
        .pixel_format = pixel_format,
        .frame_size = frame_size,
        .jpeg_quality = 12,
        .fb_count = fb_count,
        .fb_location = CAMERA_FB_IN_PSRAM,
        .grab_mode = CAMERA_GRAB_LATEST,
    };

    esp_err_t error = esp_camera_init(&camera_config);
    if (error != ESP_OK)
    {
        ESP_LOGE(TAG, "Camera init failed with error 0x%x", error);
        return;
    }

    sensor_t *sensor = esp_camera_sensor_get();
    if (sensor->id.PID == OV3660_PID || sensor->id.PID == OV2640_PID)
    {
        sensor->set_vflip(sensor, 1);
    }
    else if (sensor->id.PID == GC0308_PID)
    {
        sensor->set_hmirror(sensor, 0);
    }
    else if (sensor->id.PID == GC032A_PID)
    {
        sensor->set_vflip(sensor, 1);
    }

    if (sensor->id.PID == OV3660_PID)
    {
        sensor->set_brightness(sensor, 1);
        sensor->set_saturation(sensor, -2);
    }

    if (sensor->id.PID == GC2145_PID)
    {
        // GC2145 当前只关闭自动白平衡，其他寄存器保持驱动默认值。
        sensor->set_reg(sensor, 0xfe, 0xFF, 0);
        sensor->set_reg(sensor, 0x42, 0xFF, 0xfd);
    }

    yahboom_detection_init(&detection_context);
    xQueueFrameO = frame_queue;
    xTaskCreatePinnedToCore(task_process_handler, TAG, 3 * 1024, NULL, 5, NULL, 1);
}
