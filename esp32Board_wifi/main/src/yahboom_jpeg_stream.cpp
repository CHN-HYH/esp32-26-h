#include "yahboom_jpeg_stream.h"

#include "esp_heap_caps.h"
#include "esp_jpeg_enc.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/semphr.h"

#define YAHBOOM_JPEG_WIDTH 320
#define YAHBOOM_JPEG_HEIGHT 240
#define YAHBOOM_JPEG_QUALITY 20  // 仅降低网页图传质量，本地识别仍使用完整 YUV422 原始帧
#define YAHBOOM_JPEG_BUFFER_SIZE (128 * 1024)

static const char *TAG = "yahboom_jpeg";

static SemaphoreHandle_t s_state_mutex = NULL;
static SemaphoreHandle_t s_capture_mutex = NULL;
static SemaphoreHandle_t s_slot_semaphore = NULL;
static SemaphoreHandle_t s_ready_semaphore = NULL;
static QueueHandle_t s_capture_queue = NULL;
static QueueHandle_t s_encode_queue = NULL;
static jpeg_enc_handle_t s_encoder = NULL;
static uint8_t *s_output = NULL;
static size_t s_length = 0;
static int64_t s_ready_time_us = 0;
static int64_t s_timestamp_sec = 0;
static int32_t s_timestamp_usec = 0;
static uint32_t s_encode_time_us = 0;
static bool s_active = false;
static bool s_ready_pending = false;
static bool s_sender_owns_slot = false;
static bool s_initialized = false;
static uint32_t s_capture_consumers = 0;

static void jpeg_encode_task(void *arg);

void yahboom_jpeg_stream_init(QueueHandle_t capture_queue)
{
    if (s_initialized)
        return;

    s_state_mutex = xSemaphoreCreateMutex();
    s_capture_mutex = xSemaphoreCreateMutex();
    s_slot_semaphore = xSemaphoreCreateBinary();
    s_ready_semaphore = xSemaphoreCreateBinary();
    s_encode_queue = xQueueCreate(1, sizeof(camera_fb_t *));
    if (s_state_mutex == NULL || s_capture_mutex == NULL ||
        s_slot_semaphore == NULL || s_ready_semaphore == NULL || s_encode_queue == NULL)
    {
        ESP_LOGE(TAG, "JPEG stream semaphore allocation failed");
        return;
    }

    s_capture_queue = capture_queue;

    s_output = (uint8_t *)heap_caps_malloc(YAHBOOM_JPEG_BUFFER_SIZE,
                                           MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (s_output == NULL)
    {
        ESP_LOGE(TAG, "JPEG stream output buffer allocation failed");
        return;
    }

    jpeg_enc_config_t config = DEFAULT_JPEG_ENC_CONFIG();
    config.width = YAHBOOM_JPEG_WIDTH;
    config.height = YAHBOOM_JPEG_HEIGHT;
    config.src_type = JPEG_PIXEL_FORMAT_YCbYCr;
    config.subsampling = JPEG_SUBSAMPLE_420;
    config.quality = YAHBOOM_JPEG_QUALITY;
    config.rotate = JPEG_ROTATE_0D;
    config.task_enable = false;

    if (jpeg_enc_open(&config, &s_encoder) != JPEG_ERR_OK)
    {
        ESP_LOGE(TAG, "JPEG stream encoder initialization failed");
        free(s_output);
        s_output = NULL;
        return;
    }

    xSemaphoreGive(s_slot_semaphore);
    s_initialized = true;
    if (xTaskCreatePinnedToCore(jpeg_encode_task, "yahboom_jpeg", 5 * 1024, NULL, 4,
                                NULL, 1) != pdPASS)
    {
        ESP_LOGE(TAG, "JPEG encode task creation failed");
        s_initialized = false;
        return;
    }
    ESP_LOGI(TAG, "JPEG stream ready: %dx%d quality=%d", YAHBOOM_JPEG_WIDTH,
             YAHBOOM_JPEG_HEIGHT, YAHBOOM_JPEG_QUALITY);
}

void yahboom_jpeg_stream_start(void)
{
    if (!s_initialized || xSemaphoreTake(s_state_mutex, portMAX_DELAY) != pdTRUE)
        return;

    s_active = true;
    xSemaphoreGive(s_state_mutex);
}

void yahboom_jpeg_stream_stop(void)
{
    bool release_ready = false;
    camera_fb_t *queued_frame = NULL;

    if (!s_initialized || xSemaphoreTake(s_state_mutex, portMAX_DELAY) != pdTRUE)
        return;

    s_active = false;
    if (s_encode_queue != NULL)
        xQueueReceive(s_encode_queue, &queued_frame, 0);
    if (s_ready_pending)
    {
        s_ready_pending = false;
        release_ready = true;
    }
    xSemaphoreGive(s_state_mutex);

    if (release_ready && xSemaphoreTake(s_ready_semaphore, 0) == pdTRUE)
        xSemaphoreGive(s_slot_semaphore);
    if (queued_frame != NULL)
        esp_camera_fb_return(queued_frame);
}

static void encode_frame(camera_fb_t *frame)
{
    const int64_t encode_start_us = esp_timer_get_time();
    int encoded_size = 0;

    if (frame == NULL)
        return;

    if (frame->format != PIXFORMAT_YUV422 ||
        frame->width != YAHBOOM_JPEG_WIDTH || frame->height != YAHBOOM_JPEG_HEIGHT ||
        (((uintptr_t)frame->buf & 0x0f) != 0))
    {
        esp_camera_fb_return(frame);
        return;
    }

    if (xSemaphoreTake(s_state_mutex, portMAX_DELAY) != pdTRUE)
    {
        esp_camera_fb_return(frame);
        return;
    }
    const bool active = s_active;
    xSemaphoreGive(s_state_mutex);
    if (!active || xSemaphoreTake(s_slot_semaphore, 0) != pdTRUE)
    {
        esp_camera_fb_return(frame);
        return;
    }

    const jpeg_error_t ret = jpeg_enc_process(s_encoder, frame->buf, frame->len,
                                              s_output, YAHBOOM_JPEG_BUFFER_SIZE,
                                              &encoded_size);
    const uint32_t encode_time_us = (uint32_t)(esp_timer_get_time() - encode_start_us);

    if (ret != JPEG_ERR_OK || encoded_size <= 0)
    {
        xSemaphoreGive(s_slot_semaphore);
        esp_camera_fb_return(frame);
        return;
    }

    if (xSemaphoreTake(s_state_mutex, portMAX_DELAY) != pdTRUE)
    {
        xSemaphoreGive(s_slot_semaphore);
        esp_camera_fb_return(frame);
        return;
    }

    if (!s_active)
    {
        xSemaphoreGive(s_state_mutex);
        xSemaphoreGive(s_slot_semaphore);
        esp_camera_fb_return(frame);
        return;
    }

    s_length = (size_t)encoded_size;
    s_ready_time_us = esp_timer_get_time();
    s_timestamp_sec = frame->timestamp.tv_sec;
    s_timestamp_usec = (int32_t)frame->timestamp.tv_usec;
    s_encode_time_us = encode_time_us;
    s_ready_pending = true;
    xSemaphoreGive(s_ready_semaphore);
    xSemaphoreGive(s_state_mutex);
    esp_camera_fb_return(frame);
}

static void jpeg_encode_task(void *arg)
{
    (void)arg;

    while (true)
    {
        camera_fb_t *frame = NULL;
        if (xQueueReceive(s_encode_queue, &frame, portMAX_DELAY) == pdTRUE)
            encode_frame(frame);
    }
}

bool yahboom_jpeg_stream_submit_frame(camera_fb_t *frame)
{
    camera_fb_t *stale_frame = NULL;
    bool queued = false;

    if (frame == NULL || !s_initialized ||
        xSemaphoreTake(s_state_mutex, 0) != pdTRUE)
    {
        return false;
    }

    if (s_active && s_encode_queue != NULL)
    {
        if (xQueueSend(s_encode_queue, &frame, 0) == pdTRUE)
        {
            queued = true;
        }
        else
        {
            if (xQueueReceive(s_encode_queue, &stale_frame, 0) == pdTRUE)
                queued = xQueueSend(s_encode_queue, &frame, 0) == pdTRUE;
        }
    }

    xSemaphoreGive(s_state_mutex);
    if (stale_frame != NULL)
        esp_camera_fb_return(stale_frame);
    return queued;
}

bool yahboom_jpeg_stream_wait(yahboom_jpeg_frame_t *frame, TickType_t ticks_to_wait)
{
    if (!s_initialized || frame == NULL ||
        xSemaphoreTake(s_ready_semaphore, ticks_to_wait) != pdTRUE)
    {
        return false;
    }

    if (xSemaphoreTake(s_state_mutex, portMAX_DELAY) != pdTRUE)
    {
        xSemaphoreGive(s_slot_semaphore);
        return false;
    }

    s_ready_pending = false;
    s_sender_owns_slot = true;
    frame->data = s_output;
    frame->length = s_length;
    frame->ready_time_us = s_ready_time_us;
    frame->timestamp_sec = s_timestamp_sec;
    frame->timestamp_usec = s_timestamp_usec;
    frame->encode_time_us = s_encode_time_us;
    xSemaphoreGive(s_state_mutex);
    return true;
}

void yahboom_jpeg_stream_release(void)
{
    bool release_slot = false;

    if (!s_initialized || xSemaphoreTake(s_state_mutex, portMAX_DELAY) != pdTRUE)
        return;

    if (s_sender_owns_slot)
    {
        s_sender_owns_slot = false;
        release_slot = true;
    }
    xSemaphoreGive(s_state_mutex);

    if (release_slot)
        xSemaphoreGive(s_slot_semaphore);
}

void yahboom_capture_start(void)
{
    if (s_capture_mutex == NULL || xSemaphoreTake(s_capture_mutex, portMAX_DELAY) != pdTRUE)
        return;

    s_capture_consumers++;
    xSemaphoreGive(s_capture_mutex);
}

void yahboom_capture_stop(void)
{
    camera_fb_t *queued_frame = NULL;

    if (s_capture_mutex == NULL || xSemaphoreTake(s_capture_mutex, portMAX_DELAY) != pdTRUE)
        return;

    if (s_capture_consumers > 0)
        s_capture_consumers--;
    if (s_capture_consumers == 0 && s_capture_queue != NULL)
        xQueueReceive(s_capture_queue, &queued_frame, 0);
    xSemaphoreGive(s_capture_mutex);

    if (queued_frame != NULL)
        esp_camera_fb_return(queued_frame);
}

bool yahboom_capture_get_frame(camera_fb_t **frame, TickType_t ticks_to_wait)
{
    if (frame == NULL || s_capture_queue == NULL)
        return false;

    return xQueueReceive(s_capture_queue, frame, ticks_to_wait) == pdTRUE;
}

bool yahboom_capture_publish_frame(camera_fb_t *frame, bool *queue_replaced)
{
    camera_fb_t *stale_frame = NULL;
    bool queued = false;

    if (queue_replaced != NULL)
        *queue_replaced = false;
    if (frame == NULL || s_capture_mutex == NULL ||
        xSemaphoreTake(s_capture_mutex, 0) != pdTRUE)
    {
        return false;
    }

    if (s_capture_consumers > 0 && s_capture_queue != NULL)
    {
        if (xQueueSend(s_capture_queue, &frame, 0) == pdTRUE)
        {
            queued = true;
        }
        else
        {
            if (xQueueReceive(s_capture_queue, &stale_frame, 0) == pdTRUE &&
                queue_replaced != NULL)
            {
                *queue_replaced = true;
            }
            queued = xQueueSend(s_capture_queue, &frame, 0) == pdTRUE;
        }
    }

    xSemaphoreGive(s_capture_mutex);
    if (stale_frame != NULL)
        esp_camera_fb_return(stale_frame);
    return queued;
}
