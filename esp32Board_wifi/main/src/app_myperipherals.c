#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "app_myperipherals.h"
#include "esp_code_scanner.h"
#include "fb_gfx.h"
#include "my_usart.h"
#include "my_usart1_user.h"

static QueueHandle_t xQueueFrameI = NULL;
static QueueHandle_t xQueueEvent = NULL;
static QueueHandle_t xQueueResult = NULL;
static QueueHandle_t xQueueFrameO = NULL;

static const char *TAG = "APP_CODE_SCANNER";

static void rgb_print(camera_fb_t *fb, uint32_t color, const char *str)
{
    fb_gfx_print(fb, (fb->width - (strlen(str) * 14)) / 2, 10, color, str);
}

static int rgb_printf(camera_fb_t *fb, uint32_t color, const char *format, ...)
{
    char loc_buf[64];
    char *temp = loc_buf;
    int len;
    va_list arg;
    va_list copy;
    va_start(arg, format);
    va_copy(copy, arg);
    len = vsnprintf(loc_buf, sizeof(loc_buf), format, arg);
    va_end(copy);
    if (len >= sizeof(loc_buf))
    {
        temp = (char *)malloc(len + 1);
        if (temp == NULL)
        {
            return 0;
        }
    }
    vsnprintf(temp, len + 1, format, arg);
    va_end(arg);
    rgb_print(fb, color, temp);
    if (len > 64)
    {
        free(temp);
    }
    return len;
}

static void decode_task(void *arg)
{

    camera_fb_t *fb = NULL;
    int64_t time1, time2;
    uint8_t decode_massage[35] = {'\0'};
    while (1)
    {
        if (xQueueReceive(xQueueFrameI, &fb, portMAX_DELAY))
        {
            time1 = esp_timer_get_time();
            // Decode Progress
            esp_image_scanner_t *esp_scn = esp_code_scanner_create();
            esp_code_scanner_config_t config = {ESP_CODE_SCANNER_MODE_FAST, ESP_CODE_SCANNER_IMAGE_RGB565, fb->width, fb->height};
            esp_code_scanner_set_config(esp_scn, config);
            int decoded_num = esp_code_scanner_scan_image(esp_scn, fb->buf);

            if(decoded_num){
                esp_code_scanner_symbol_t result = esp_code_scanner_result(esp_scn);
                time2 = esp_timer_get_time();

                rgb_printf(fb, 0x07E0, "%s", result.data); //画到图上 显示结果 绿色

                ESP_LOGI(TAG, "Decode time in %lld ms.", (time2 - time1) / 1000);
                ESP_LOGI(TAG, "Decoded %s symbol \"%s\"\n", result.type_name, result.data);

                sprintf((char*)decode_massage,"$%s#",result.data);
                Uart_Send_Data(decode_massage,strlen((char*)decode_massage)); 

                //串口1同步发送
                Uart1_Send_Data(decode_massage,strlen((char*)decode_massage));
            }
            esp_code_scanner_destroy(esp_scn);
            
            if (xQueueFrameO)
            {
                xQueueSend(xQueueFrameO, &fb, portMAX_DELAY);
            }
            //esp_camera_fb_return(fb);
            //vTaskDelay(10 / portTICK_PERIOD_MS);
        }
    }
}


void regsitig_decode(   const QueueHandle_t frame_i,
                        const QueueHandle_t event,
                        const QueueHandle_t result,
                        const QueueHandle_t frame_o)
{

    xQueueFrameI = frame_i;
    xQueueFrameO = frame_o;
    xQueueEvent = event;
    xQueueResult = result;

    xTaskCreatePinnedToCore(decode_task, TAG, 4 * 1024, NULL, 6, NULL, 0);
}
