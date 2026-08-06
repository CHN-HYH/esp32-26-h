#include "yahboom_msp_uart.h"

#include <stddef.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "my_usart1_user.h"

static const char *TAG = "yahboom_msp_uart";
static QueueHandle_t s_packet_queue;
static uint8_t s_sequence;

static uint8_t calculate_checksum(const uint8_t *data, size_t length)
{
    uint16_t sum = 0;
    for (size_t index = 0; index < length; index++)
        sum += data[index];

    return (uint8_t)(sum & 0xFF);
}

static void sender_task(void *arg)
{
    (void)arg;
    uint8_t packet[YAHBOOM_MSP_UART_PACKET_SIZE];
    TickType_t last_warning_tick = 0;

    while (true)
    {
        if (xQueueReceive(s_packet_queue, packet, portMAX_DELAY) != pdTRUE)
            continue;

        int sent = Uart1_Send_Data(packet, YAHBOOM_MSP_UART_PACKET_SIZE);
        if (sent != YAHBOOM_MSP_UART_PACKET_SIZE)
        {
            TickType_t now = xTaskGetTickCount();
            if (last_warning_tick == 0 ||
                now - last_warning_tick >= pdMS_TO_TICKS(1000))
            {
                last_warning_tick = now;
                ESP_LOGW(TAG, "MSP packet send failed: %d/%d bytes", sent,
                         YAHBOOM_MSP_UART_PACKET_SIZE);
            }
        }
    }
}

void yahboom_msp_uart_init(void)
{
    if (s_packet_queue != NULL)
        return;

    s_packet_queue = xQueueCreate(1, YAHBOOM_MSP_UART_PACKET_SIZE);
    if (s_packet_queue == NULL)
    {
        ESP_LOGE(TAG, "MSP packet queue allocation failed");
        return;
    }

    if (xTaskCreate(sender_task, "msp_uart_tx", 2048, NULL, 4, NULL) != pdPASS)
    {
        ESP_LOGE(TAG, "MSP packet sender task creation failed");
        vQueueDelete(s_packet_queue);
        s_packet_queue = NULL;
    }
}

bool yahboom_msp_uart_send(bool valid, uint16_t center_x, uint16_t width)
{
    if (s_packet_queue == NULL)
        return false;

    if (center_x > 319)
        center_x = 319;
    if (width > 255)
        width = 255;

    uint8_t packet[YAHBOOM_MSP_UART_PACKET_SIZE] = {
        0xAA,
        0x55,
        valid ? YAHBOOM_MSP_FLAG_VALID : 0,
        s_sequence++,
        (uint8_t)(center_x & 0xFF),
        (uint8_t)(center_x >> 8),
        (uint8_t)width,
        0,
    };

    if (!valid)
    {
        packet[4] = 0;
        packet[5] = 0;
        packet[6] = 0;
    }
    packet[7] = calculate_checksum(packet, YAHBOOM_MSP_UART_PACKET_SIZE - 1);

    // 队列只保留最新状态，避免串口发送落后时堆积旧坐标。
    return xQueueOverwrite(s_packet_queue, packet) == pdPASS;
}
