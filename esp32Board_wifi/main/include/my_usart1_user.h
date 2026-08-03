#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "sdkconfig.h"
#include "esp_err.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_chip_info.h"
#include "esp_system.h"
#include "esp_flash.h"


#define UART1_MUN         UART_NUM_1
#define UART1_GPIO_TXD   (GPIO_NUM_36)
#define UART1_GPIO_RXD   (GPIO_NUM_35)

// void My_Uart_Init(void);
void My_Uart1_user_Init(const QueueHandle_t key_state_i);
int Uart1_Send_Data(uint8_t* data, uint16_t len);
int Uart1_Send_Byte(uint8_t data);

void Deal_uart1_massage(uint8_t *buff,uint16_t len);


#ifdef __cplusplus
}
#endif