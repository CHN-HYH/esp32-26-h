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


#define UART_MUN         UART_NUM_0
#define UART_GPIO_TXD   (GPIO_NUM_43)
#define UART_GPIO_RXD   (GPIO_NUM_44)

// void My_Uart_Init(void);
void My_Uart_Init(const QueueHandle_t key_state_i);
int Uart_Send_Data(uint8_t* data, uint16_t len);
int Uart_Send_Byte(uint8_t data);

void Deal_uart_massage(uint8_t *buff,uint16_t len);

typedef enum
{
    myIDLE = 0,
    myDETECT,
    myENROLL,
    myRECOGNIZE,
    myDELETE,
} myrecognizer_state_t;

typedef enum
{
    Nornal_AI = 0,//不检测
    Cat_Dog_AI, //猫狗检测
    FACE_AI, //人脸检测
    COLOR_AI ,//颜色检测
    REFACE_AI, //人脸识别
    QR_AI, //二维码识别
    AI_MAX//最大值
} myAI_mode_t;

typedef enum
{
    myCOLOR_DETECTION_IDLE = 0,
    myOPEN_REGISTER_COLOR_BOX,
    myCLOSE_REGISTER_COLOR_BOX,
    myREGISTER_COLOR,
    myDELETE_COLOR,
    myINCREASE_COLOR_AREA,
    myDECREASE_COLOR_AREA,
    mySWITCH_RESULT,
} mycolor_detection_state_t;

#ifdef __cplusplus
}
#endif