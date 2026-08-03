#include "yahboom_camera.h"
#include "app_mywifi.h"
#include "app_myhttpd.hpp"
#include "app_mymdns.h"

#include "esp_log.h"
#include "driver/spi_common.h"
#include "esp_chip_info.h"
#include "esp_system.h"
#include "esp_flash.h"

#include "my_usart.h"
#include "my_usart1_user.h"
#include "my_user_iic.h"
#include "mykey.h"
#include <cstring>


static QueueHandle_t xQueueAIFrame = NULL;
static QueueHandle_t xQueueAIUSERFrame = NULL;
static QueueHandle_t xQueuemyvirtualKey = NULL;

static const char TAG[] = "main_AI_version";
char Version[] = "AI_V1.5.1";
uint16_t wifi_Mode = 2; // 0: AP, 1: STA, 2: AP+STA
extern uint8_t sta_ip_connect[4];


extern "C" void app_main(void)
{
    uint8_t ver_data[50]="\0";

    xQueueAIFrame = xQueueCreate(1, sizeof(camera_fb_t *));

    xQueueAIUSERFrame = xQueueCreate(2, sizeof(camera_fb_t *));

    xQueuemyvirtualKey = xQueueCreate(1, sizeof(int *));

    My_Uart1_user_Init(xQueuemyvirtualKey);//串口1初始化

    app_mywifi_main();

    //摄像头相关
    my_register_camera(PIXFORMAT_YUV422, FRAMESIZE_QVGA, 2, xQueueAIFrame);
    app_mymdns_main();

    register_httpd(xQueueAIFrame, NULL, true);//图传服务
 
    My_Uart_Init(xQueuemyvirtualKey);

    //直接把版本号发送出来+ 串口助手也能查询
    sprintf((char*)ver_data,"YAHBOOM Board Ver:%s\r\n",Version);
    Uart_Send_Data(ver_data,strlen((char*)ver_data)); 

    sprintf((char*)ver_data,"sta_ip:%d.%d.%d.%d\r\n",sta_ip_connect[0],sta_ip_connect[1],sta_ip_connect[2],sta_ip_connect[3]);
    Uart_Send_Data(ver_data,strlen((char*)ver_data));

    sprintf((char*)ver_data,"ap_ip:192.168.4.1\r\n"); //固定就好
    Uart_Send_Data(ver_data,strlen((char*)ver_data));

}
