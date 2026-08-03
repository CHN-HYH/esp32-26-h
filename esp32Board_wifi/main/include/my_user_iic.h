#pragma once

#ifdef __cplusplus
extern "C" {
#endif


#include "sdkconfig.h"
#include "esp_err.h"
#include "driver/i2c.h"
#include "esp_intr_alloc.h"
#include "driver/gpio.h"
#include "esp_chip_info.h"
#include "esp_system.h"
#include "esp_flash.h"



#define MYI2C_SLAVE_NUM  0
#define MYI2C_SLAVE_SDA (38)
#define MYI2C_SLAVE_SCL (37)

#define Camera_IIC_ADDR  (0x33)

#define I2C_SLAVE_RX_BUF_LEN (1024)
#define I2C_SLAVE_TX_BUF_LEN (1)


//REG 寄存器
#define Horizontal_IMG 0x01 //水平翻转寄存器
#define Vertical_IMG   0x02 //垂直翻转寄存器

#define Model_SECLCE   0x03 //模式选择
#define Model_RESET    0x04 //模块复位重启
#define VIRTUAL_KEY    0x08 //虚拟按键

#define Left_X_High   0x20 // 左X的坐标 高位
#define Left_X_LOW    0x21 // 左X的坐标 低位
#define Left_Y_High   0x22 // 左Y的坐标 高位
#define Left_Y_LOW    0x23 // 左Y的坐标 低位

#define Right_X_High   0x24 // 右X的坐标 高位
#define Right_X_LOW    0x25 // 右X的坐标 低位
#define Right_Y_High   0x26 // 右Y的坐标 高位
#define Right_Y_LOW    0x27 // 右Y的坐标 低位

#define middle_X_High   0x28 // 右X的坐标 高位
#define middle_X_LOW    0x29 // 右X的坐标 低位
#define middle_Y_High   0x2A // 右Y的坐标 高位
#define middle_Y_LOW    0x2B // 右Y的坐标 低位

#define Face_ID_High   0x2C // 人脸的id 高位
#define Face_ID_LOW    0x2D // 人脸的id 低位

#define Area_High   0x2E // 人脸的id 高位
#define Area_LOW    0x2F // 人脸的id 低位

//AI识别的信息结构体
typedef struct IIC_REG_Data_t 
{
    uint16_t msg_lx; //左上角
    uint16_t msg_ly; //左上角
    uint16_t msg_rx; //右下角
    uint16_t msg_ry; //右下角
    uint16_t msg_mx; //中心点坐标 X
    uint16_t msg_my; //中心点坐标 Y
    int16_t msg_id; //id号
    int area; //识别框的第一个面积
}IIC_REG_Data;



void My_i2c_init(const QueueHandle_t key_state_i);
int I2C_Send_Byte(uint8_t data);
int I2C_Send_Data(uint8_t* data, uint16_t len);
void set_IIC_data(int16_t lx,int16_t ly,int16_t rx,int16_t ry);
void set_IIC_data_id(int16_t temp_id);


void switch_i2c_control(uint8_t REG_flag,uint8_t data);



void recv_i2c_data(uint8_t REG_flag);


// void send_leftX_high(void);
// void send_leftX_Low(void);
// void send_leftY_high(void);
// void send_leftY_Low(void);

// void send_rightX_high(void);
// void send_rightX_Low(void);
// void send_rightY_high(void);
// void send_rightY_Low(void);

// void send_FaceID_high(void);
// void send_FaceID_Low(void);

#ifdef __cplusplus
}
#endif