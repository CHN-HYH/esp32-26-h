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


#define KEY_GPIO_BOOT0        0



#define KEY_STATE_PRESS       1
#define KEY_STATE_RELEASE     0


#define KEY_MODE_ONE_TIME     1
#define KEY_MODE_ALWAYS       0

#define KEY1 gpio_get_level(KEY_GPIO_BOOT0)

typedef enum _KEY_ID {
    KEY_ID_BOOT0,
} key_id_t;



void Key_Init(const QueueHandle_t key_state_i);
void Key_Handle(void);
uint8_t Long_Press(void);
uint8_t click_N_Double (uint16_t time);
void  Key_data_upgrade(uint8_t keydata);
void Get_key_data(void);

#ifdef __cplusplus
}
#endif
