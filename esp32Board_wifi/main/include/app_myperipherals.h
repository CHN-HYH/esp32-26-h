#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_camera.h"

#ifdef __cplusplus
extern "C"
{
#endif

void regsitig_decode(      const QueueHandle_t frame_i,
                                const QueueHandle_t event,
                                const QueueHandle_t result,
                                const QueueHandle_t frame_o);

    
#ifdef __cplusplus
}
#endif