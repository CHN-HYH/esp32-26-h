#include "who_ai_utils.hpp"

#include "esp_log.h"
#include "esp_camera.h"

#include "dl_image.hpp"
#include "my_usart.h"
#include "my_user_iic.h"
#include "my_usart1_user.h"

static const char *TAG = "ai_utils";

// extern uint16_t AI_Mode ;

// +-------+--------------------+----------+
// |       |       RGB565       |  RGB888  |
// +=======+====================+==========+
// |  Red  | 0b0000000011111000 | 0x0000FF |
// +-------+--------------------+----------+
// | Green | 0b1110000000000111 | 0x00FF00 |
// +-------+--------------------+----------+
// |  Blue | 0b0001111100000000 | 0xFF0000 |
// +-------+--------------------+----------+

void draw_detection_result(uint16_t *image_ptr, int image_height, int image_width, std::list<dl::detect::result_t> &results)
{
    int i = 0;
    for (std::list<dl::detect::result_t>::iterator prediction = results.begin(); prediction != results.end(); prediction++, i++)
    {
        dl::image::draw_hollow_rectangle(image_ptr, image_height, image_width,
                                         DL_MAX(prediction->box[0], 0),
                                         DL_MAX(prediction->box[1], 0),
                                         DL_MAX(prediction->box[2], 0),
                                         DL_MAX(prediction->box[3], 0),
                                         0b1110000000000111);

        if (prediction->keypoint.size() == 10)
        {
            dl::image::draw_point(image_ptr, image_height, image_width, DL_MAX(prediction->keypoint[0], 0), DL_MAX(prediction->keypoint[1], 0), 4, 0b0000000011111000); // left eye
            dl::image::draw_point(image_ptr, image_height, image_width, DL_MAX(prediction->keypoint[2], 0), DL_MAX(prediction->keypoint[3], 0), 4, 0b0000000011111000); // mouth left corner
            dl::image::draw_point(image_ptr, image_height, image_width, DL_MAX(prediction->keypoint[4], 0), DL_MAX(prediction->keypoint[5], 0), 4, 0b1110000000000111); // nose
            dl::image::draw_point(image_ptr, image_height, image_width, DL_MAX(prediction->keypoint[6], 0), DL_MAX(prediction->keypoint[7], 0), 4, 0b0001111100000000); // right eye
            dl::image::draw_point(image_ptr, image_height, image_width, DL_MAX(prediction->keypoint[8], 0), DL_MAX(prediction->keypoint[9], 0), 4, 0b0001111100000000); // mouth right corner
        }
    }
}

void draw_detection_result(uint8_t *image_ptr, int image_height, int image_width, std::list<dl::detect::result_t> &results)
{
    int i = 0;
    for (std::list<dl::detect::result_t>::iterator prediction = results.begin(); prediction != results.end(); prediction++, i++)
    {
        dl::image::draw_hollow_rectangle(image_ptr, image_height, image_width,
                                         DL_MAX(prediction->box[0], 0),
                                         DL_MAX(prediction->box[1], 0),
                                         DL_MAX(prediction->box[2], 0),
                                         DL_MAX(prediction->box[3], 0),
                                         0x00FF00);

        if (prediction->keypoint.size() == 10)
        {
            dl::image::draw_point(image_ptr, image_height, image_width, DL_MAX(prediction->keypoint[0], 0), DL_MAX(prediction->keypoint[1], 0), 4, 0x0000FF); // left eye
            dl::image::draw_point(image_ptr, image_height, image_width, DL_MAX(prediction->keypoint[2], 0), DL_MAX(prediction->keypoint[3], 0), 4, 0x0000FF); // mouth left corner
            dl::image::draw_point(image_ptr, image_height, image_width, DL_MAX(prediction->keypoint[4], 0), DL_MAX(prediction->keypoint[5], 0), 4, 0x00FF00); // nose
            dl::image::draw_point(image_ptr, image_height, image_width, DL_MAX(prediction->keypoint[6], 0), DL_MAX(prediction->keypoint[7], 0), 4, 0xFF0000); // right eye
            dl::image::draw_point(image_ptr, image_height, image_width, DL_MAX(prediction->keypoint[8], 0), DL_MAX(prediction->keypoint[9], 0), 4, 0xFF0000); // mouth right corner
        }
    }
}


uint8_t result_data[35]="\0";

char llx[4]="\0";
char lly[4]="\0";
char rrx[4]="\0";
char rry[4]="\0";

int mymyabs(int a)
{
    if(a < 0)
        return -a;
    return a;
}

//把数字转成字符串，并且3位对好 带负数不要了，算丢包
static void ccinttostr(int num,char *buf)
{
    if(num >=0)
    {
        if(num<=9) //只有1位
        {
            sprintf(buf,"00%d",num);
        }
        else if(num<=99)//只有两位
        {
            sprintf(buf,"0%d",num);
        }
        else
        {
            sprintf(buf,"%d",num);
        }
    }
    else
    {
        if(num>=-9) //只有1位
        {
            num = mymyabs(num);
            sprintf(buf,"-00%d",num);
        }
        else if(num>=-99)//只有两位
        {
            num = mymyabs(num);
            sprintf(buf,"-0%d",num);
        }
        else
        {
            sprintf(buf,"%d",num);
        }
    }
    
}


void print_detection_result(std::list<dl::detect::result_t> &results)
{
    int i = 0;
    for (std::list<dl::detect::result_t>::iterator prediction = results.begin(); prediction != results.end(); prediction++, i++)
    {
        ESP_LOGI("detection_result", "[%2d]: (%3d, %3d, %3d, %3d)", i, prediction->box[0], prediction->box[1], prediction->box[2], prediction->box[3]);
        
        ccinttostr(prediction->box[0],llx);
        ccinttostr(prediction->box[1],lly);
        ccinttostr(prediction->box[2],rrx);
        ccinttostr(prediction->box[3],rry);

        sprintf((char*)result_data,"$%s,%s,%s,%s,#",llx, lly, rrx, rry); //左上角坐标 右下角坐标
        Uart_Send_Data(result_data,strlen((char*)result_data)); 

        //串口1同步
        Uart1_Send_Data(result_data,strlen((char*)result_data)); 

        //同时IIC获取识别结果
        set_IIC_data(prediction->box[0], prediction->box[1], prediction->box[2], prediction->box[3]);

        if (prediction->keypoint.size() == 10)
        {
            ESP_LOGI("detection_result", "      left eye: (%3d, %3d), right eye: (%3d, %3d), nose: (%3d, %3d), mouth left: (%3d, %3d), mouth right: (%3d, %3d)",
                     prediction->keypoint[0], prediction->keypoint[1],  // left eye
                     prediction->keypoint[6], prediction->keypoint[7],  // right eye
                     prediction->keypoint[4], prediction->keypoint[5],  // nose
                     prediction->keypoint[2], prediction->keypoint[3],  // mouth left corner
                     prediction->keypoint[8], prediction->keypoint[9]); // mouth right corner
        }
    }
}

void *app_camera_decode(camera_fb_t *fb)
{
    if (fb->format == PIXFORMAT_RGB565)
    {
        return (void *)fb->buf;
    }
    else
    {
        uint8_t *image_ptr = (uint8_t *)malloc(fb->height * fb->width * 3 * sizeof(uint8_t));
        if (image_ptr)
        {
            if (fmt2rgb888(fb->buf, fb->len, fb->format, image_ptr))
            {
                return (void *)image_ptr;
            }
            else
            {
                ESP_LOGE(TAG, "fmt2rgb888 failed");
                dl::tool::free_aligned(image_ptr);
            }
        }
        else
        {
            ESP_LOGE(TAG, "malloc memory for image rgb888 failed");
        }
    }
    return NULL;
}