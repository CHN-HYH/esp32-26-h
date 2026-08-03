#include "mykey.h"
#include "my_usart.h"
#include "stdio.h"
#include "stdint.h"
#include "string.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "driver/gpio.h"
#include "my_user_iic.h"

#define u8 uint8_t
#define u16 uint16_t




const static char *TAG = "KEY";


uint16_t AI_Mode = 0; 
volatile u8 g_shote_flag = 0;//没按下为0 ,短按置1
volatile u8 g_long_flag = 0;//没按下为0 ,长按置1
volatile u8 g_double_flag = 0;//没按下为0  ,双击置1

static QueueHandle_t xQueuevirtualKeystate = NULL; //虚拟按键
static myrecognizer_state_t recognizer_state = myDETECT;//人脸检测的标志,默认就是检测
static mycolor_detection_state_t mydetector_state = myCOLOR_DETECTION_IDLE;//颜色识别的标志


static uint8_t key0_data_flag = 1; //短按一次，发送一次  双击减1 长按加1
static uint8_t key_state[100]={'\0'};//按键定义发送

static void Key_GPIO_Init(void)
{
    // zero-initialize the config structure.
    gpio_config_t io_conf = {};
    //disable interrupt
    io_conf.intr_type = GPIO_INTR_DISABLE;
    //set as input mode
    io_conf.mode = GPIO_MODE_INPUT;
    //bit mask of the pins that you want to set
    io_conf.pin_bit_mask = (1ULL<<KEY_GPIO_BOOT0);
    //disable pull-down mode
    io_conf.pull_down_en = 0;
    //enable pull-up mode
    io_conf.pull_up_en = 1;
    //configure GPIO with the given settings
    gpio_config(&io_conf);
    
}


static void Key_Task(void *arg)
{
    //ESP_LOGD(TAG, "Start Key_Task with core:%d", xPortGetCoreID());
    while (1)
    {
        //printf("key i am coming\r\n");
        Key_Handle();
        Get_key_data();
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    vTaskDelete(NULL);
}


/**************************************************************************
Function: Long press detection
Input   : none
Output  : 0：No action；1：Long press for 2 seconds；
函数功能：长按检测
入口参数：无
返回  值：按键状态 0：无动作 1：长按2s
**************************************************************************/
uint8_t Long_Press(void)
{
  static uint16_t Long_Press_count,Long_Press;
  if(Long_Press==0&&KEY1==0) 
  {
    Long_Press_count++;   //长按标志位未置1
  } 
  
  else   
  {
    Long_Press_count=0; 
  }                    
    
    if(Long_Press_count>150)		//10ms扫描一次
    {
      Long_Press=1;	
      Long_Press_count=0;
      return 1;
     }				
    if(Long_Press==1)     //长按标志位置1
    {
        Long_Press=0;
    }
    return 0;
}


/**************************************************************************
Function: Key scan
Input   : Double click the waiting time
Output  : 0：No action；1：click；2：Double click
函数功能：按键扫描
入口参数：双击等待时间
返回  值：按键状态 0：无动作 1：单击 2：双击 
**************************************************************************/
uint8_t click_N_Double (uint16_t time)
{
  static	uint8_t flag_key,count_key,double_key;	
  static	uint16_t count_single,Forever_count;
  if(KEY1==0)  Forever_count++;   //长按标志位未置1
  else        Forever_count=0;
  if(0==KEY1&&0==flag_key)		flag_key=1;	//第一次按下
  if(0==count_key)
  {
    if(flag_key==1) 
    {
        double_key++;
        count_key=1;			//标记按下一次
    }
    if(double_key==2) 
    {					//按下两次
        double_key=0;
        count_single=0;
        return 2;			//双击执行的指令
    }
  }
    if(1==KEY1)
    {
       flag_key=0,count_key=0;
    }

    if(1==double_key)
    {
      count_single++;
      if(count_single>time&&Forever_count<time)
      {
        double_key=0;
        count_single=0;	//超时不标记为双击
        return 1;//单击执行的指令
      }
      if(Forever_count>time)
      {
        double_key=0;
        count_single=0;	
      }
    }	
    return 0;
}



// 每10毫秒调用一次。
void Key_Handle(void)
{
    uint8_t tmp,tmp2;
    tmp=click_N_Double(50); 
    if(tmp==1)
    { 
        g_shote_flag = 1;//短按 
            
    }
    if(tmp == 2)
    {
        g_double_flag = 1; //双击
        
    }
    
    tmp2=Long_Press();                   
    if(tmp2==1) 
    {
        g_long_flag = 1; //长按
        
    }


}


extern bool register_mode ;//引入串口0颜色识别的注册标志 使其同步
void  Key_data_upgrade(uint8_t keydata)
{
    
    // if (keydata == 1 )//eg:KEY_MENU
    // {
    //     // //人脸识别
    //     if(AI_Mode == REFACE_AI)
    //     {
    //         recognizer_state = myENROLL;//登记
    //         xQueueSend(xQueuevirtualKeystate, &recognizer_state, portMAX_DELAY);
    //     }
    //     else if (AI_Mode == COLOR_AI)
    //     {
    //         //颜色识别
    //         mydetector_state = register_mode ? myCLOSE_REGISTER_COLOR_BOX : myOPEN_REGISTER_COLOR_BOX;
    //         register_mode = !register_mode;
    //         xQueueSend(xQueuevirtualKeystate, &mydetector_state, portMAX_DELAY);
    //     }
    //     //printf("KEY_MENU\r\n");
    // }
    // else if (keydata == 2)//eg:KEY_play
    // {
    //     //人脸识别
    //     if(AI_Mode == REFACE_AI)
    //     {
    //         recognizer_state = myRECOGNIZE;//识别
    //         xQueueSend(xQueuevirtualKeystate, &recognizer_state, portMAX_DELAY);
    //     }
    //     else if (AI_Mode == COLOR_AI)//颜色识别
    //    {
    //         mydetector_state = register_mode ? myREGISTER_COLOR : myDELETE_COLOR;
    //         register_mode = false;
    //         xQueueSend(xQueuevirtualKeystate, &mydetector_state, portMAX_DELAY);
    //     }
    //     //printf("keydata:2\r\n");
    // }
    // else if (keydata == 3)//eg:KEY_upup
    // {
    //     //人脸识别
    //     if(AI_Mode == REFACE_AI)
    //     {
    //         recognizer_state = myDETECT;//检测
    //         xQueueSend(xQueuevirtualKeystate, &recognizer_state, portMAX_DELAY);
    //     }

    //     else if (AI_Mode == COLOR_AI)//颜色识别
    //     {
    //         mydetector_state = myINCREASE_COLOR_AREA;
    //         xQueueSend(xQueuevirtualKeystate, &mydetector_state, portMAX_DELAY);
    //     }
    //     //printf("keydata:3\r\n");
    // }
    // else if (keydata == 4)//eg:KEY_down
    // {
    //     //人脸识别
    //     if(AI_Mode == REFACE_AI)
    //     {
    //         recognizer_state = myDELETE;//删除
    //         xQueueSend(xQueuevirtualKeystate, &recognizer_state, portMAX_DELAY);
    //     }
    //    else if (AI_Mode == COLOR_AI)//颜色识别
    //    {
    //         mydetector_state = myDECREASE_COLOR_AREA;
    //         xQueueSend(xQueuevirtualKeystate, &mydetector_state, portMAX_DELAY);
    //    }
    printf("keydata:4\r\n");
    // }
    

}


void Get_key_data(void)
{
    //printf("key data coming\r\n");
    if(g_shote_flag == 1)
    {
        g_shote_flag = 0;

        Key_data_upgrade(key0_data_flag);

        sprintf((char*)key_state,"key_down!! flag:%d\r\n",key0_data_flag);
        Uart_Send_Data(key_state,strlen((char*)key_state)); 

    }
    else if (g_double_flag == 1)
    {
        g_double_flag = 0;

        key0_data_flag -=1; //双击减一
        if(key0_data_flag>4 || key0_data_flag ==0)
        {
            key0_data_flag = 4;
        } 

        sprintf((char*)key_state,"key_double!! \r\n");
        Uart_Send_Data(key_state,strlen((char*)key_state)); 

    }
    else if(g_long_flag == 1)
    {
        g_long_flag = 0;

        key0_data_flag = (key0_data_flag+1)%5; //1-4 //长按增1

        if(key0_data_flag == 0)
        {
            key0_data_flag = 1;
        }
        
        sprintf((char*)key_state,"key_long_down!! \r\n");
        Uart_Send_Data(key_state,strlen((char*)key_state)); 
    }
}



void Key_Init(const QueueHandle_t key_state_i)
{
    //引入虚拟按键 virtual
    xQueuevirtualKeystate = key_state_i; 

    Key_GPIO_Init();

    xTaskCreatePinnedToCore(Key_Task, "Key_Task", 2*1024, NULL, configMAX_PRIORITIES, NULL, 1); //任务在核心1运行

    //此任务不开了，会导致一些模块异常
    //xTaskCreatePinnedToCore(Key_User_Task, "Key_User_Task", 2*1024, NULL, configMAX_PRIORITIES, NULL, 1);//任务在核心1运行
    
}
