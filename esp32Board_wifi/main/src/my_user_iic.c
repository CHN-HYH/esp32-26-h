#include "stdio.h"
#include "my_user_iic.h"
#include "string.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "freertos/ringbuf.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "my_usart.h"
#include "yahboom_camera.h"


//这个是从机i2c文件 -- 还要优化速度方面的
#define DATA_LENGTH 1024

static const char *IICTAG = "MY_IIC";
static IIC_REG_Data IIC_Data={0,0,320,240,160,120,0,0};//信息结构体

static QueueHandle_t xQueuevirtualKeystate = NULL; //虚拟按键

static myrecognizer_state_t recognizer_state = myDETECT;//人脸检测的标志,默认就是检测
static mycolor_detection_state_t mydetector_state = myCOLOR_DETECTION_IDLE;//颜色识别的标志

//i2c接收的数据 {'\0'}; //这个都没有寄存器对上了可言性了
static uint8_t IIC_Salve_data[I2C_SLAVE_TX_BUF_LEN] =  {'\0'};
//{0x01, 0x00, 0x02, 0x00, 0x03,0x00,0x04,0x00,0x05,0x00,0x06,0x00,0x07,0x00,0x08,0x00,0x09,0x00,0x0A,0x00}

//外部引入变量
int vflip_data=0 ; //垂直
int mirror_data = 0;//水平

static int myabs(int a)
{
    if(a<0)
        return a;
    return a; 
     
}

//框的范围
void set_IIC_data(int16_t lx,int16_t ly,int16_t rx,int16_t ry)//设置i2c信息结构体的数据
{
    if(lx < 0 || ly <0 || rx<0 || ry<0)
    {
        return; //为负的，不操作
    }
    
    IIC_Data.msg_lx = lx;
    IIC_Data.msg_ly = ly;
    IIC_Data.msg_rx = rx;
    IIC_Data.msg_ry = ry;

    IIC_Data.msg_mx=(IIC_Data.msg_rx-IIC_Data.msg_lx)/2+IIC_Data.msg_lx;
    IIC_Data.msg_my=(IIC_Data.msg_ry-IIC_Data.msg_ly)/2+IIC_Data.msg_ly;
    IIC_Data.area = (IIC_Data.msg_rx - IIC_Data.msg_lx)*(IIC_Data.msg_ry-IIC_Data.msg_lx);

    if(IIC_Data.area > 320*240)
    {
        IIC_Data.area = 65535; //因为iic只用了 16位的数据
    }
}

//单独设置id
void set_IIC_data_id(int16_t temp_id)
{
    IIC_Data.msg_id = temp_id;
}

// i2c从机接收任务
static void My_IIC_Slave_Task(void *arg)
{
   uint8_t* RXdata = (uint8_t*) malloc(DATA_LENGTH);

    //先把数组x赋值
    memset(IIC_Salve_data,(IIC_Data.msg_mx>>8),I2C_SLAVE_TX_BUF_LEN); //x的高位

    while (1) 
    {
        
        // 读取数据 需要调试
        int size = i2c_slave_read_buffer(MYI2C_SLAVE_NUM, RXdata, DATA_LENGTH, 0);//portTICK_PERIOD_MS
        if (size>1)
        {
            //写入控制操作
           switch_i2c_control(RXdata[0],RXdata[1]);
        }
        else if(size==1)
        {
            //读取预操作
            recv_i2c_data(RXdata[0]);//更新赋值
        }

        vTaskDelay(pdMS_TO_TICKS(10)); //至少10ms 否则无法正常打开摄像头数据
    }
    free(RXdata);
    vTaskDelete(NULL);
}


static void i2c_slave_init(void)
{
    int i2c_slave_port = MYI2C_SLAVE_NUM;
    i2c_config_t conf_slave = {
        .mode = I2C_MODE_SLAVE,
        .sda_io_num = MYI2C_SLAVE_SDA,
        .scl_io_num = MYI2C_SLAVE_SCL,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .slave.addr_10bit_en = 0,
        .slave.slave_addr = Camera_IIC_ADDR, //从机i2c地址  0x33
    };
    i2c_param_config(i2c_slave_port, &conf_slave);
    i2c_driver_install(i2c_slave_port, conf_slave.mode, I2C_SLAVE_RX_BUF_LEN, I2C_SLAVE_TX_BUF_LEN, 0);

}

// 通过i2c发送一串数据 
int I2C_Send_Data(uint8_t* data, uint16_t len)
{
    const int txBytes = i2c_slave_write_buffer(MYI2C_SLAVE_NUM, data, len,1);///portTICK_PERIOD_MS
    //printf("data:=%d \r\n",txBytes);
    return txBytes;
}

// 通过i2c发送一个字节 
int I2C_Send_Byte(uint8_t data)
{
    uint8_t data1 = data;
    const int txBytes = i2c_slave_write_buffer(MYI2C_SLAVE_NUM, &data1, 1,1000/portTICK_PERIOD_MS); //这个是个队列,主机需要才会把这个队列传输过去,怪异i2c
    return txBytes;
}

//i2c 初始化，给外部调用
void My_i2c_init(const QueueHandle_t key_state_i)
{
    //引入虚拟按键 virtual
    xQueuevirtualKeystate = key_state_i; 

    i2c_slave_init();

    xTaskCreate(My_IIC_Slave_Task, "My_IIC_Slave_Task", 5*1024, NULL, 20, NULL);

}



void switch_i2c_control(uint8_t REG_flag,uint8_t data)
{
    sensor_t *s = esp_camera_sensor_get();
    if(REG_flag == Horizontal_IMG) //水平翻转
    {
        if(data>1)
        {
            mirror_data = 1;
        }
        else
        {
            mirror_data = data;
        }
        s->set_hmirror(s, mirror_data);
    }
    else if(REG_flag == Vertical_IMG)//垂直翻转
    {
        if(data>1)
        {
            vflip_data = 1;
        }
        else
        {
            vflip_data = data;
        }
         s->set_vflip(s, vflip_data);
    }
    else if(REG_flag == Model_RESET )//自动复位重启
    {
        if(data == 0x01)
        {
            esp_restart();//自动复位
        }
        
    }



}

extern bool register_mode ;//引入串口0颜色识别的注册标志 使其同步

//数据传输部分
void recv_i2c_data(uint8_t REG_flag)
{
    if (REG_flag == middle_X_High)
    {
        memset(IIC_Salve_data,((IIC_Data.msg_mx >>8) & 0xFF),I2C_SLAVE_TX_BUF_LEN);
    }
    else if (REG_flag == middle_X_LOW)
    {
        memset(IIC_Salve_data,((IIC_Data.msg_mx ) & 0xFF),I2C_SLAVE_TX_BUF_LEN);
    }
    else if (REG_flag == middle_Y_High)
    {
        memset(IIC_Salve_data,((IIC_Data.msg_my>>8) & 0xFF),I2C_SLAVE_TX_BUF_LEN);
    }
    else if (REG_flag == middle_Y_LOW)
    {
        memset(IIC_Salve_data,((IIC_Data.msg_my) & 0xFF),I2C_SLAVE_TX_BUF_LEN);
    }

    else if (REG_flag == Face_ID_High)
    {
        memset(IIC_Salve_data,((IIC_Data.msg_id >>8 ) & 0xFF),I2C_SLAVE_TX_BUF_LEN);
    }
    else if (REG_flag == Face_ID_LOW)
    {
        memset(IIC_Salve_data,((IIC_Data.msg_id) & 0xFF),I2C_SLAVE_TX_BUF_LEN);
    }
    else if (REG_flag == Area_High)
    {
        memset(IIC_Salve_data,((IIC_Data.area)>>8 & 0xFF),I2C_SLAVE_TX_BUF_LEN);
    }
    else if (REG_flag == Area_LOW)
    {
        memset(IIC_Salve_data,((IIC_Data.area) & 0xFF),I2C_SLAVE_TX_BUF_LEN);
    }



    I2C_Send_Data(IIC_Salve_data,I2C_SLAVE_TX_BUF_LEN); 

}
