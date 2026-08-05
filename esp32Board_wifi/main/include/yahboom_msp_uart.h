#pragma once

#include <stdbool.h>
#include <stdint.h>

#define YAHBOOM_MSP_UART_PACKET_SIZE 8
#define YAHBOOM_MSP_FLAG_VALID       0x01

#ifdef __cplusplus
extern "C" {
#endif

/*

| 0 | `AA`    | 固定帧头 1      |
| 1 | `55`    | 固定帧头 2      |                                               
| 2 | `FLAGS` | 状态位；`bit0=1` 表示钢珠坐标有效，`bit0=0` 表示当前未识别到钢珠 |
| 3 | `SEQ`   | 序号，每发一帧加 1，`FF` 后回到 `00` |
| 4 | `X_L`   | 钢珠中心 X 坐标低字节 |
| 5 | `X_H`   | 钢珠中心 X 坐标高字节 |
| 6 | `WIDTH` | 当前钢珠候选的横向宽度，单位像素 |
| 7 | `CRC8`  | 对前 7 字节计算的 CRC8 |

*/

// 初始化 MSPM0 数据包发送任务；底层复用已有 UART1 驱动。
void yahboom_msp_uart_init(void);

// 将最新钢珠状态放入发送队列；函数不会等待 UART 发送完成。
bool yahboom_msp_uart_send(bool valid, uint16_t center_x, uint16_t width);

#ifdef __cplusplus
}
#endif
