#ifndef UART_PROTOCOL_H
#define UART_PROTOCOL_H

#include <stdint.h>

// MV4 数据帧格式
// 帧头(0xAA) + 颜色ID(1B) + X_L(1B) + X_H(1B) + Y_L(1B) + Y_H(1B) + CRC8(1B) + 帧尾(0x55)
#define FRAME_HEAD    0xAA
#define FRAME_TAIL    0x55
#define FRAME_LENGTH  8

// 颜色ID定义（根据MV4实际配置）
#define COLOR_RED     1
#define COLOR_GREEN   2
#define COLOR_BLUE    3
#define COLOR_YELLOW  4

// 最新接收到的抓取指令（volatile 供中断和主循环共享）
extern volatile uint8_t  g_mv4_color;
extern volatile int16_t  g_mv4_px;
extern volatile int16_t  g_mv4_py;
extern volatile uint8_t  g_mv4_updated;

// 初始化串口接收状态
void UART_Init(void);

// CRC8 计算（多项式 0x31，初始 0xFF）
uint8_t CRC8_Calc(const uint8_t* data, uint8_t len);

// 帧解析（由USART1中断每收到一个字节调用一次）
// 返回 1=收到完整有效帧，0=未完成或无效
uint8_t UART_FeedByte(uint8_t byte);

#endif
