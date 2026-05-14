#ifndef UART_PROTOCOL_H
#define UART_PROTOCOL_H

#include <stdint.h>

// MV4 数据帧格式
// 帧头(0xAA) + 颜色ID(1B) + X_L(1B) + X_H(1B) + Y_L(1B) + Y_H(1B) + CRC8(1B) + 帧尾(0x55)
#define FRAME_HEAD    0xAA
#define FRAME_TAIL    0x55
#define FRAME_LENGTH  8

// 帧内偏移（避免硬编码魔法数字）
#define FRAME_IDX_HEAD    0
#define FRAME_IDX_COLOR   1
#define FRAME_IDX_XL      2
#define FRAME_IDX_XH      3
#define FRAME_IDX_YL      4
#define FRAME_IDX_YH      5
#define FRAME_IDX_CRC     6
#define FRAME_IDX_TAIL    7
#define FRAME_DATA_LEN    6   // CRC 覆盖的字节数（头+颜色+XL+XH+YL+YH）

// 颜色ID定义（根据MV4实际配置）
#define COLOR_RED     1
#define COLOR_GREEN   2
#define COLOR_BLUE    3
#define COLOR_YELLOW  4

// 初始化串口接收状态
void UARTProtocol_Init(void);

// CRC8 计算（多项式 0x31，初始 0xFF）
uint8_t CRC8_Calc(const uint8_t* data, uint8_t len);

// 帧解析（由USART1中断每收到一个字节调用一次）
uint8_t UART_FeedByte(uint8_t byte);

// 中断安全地读取最新帧（返回 1=有新数据，0=无新数据）
// 内部禁用 USART1 中断以防止 ISR 与主循环的竞争条件
uint8_t UART_ReadFrame(uint8_t* color, int16_t* px, int16_t* py);

#endif
