#include "uart_protocol.h"

// ====== 全局变量（volatile 跨中断/主循环共享） ======
volatile uint8_t  g_mv4_color    = 0;
volatile int16_t  g_mv4_px       = 0;
volatile int16_t  g_mv4_py       = 0;
volatile uint8_t  g_mv4_updated  = 0;

// ====== 内部接收缓冲区 ======
static uint8_t  g_rx_buf[FRAME_LENGTH];
static uint8_t  g_rx_idx = 0;

// ====== CRC8 实现（多项式 0x31） ======
uint8_t CRC8_Calc(const uint8_t* data, uint8_t len) {
    uint8_t crc = 0xFF;
    for (uint8_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x80) {
                crc = (crc << 1) ^ 0x31;
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

// ====== 帧解析 ======
uint8_t UART_FeedByte(uint8_t byte) {
    // 检测帧头 — 重置接收位置
    if (byte == FRAME_HEAD) {
        g_rx_idx = 0;
    }

    // 防止缓冲区溢出
    if (g_rx_idx < FRAME_LENGTH) {
        g_rx_buf[g_rx_idx++] = byte;
    }

    // 检查是否累积了一个完整帧
    if (g_rx_idx >= FRAME_LENGTH) {
        g_rx_idx = 0;  // 无论有效无效，重置接收状态

        // 校验帧尾
        if (g_rx_buf[FRAME_LENGTH - 1] != FRAME_TAIL) {
            return 0;  // 帧尾不匹配
        }

        // CRC8 校验（数据部分：帧头+颜色+XL+XH+YL+YH = 6字节）
        uint8_t recv_crc = g_rx_buf[6];
        uint8_t calc_crc = CRC8_Calc(g_rx_buf, 6);
        if (recv_crc != calc_crc) {
            return 0;  // CRC 错误
        }

        // 解析有效数据
        g_mv4_color = g_rx_buf[1];
        g_mv4_px    = (int16_t)(g_rx_buf[2] | (uint16_t)(g_rx_buf[3] << 8));
        g_mv4_py    = (int16_t)(g_rx_buf[4] | (uint16_t)(g_rx_buf[5] << 8));
        g_mv4_updated = 1;

        return 1;  // 有效帧
    }

    return 0;  // 还在收
}

void UART_Init(void) {
    g_rx_idx = 0;
    g_mv4_updated = 0;
}
