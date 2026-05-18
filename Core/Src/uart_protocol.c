#include "uart_protocol.h"
#include "platform_hal.h"

// ====== 内部接收状态（static 隐藏实现细节） ======
static volatile uint8_t  g_mv4_color    = 0;
static volatile int16_t  g_mv4_px       = 0;
static volatile int16_t  g_mv4_py       = 0;
static volatile uint8_t  g_mv4_updated  = 0;

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

// ====== 帧解析（在 USART1 中断中调用） ======
uint8_t UART_FeedByte(uint8_t byte) {
    if (byte == FRAME_HEAD) {
        g_rx_idx = 0;
    }

    if (g_rx_idx < FRAME_LENGTH) {
        g_rx_buf[g_rx_idx++] = byte;
    }

    if (g_rx_idx >= FRAME_LENGTH) {
        g_rx_idx = 0;

        if (g_rx_buf[FRAME_IDX_TAIL] != FRAME_TAIL) {
            return 0;
        }

        uint8_t recv_crc = g_rx_buf[FRAME_IDX_CRC];
        uint8_t calc_crc = CRC8_Calc(g_rx_buf, FRAME_DATA_LEN);
        if (recv_crc != calc_crc) {
            return 0;
        }

        g_mv4_color = g_rx_buf[FRAME_IDX_COLOR];
        g_mv4_px    = (int16_t)(g_rx_buf[FRAME_IDX_XL] | (uint16_t)(g_rx_buf[FRAME_IDX_XH] << 8));
        g_mv4_py    = (int16_t)(g_rx_buf[FRAME_IDX_YL] | (uint16_t)(g_rx_buf[FRAME_IDX_YH] << 8));
        g_mv4_updated = 1;

        return 1;
    }

    return 0;
}

// ====== 中断安全读取（禁用中断防止竞态条件） ======
uint8_t UART_ReadFrame(uint8_t* color, int16_t* px, int16_t* py) {
    uint8_t updated;
    uint8_t c;
    int16_t x, y;

    __disable_irq();
    updated = g_mv4_updated;
    if (updated) {
        c = g_mv4_color;
        x = g_mv4_px;
        y = g_mv4_py;
        g_mv4_updated = 0;
    }
    __enable_irq();

    if (updated) {
        *color = c;
        *px = x;
        *py = y;
    }

    return updated;
}

void UARTProtocol_Init(void) {
    g_rx_idx = 0;
    g_mv4_updated = 0;
}
