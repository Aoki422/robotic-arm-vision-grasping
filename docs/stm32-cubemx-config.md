# STM32 CubeMX 配置指南

## 1. 新建工程
- 打开 STM32CubeMX
- 选择芯片型号（如 STM32F103C8T6）
- 点击 "Start Project"

## 2. 时钟配置（RCC）
- HSE → Crystal/Ceramic Resonator
- Clock Configuration → HCLK = 72MHz（F103最大频率）

## 3. TIM2 配置（舵机PWM J1~J4）
- 选择 TIM2
- Channel1~4 → PWM Generation CH1~CH4
- 参数设置：
  - Prescaler (PSC) = 71 （72MHz / (71+1) = 1MHz → 1us计数）
  - Counter Period (ARR) = 19999 （1MHz / (19999+1) = 50Hz）
  - Pulse = 1500 （初始90°位置：脉宽1500us）

## 4. TIM3 配置（舵机PWM J5~J6）
- 选择 TIM3
- Channel1~2 → PWM Generation CH1~CH2
- 参数同上：PSC=71, ARR=19999

## 5. USART1 配置（MV4通信）
- 选择 USART1
- Mode → Asynchronous
- Baud Rate = 115200
- Word Length = 8
- Parity = None
- Stop Bits = 1
- NVIC Settings → 勾选 USART1 global interrupt

## 6. 生成代码
- Project → Generate Code
- 选择 MDK-ARM (Keil) 或 SW4STM32（根据你的IDE）
- 生成后，将本仓库的 Core/Src/*.c 和 Core/Inc/*.h 加入项目

## 7. USART1 中断处理

在 CubeMX 生成的 `stm32f1xx_it.c`（或 stm32f4xx_it.c）中找到 `USART1_IRQHandler`，加入：

```c
void USART1_IRQHandler(void) {
    if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_RXNE)) {
        uint8_t byte = (uint8_t)(huart1.Instance->DR & 0xFF);
        UART_FeedByte(byte);
        __HAL_UART_CLEAR_FLAG(&huart1, UART_FLAG_RXNE);
    }
}
```

## 8. 主程序 main.c

在 CubeMX 生成的 main.c 中，在 /* USER CODE BEGIN 2 */ 区域加入初始化：

```c
#include "servo_control.h"
#include "uart_protocol.h"
#include "calibration.h"
#include "kinematics.h"
#include "grasp_state_machine.h"

int main(void) {
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_TIM2_Init();    // J1-J4
    MX_TIM3_Init();    // J5-J6
    MX_USART1_UART_Init();

    // ====== 用户初始化 ======
    AffineMatrix mat = {
        .a = 0.5f, .b = 0.0f, .c = -100.0f,
        .d = 0.0f, .e = 0.5f, .f = -80.0f
    };
    Calib_Init(&mat);

    int home[6] = {90, 90, 90, 90, 0, 0};
    IK_SetHome(home);

    Servo_Init();
    Grasp_Init();
    UARTProtocol_Init();

    while (1) {
        Servo_Tick();
        Grasp_Tick();
    }
}
```

## 9. 接线表

| STM32 Pin | 功能 | 连接 |
|-----------|------|------|
| PA0 (TIM2_CH1) | J1 底座 | 舵机驱动板 CH1 |
| PA1 (TIM2_CH2) | J2 大臂 | 舵机驱动板 CH2 |
| PA2 (TIM2_CH3) | J3 小臂 | 舵机驱动板 CH3 |
| PA3 (TIM2_CH4) | J4 腕部俯仰 | 舵机驱动板 CH4 |
| PA6 (TIM3_CH1) | J5 腕部旋转 | 舵机驱动板 CH5 |
| PA7 (TIM3_CH2) | J6 夹爪 | 舵机驱动板 CH6 |
| PA9 (USART1_TX) | 串口发送 | (不用可悬空) |
| PA10 (USART1_RX) | 串口接收 | MV4 TX 输出 |
| GND | 共地 | MV4 GND + 电源GND |

## 10. 舵机脉宽标定

每个ZX30D舵机的0°和180°对应的脉宽可能有差异。在 main.c 中临时加入以下代码进行标定：

```c
// 逐个舵机标定：
for (int i = 0; i < 6; i++) {
    Servo_SetAngle(i, 0);   HAL_Delay(2000);
    Servo_SetAngle(i, 90);  HAL_Delay(2000);
    Servo_SetAngle(i, 180); HAL_Delay(2000);
}
```

观察机械臂的物理活动范围，调整 SERVO_MIN_PULSE/SERVO_MAX_PULSE（在 servo_control.h 中）。
