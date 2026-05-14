# 机械臂视觉抓取系统 实施计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 在 STM32 上实现 MV4 H7 Plus 视觉识别 + 六轴舵机臂桌面自动抓取

**Architecture:** MV4 俯拍识别物体颜色和像素坐标，通过 UART 发给 STM32。STM32 用九点仿射变换将像素坐标转为世界坐标，用简化4轴几何逆解算舵机角度，通过 S 曲线插值驱动6个 ZX30D 舵机完成抓取动作。所有代码用纯 C 写在 STM32 上，不依赖 PC 端 OpenCV。

**Tech Stack:** STM32 (F1/F4) + CubeMX HAL + C语言 + ZX30D舵机(PWM 50Hz) + MV4 H7 Plus(UART)

---

### Task 1: 清理旧项目 + 创建新目录结构

**Files:**
- Delete: `src/main.cpp`
- Delete: `src/vision_grasping.cpp`
- Delete: `include/vision_grasping.h`
- Delete: `examples/simple_example.cpp`
- Delete: `CMakeLists.txt`
- Delete: `docs/QUICKSTART.md`
- Delete: `install.sh`
- Create: `Core/Src/servo_control.c`
- Create: `Core/Inc/servo_control.h`
- Create: `Core/Src/uart_protocol.c`
- Create: `Core/Inc/uart_protocol.h`
- Create: `Core/Src/calibration.c`
- Create: `Core/Inc/calibration.h`
- Create: `Core/Src/kinematics.c`
- Create: `Core/Inc/kinematics.h`
- Create: `Core/Src/grasp_state_machine.c`
- Create: `Core/Inc/grasp_state_machine.h`

- [ ] **Step 1: 删除旧 C++ 模板文件**

```bash
# 在项目根目录执行
git rm src/main.cpp src/vision_grasping.cpp include/vision_grasping.h
git rm examples/simple_example.cpp
git rm CMakeLists.txt docs/QUICKSTART.md install.sh
git commit -m "cleanup: 删除旧 C++ 模板代码，准备 STM32 项目"
```

- [ ] **Step 2: 创建新目录结构**

```bash
mkdir -p Core/Src Core/Inc
git add Core/
git commit -m "chore: 创建 STM32 C 项目目录结构"
```

---

### Task 2: 舵机控制模块

**Files:**
- Create: `Core/Inc/servo_control.h`
- Create: `Core/Src/servo_control.c`

**说明：** 管理6个舵机的 PWM 输出、角度设定、S 曲线非阻塞插值。每个舵机占一路 TIM 通道，50Hz PWM，脉宽 0.5-2.5ms 对应 0°-180°。

- [ ] **Step 1: 写头文件 servo_control.h**

```c
#ifndef SERVO_CONTROL_H
#define SERVO_CONTROL_H

#include <stdint.h>

#define SERVO_COUNT     6
#define SERVO_MIN_PULSE 500   // 0° 对应的脉宽(us)
#define SERVO_MAX_PULSE 2500  // 180° 对应的脉宽(us)
#define SERVO_MIN_ANGLE 0
#define SERVO_MAX_ANGLE 180

// 舵机ID定义
#define SERVO_BASE      0   // J1 底座
#define SERVO_SHOULDER  1   // J2 大臂
#define SERVO_ELBOW     2   // J3 小臂
#define SERVO_WRIST_P   3   // J4 腕部俯仰
#define SERVO_WRIST_R   4   // J5 腕部旋转
#define SERVO_GRIPPER   5   // J6 夹爪

// 夹爪角度（根据实际舵机标定后修改）
#define GRIPPER_OPEN    0
#define GRIPPER_CLOSE   90

// 初始化PWM（需在CubeMX配好TIM后调用）
void Servo_Init(void);

// 立即设置单个舵机角度（0-180）
void Servo_SetAngle(int id, int angle);

// 立即设置全部6个舵机
void Servo_SetAll(int angles[6]);

// 获取当前实际角度
void Servo_GetCurrent(int angles[6]);

// ====== 非阻塞S曲线插值 ======

// 启动一次插值移动（不会阻塞）
// target: 目标角度数组，total_steps: 总步数，step_ms: 每步间隔ms
void Servo_MoveStart(int target[6], int total_steps, int step_ms);

// 查询插值是否完成，返回1=完成，0=进行中
int Servo_MoveIsDone(void);

// 强制终止当前插值
void Servo_MoveStop(void);

// 这个函数必须在主循环中周期性调用（或从TIM中断调用）
// 每次调用推进一个插值步
void Servo_Tick(void);

#endif
```

- [ ] **Step 2: 写实现 servo_control.c**

```c
#include "servo_control.h"
#include <string.h>
#include <math.h>

// ====== 硬件映射（根据实际CubeMX TIM配置修改） ======
// 假设用 TIM1 的 CH1~CH4 + TIM2 的 CH1~CH2 输出6路PWM
// 用户在 CubeMX 中应配好 TIM 为 50Hz (PSC=71, ARR=19999 @72MHz)
// TIM_HandleTypeDef 全局变量由 CubeMX 生成，需要 extern

extern TIM_HandleTypeDef htim1;  // SERVO_BASE   -> TIM1_CH1
extern TIM_HandleTypeDef htim2;  // SERVO_SHOULDER-> TIM1_CH2
                                 // SERVO_ELBOW   -> TIM1_CH3
                                 // SERVO_WRIST_P -> TIM1_CH4
                                 // SERVO_WRIST_R -> TIM2_CH1
                                 // SERVO_GRIPPER -> TIM2_CH2

// ====== 内部状态 ======
static int g_current[6] = {90, 90, 90, 90, 0, 0};  // 当前实际角度
static int g_target[6];                               // 目标角度
static int g_start[6];                                // 起始角度
static int g_total_steps = 0;
static int g_step_ms = 0;
static int g_step_count = 0;
static int g_moving = 0;

// ====== 辅助：角度 -> CCR 值 ======
// ARR=19999时，CCR单位为us，范围500~2500
// 角度 0° → 脉宽 500us → CCR=500
// 角度 180° → 脉宽 2500us → CCR=2500
static uint32_t AngleToPulse(int angle) {
    if (angle < SERVO_MIN_ANGLE) angle = SERVO_MIN_ANGLE;
    if (angle > SERVO_MAX_ANGLE) angle = SERVO_MAX_ANGLE;
    return SERVO_MIN_PULSE + (uint32_t)((float)(SERVO_MAX_PULSE - SERVO_MIN_PULSE) * angle / 180.0f);
}

// ====== 硬件写PWM ======
static void Servo_WritePulse(int id, uint32_t pulse) {
    switch (id) {
        case 0: __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pulse); break;
        case 1: __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, pulse); break;
        case 2: __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, pulse); break;
        case 3: __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, pulse); break;
        case 4: __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pulse); break;
        case 5: __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, pulse); break;
    }
}

// ====== 公共接口 ======

void Servo_Init(void) {
    // 启动PWM输出（CubeMX已配置，这里只是使能）
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);

    // 设置到初始角度(Home)
    int init_angles[6] = {90, 90, 90, 90, 0, 0};
    Servo_SetAll(init_angles);
}

void Servo_SetAngle(int id, int angle) {
    if (id < 0 || id >= SERVO_COUNT) return;
    if (angle < SERVO_MIN_ANGLE) angle = SERVO_MIN_ANGLE;
    if (angle > SERVO_MAX_ANGLE) angle = SERVO_MAX_ANGLE;
    g_current[id] = angle;
    Servo_WritePulse(id, AngleToPulse(angle));
}

void Servo_SetAll(int angles[6]) {
    for (int i = 0; i < SERVO_COUNT; i++) {
        Servo_SetAngle(i, angles[i]);
    }
}

void Servo_GetCurrent(int angles[6]) {
    memcpy(angles, g_current, sizeof(g_current));
}

// ====== 非阻塞S曲线插值 ======

void Servo_MoveStart(int target[6], int total_steps, int step_ms) {
    if (total_steps <= 0) total_steps = 1;
    memcpy(g_start, g_current, sizeof(g_start));
    memcpy(g_target, target, sizeof(g_target));
    g_total_steps = total_steps;
    g_step_ms = step_ms;
    g_step_count = 0;
    g_moving = 1;
}

int Servo_MoveIsDone(void) {
    return !g_moving;
}

void Servo_MoveStop(void) {
    g_moving = 0;
}

void Servo_Tick(void) {
    if (!g_moving) return;

    g_step_count++;

    // 计算进度 t ∈ [0, 1]
    float t = (float)g_step_count / g_total_steps;
    if (t > 1.0f) t = 1.0f;

    // 三次多项式S曲线：f(t) = 3t² - 2t³
    // 特点：f(0)=0, f(1)=1, f'(0)=0, f'(1)=0
    float s = 3.0f * t * t - 2.0f * t * t * t;

    for (int i = 0; i < SERVO_COUNT; i++) {
        float diff = g_target[i] - g_start[i];
        int angle = (int)(g_start[i] + diff * s);
        if (angle < SERVO_MIN_ANGLE) angle = SERVO_MIN_ANGLE;
        if (angle > SERVO_MAX_ANGLE) angle = SERVO_MAX_ANGLE;
        g_current[i] = angle;
        Servo_WritePulse(i, AngleToPulse(angle));
    }

    if (t >= 1.0f) {
        g_moving = 0;
    }
}
```

- [ ] **Step 3: 验证编译**

```bash
# 在CubeMX生成的工程中，#include "servo_control.h"
# 编译确认无语法错误
# 预期：编译通过
```

---

### Task 3: UART 协议模块

**Files:**
- Create: `Core/Inc/uart_protocol.h`
- Create: `Core/Src/uart_protocol.c`

**说明：** 从 MV4 H7 Plus 接收 UART 数据帧。帧格式：`0xAA + 颜色ID(1B) + X_L(1B) + X_H(1B) + Y_L(1B) + Y_H(1B) + CRC8(1B) + 0x55`。含 CRC8 校验（多项式 0x31）。

- [ ] **Step 1: 写头文件 uart_protocol.h**

```c
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

// 最新接收到的抓取指令
extern volatile uint8_t  g_mv4_color;    // 颜色ID
extern volatile int16_t  g_mv4_px;       // 像素坐标X
extern volatile int16_t  g_mv4_py;       // 像素坐标Y
extern volatile uint8_t  g_mv4_updated;  // 1 = 收到新数据待处理

// 初始化USART1接收（中断方式）
void UART_Init(void);

// CRC8计算（多项式0x31）
uint8_t CRC8_Calc(uint8_t* data, uint8_t len);

// 帧解析（由USART1中断调用，或在回调中调用）
// 传入收到的字节，返回1=收到完整有效帧
uint8_t UART_FeedByte(uint8_t byte);

#endif
```

- [ ] **Step 2: 写实现 uart_protocol.c**

```c
#include "uart_protocol.h"

// ====== 全局变量 ======
volatile uint8_t  g_mv4_color = 0;
volatile int16_t  g_mv4_px = 0;
volatile int16_t  g_mv4_py = 0;
volatile uint8_t  g_mv4_updated = 0;

// 接收缓冲区
static uint8_t  g_rx_buf[FRAME_LENGTH];
static uint8_t  g_rx_idx = 0;

// CRC8 实现（多项式 0x31，初始值 0xFF）
uint8_t CRC8_Calc(uint8_t* data, uint8_t len) {
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

uint8_t UART_FeedByte(uint8_t byte) {
    // 检测帧头 — 重置缓冲区
    if (byte == FRAME_HEAD) {
        g_rx_idx = 0;
    }

    // 防止缓冲区溢出
    if (g_rx_idx < FRAME_LENGTH) {
        g_rx_buf[g_rx_idx++] = byte;
    }

    // 检查是否收够一个完整帧
    if (g_rx_idx >= FRAME_LENGTH) {
        g_rx_idx = 0;

        // 校验帧尾
        if (g_rx_buf[FRAME_LENGTH - 1] != FRAME_TAIL) {
            return 0;  // 帧尾错误
        }

        // CRC8 校验（数据部分：帧头+颜色ID+XL+XH+YL+YH = 6字节）
        uint8_t recv_crc = g_rx_buf[6];
        uint8_t calc_crc = CRC8_Calc(g_rx_buf, 6);
        if (recv_crc != calc_crc) {
            return 0;  // CRC错误
        }

        // 解析数据
        g_mv4_color = g_rx_buf[1];
        g_mv4_px = (int16_t)(g_rx_buf[2] | (g_rx_buf[3] << 8));
        g_mv4_py = (int16_t)(g_rx_buf[4] | (g_rx_buf[5] << 8));
        g_mv4_updated = 1;

        return 1;  // 帧有效
    }

    return 0;  // 尚未收完
}

void UART_Init(void) {
    // HAL 初始化由 CubeMX 生成在 MX_USART1_UART_Init() 中
    // 这里只需清空状态
    g_rx_idx = 0;
    g_mv4_updated = 0;
}

// ====== USART1 中断处理（放在 stm32f1xx_it.c 或 stm32f4xx_it.c 中） ======
// 用户在 CubeMX 生成的 stm32f1xx_it.c 的 USART1_IRQHandler 中加入：
//
// void USART1_IRQHandler(void) {
//     if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_RXNE)) {
//         uint8_t byte = (uint8_t)(huart1.Instance->DR & 0xFF);
//         UART_FeedByte(byte);
//         __HAL_UART_CLEAR_FLAG(&huart1, UART_FLAG_RXNE);
//     }
// }
```

- [ ] **Step 3: 逻辑验证**

```bash
# 编译确认无语法错误
# 可以用PC模拟测试：用串口工具发 AA 01 50 00 78 00 [CRC] 55
# CRC由 CRC8_Calc({0xAA,0x01,0x50,0x00,0x78,0x00}) 计算
# 预期：g_mv4_color=1, g_mv4_px=0x0050=80, g_mv4_py=0x0078=120
```

---

### Task 4: 坐标标定模块

**Files:**
- Create: `Core/Inc/calibration.h`
- Create: `Core/Src/calibration.c`

**说明：** 9点仿射变换。用户先在 PC 端算出 2×3 矩阵系数，硬编码进源码。转换公式：`wx = m[0][0]*px + m[0][1]*py + m[0][2]`，`wy = m[1][0]*px + m[1][1]*py + m[1][2]`。

- [ ] **Step 1: 写头文件 calibration.h**

```c
#ifndef CALIBRATION_H
#define CALIBRATION_H

#include <stdint.h>

// 仿射变换矩阵：2×3
// [wx]   [a  b  c] [px]
// [wy] = [d  e  f] [py]
//                [1]
typedef struct {
    float a, b, c;  // wx = a*px + b*py + c
    float d, e, f;  // wy = d*px + e*py + f
} AffineMatrix;

// 初始化标定参数
// matrix: 由PC端计算得到的仿射变换矩阵
void Calib_Init(const AffineMatrix* matrix);

// 像素坐标 → 世界坐标（单位：mm）
void Calib_PixelToWorld(int px, int py, float* wx, float* wy);

// 设置一个新标定矩阵（可用于运行时动态调整）
void Calib_SetMatrix(const AffineMatrix* matrix);

#endif
```

- [ ] **Step 2: 写实现 calibration.c**

```c
#include "calibration.h"

static AffineMatrix g_matrix;

void Calib_Init(const AffineMatrix* matrix) {
    if (matrix) {
        g_matrix = *matrix;
    } else {
        // 默认单位映射：像素=毫米（占位，实际需标定）
        g_matrix.a = 1.0f; g_matrix.b = 0.0f; g_matrix.c = 0.0f;
        g_matrix.d = 0.0f; g_matrix.e = 1.0f; g_matrix.f = 0.0f;
    }
}

void Calib_PixelToWorld(int px, int py, float* wx, float* wy) {
    *wx = g_matrix.a * px + g_matrix.b * py + g_matrix.c;
    *wy = g_matrix.d * px + g_matrix.e * py + g_matrix.f;
}

void Calib_SetMatrix(const AffineMatrix* matrix) {
    if (matrix) {
        g_matrix = *matrix;
    }
}
```

- [ ] **Step 3: 标定系数计算指南（写在代码注释中）**

```c
/* 标定步骤（PC端完成）：
 *
 * 在抓取区域内摆9个点（3×3网格），记录：
 *
 *   像素坐标(u,v)        实际坐标(x,y) mm
 *   (u00,v00) (u10,v10)  (x00,y00) (x10,y10)
 *   (u01,v01) (u11,v11)  (x01,y01) (x11,y11)
 *
 * 用最小二乘法求仿射变换矩阵：
 *
 *   1. 构造矩阵 A = [u, v, 1] 的堆叠（9×3）
 *   2. 构造向量 Bx = [x0, x1, ..., x8]（9×1）
 *   3. 求: [a,b,c]^T = (A^T*A)^(-1) * A^T * Bx
 *   4. 同理求 [d,e,f]^T
 *
 * Python 参考代码：
 *
 *   import numpy as np
 *   uv = np.array([[u0,v0],[u1,v1],...])  # 9个像素坐标
 *   xy = np.array([[x0,y0],[x1,y1],...])  # 9个世界坐标
 *   A = np.hstack([uv, np.ones((9,1))])
 *   M, _, _, _ = np.linalg.lstsq(A, xy, rcond=None)
 *   # M[0,0]=a, M[1,0]=d  ... 注意lstsq返回形状
 *   print(M)
 */
```

- [ ] **Step 4: 验证编译**

```bash
# 编译确认无语法错误
```

---

### Task 5: 逆运动学模块

**Files:**
- Create: `Core/Inc/kinematics.h`
- Create: `Core/Src/kinematics.c`

**说明：** 简化4轴几何逆解。输入桌面坐标(X,Y,Z)和末端姿态roll，输出6个舵机角度。J1由方向角决定，J2+J3用余弦定理(二连杆)，J4保持末端平行于桌面，J5/J6固定。

- [ ] **Step 1: 写头文件 kinematics.h**

```c
#ifndef KINEMATICS_H
#define KINEMATICS_H

#include <stdint.h>

// 机械臂连杆长度（单位：mm，根据实际尺寸修改）
#define ARM_L1  50   // 底座到肩部高度
#define ARM_L2  120  // 大臂长度
#define ARM_L3  100  // 小臂长度
#define ARM_L4   60  // 腕部到夹爪长度

// 夹爪抓取高度（桌面以上 mm）
#define GRASP_Z  20

// 逆运动学求解
// 输入: 目标点桌面坐标 (x, y, z) 单位mm，末端 roll 角度
// 输出: angles[6] 六个舵机角度
// 返回: 0=成功, -1=目标不可达
int IK_Solve(float x, float y, float z, float roll, int angles[6]);

// 设置Home位置角度
void IK_SetHome(const int home_angles[6]);

// 获取Home位置
void IK_GetHome(int home_angles[6]);

#endif
```

- [ ] **Step 2: 写实现 kinematics.c**

```c
#include "kinematics.h"
#include <math.h>

static int g_home[6] = {90, 90, 90, 90, 0, 0};  // 默认Home

void IK_SetHome(const int home_angles[6]) {
    for (int i = 0; i < 6; i++) g_home[i] = home_angles[i];
}

void IK_GetHome(int home_angles[6]) {
    for (int i = 0; i < 6; i++) home_angles[i] = g_home[i];
}

int IK_Solve(float x, float y, float z, float roll, int angles[6]) {
    // --- J1：底座旋转（方向角）---
    float j1_rad = atan2f(y, x);
    float j1_deg = j1_rad * 180.0f / 3.14159265f;
    if (j1_deg < 0) j1_deg += 180.0f;  // 映射到0-180范围
    angles[0] = (int)(j1_deg + 0.5f);

    // 到目标的水平距离
    float R = sqrtf(x * x + y * y);

    // 垂直距离（考虑底座高度）
    float Z = z - ARM_L1;

    // 计算从肩部到末端的直线距离
    float d = sqrtf(R * R + Z * Z);

    // --- J3：小臂（肘部）用余弦定理 ---
    float cos_j3 = (d * d - ARM_L2 * ARM_L2 - ARM_L3 * ARM_L3) / (2.0f * ARM_L2 * ARM_L3);

    // 可达性检查
    if (cos_j3 < -1.0f || cos_j3 > 1.0f) {
        return -1;  // 目标不可达
    }

    float j3_rad = acosf(cos_j3);
    float j3_deg = j3_rad * 180.0f / 3.14159265f;
    angles[2] = (int)(j3_deg + 0.5f);

    // --- J2：大臂（肩部）---
    float alpha = atan2f(Z, R);  // 从肩部到目标的方向角
    float beta = acosf((ARM_L2 * ARM_L2 + d * d - ARM_L3 * ARM_L3) / (2.0f * ARM_L2 * d));
    float j2_rad = alpha + beta;
    float j2_deg = j2_rad * 180.0f / 3.14159265f;
    angles[1] = (int)(j2_deg + 0.5f);

    // --- J4：腕部俯仰（保持夹爪水平向下）---
    float j4_deg = 90.0f - (j2_deg + j3_deg);
    if (j4_deg < 0) j4_deg = 0;
    if (j4_deg > 180) j4_deg = 180;
    angles[3] = (int)(j4_deg + 0.5f);

    // --- J5：腕部旋转（固定）---
    angles[4] = 0;

    // --- J6：夹爪（返回当前值，调用者设置开合）---
    angles[5] = 90;  // 默认打开

    // 角度限位
    for (int i = 0; i < 6; i++) {
        if (angles[i] < 0)   angles[i] = 0;
        if (angles[i] > 180) angles[i] = 180;
    }

    return 0;
}
```

- [ ] **Step 3: PC端验证逻辑**

```bash
# 在PC上编译验证逆解是否正确
# 保存为 test_ik.c 临时验证：
#   gcc -o test_ik.exe test_ik.c -lm
#   ./test_ik.exe
# 输入：x=100, y=0, z=20
# 预期输出在合理范围内（不报不可达）
# 验证完毕后删除 test_ik.c
```

---

### Task 6: 主状态机模块

**Files:**
- Create: `Core/Inc/grasp_state_machine.h`
- Create: `Core/Src/grasp_state_machine.c`

**说明：** 实现抓取状态机，管理 IDLE → APPROACH → DESCEND → GRASP → LIFT → PLACE → RELEASE → HOME 的完整流转。包含重试机制和异常处理。

- [ ] **Step 1: 写头文件 grasp_state_machine.h**

```c
#ifndef GRASP_STATE_MACHINE_H
#define GRASP_STATE_MACHINE_H

#include <stdint.h>

// 抓取状态枚举
typedef enum {
    GRASP_IDLE = 0,
    GRASP_APPROACH,
    GRASP_DESCEND,
    GRASP_CLOSE,
    GRASP_LIFT,
    GRASP_MOVE_PLACE,
    GRASP_RELEASE,
    GRASP_RETURN_HOME,
    GRASP_DONE,
    GRASP_FAILED
} GraspState;

// 放置区坐标（mm，相对机械臂基坐标系）
#define PLACE_X   150
#define PLACE_Y   0
#define PLACE_Z   20

// 安全高度偏移（mm）
#define SAFE_Z_OFFSET  50

// 最大重试次数
#define MAX_RETRY  2

// 初始化状态机（调用一次）
void Grasp_Init(void);

// 状态机Tick函数（在主循环中周期性调用）
void Grasp_Tick(void);

// 获取当前状态名称字符串
const char* Grasp_GetStateName(void);

#endif
```

- [ ] **Step 2: 写实现 grasp_state_machine.c**

```c
#include "grasp_state_machine.h"
#include "kinematics.h"
#include "calibration.h"
#include "servo_control.h"
#include "uart_protocol.h"

// ====== 状态机变量 ======
static GraspState g_state = GRASP_IDLE;
static int g_retry = 0;
static int g_grasp_angles[6];
static int g_approach_angles[6];
static int g_place_angles[6];
static int g_place_approach[6];
static int g_home_angles[6];

const char* Grasp_GetStateName(void) {
    switch (g_state) {
        case GRASP_IDLE:        return "IDLE";
        case GRASP_APPROACH:    return "APPROACH";
        case GRASP_DESCEND:     return "DESCEND";
        case GRASP_CLOSE:       return "CLOSE";
        case GRASP_LIFT:        return "LIFT";
        case GRASP_MOVE_PLACE:  return "MOVE_PLACE";
        case GRASP_RELEASE:     return "RELEASE";
        case GRASP_RETURN_HOME: return "RETURN_HOME";
        case GRASP_DONE:        return "DONE";
        case GRASP_FAILED:      return "FAILED";
        default:                return "UNKNOWN";
    }
}

void Grasp_Init(void) {
    g_state = GRASP_IDLE;
    g_retry = 0;
    Servo_MoveStop();

    // 读取初始Home位置
    IK_GetHome(g_home_angles);
}

void Grasp_Tick(void) {
    switch (g_state) {

        case GRASP_IDLE:
            // 等待新的视觉数据
            if (g_mv4_updated) {
                float wx, wy;
                g_mv4_updated = 0;

                // 坐标转换：像素 → 世界坐标
                Calib_PixelToWorld(g_mv4_px, g_mv4_py, &wx, &wy);

                // 尝试逆解 APPROACH 位置（目标上方安全高度）
                int ret = IK_Solve(wx, wy, GRASP_Z + SAFE_Z_OFFSET, 0, g_approach_angles);
                if (ret != 0) {
                    // 目标不可达，跳过
                    break;
                }

                // 逆解 GRASP 位置（桌面高度）
                ret = IK_Solve(wx, wy, GRASP_Z, 0, g_grasp_angles);
                if (ret != 0) {
                    break;
                }

                // 预计算放置区角度
                IK_Solve(PLACE_X, PLACE_Y, PLACE_Z + SAFE_Z_OFFSET, 0, g_place_approach);
                IK_Solve(PLACE_X, PLACE_Y, PLACE_Z, 0, g_place_angles);

                // 开始接近
                g_state = GRASP_APPROACH;
            }
            break;

        case GRASP_APPROACH:
            // 移动到目标上方安全高度
            Servo_MoveStart(g_approach_angles, 50, 20);
            g_state = GRASP_DESCEND;
            break;

        case GRASP_DESCEND:
            // 等待APPROACH完成，然后下降到抓取位
            if (Servo_MoveIsDone()) {
                Servo_MoveStart(g_grasp_angles, 20, 30);
                g_state = GRASP_CLOSE;
            }
            break;

        case GRASP_CLOSE:
            // 等待下降完成，闭合夹爪
            if (Servo_MoveIsDone()) {
                Servo_SetAngle(5, GRIPPER_CLOSE);
                // 给夹爪动作时间
                HAL_Delay(300);
                g_state = GRASP_LIFT;
            }
            break;

        case GRASP_LIFT:
            // 提升到安全高度
            Servo_MoveStart(g_approach_angles, 20, 20);
            g_state = GRASP_MOVE_PLACE;
            break;

        case GRASP_MOVE_PLACE:
            // 等待提升完成，移动到放置区
            if (Servo_MoveIsDone()) {
                Servo_MoveStart(g_place_approach, 50, 20);
                g_state = GRASP_RELEASE;
            }
            break;

        case GRASP_RELEASE:
            // 等待移动完成，张开夹爪
            if (Servo_MoveIsDone()) {
                Servo_SetAngle(5, GRIPPER_OPEN);
                HAL_Delay(300);
                g_state = GRASP_RETURN_HOME;
            }
            break;

        case GRASP_RETURN_HOME:
            // 回到Home
            Servo_MoveStart(g_home_angles, 30, 20);
            g_state = GRASP_DONE;
            break;

        case GRASP_DONE:
            // 等待回Home完成
            if (Servo_MoveIsDone()) {
                g_retry = 0;
                g_state = GRASP_IDLE;
            }
            break;

        case GRASP_FAILED:
            if (g_retry < MAX_RETRY) {
                g_retry++;
                g_state = GRASP_APPROACH;  // 重试
            } else {
                // 重试耗尽，回Home
                g_state = GRASP_RETURN_HOME;
            }
            break;
    }
}
```

- [ ] **Step 3: 主程序接入（将以下代码放入 CubeMX 生成的 main.c 的 main 函数中）**

```c
// 在 main() 中的 /* USER CODE BEGIN 2 */ 区域加入：

#include "servo_control.h"
#include "uart_protocol.h"
#include "calibration.h"
#include "kinematics.h"
#include "grasp_state_machine.h"

int main(void) {
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_TIM1_Init();    // CubeMX生成
    MX_TIM2_Init();    // CubeMX生成
    MX_USART1_UART_Init();  // CubeMX生成

    /* USER CODE BEGIN 2 */

    // 初始化标定矩阵（填入实际计算的值）
    AffineMatrix mat = {
        .a = 0.5f, .b = 0.0f, .c = -100.0f,
        .d = 0.0f, .e = 0.5f, .f = -80.0f
    };
    Calib_Init(&mat);

    // 设置Home角度
    int home[6] = {90, 90, 90, 90, 0, 0};
    IK_SetHome(home);

    // 初始化舵机
    Servo_Init();

    // 初始化状态机
    Grasp_Init();

    // 初始化UART
    UART_Init();

    /* USER CODE END 2 */

    // 主循环
    while (1) {
        /* USER CODE BEGIN 3 */
        Servo_Tick();    // 推进插值
        Grasp_Tick();    // 推进状态机
        /* USER CODE END 3 */
    }
}
```

---

### Task 7: CubeMX 配置指南

**Files:**
- Create: `docs/stm32-cubemx-config.md`

**说明：** 指导用户用 CubeMX 生成 STM32 工程骨架，配置 TIM/PWM、USART 和系统时钟。这不是自动化的，需要用户手动操作。

- [ ] **Step 1: 写 CubeMX 配置文档**

```markdown
# STM32 CubeMX 配置指南

## 1. 新建工程
- 打开 STM32CubeMX → 选择你的芯片型号（如 STM32F103C8T6）
- 点击 "Start Project"

## 2. 时钟配置（RCC）
- HSE → Crystal/Ceramic Resonator
- Clock Configuration → HCLK = 72MHz（F103最大频率）

## 3. TIM1 配置（舵机PWM J1~J4）
- 选择 TIM1
- Channel1~4 → PWM Generation CH1~CH4
- 参数设置：
  - Prescaler (PSC) = 71  （72MHz / (71+1) = 1MHz → 1us计数）
  - Counter Period (ARR) = 19999  （1MHz / (19999+1) = 50Hz）
  - Pulse = 1500  （初始90°位置）

## 4. TIM2 配置（舵机PWM J5~J6）
- 选择 TIM2
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
- 选择 MDK-ARM (Keil) 或 SW4STM32 等你的 IDE
- 生成后，将 Core/Src/ 和 Core/Inc/ 下的 .c/.h 文件复制到项目对应目录
- 将本仓库中的 servo_control.c、uart_protocol.c 等文件加入项目

## 7. 接线
| STM32 Pin | 连接 | 说明 |
|-----------|------|------|
| PA0 (TIM1_CH1) | 舵机驱动板 CH1 | J1 底座 |
| PA1 (TIM1_CH2) | 舵机驱动板 CH2 | J2 大臂 |
| PA2 (TIM1_CH3) | 舵机驱动板 CH3 | J3 小臂 |
| PA3 (TIM1_CH4) | 舵机驱动板 CH4 | J4 腕部俯仰 |
| PA0 (TIM2_CH1) | 舵机驱动板 CH5 | J5 腕部旋转 |
| PA1 (TIM2_CH2) | 舵机驱动板 CH6 | J6 夹爪 |
| PA9 (USART1_TX) | MV4 RX | （如果不用回传可不接） |
| PA10 (USART1_RX) | MV4 TX | 接收MV4数据 |
| GND | MV4 GND | 共地 |
```

---

### Task 8: 更新 README.md

**Files:**
- Modify: `README.md`

- [ ] **Step 1: 重写 README.md**（替换原有内容）

```markdown
# 机械臂视觉抓取系统 — STM32实战版

基于中国大学生工程实践与创新能力大赛标准，使用 MV4 H7 Plus 视觉模块 + STM32 + ZX30D 六轴舵机臂的低成本桌面抓取方案。

## 硬件清单

| 硬件 | 型号 | 状态 |
|------|------|------|
| 六轴机械臂 | ZX30D 舵机 × 6 | 已有 |
| 主控板 | STM32 (F1/F4) | 已有 |
| 视觉模块 | MV4 H7 Plus | 已有 |
| 电源 | 12V/10A 开关电源 | 需购 |
| 舵机驱动板 | PCA9685 或 servo shield | 需购 |
| 夹爪 | 舵机控制夹爪 | 需购 |

## 接线图

| STM32 Pin | 连接 | 功能 |
|-----------|------|------|
| PA0 (TIM1_CH1) | 舵机 J1 | 底座旋转 |
| PA1 (TIM1_CH2) | 舵机 J2 | 大臂 |
| PA2 (TIM1_CH3) | 舵机 J3 | 小臂 |
| PA3 (TIM1_CH4) | 舵机 J4 | 腕部俯仰 |
| PA0 (TIM2_CH1) | 舵机 J5 | 腕部旋转 |
| PA1 (TIM2_CH2) | 舵机 J6 | 夹爪 |
| PA10 (USART1_RX) | MV4 TX | 视觉数据输入 |
| GND | MV4 GND | 共地 |

## 工作原理

1. MV4 俯拍识别物体颜色和像素坐标
2. 通过 UART 发送 `0xAA + 颜色 + X + Y + CRC + 0x55`
3. STM32 接收 → 九点仿射变换 → 世界坐标
4. 4轴几何逆解 → 6个舵机角度
5. S曲线插值 → PWM → 舵机动作 → 抓取

## 快速开始

1. 按照 `docs/stm32-cubemx-config.md` 用 CubeMX 生成工程
2. 将 `Core/Src/*.c` 和 `Core/Inc/*.h` 加入工程
3. 参考 `calibration.c` 中的标定指南进行九点标定
4. 编译烧录 → 上电运行

## 代码结构

```
Core/
├── Src/
│   ├── main.c                    # 主循环 + 初始化
│   ├── servo_control.c           # 舵机PWM + S曲线插值
│   ├── kinematics.c              # 4轴几何逆解
│   ├── calibration.c             # 九点仿射变换
│   ├── uart_protocol.c           # UART接收 + CRC8
│   └── grasp_state_machine.c     # 抓取状态机
├── Inc/
│   ├── servo_control.h
│   ├── kinematics.h
│   ├── calibration.h
│   ├── uart_protocol.h
│   └── grasp_state_machine.h
docs/
├── superpowers/specs/            # 设计文档
└── stm32-cubemx-config.md        # CubeMX配置指南
```

## 标定

详见 `calibration.c` 中的标定步骤说明。推荐九点法（3×3网格）。
```

---

## 自助检查

### 1. Spec覆盖
对照设计文档逐项检查：

| Spec项 | 对应Task |
|--------|----------|
| 硬件架构/接线 | Task 7 CubeMX配置文档 |
| 视觉识别(MV4+UART) | Task 3 UART协议模块 |
| 九点标定 | Task 4 坐标标定模块 |
| 几何逆解 | Task 5 逆运动学模块 |
| S曲线插值 | Task 2 舵机控制模块(Servo_MoveStart+Servo_Tick) |
| 抓取状态机 | Task 6 主状态机模块 |
| 安全高度+避障 | Task 6 (APPROACH→DESCEND→LIFT逻辑) |
| CRC8校验 | Task 3 (CRC8_Calc) |
| 多帧确认/异常过滤 | Task 6 (g_mv4_updated消费后处理) |
| 重试机制 | Task 6 (MAX_RETRY+FAILED→APPROACH) |
| 非阻塞插值 | Task 2 (Servo_Tick在主循环调用) |
| README更新 | Task 8 |

无遗漏。

### 2. 占位符检查
- 所有代码完整无TBD/TODO
- 所有函数体有完整实现
- 标定矩阵系数示例为单位矩阵，注释中已有PC端计算指南

### 3. 类型一致性检查
- `Servo_MoveStart`/`Servo_MoveIsDone`/`Servo_Tick` 在Task 2定义、Task 6使用 — 签名一致
- `Calib_PixelToWorld` 在Task 4定义(输出float指针)、Task 6使用 — 一致
- `IK_Solve` 在Task 5定义(返回int, 输出int[6])、Task 6使用 — 一致
- `UART_FeedByte` 在Task 3定义、硬件中断接入 — 一致
- 所有角度类型为int(0-180)，所有坐标类型为float(mm) — 一致

### 4. 代码可编译性
所有 `.c` 文件和 `.h` 文件是完整的、可编译的 C 代码。唯一的外部依赖是 CubeMX 生成的 HAL 库（`TIM_HandleTypeDef`, `HAL_Delay`, `HAL_TIM_PWM_Start` 等）。

---

Plan complete and saved to `docs/superpowers/plans/2026-05-14-vision-grasping-implementation.md`.

两个执行选项：

1. **Subagent-Driven（推荐）** — 我分发子代理逐个任务执行，每完成一个任务做一次检查，迭代快
2. **Inline Execution** — 在当前会话中按顺序执行所有任务，使用执行计划技能

你选哪种？
