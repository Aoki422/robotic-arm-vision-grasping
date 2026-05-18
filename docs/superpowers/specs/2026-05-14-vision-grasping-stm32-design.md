# 机械臂视觉抓取系统 — STM32实战方案设计

> 历史存档说明：本文是早期实机方向设计草案，不是当前项目的运行前提。
> 当前主线已切换为无真实硬件的 PC/仿真闭环，运行方式以 `README.md` 和 `docs/pc-simulation-guide.md` 为准。

## 概述

基于中国大学生工程实践与创新能力大赛（工训大赛）标准，使用已有硬件实现低成本、可落地的视觉识别+机械臂自动抓取系统。核心策略：**简单、可靠、不依赖复杂算法**。

## 硬件架构

```
MV4 H7 Plus（俯拍固定）
     │
     │ UART  →  颜色ID + 像素坐标(px, py)
     ▼
STM32（F1/F4系列）
     ├── PWM × 6路 → 舵机驱动板 → ZX30D舵机 × 6
     ├── UART ←→ PC（调试/参数配置）
     └── I2C → OLED（状态显示，可选）
```

### 已有硬件
- 六轴机械臂结构件
- ZX30D舵机 × 6
- STM32控制板
- MV4 H7 Plus 智能视觉识别模块

### 需采购硬件
| 硬件 | 说明 | 必须？ |
|------|------|--------|
| 12V/10A 开关电源 | ZX30D启动电流大，需独立供电。建议加软启动或过流保护 | 必须 |
| 舵机驱动板 | 方案A: STM32 servo shield（直插）方案B: PCA9685(I2C) | 必须 |
| 夹爪 | 舵机控制夹爪（40-80元） | 必须 |
| 彩色方块/圆柱 | 红绿蓝三色，30-50mm | 必须 |

### 可选硬件
| 硬件 | 用途 |
|------|------|
| USB转TTL（带LED指示） | 调试时观察UART通信状态 |
| 0.96寸 OLED I2C | 实时显示坐标和抓取状态 |
| 均匀光源（LED灯板） | 减少环境光对颜色识别干扰 |

### 安装注意事项
- MV4支架建议用可调角度云台，方便快速微调焦距和视角
- 在抓取区域上方加漫反射LED光源，减少阴影和反光
- 确保桌面平坦、高度固定

## 视觉识别方案

### MV4 H7 Plus 配置
1. 俯拍固定安装（距桌面30-40cm，根据视野范围调整）
2. 配置颜色识别3-4通道：红色、绿色、蓝色（+黄色可选）
3. 设置输出频率（建议每秒5-10帧，避免STM32处理不过来）
4. MV4输出数据格式（UART）：

```
帧头(0xAA) + 颜色ID(1B) + 坐标X(2B) + 坐标Y(2B) + CRC8(1B) + 帧尾(0x55)
```

### STM32端颜色容错
- **多帧确认**：同一颜色连续3帧坐标变化在合理范围内才触发抓取，避免闪烁误触发
- **异常值过滤**：像素坐标突变超过阈值（如跳动>50像素）则丢弃该帧
- **无物体超时**：连续5秒未收到有效数据则回到Home待机

## 坐标标定方案

### 九点标定法（推荐，四点法的升级版）

在抓取区域内摆放3×3共9个标定点：

```
像素坐标                    世界坐标（机械臂基坐标系）
(u00, v00)  (u10, v10)  (u20, v20)    (x00,y00)  (x10,y10)  (x20,y20)
(u01, v01)  (u11, v11)  (u21, v21)    (x01,y01)  (x11,y11)  (x21,y21)
(u02, v02)  (u12, v12)  (u22, v22)    (x02,y02)  (x12,y12)  (x22,y22)
```

通过最小二乘法求解仿射变换矩阵 M（2×3）：

```
[x]   [a  b  c] [u]
[y] = [d  e  f] [v]
                [1]
```

运行时转换：
```
world_x = a*u + b*v + c;
world_y = d*u + e*v + f;
```

**标定步骤：**
1. 把机械臂末端移动到9个位置，记录每个位置的实际坐标 (x_i, y_i)
2. 在每个位置，用MV4读出对应的像素坐标 (u_i, v_i)
3. 在PC上用最小二乘算出矩阵系数（或用在线工具计算）
4. 将系数硬编码进STM32代码

**为什么用九点而不是四点：** 九点能平均分布误差，对镜头畸变有一定容忍度——工训大赛实测四点法在视野边缘偏差较大，九点法改善明显。

## 运动控制方案

### 关节配置
| 关节 | 舵机 | 范围 | 说明 |
|------|------|------|------|
| J1 | ZX30D #1 | 0-180° | 底座旋转（水平） |
| J2 | ZX30D #2 | 0-180° | 大臂俯仰 |
| J3 | ZX30D #3 | 0-180° | 小臂俯仰 |
| J4 | ZX30D #4 | 0-180° | 腕部俯仰 |
| J5 | ZX30D #5 | 0-180° | 腕部旋转 |
| J6 | ZX30D #6 | 夹爪开合 | 舵机控制夹爪 |

### 逆运动学——简化4轴几何法
完整6轴逆解计算量太大，工训大赛标准做法是简化为4轴：

- **J1**（底座旋转）：由目标点方向角 atan2(Y, X) 决定
- **J2+J3**（大臂+小臂）：视为平面二连杆，用余弦定理求解
  - 已知连杆长度 L2（大臂）、L3（小臂）和目标点到肩部的距离 R
  - cos(J3) = (R² - L2² - L3²) / (2*L2*L3)
  - J2 = atan2(Y, X) - atan2(L3*sin(J3), L2 + L3*cos(J3))
- **J4**（腕部俯仰）：保持末端夹爪水平朝下
  - J4 = 90° - (J2 + J3)
- **J5**（腕部旋转）：固定0°（或根据物体角度简单调节）
- **J6**（夹爪）：只控制开合，无关节角度

**不可达处理：** 若 IK_Solve 返回不可达（目标超出臂展），自动选择最近可达点或跳过该目标。

### 运动插值——三次多项式S型曲线

为避免线性插值存在的速度突变和机械冲击，直接使用三次多项式插值（S曲线）：

```
// 三次多项式：角度 = a0 + a1*t + a2*t² + a3*t³
// 满足边界条件：t=0时角度=起始，t=1时角度=目标，且起止速度=0
// 解得：
//   a0 = start
//   a1 = 0
//   a2 = 3*(target - start)
//   a3 = -2*(target - start)

void Servo_MoveTo(int target[6], int total_steps, int step_ms) {
    int start[6];
    for (int i = 0; i < 6; i++) start[i] = current_angles[i];

    for (int s = 0; s <= total_steps; s++) {
        float t = (float)s / total_steps;
        for (int i = 0; i < 6; i++) {
            float diff = target[i] - start[i];
            float ang = start[i] + diff * (3*t*t - 2*t*t*t);  // S曲线
            Servo_SetAngle(i, (int)ang);
        }
        delay(step_ms);
    }
}
```

上述 delay 方式适合初版。后续可改为**定时器中断驱动**——每隔 step_ms 触发一次中断，更新一步角度，主循环不被阻塞，可同时处理UART数据和异常检测。

### 安全高度与避障路径

完整抓取路径分7步，每步有明确的安全缓冲：

```
状态         动作                          安全机制
────────────────────────────────────────────────────────
IDLE        等待MV4数据                    舵机切到保持力矩
APPROACH    移动到目标上方+5cm             S曲线平滑
DESCEND     缓慢下降到抓取高度             速度减半，可随时停止
GRASP       闭合夹爪                      限力矩PWM
LIFT        提升到安全高度                 确保夹爪不碰桌面
MOVE_PLACE  移动到放置区上方               路径经过中间避障点
RELEASE     张开夹爪                      确认物体已释放
RETURN_HOME 回到初始待机位置               最后一步
```

**中间避障点：** 当 APPROACH 位置与 HOME 之间有障碍物（或经过工作区边缘），可在路径中插入一个中间高点作为避障中转。

## 软件架构

### 文件结构
```
Core/
├── Src/
│   ├── main.c                // CubeMX生成，主循环
│   ├── servo_control.c       // 舵机PWM控制 + S曲线插值
│   ├── kinematics.c          // 逆运动学
│   ├── calibration.c         // 坐标标定
│   └── uart_protocol.c       // UART接收 + CRC校验
├── Inc/
│   ├── servo_control.h
│   ├── kinematics.h
│   ├── calibration.h
│   └── uart_protocol.h
```

### 抓取状态机

抓取序列用枚举状态机实现，取代顺序函数调用：

```c
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

GraspState g_state = GRASP_IDLE;
int g_retry_count = 0;

void Grasp_Tick(void) {
    switch (g_state) {
        case GRASP_IDLE:
            if (grasp_pending) {
                // 检查是否可到达目标
                float wx, wy;
                Calib_PixelToWorld(grasp_px, grasp_py, &wx, &wy);
                if (IK_Solve(wx, wy, GRASP_Z, 0, arm_angles) != 0) {
                    // 目标不可达，跳过
                    grasp_pending = 0;
                    return;
                }
                g_state = GRASP_APPROACH;
            }
            break;

        case GRASP_APPROACH:
            // 移动到目标上方安全高度
            if (Servo_MoveTo_NonBlock(approach_angles, 50, 20)) {
                g_state = GRASP_DESCEND;
            }
            break;

        case GRASP_DESCEND:
            if (Servo_MoveTo_NonBlock(grasp_angles, 20, 30)) {
                g_state = GRASP_CLOSE;
            }
            break;

        case GRASP_CLOSE:
            Servo_SetAngle(6, CLOSE_ANGLE);
            g_state = GRASP_LIFT;
            break;

        case GRASP_LIFT:
            if (Servo_MoveTo_NonBlock(approach_angles, 20, 20)) {
                g_state = GRASP_MOVE_PLACE;
            }
            break;

        case GRASP_MOVE_PLACE:
            if (Servo_MoveTo_NonBlock(place_angles, 50, 20)) {
                g_state = GRASP_RELEASE;
            }
            break;

        case GRASP_RELEASE:
            Servo_SetAngle(6, OPEN_ANGLE);
            g_state = GRASP_RETURN_HOME;
            break;

        case GRASP_RETURN_HOME:
            if (Servo_MoveTo_NonBlock(home_angles, 30, 20)) {
                g_state = GRASP_DONE;
            }
            break;

        case GRASP_DONE:
            grasp_pending = 0;
            g_retry_count = 0;
            g_state = GRASP_IDLE;
            break;

        case GRASP_FAILED:
            if (g_retry_count < 2) {
                g_retry_count++;
                g_state = GRASP_APPROACH;     // 重试
            } else {
                // 重试耗尽，回Home
                g_state = GRASP_RETURN_HOME;
            }
            break;
    }
}
```

**状态机架构的优势：**
- 每步由定时器驱动的 "Tick" 函数推进，主循环不阻塞
- 容易插入重试、超时、异常处理
- 容易扩展多物体顺序抓取

### UART数据接收

```c
#define RX_BUF_SIZE  16    // 加大缓冲区防丢帧

uint8_t rx_buf[RX_BUF_SIZE];
uint8_t rx_idx = 0;

// CRC8 校验（多项式 0x31）
uint8_t CRC8(uint8_t* data, uint8_t len) {
    uint8_t crc = 0xFF;
    for (uint8_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x80) crc = (crc << 1) ^ 0x31;
            else crc <<= 1;
        }
    }
    return crc;
}

void USART1_IRQHandler(void) {
    uint8_t byte = USART1->DR;

    if (byte == 0xAA) {
        rx_idx = 0;
    }

    if (rx_idx < RX_BUF_SIZE) {
        rx_buf[rx_idx++] = byte;
    }

    // 完整帧：帧头(1) + 颜色(1) + X(2) + Y(2) + CRC(1) + 帧尾(1) = 8字节
    if (rx_idx >= 8 && rx_buf[7] == 0x55) {
        // CRC校验
        uint8_t recv_crc = rx_buf[6];
        uint8_t calc_crc = CRC8(rx_buf, 6);
        if (recv_crc == calc_crc) {
            grasp_color = rx_buf[1];
            grasp_px = rx_buf[2] | (rx_buf[3] << 8);
            grasp_py = rx_buf[4] | (rx_buf[5] << 8);
            grasp_pending = 1;
        }
        rx_idx = 0;
    }
}
```

## 安全与异常处理

| 场景 | 处理方式 |
|------|----------|
| 逆解不可达 | 自动跳过该目标，回Home |
| 舵机角度超限 | 软限位保护，超过设定范围不执行 |
| 夹爪卡住 | 限制PWM脉宽，防止堵转过流 |
| 抓取超时 | 某步动作超过设定时间（如5秒）未完成，视为失败 |
| 连续抓取失败 | 最多重试2次，仍失败则回Home报错 |
| MV4数据中断 | 超过5秒无有效帧，回Home待机 |

## 核心接口

```c
// servo_control.c
void Servo_Init(void);
void Servo_SetAngle(int id, int angle);
void Servo_SetAll(int angles[6]);
int  Servo_MoveTo_NonBlock(int target[6], int total_steps, int step_ms);
// 非阻塞移动，每调用一次推进一步，完成后返回1

// kinematics.c
int IK_Solve(float x, float y, float z, float roll, int angles[6]);
// 输入：(X,Y)桌面坐标，Z抓取高度，roll末端姿态
// 输出：6个舵机角度
// 返回：0=成功，-1=不可达

// calibration.c
void Calib_Init(float matrix[2][3]);
void Calib_PixelToWorld(int px, int py, float* wx, float* wy);
// 像素坐标→世界坐标    wx = matrix[0][0]*px + matrix[0][1]*py + matrix[0][2]

// uart_protocol.c
void UART_Init(void);
uint8_t CRC8(uint8_t* data, uint8_t len);
// USART1_IRQHandler 在 stm32f1xx_it.c 中实现

// main.c
// 主循环：
//   while (1) {
//       Grasp_Tick();           // 状态机推进
//       Servo_Tick();           // 非阻塞插值定时刷新
//       Handle_Debug_CMD();     // 串口调试命令（可选）
//   }
```

## 项目改造方案

对桌面已有 `robotic-arm-vision-grasping` 项目做以下改造：

1. **删除** 现有 C++ 模板代码（`src/`, `include/`, `examples/`, `CMakeLists.txt`）
2. **删除** 旧的文档（`QUICKSTART.md`, `README.md` —— 后续重写）
3. **新建** STM32 项目目录结构（`Core/Src/`, `Core/Inc/`）
4. **书写** 完整代码：`main.c`, `servo_control.c`, `kinematics.c`, `calibration.c`, `uart_protocol.c`
5. **更新** README.md 描述新硬件方案和接线说明

## 逐步实施计划

### 第1周：硬件搭建与测试
1. 组装机械臂结构，安装MV4俯拍支架（可调角度）
2. 配置12V电源、安装舵机驱动板
3. 接线：STM32 PWM → 驱动板 → 6个ZX30D
4. 接线：MV4 TX → STM32 USART1 RX（共地）
5. CubeMX生成最小工程：配6路PWM + 1路UART
6. 写 Servo_Init + Servo_SetAngle，逐个舵机标定0°-180°对应的PWM脉宽

### 第2周：UART通信 + 视觉接入
7. 用USB转TTL监听MV4输出，确认数据格式和帧结构
8. 在STM32上实现 UART_Init + CRC8 + USART1_IRQHandler
9. 验证STM32正确接收MV4数据（用串口打印到PC）

### 第3周：坐标标定 + 逆解 + 插值
10. 在抓取区域放置9个标定点，记录像素坐标和实际坐标
11. 在PC上计算仿射变换矩阵系数，写入 Calib_Init
12. 实现 I K_Solve（余弦定理二连杆）
13. 实现 Servo_MoveTo_NonBlock（三次多项式S曲线）+ 状态机

### 第4周：联调
14. 全流程跑通：MV4 → UART → 坐标转换 → 逆解 → 插值 → 舵机动作
15. 调整标定参数、运动速度、夹爪力度
16. 测试不同位置、不同颜色的抓取成功率
17. 安全机制验证：超时保护、不可达处理、重试机制

## 后续优化方向（按优先级）

1. **非阻塞定时器驱动插值**：用TIM中断替代delay，主循环实时响应
2. **抓取成功率统计**：记录总次数/成功/失败，串口输出
3. **PC调试工具**：PC端发串口命令实时修改参数（速度、角度、标定系数）
4. **多物体顺序抓取**：MV4发多个坐标，状态机逐个处理
5. **自适应夹爪力**：根据物体颜色/大小调节夹持力
6. **动态标定校正**：每次抓取前检测标定板，自动微调矩阵系数
