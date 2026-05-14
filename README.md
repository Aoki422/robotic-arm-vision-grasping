# 机械臂视觉抓取系统 — STM32实战版

基于中国大学生工程实践与创新能力大赛（工训大赛）标准，使用 MV4 H7 Plus 视觉模块 + STM32 + ZX30D 六轴舵机臂的低成本桌面抓取方案。

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

| STM32 Pin | 功能 | 连接 |
|-----------|------|------|
| PA0 (TIM1_CH1) | J1 底座 | 舵机驱动板 CH1 |
| PA1 (TIM1_CH2) | J2 大臂 | 舵机驱动板 CH2 |
| PA2 (TIM1_CH3) | J3 小臂 | 舵机驱动板 CH3 |
| PA3 (TIM1_CH4) | J4 腕部俯仰 | 舵机驱动板 CH4 |
| PA0 (TIM2_CH1) | J5 腕部旋转 | 舵机驱动板 CH5 |
| PA1 (TIM2_CH2) | J6 夹爪 | 舵机驱动板 CH6 |
| PA10 (USART1_RX) | 视觉数据 | MV4 TX 输出 |
| GND | 共地 | MV4 GND + 电源GND |

## 工作原理

1. **MV4 H7 Plus** 俯拍识别物体颜色和像素坐标
2. 通过 UART 发送数据帧：`0xAA + 颜色ID + X坐标 + Y坐标 + CRC8 + 0x55`
3. **STM32** 接收帧 → CRC校验 → 九点仿射变换转世界坐标
4. **4轴几何逆解** → 6个舵机角度
5. **S曲线插值** → PWM → 舵机动作 → 自动抓取

## 快速开始

1. 参考 `docs/stm32-cubemx-config.md` 用 CubeMX 生成工程
2. 将 `Core/Src/*.c` 和 `Core/Inc/*.h` 加入工程
3. 参考 `Core/Src/calibration.c` 中的标定指南进行九点标定
4. 编译烧录 → 上电运行

## 代码结构

```
Core/
├── Src/
│   ├── main.c                    # 主循环 + 初始化
│   ├── servo_control.c           # 舵机PWM + S曲线插值
│   ├── kinematics.c              # 4轴几何逆解
│   ├── calibration.c             # 九点仿射变换标定
│   ├── uart_protocol.c           # UART接收 + CRC8校验
│   └── grasp_state_machine.c     # 抓取状态机
├── Inc/
│   ├── servo_control.h
│   ├── kinematics.h
│   ├── calibration.h
│   ├── uart_protocol.h
│   └── grasp_state_machine.h
docs/
├── superpowers/                  # 设计文档和实施计划
└── stm32-cubemx-config.md        # CubeMX配置指南
```

## 核心算法

### 1. 九点标定（像素→世界坐标）

在抓取区域摆放3×3共9个点，用最小二乘法求解仿射变换矩阵：
```
world_x = a*px + b*py + c
world_y = d*px + e*py + f
```
详见 `Core/Src/calibration.c` 顶部的标定指南。

### 2. 4轴几何逆解

- J1（底座）：由目标点方向角决定
- J2+J3（大臂+小臂）：余弦定理求解二连杆
- J4（腕部）：保持夹爪水平向下
- J5/J6：固定姿态

### 3. S曲线插值

三次多项式 S 曲线保证起止速度为零，减少机械冲击：
```
s = 3t² - 2t³, t ∈ [0, 1]
```

## 标定步骤

1. 把机械臂末端移动到抓取区域9个位置，记录实际坐标
2. 用 MV4 读出每个位置的像素坐标
3. 在 PC 上用 Python/numpy 算出仿射变换矩阵系数
4. 将系数填入 `Calib_Init` 调用中（在 main.c）

## 开发计划

| 阶段 | 内容 |
|------|------|
| 第1周 | 硬件搭建、CubeMX工程、单舵机测试 |
| 第2周 | UART通信、MV4数据接入 |
| 第3周 | 坐标标定、逆解算法、插值运动 |
| 第4周 | 全流程联调、参数优化 |

## 许可证

MIT
