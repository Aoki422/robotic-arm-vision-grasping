# 机械臂视觉抓取系统 — STM32实战方案设计

## 概述

基于中国大学生工程实践与创新能力大赛（工训大赛）标准，使用已有硬件实现低成本、可落地的视觉识别+机械臂自动抓取系统。

## 硬件架构

```
MV4 H7 Plus（俯拍固定）
     │
     │ UART  →  颜色ID + 像素坐标(px, py)
     ▼
STM32（F1/F4系列）
     │
     │ PWM × 6路 → 舵机驱动板
     ▼
ZX30D舵机 × 6（六轴机械臂 + 夹爪）
     │
     └── USB串口 ←→ PC（调试/参数配置）
```

### 已有硬件
- 六轴机械臂结构件
- ZX30D舵机 × 6
- STM32控制板（F1/F4系列）
- MV4 H7 Plus 智能视觉识别模块

### 需采购硬件
- 舵机电源（12V/10A+开关电源）
- 舵机驱动板（STM32 servo shield 或 PCA9685方案）
- 夹爪（舵机控制夹爪）
- 被抓取物（红/绿/蓝色方块或圆柱）

### 可选硬件
- USB转TTL调试模块
- 0.96寸OLED I2C显示屏（状态显示）

## 视觉识别方案

### MV4 H7 Plus 配置
1. 俯拍固定安装（距桌面30-40cm），确保画面覆盖抓取区域
2. 配置颜色识别：红色、绿色、蓝色三通道
3. MV4输出数据格式（UART）：

```
帧头(0xAA) + 颜色ID(1B) + 坐标X(2B) + 坐标Y(2B) + 校验(1B) + 帧尾(0x55)
```

### 识别流程
- MV4自行完成：图像采集 → 颜色识别 → 像素坐标计算
- STM32只需：接收UART数据 → 解析 → 坐标转换
- 无需OpenCV、无需深度学习

## 坐标标定方案

### 四点仿射变换（标准工训做法）
1. 桌面上放4个标定点（A4纸四角或棋盘格四点）
2. 用MV4读每个点的像素坐标 (u_i, v_i)
3. 用尺子量每个点在机械臂基坐标下的实际坐标 (x_i, y_i)
4. 求解仿射变换矩阵 M（2×3）：

```
[x]   [a  b  c] [u]
[y] = [d  e  f] [v]
                [1]
```

5. 运行时：
```
world_x = a*u + b*v + c;
world_y = d*u + e*v + f;
```

### 优化方向（后续）
- 扩展为9点标定（3×3网格）减少误差
- 加入运行中自检：偏差超阈值触发微调

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

### 逆运动学
使用简化4轴几何法 + 2轴固定姿态：

- **J1**（底座）：由目标点方向角决定
- **J2 + J3**（大臂+小臂）：由余弦定理求解（二连杆模型）
- **J4**（腕部）：保持末端水平（J4 = 90° - (J2 + J3)）
- **J5 + J6**：固定姿态或简单调节

### 运动插值

初始版本使用**时间归一化线性插值**（优于固定delay）：

```
// 将一次运动分成N步，每步计算当前进度占比
for (step = 0; step <= N; step++) {
    t = (float)step / N;                // 进度 0.0 → 1.0
    for (i = 0; i < 6; i++) {
        cur[i] = start[i] + (target[i] - start[i]) * t;
    }
    Servo_SetAll(cur);
    delay(10);
}
```

优化方向（后续）：
- 三次多项式插值（S曲线），减少机械冲击
- 非阻塞延时循环，保证主循环可同时处理其他数据

### 安全高度机制
抓取路径加入缓冲高度：
```
Home → Approach_Point（目标上方5cm安全高度）
     → Grasp_Point（缓慢下降至桌面）
     → 夹爪闭合
     → Approach_Point（提升）
     → Place_Approach（放置区上方）
     → Place_Point（放下）
     → 夹爪张开
     → Home
```

## STM32代码架构

### 文件结构
```
Core/
├── Src/
│   ├── main.c                // CubeMX生成，主循环
│   ├── servo_control.c       // 舵机PWM控制
│   ├── kinematics.c          // 逆运动学
│   └── calibration.c         // 坐标标定
├── Inc/
│   ├── servo_control.h
│   ├── kinematics.h
│   └── calibration.h
```

### 核心接口

```c
// servo_control.c
void Servo_Init(void);                                // 初始化PWM
void Servo_SetAngle(int id, int angle);               // 设置单舵机角度0-180
void Servo_SetAll(int angles[6]);                     // 设置全部6舵机
void Servo_MoveTo(int target[6], int steps, int ms);  // 插值移动到目标

// kinematics.c
int IK_Solve(float x, float y, float z, float roll, int angles[6]);
// 输入：桌面坐标(X,Y)，抓取高度Z，末端姿态roll
// 输出：6个舵机角度
// 返回：0=成功，-1=不可达

// calibration.c
void Calib_Init(void);                                // 初始化标定参数
void Calib_PixelToWorld(int px, int py, float* wx, float* wy);
// 像素坐标 → 世界坐标（四点仿射变换）

// main.c 主流程
void Grasp_Sequence(int color_id, int px, int py) {
    int angles[6];
    float wx, wy;

    // 坐标转换
    Calib_PixelToWorld(px, py, &wx, &wy);

    // 逆解
    if (IK_Solve(wx, wy, GRASP_Z, 0, angles) != 0) {
        // 不可达，跳过
        return;
    }

    // 1. 移动至目标上方
    angles[4] = 90;  // 夹爪朝下
    Servo_MoveTo(angles, 50, 20);

    // 2. 下降至桌面
    angles[3] += 15; // 腕部微调
    Servo_MoveTo(angles, 20, 30);

    // 3. 闭合夹爪
    Servo_SetAngle(6, CLOSE_ANGLE);

    // 4. 提升
    angles[3] -= 15;
    Servo_MoveTo(angles, 20, 20);

    // 5. 移动至放置区
    int place_angles[6];
    IK_Solve(PLACE_X, PLACE_Y, PLACE_Z, 0, place_angles);
    Servo_MoveTo(place_angles, 50, 20);

    // 6. 张开夹爪
    Servo_SetAngle(6, OPEN_ANGLE);

    // 7. 回到Home
    Servo_MoveTo(home_angles, 30, 20);
}
```

### UART数据接收（中断方式）

```c
// USART1 接收中断
uint8_t rx_buffer[6];   // 帧头+颜色+XL+XH+YL+YH
uint8_t rx_index = 0;

void USART1_IRQHandler(void) {
    uint8_t byte = USART1->DR;
    if (byte == 0xAA) {
        rx_index = 0;   // 帧头，重新计数
    }
    rx_buffer[rx_index++] = byte;
    if (rx_index >= 6) {
        // 收到完整数据帧，解析
        int color_id = rx_buffer[1];
        int px = rx_buffer[2] | (rx_buffer[3] << 8);
        int py = rx_buffer[4] | (rx_buffer[5] << 8);
        // 触发抓取（设置标志位，主循环中处理）
        grasp_pending = 1;
        grasp_color = color_id;
        grasp_px = px;
        grasp_py = py;
        rx_index = 0;
    }
}
```

## 安全与异常处理

- 逆解失败时停止动作，返回Home
- 设置舵机角度软限位（防止机械碰撞）
- 抓取超时保护（某一步卡住超过设定时间则复位）
- 夹爪力控制（通过PWM脉宽限制最大力矩）
- 抓取失败重试机制（最多重试2次）

## 项目改造方案

对桌面已有 `robotic-arm-vision-grasping` 项目：

1. **删除**现有 C++ 模板代码（`src/main.cpp`, `src/vision_grasping.cpp`, `include/vision_grasping.h`, `examples/`）
2. **删除**原有的 CMakeLists.txt、README.md、QUICKSTART.md
3. **新建** STM32 项目文件（`Core/Src/`, `Core/Inc/`）
4. **更新** README.md 为新硬件方案描述

## 逐步实施计划

### 第1周：硬件搭建
1. 组装机械臂结构，固定MV4俯拍支架
2. 接线：STM32 PWM输出 → 舵机驱动板 → 6个ZX30D
3. 接线：MV4 TX → STM32 USART1 RX（共地）
4. 配12V电源给舵机驱动板

### 第2周：单体测试
5. CubeMX建工程：配TIM1/2输出6路PWM，配USART1接收
6. 写 Servo_Init + Servo_SetAngle，逐个舵机标定角度范围
7. 用USB转TTL工具观察MV4输出数据，验证UART接收

### 第3周：核心算法
8. 实现四点标定（临时在PC上算矩阵系数，硬编码进STM32）
9. 实现几何逆解（二连杆 + 余弦定理）
10. 实现 Servo_MoveTo 插值运动

### 第4周：联调
11. 全流程跑通：MV4识别 → UART → STM32 → 坐标转换 → 逆解 → 舵机动作
12. 调整标定参数和运动速度
13. 测试不同位置、不同颜色物体的抓取成功率

## 后续优化方向

- 运动平滑：三次多项式/S型轨迹插值
- 标定精度：四点→九点，加入误差自检
- 数据校验：UART帧加入CRC8校验
- 状态机架构：抓取序列改为状态机，提高可扩展性
- 非阻塞延时：用定时器中断替代delay，提高实时性
- PC调试工具：USB串口通信，实时调整参数和监控状态
