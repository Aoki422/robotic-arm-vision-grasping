# 机械臂视觉抓取系统 - 纯PC/仿真闭环版

本项目用于在**无真实硬件、无实物机械臂、无真实传感器、无外部硬件链路**的条件下验证机械臂视觉抓取核心逻辑。

当前主流程只依赖 PC 构建、单元测试和 CoppeliaSim 可视化仿真。STM32 相关代码通过 `platform_hal.h` 做接口隔离：PC 模式使用模拟 HAL 桩函数，未来实机适配时再切换到 CubeMX HAL。

## 当前能力

- PC 端可编译核心 C 逻辑，不依赖真实 STM32 HAL。
- 自动化测试覆盖 UART 帧解析、CRC 校验、9点仿射标定、IK 可达/不可达边界和抓取状态机最小闭环。
- 状态机支持多帧确认、目标丢失超时、运动超时、不可达保护、放置区 IK 检查、失败回 Home。
- 运动学支持 ARM_L4 末端偏移、舵机零位/方向映射、关节软限位、正逆运动学回算验证。
- CoppeliaSim 脚本提供两种模式：纯 PC 逻辑仿真、UR5+RG2 可视化仿真。
- 仿真指标包含工作区范围、定位误差阈值、抓取成功率统计。

## 目录结构

```text
Core/
  Inc/                       # 核心模块头文件、PC HAL隔离、仿真配置/指标接口
  Src/                       # 核心逻辑实现
coppeliasim/
  visual_grasping.py         # pc / coppeliasim 两种仿真入口
config/
  sim_params.ini             # 仿真参数配置，不需要修改核心代码
tests/
  test_core.c                # PC端核心逻辑单元测试
  run_tests.py               # 一键测试脚本
docs/
  pc-simulation-guide.md     # 无实机运行指南
```

## 一键运行

### 1. PC 端单元测试

安装任意一种 C 编译工具链后运行：

```bash
python tests/run_tests.py
```

脚本优先使用 CMake；没有 CMake 时会尝试 `gcc`、`clang` 或 `cl`。

### 2. 纯 PC 逻辑仿真

```bash
python coppeliasim/visual_grasping.py --mode pc --attempts 20
```

该模式不需要打开 CoppeliaSim，也不需要任何外设。

### 3. CoppeliaSim 可视化仿真

先打开 CoppeliaSim，加载包含 `/UR5_target`、`/UR5_tip`、`/UR5_joint1..6`、`/VisionSensor`、RG2 的示例场景，然后运行：

```bash
python coppeliasim/visual_grasping.py --mode coppeliasim
```

注意：该模式只验证 UR5+RG2 的视觉抓取概念，不代表真实 ZX30D 舵机臂实机验证。

## 参数修改

修改 [config/sim_params.ini](config/sim_params.ini) 即可调整：

- 连杆长度：`arm_l1_mm` ~ `arm_l4_mm`
- 工作区：默认 200mm x 150mm
- 定位精度阈值：默认 `positioning_tolerance_mm = 5`
- 目标多帧确认：`target_confirm_frames`
- 舵机零位/方向/软限位：`joint_zero_deg`、`joint_direction`、`joint_soft_min_deg`、`joint_soft_max_deg`

## 构建说明

CMake 构建：

```bash
cmake -S . -B build -DPC_SIM=ON
cmake --build build
ctest --test-dir build --output-on-failure
```

Makefile 构建：

```bash
make test
```

## 原则

本仓库当前不要求真实硬件、实物标定或烧录才能运行。所有硬件相关逻辑均通过 `platform_hal.h` 和 PC 模拟桩隔离。
