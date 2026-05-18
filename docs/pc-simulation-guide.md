# 无实机 PC 仿真运行指南

本文档是当前项目的主运行路径，所有步骤都不需要真实硬件。

## 1. 安装 PC 编译工具链

任选一种：

- CMake + GCC
- CMake + Clang
- Visual Studio Build Tools 的 `cl`

如果只运行 Python 纯逻辑仿真，则不需要 C 编译器；如果要跑 C 单元测试，需要 C 编译器。

## 2. 运行核心逻辑单元测试

在项目根目录执行：

```bash
python tests/run_tests.py
```

测试覆盖：

- UART 正常帧解析
- CRC 错误帧拒绝
- 帧尾错误拒绝
- 虚拟 9 点标定矩阵求解
- 重投影误差验证
- IK 可达目标求解
- IK 工作区外目标拒绝
- 抓取状态机最小闭环

## 3. 运行纯 PC 逻辑仿真

```bash
python coppeliasim/visual_grasping.py --mode pc --attempts 20
```

输出包含：

- 虚拟像素坐标
- 世界坐标
- 定位误差
- IK 接受/拒绝结果
- 抓取成功率统计

## 4. 运行 CoppeliaSim 可视化仿真

1. 打开 CoppeliaSim。
2. 加载包含 UR5、RG2、VisionSensor 的场景。
3. 确保对象命名符合 `coppeliasim/README.md`。
4. 运行：

```bash
python coppeliasim/visual_grasping.py --mode coppeliasim
```

## 5. 修改仿真参数

编辑：

```text
config/sim_params.ini
```

可改内容：

- `arm_l1_mm` ~ `arm_l4_mm`：连杆长度与末端偏移
- `workspace_*`：工作区范围，默认 200mm x 150mm
- `positioning_tolerance_mm`：定位精度阈值，默认 ±5mm
- `target_confirm_frames`：目标多帧确认数量
- `joint_zero_deg`：舵机零位
- `joint_direction`：舵机方向
- `joint_soft_min_deg` / `joint_soft_max_deg`：关节软限位

## 6. 当前无法无实机验证的内容

真实舵机力矩、夹爪接触力、真实 MV4 成像噪声、供电压降、机械结构间隙无法在无硬件条件下被真实验证。

替代方案：

- 用 PC 单元测试验证核心算法边界。
- 用 CoppeliaSim 验证视觉抓取概念。
- 用 `config/sim_params.ini` 注入噪声、工作区和关节限制，做逻辑压力测试。
