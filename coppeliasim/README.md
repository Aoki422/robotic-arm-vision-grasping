# CoppeliaSim 可视化仿真说明

本目录只提供可视化概念验证，不作为真实机械臂或真实传感器验证依据。

## 两种模式

### 1. 纯 PC 逻辑仿真

```bash
python coppeliasim/visual_grasping.py --mode pc --attempts 20
```

特点：

- 不需要 CoppeliaSim。
- 不需要真实相机、机械臂、串口或传感器。
- 使用 `config/sim_params.ini` 中的工作区、连杆和精度参数。
- 输出定位误差和抓取成功率统计。

### 2. CoppeliaSim 可视化仿真

```bash
python coppeliasim/visual_grasping.py --mode coppeliasim
```

场景对象命名要求：

```text
/UR5_target
/UR5_tip
/UR5_joint1
/UR5_joint2
/UR5_joint3
/UR5_joint4
/UR5_joint5
/UR5_joint6
/VisionSensor
RG2 gripper with RG2_open signal
```

依赖：

```bash
pip install coppeliasim-zmqremoteapi-client opencv-python numpy
```

该模式会把视觉目标转换成与 Core 侧相同概念的“目标点 -> IK检查 -> 接近 -> 抓取 -> 抬升 -> 回Home”流程，但 UR5+RG2 与真实 ZX30D 舵机臂不是同一硬件模型。
