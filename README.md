# 机械臂视觉抓取系统

一个用C++实现的机械臂视觉抓取模板系统，支持计算机视觉、物体检测、抓取规划和机器人控制。

## 🌟 特性

- **计算机视觉集成**: 使用OpenCV进行图像处理和物体检测
- **模块化设计**: 相机、检测器、抓取规划器、机器人控制器完全解耦
- **易于扩展**: 可以轻松替换不同的检测算法和机器人控制接口
- **模拟环境**: 内置模拟机器人用于测试和开发
- **调试支持**: 可视化工具帮助调试抓取过程
- **跨平台**: 支持Windows、Linux和macOS

## 📋 系统要求

### 必需依赖
- **C++ 编译器**: GCC 7+ / Clang 5+ / MSVC 2017+
- **CMake**: 版本 3.10 或更高
- **OpenCV**: 版本 4.0 或更高

### 可选依赖
- **深度学习框架**: 用于高级物体检测（如YOLO、SSD等）
- **ROS/ROS2**: 用于与真实机器人通信
- **PCL**: 点云处理库

## 🚀 快速开始

### 1. 克隆仓库

```bash
git clone https://github.com/your-username/robotic-arm-vision-grasping.git
cd robotic-arm-vision-grasping
```

### 2. 安装依赖

#### Ubuntu/Debian
```bash
sudo apt-get update
sudo apt-get install build-essential cmake
sudo apt-get install libopencv-dev
```

#### macOS
```bash
brew install cmake
brew install opencv
```

#### Windows
1. 下载并安装 [CMake](https://cmake.org/download/)
2. 下载并安装 [OpenCV](https://opencv.org/releases/)

### 3. 编译项目

```bash
mkdir build
cd build
cmake ..
make
```

或者在Windows上使用Visual Studio：
```bash
mkdir build
cd build
cmake ..
# 使用Visual Studio打开生成的解决方案文件
```

### 4. 运行程序

```bash
./vision_grasping --help
```

## 📖 使用方法

### 基本用法

```bash
# 使用默认设置运行
./vision_grasping

# 启用调试模式
./vision_grasping --debug

# 指定相机ID
./vision_grasping --camera 1

# 执行指定次数的抓取
./vision_grasping --number 5

# 设置抓取间隔
./vision_grasping --interval 2000
```

### 命令行参数

| 参数 | 说明 | 默认值 |
|------|------|--------|
| `-h, --help` | 显示帮助信息 | - |
| `-d, --debug` | 启用调试模式 | false |
| `-c, --camera ID` | 指定相机ID | 0 |
| `-n, --number NUM` | 执行NUM次抓取 | 10 |
| `-i, --interval MS` | 每次抓取间隔毫秒数 | 1000 |

### 示例

#### 调试模式运行
```bash
./vision_grasping --debug
```

#### 使用第二个相机，执行5次抓取
```bash
./vision_grasping --camera 1 --number 5
```

#### 高级调试
```bash
./vision_grasping --debug --number 3 --interval 500
```

## 🏗️ 项目结构

```
robotic-arm-vision-grasping/
├── CMakeLists.txt          # CMake构建配置
├── README.md              # 项目说明文档
├── .gitignore             # Git忽略文件配置
├── include/               # 头文件目录
│   └── vision_grasping.h  # 主要接口定义
├── src/                   # 源代码目录
│   ├── main.cpp           # 主程序入口
│   └── vision_grasping.cpp # 主要实现
├── examples/              # 示例代码
│   └── simple_example.cpp # 简单使用示例
├── docs/                  # 文档目录
├── data/                  # 数据目录
└── build/                 # 编译输出目录
```

## 🎯 核心模块

### 1. 相机模块 (Camera)
- 处理视频流的捕获
- 支持多种相机接口
- 图像预处理功能

### 2. 视觉检测模块 (VisionDetector)
- 基础物体检测接口
- 简单Blob检测器实现
- 易于扩展支持深度学习模型

### 3. 抓取规划模块 (GraspPlanner)
- 生成抓取候选点
- 评估抓取质量
- 支持多种抓取策略

### 4. 机器人控制模块 (RobotController)
- 标准机器人控制接口
- 模拟机器人实现
- 易于适配真实硬件

### 5. 集成系统 (VisionGraspingSystem)
- 整合所有模块
- 协调抓取流程
- 调试和可视化支持

## 🔧 自定义和扩展

### 添加新的检测器

```cpp
#include "vision_grasping.h"

class CustomDetector : public VisionDetector {
public:
    CustomDetector() {
        // 初始化你的检测器
    }

protected:
    std::vector<DetectedObject> detect_impl(const cv::Mat& frame) override {
        // 实现你的检测逻辑
        std::vector<DetectedObject> objects;

        // 你的检测代码...
        // 例如：使用YOLO、SSD等深度学习模型

        return objects;
    }
};
```

### 连接真实机器人

```cpp
#include "vision_grasping.h"

class RealRobot : public RobotController {
public:
    RealRobot(const std::string& ip, int port) {
        // 初始化机器人连接
        // 例如：连接UR机器人、Franka Emika等
    }

    bool connect() override {
        // 实现连接逻辑
        return true;
    }

    bool move_to_position(float x, float y, float z) override {
        // 实现移动逻辑
        return true;
    }

    bool execute_grasp(float width, float force) override {
        // 实现抓取逻辑
        return true;
    }

    bool release_grasp() override {
        // 实现释放逻辑
        return true;
    }

    bool is_connected() const override { return connected_; }
    bool is_ready() const override { return ready_; }
};
```

### 使用自定义组件

```cpp
int main() {
    VisionGraspingSystem system;

    // 使用自定义检测器
    auto custom_detector = std::make_unique<CustomDetector>();
    system.set_detector(std::move(custom_detector));

    // 使用真实机器人
    auto real_robot = std::make_unique<RealRobot>("192.168.1.100", 30003);
    system.set_robot_controller(std::move(real_robot));

    // 启用调试模式
    system.enable_debug_mode(true);

    // 初始化并运行
    if (system.initialize()) {
        system.run_grasping_sequence();
        system.shutdown();
    }

    return 0;
}
```

## 🎓 学习资源

### 相关项目
- [GraspNet-1Billion](https://github.com/graspnet/GraspNet-1Billion) - 大规模抓取数据集
- [AnyGrasp](https://github.com/graspnet/anygrasp) - 通用抓取系统
- [dex-net](https://github.com/google-research/dex-net) - Google的抓取学习系统

### 技术文档
- [OpenCV官方文档](https://docs.opencv.org/)
- [ROS官方教程](https://wiki.ros.org/)
- [MoveIt运动规划](https://moveit.ros.org/)

### 学术论文
- "Dex-Net 2.0: Deep Learning to Plan Robust Grasps with Synthetic Point Clouds and Analytic Grasp Metrics"
- "Grasp Quality CNNs: Learning to Evaluate Grasp Stability from RGB-D Images"

## 🐛 故障排除

### 相机无法打开
```bash
# 检查相机设备
ls /dev/video*  # Linux
```

### OpenCV找不到
```bash
# 设置OpenCV路径
export OpenCV_DIR=/path/to/opencv/build  # Linux
```

### 编译错误
```bash
# 清理并重新编译
cd build
make clean
cmake ..
make
```

## 🤝 贡献指南

欢迎贡献代码！请遵循以下步骤：

1. Fork 本仓库
2. 创建特性分支 (`git checkout -b feature/AmazingFeature`)
3. 提交更改 (`git commit -m 'Add some AmazingFeature'`)
4. 推送到分支 (`git push origin feature/AmazingFeature`)
5. 开启 Pull Request

## 📄 许可证

本项目采用 MIT 许可证 - 详见 [LICENSE](LICENSE) 文件

## 👥 作者

- **您的名字** - 初始工作

## 🙏 致谢

- OpenCV开发团队
- ROS开发团队
- 所有为开源机器人项目做出贡献的开发者

## 📧 联系方式

如有问题或建议，请：
- 提交 [Issue](https://github.com/your-username/robotic-arm-vision-grasping/issues)
- 发送邮件至: your.email@example.com

---

**注意**: 这是一个模板项目，请根据您的具体需求进行修改和扩展。对于真实环境使用，请确保添加适当的安全措施和错误处理。