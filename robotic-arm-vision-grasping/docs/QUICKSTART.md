# 快速开始指南

本指南将帮助您在15分钟内开始使用机械臂视觉抓取系统。

## 🎯 快速设置

### 步骤1: 获取代码

```bash
git clone https://github.com/your-username/robotic-arm-vision-grasping.git
cd robotic-arm-vision-grasping
```

### 步骤2: 安装依赖

#### Ubuntu/Debian
```bash
sudo apt-get update
sudo apt-get install build-essential cmake libopencv-dev
```

#### macOS
```bash
brew install cmake opencv
```

#### Windows
- 安装 [CMake](https://cmake.org/download/)
- 安装 [OpenCV](https://opencv.org/releases/)

### 步骤3: 编译项目

```bash
mkdir build
cd build
cmake ..
make  # 或在Windows上使用Visual Studio打开解决方案
```

### 步骤4: 运行测试

```bash
# 查看帮助信息
./vision_grasping --help

# 基本运行（使用模拟相机和机器人）
./vision_grasping --debug
```

## 🖼️ 第一个示例

### 使用网络摄像头

```bash
# 使用默认摄像头
./vision_grasping --debug

# 使用第二个摄像头
./vision_grasping --debug --camera 1
```

### 解释
- `--debug`: 启用可视化，实时显示检测结果
- 程序会自动检测图像中的物体
- 显示抓取点的位置和置信度
- 在模拟环境中执行抓取动作

## 🔧 自定义配置

### 修改相机设置

编辑 `src/vision_grasping.cpp` 中的相机初始化代码：

```cpp
// 在 Camera::open() 函数中
capture_.set(cv::CAP_PROP_FRAME_WIDTH, 1280);  // 分辨率
capture_.set(cv::CAP_PROP_FRAME_HEIGHT, 720);
capture_.set(cv::CAP_PROP_FPS, 60);            // 帧率
```

### 调整检测参数

```cpp
// 在主程序中修改
detector->set_parameters(0.7, 0.3);  // 置信度, NMS阈值
```

### 修改抓取参数

```cpp
// 在主程序中修改
grasp_planner->set_parameters(10, 0.8);  // 抓取点数量, 最小置信度
```

## 📝 常用命令

### 基本操作

```bash
# 运行一次抓取
./vision_grasping --number 1

# 连续抓取10次，间隔2秒
./vision_grasping --number 10 --interval 2000

# 调试模式，5次抓取
./vision_grasping --debug --number 5
```

### 编译相关

```bash
# 清理并重新编译
cd build
make clean
cmake ..
make

# 安装到系统
sudo make install  # Linux/macOS

# 卸载
sudo make uninstall
```

## 🎓 学习路径

### 初学者
1. 阅读 `README.md` 了解项目结构
2. 运行 `examples/simple_example.cpp` 中的不同示例
3. 尝试修改参数观察效果变化
4. 理解各模块的基本功能

### 中级用户
1. 创建自定义检测器（见README中的"自定义和扩展"章节）
2. 实现简单的抓取算法改进
3. 添加新的可视化功能
4. 熟悉代码结构，准备进行更复杂的修改

### 高级用户
1. 集成真实的机器人控制接口
2. 实现基于深度学习的物体检测
3. 优化抓取规划算法
4. 添加运动规划、碰撞检测等功能
5. 参与项目开发，提交PR

## 🐛 常见问题

### Q1: 找不到相机
```bash
# 检查可用的相机设备
# Linux:
ls /dev/video*

# macOS:
system_profiler SPDisplaysDataType

# 如果问题持续，尝试使用视频文件作为输入
```

### Q2: OpenCV找不到
```bash
# 设置OpenCV路径
# Linux/macOS:
export OpenCV_DIR=/usr/local/lib/cmake/opencv4

# Windows: 在CMake中设置OPENCV_DIR变量
```

### Q3: 编译错误
```bash
# 完全清理并重新开始
cd ..
rm -rf build
mkdir build
cd build
cmake ..
make
```

### Q4: 运行时错误
```bash
# 检查依赖库
ldd ./vision_grasping  # Linux
otool -L ./vision_grasping  # macOS

# 确保所有依赖都正确安装
```

## 🚀 下一步

1. **查看示例**: 运行 `examples/simple_example.cpp` 中的不同示例
2. **阅读文档**: 仔细阅读 `README.md` 的完整文档
3. **实验修改**: 尝试修改参数和算法
4. **集成硬件**: 当你熟悉系统后，可以开始集成真实硬件

## 📞 获取帮助

- 查看 [README.md](../README.md) 获取详细文档
- 提交 [Issue](https://github.com/your-username/robotic-arm-vision-grasping/issues)
- 查看相关项目的源码学习（见README中的"学习资源"）

## 💡 提示

- 从简单开始，逐步增加复杂度
- 多使用 `--debug` 模式观察系统行为
- 定期保存代码版本
- 在修改复杂功能前先备份代码
- 遇到问题时，先检查基本的配置和依赖

祝您使用愉快！ 🎉