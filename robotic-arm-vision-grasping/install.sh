#!/bin/bash

# 机械臂视觉抓取系统安装脚本
# 适用于Ubuntu/Debian系统

set -e

echo "====================================="
echo "机械臂视觉抓取系统安装脚本"
echo "====================================="

# 检查是否为root用户
if [ "$EUID" -ne 0 ]; then
    echo "请使用sudo运行此脚本"
    exit 1
fi

# 更新包管理器
echo "更新包管理器..."
apt-get update

# 安装基本构建工具
echo "安装基本构建工具..."
apt-get install -y \
    build-essential \
    cmake \
    git \
    pkg-config

# 安装OpenCV
echo "安装OpenCV..."
apt-get install -y \
    libopencv-dev \
    python3-opencv

# 可选：安装深度学习依赖
read -p "是否安装深度学习依赖? (y/n): " install_dl
if [ "$install_dl" = "y" ]; then
    echo "安装深度学习依赖..."
    apt-get install -y \
        libtorch-dev \
        python3-torch \
        python3-torchvision
fi

# 创建构建目录
echo "创建构建目录..."
mkdir -p build

# 编译项目
echo "编译项目..."
cd build
cmake ..
make

# 安装
echo "安装程序..."
sudo make install

echo "====================================="
echo "安装完成！"
echo "====================================="
echo "可执行文件位置: /usr/local/bin/vision_grasping"
echo "运行程序: vision_grasping --help"
echo "====================================="