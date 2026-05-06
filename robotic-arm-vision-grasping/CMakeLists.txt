cmake_minimum_required(VERSION 3.10)
project(RoboticArmVisionGrasping)

# C++ 标准
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 查找依赖包
find_package(OpenCV REQUIRED)
find_package(Threads REQUIRED)

# 头文件目录
include_directories(${CMAKE_SOURCE_DIR}/include)

# 源文件
file(GLOB_RECURSE SOURCES "src/*.cpp")

# 可执行文件
add_executable(vision_grasping ${SOURCES})

# 链接库
target_link_libraries(vision_grasping ${OpenCV_LIBS} Threads::Threads)

# 编译选项
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(vision_grasping PRIVATE -Wall -Wextra)
elseif(MSVC)
    target_compile_options(vision_grasping PRIVATE /W4)
endif()

# 安装规则
install(TARGETS vision_grasping DESTINATION bin)