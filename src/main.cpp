#include "vision_grasping.h"
#include <iostream>
#include <chrono>
#include <thread>

using namespace robotic;

void print_usage() {
    std::cout << "机械臂视觉抓取系统" << std::endl;
    std::cout << "=================" << std::endl;
    std::cout << "使用方法:" << std::endl;
    std::cout << "  ./vision_grasping [选项]" << std::endl;
    std::cout << std::endl;
    std::cout << "选项:" << std::endl;
    std::cout << "  -h, --help         显示帮助信息" << std::endl;
    std::cout << "  -d, --debug        启用调试模式 (显示视觉输出)" << std::endl;
    std::cout << "  -c, --camera ID    指定相机ID (默认: 0)" << std::endl;
    std::cout << "  -n, --number NUM   执行NUM次抓取 (默认: 10)" << std::endl;
    std::cout << "  -i, --interval MS  每次抓取间隔毫秒数 (默认: 1000)" << std::endl;
    std::cout << std::endl;
    std::cout << "示例:" << std::endl;
    std::cout << "  ./vision_grasping -d              # 调试模式运行" << std::endl;
    std::cout << "  ./vision_grasping -n 5            # 执行5次抓取" << std::endl;
    std::cout << "  ./vision_grasping -d -n 3 -i 500  # 调试模式,3次抓取,500ms间隔" << std::endl;
}

int main(int argc, char* argv[]) {
    // 解析命令行参数
    bool debug_mode = false;
    int camera_id = 0;
    int grasp_count = 10;
    int interval_ms = 1000;

    for (int i = 1; i < argc; i++) {
        std::string arg = argv[i];

        if (arg == "-h" || arg == "--help") {
            print_usage();
            return 0;
        } else if (arg == "-d" || arg == "--debug") {
            debug_mode = true;
        } else if (arg == "-c" || arg == "--camera") {
            if (i + 1 < argc) {
                camera_id = std::atoi(argv[++i]);
            }
        } else if (arg == "-n" || arg == "--number") {
            if (i + 1 < argc) {
                grasp_count = std::atoi(argv[++i]);
            }
        } else if (arg == "-i" || arg == "--interval") {
            if (i + 1 < argc) {
                interval_ms = std::atoi(argv[++i]);
            }
        } else {
            std::cerr << "未知参数: " << arg << std::endl;
            print_usage();
            return 1;
        }
    }

    std::cout << "===================================" << std::endl;
    std::cout << "机械臂视觉抓取系统" << std::endl;
    std::cout << "===================================" << std::endl;
    std::cout << "调试模式: " << (debug_mode ? "启用" : "禁用") << std::endl;
    std::cout << "相机ID: " << camera_id << std::endl;
    std::cout << "抓取次数: " << grasp_count << std::endl;
    std::cout << "间隔: " << interval_ms << " ms" << std::endl;
    std::cout << "===================================" << std::endl;

    try {
        // 创建抓取系统
        VisionGraspingSystem system;

        // 启用调试模式
        system.enable_debug_mode(debug_mode);
        if (debug_mode) {
            system.set_output_window("机械臂视觉抓取");
        }

        // 初始化系统
        std::cout << "\n正在初始化系统..." << std::endl;
        if (!system.initialize()) {
            std::cerr << "系统初始化失败" << std::endl;
            return 1;
        }

        std::cout << "系统初始化成功" << std::endl;

        // 主循环 - 执行抓取序列
        int success_count = 0;
        int fail_count = 0;

        for (int i = 0; i < grasp_count; i++) {
            std::cout << "\n======== 抓取 " << (i + 1) << "/" << grasp_count << " ========" << std::endl;

            bool success = system.run_grasping_sequence();
            if (success) {
                success_count++;
            } else {
                fail_count++;
            }

            // 等待指定间隔
            if (i < grasp_count - 1) {  // 最后一次不等待
                std::this_thread::sleep_for(std::chrono::milliseconds(interval_ms));
            }
        }

        // 打印统计信息
        std::cout << "\n===================================" << std::endl;
        std::cout << "抓取统计" << std::endl;
        std::cout << "===================================" << std::endl;
        std::cout << "总次数: " << grasp_count << std::endl;
        std::cout << "成功: " << success_count << std::endl;
        std::cout << "失败: " << fail_count << std::endl;
        std::cout << "成功率: " << (100.0 * success_count / grasp_count) << "%" << std::endl;
        std::cout << "===================================" << std::endl;

        // 关闭系统
        std::cout << "\n正在关闭系统..." << std::endl;
        system.shutdown();

        std::cout << "\n程序执行完成!" << std::endl;

        return 0;

    } catch (const std::exception& e) {
        std::cerr << "程序异常: " << e.what() << std::endl;
        return 1;
    }
}