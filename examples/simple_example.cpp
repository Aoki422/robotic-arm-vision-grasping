#include "vision_grasping.h"
#include <iostream>

using namespace robotic;

// 简单示例：基础的物体检测和抓取
void simple_detection_example() {
    std::cout << "简单检测示例" << std::endl;

    // 创建相机
    Camera camera(0);
    if (!camera.is_open()) {
        std::cerr << "无法打开相机" << std::endl;
        return;
    }

    // 创建检测器
    SimpleObjectDetector detector;
    detector.set_parameters(0.5, 0.4);  // 置信度阈值, NMS阈值

    // 捕获一帧
    cv::Mat frame = camera.capture_frame();
    if (frame.empty()) {
        std::cerr << "无法捕获图像" << std::endl;
        return;
    }

    // 检测物体
    std::vector<DetectedObject> objects = detector.detect_objects(frame);

    std::cout << "检测到 " << objects.size() << " 个物体" << std::endl;

    // 显示结果
    cv::Mat result = frame.clone();
    for (const auto& obj : objects) {
        cv::rectangle(result, obj.bbox, cv::Scalar(0, 255, 0), 2);
        std::string label = obj.label + " (" +
                          std::to_string(static_cast<int>(obj.confidence * 100)) + "%)";
        cv::putText(result, label,
                   cv::Point(obj.bbox.x, obj.bbox.y - 10),
                   cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 2);
    }

    cv::imshow("检测结果", result);
    cv::waitKey(0);

    camera.close();
}

// 抓取规划示例
void grasp_planning_example() {
    std::cout << "抓取规划示例" << std::endl;

    // 创建检测器
    SimpleObjectDetector detector;

    // 创建抓取规划器
    GraspPlanner planner;
    planner.set_parameters(5, 0.6);  // 每个物体5个抓取点, 最小置信度0.6

    // 创建相机
    Camera camera(0);
    if (!camera.is_open()) {
        std::cerr << "无法打开相机" << std::endl;
        return;
    }

    // 捕获图像
    cv::Mat frame = camera.capture_frame();
    if (frame.empty()) {
        std::cerr << "无法捕获图像" << std::endl;
        return;
    }

    // 检测物体
    std::vector<DetectedObject> objects = detector.detect_objects(frame);

    if (objects.empty()) {
        std::cout << "未检测到物体" << std::endl;
        camera.close();
        return;
    }

    // 为第一个物体规划抓取
    const DetectedObject& obj = objects[0];
    std::vector<GraspPoint> grasps = planner.plan_grasps(obj, frame);

    std::cout << "为物体 " << obj.label << " 生成了 " << grasps.size() << " 个抓取点" << std::endl;

    // 可视化抓取点
    cv::Mat result = frame.clone();

    // 绘制物体边界框
    cv::rectangle(result, obj.bbox, cv::Scalar(0, 255, 0), 2);

    // 绘制抓取点
    for (size_t i = 0; i < grasps.size(); i++) {
        const GraspPoint& grasp = grasps[i];

        // 绘制抓取中心
        cv::Scalar color = (i == 0) ? cv::Scalar(255, 0, 0) : cv::Scalar(0, 0, 255);
        int thickness = (i == 0) ? 3 : 1;

        cv::circle(result, grasp.center, 5, color, -1);

        // 绘制抓取方向
        float line_length = 30.0f;
        cv::Point2f direction(std::cos(grasp.angle), std::sin(grasp.angle));
        cv::Point2f start = grasp.center - direction * line_length;
        cv::Point2f end = grasp.center + direction * line_length;

        cv::line(result, start, end, color, thickness);

        // 绘制置信度
        std::string conf_text = "G" + std::to_string(i) + ": " +
                              std::to_string(static_cast<int>(grasp.confidence * 100)) + "%";
        cv::putText(result, conf_text,
                   cv::Point(grasp.center.x + 10, grasp.center.y + 10),
                   cv::FONT_HERSHEY_SIMPLEX, 0.4, color, 1);
    }

    cv::imshow("抓取规划结果", result);
    cv::waitKey(0);

    camera.close();
}

// 机器人控制示例
void robot_control_example() {
    std::cout << "机器人控制示例" << std::endl;

    // 创建模拟机器人
    SimulatedRobot robot;

    // 连接机器人
    if (!robot.connect()) {
        std::cerr << "无法连接机器人" << std::endl;
        return;
    }

    // 移动到不同位置
    float positions[][3] = {
        {0.1f, 0.1f, 0.1f},
        {0.2f, 0.0f, 0.15f},
        {0.0f, 0.2f, 0.1f}
    };

    for (const auto& pos : positions) {
        robot.move_to_position(pos[0], pos[1], pos[2]);

        // 执行抓取
        robot.execute_grasp(0.05f, 50.0f);

        // 稍作等待
        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        // 释放抓取
        robot.release_grasp();
    }

    // 获取当前位置
    float x, y, z;
    robot.get_current_position(x, y, z);
    std::cout << "当前位置: (" << x << ", " << y << ", " << z << ")" << std::endl;

    // 断开连接
    robot.disconnect();
}

// 完整系统示例
void complete_system_example() {
    std::cout << "完整系统示例" << std::endl;

    VisionGraspingSystem system;

    // 启用调试模式
    system.enable_debug_mode(true);
    system.set_output_window("完整系统示例");

    // 初始化系统
    if (!system.initialize()) {
        std::cerr << "系统初始化失败" << std::endl;
        return;
    }

    // 执行抓取
    for (int i = 0; i < 3; i++) {
        std::cout << "执行抓取 " << (i + 1) << "/3" << std::endl;
        system.run_grasping_sequence();

        // 等待按键继续
        int key = cv::waitKey(0);
        if (key == 27) {  // ESC键
            break;
        }
    }

    // 关闭系统
    system.shutdown();
}

int main() {
    std::cout << "选择示例:" << std::endl;
    std::cout << "1. 简单检测" << std::endl;
    std::cout << "2. 抓取规划" << std::endl;
    std::cout << "3. 机器人控制" << std::endl;
    std::cout << "4. 完整系统" << std::endl;
    std::cout << "请输入选项 (1-4): ";

    int choice;
    std::cin >> choice;

    switch (choice) {
        case 1:
            simple_detection_example();
            break;
        case 2:
            grasp_planning_example();
            break;
        case 3:
            robot_control_example();
            break;
        case 4:
            complete_system_example();
            break;
        default:
            std::cout << "无效选项" << std::endl;
            return 1;
    }

    cv::destroyAllWindows();

    return 0;
}