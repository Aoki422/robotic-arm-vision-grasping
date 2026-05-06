#include "vision_grasping.h"
#include <iostream>
#include <algorithm>
#include <cmath>

namespace robotic {

// ============================================================================
// Camera 实现
// ============================================================================

Camera::Camera(int device_id) : opened_(false) {
    open(device_id);
}

Camera::~Camera() {
    close();
}

bool Camera::open(int device_id) {
    capture_.open(device_id);
    if (capture_.isOpened()) {
        opened_ = true;
        capture_.set(cv::CAP_PROP_FRAME_WIDTH, 640);
        capture_.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
        capture_.set(cv::CAP_PROP_FPS, 30);
        return true;
    }
    return false;
}

cv::Mat Camera::capture_frame() {
    cv::Mat frame;
    if (opened_) {
        capture_ >> frame;
    }
    return frame;
}

void Camera::close() {
    if (opened_) {
        capture_.release();
        opened_ = false;
    }
}

// ============================================================================
// VisionDetector 实现
// ============================================================================

VisionDetector::VisionDetector()
    : confidence_threshold_(0.5), nms_threshold_(0.4) {}

VisionDetector::~VisionDetector() = default;

void VisionDetector::set_parameters(double confidence_threshold,
                                   double nms_threshold) {
    confidence_threshold_ = confidence_threshold;
    nms_threshold_ = nms_threshold;
}

std::vector<DetectedObject> VisionDetector::detect_objects(const cv::Mat& frame) {
    return detect_impl(frame);
}

// ============================================================================
// SimpleObjectDetector 实现
// ============================================================================

SimpleObjectDetector::SimpleObjectDetector() {
    cv::SimpleBlobDetector::Params params;
    params.filterByArea = true;
    params.minArea = 500;
    params.maxArea = 100000;
    params.filterByCircularity = false;
    params.filterByConvexity = false;
    params.filterByInertia = false;

    blob_detector_ = cv::SimpleBlobDetector::create(params);
}

std::vector<DetectedObject> SimpleObjectDetector::detect_impl(const cv::Mat& frame) {
    std::vector<DetectedObject> objects;

    // 转换为灰度图
    cv::Mat gray;
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

    // 高斯模糊
    cv::Mat blurred;
    cv::GaussianBlur(gray, blurred, cv::Size(9, 9), 2);

    // 检测关键点
    std::vector<cv::KeyPoint> keypoints;
    blob_detector_->detect(blurred, keypoints);

    // 将关键点转换为检测对象
    for (const auto& kp : keypoints) {
        DetectedObject obj;
        obj.label = "blob_" + std::to_string(objects.size());
        obj.confidence = kp.response / 100.0f;

        // 计算边界框
        int radius = static_cast<int>(kp.size / 2);
        cv::Point center(static_cast<int>(kp.pt.x), static_cast<int>(kp.pt.y));

        obj.bbox = cv::Rect(
            center.x - radius,
            center.y - radius,
            radius * 2,
            radius * 2
        );

        // 确保边界框在图像范围内
        obj.bbox &= cv::Rect(0, 0, frame.cols, frame.rows);

        if (obj.bbox.area() > 0 && obj.confidence >= confidence_threshold_) {
            objects.push_back(obj);
        }
    }

    return objects;
}

// ============================================================================
// GraspPlanner 实现
// ============================================================================

GraspPlanner::GraspPlanner()
    : grasp_points_per_object_(5), min_confidence_(0.6) {}

void GraspPlanner::set_parameters(int grasp_points_per_object,
                                  double min_confidence) {
    grasp_points_per_object_ = grasp_points_per_object;
    min_confidence_ = min_confidence;
}

std::vector<GraspPoint> GraspPlanner::plan_grasps(const DetectedObject& object,
                                                 const cv::Mat& frame) {
    std::vector<GraspPoint> grasps;

    // 预处理图像区域
    cv::Mat region = preprocess_for_grasp(frame, object.bbox);

    if (region.empty()) {
        return grasps;
    }

    // 生成抓取候选
    std::vector<GraspPoint> candidates = generate_grasp_candidates(region);

    // 评估并排序抓取质量
    for (auto& grasp : candidates) {
        grasp.confidence = evaluate_grasp_quality(region, grasp);
        // 转换回原始图像坐标
        grasp.center.x += object.bbox.x;
        grasp.center.y += object.bbox.y;
    }

    // 按置信度排序
    std::sort(grasps.begin(), grasps.end(),
        [](const GraspPoint& a, const GraspPoint& b) {
            return a.confidence > b.confidence;
        });

    // 返回top-N抓取点
    int num_grasps = std::min(static_cast<int>(grasps.size()), grasp_points_per_object_);
    grasps.resize(num_grasps);

    return grasps;
}

cv::Mat GraspPlanner::preprocess_for_grasp(const cv::Mat& frame, const cv::Rect& bbox) {
    cv::Rect safe_bbox = bbox & cv::Rect(0, 0, frame.cols, frame.rows);
    if (safe_bbox.area() <= 0) {
        return cv::Mat();
    }

    cv::Mat region = frame(safe_bbox).clone();
    return region;
}

std::vector<GraspPoint> GraspPlanner::generate_grasp_candidates(const cv::Mat& region) {
    std::vector<GraspPoint> candidates;

    // 简单的抓取候选生成策略
    // 在区域中心周围生成多个角度的抓取点

    cv::Point center(region.cols / 2, region.rows / 2);
    int radius = std::min(region.cols, region.rows) / 4;

    // 生成不同角度的抓取点
    for (int i = 0; i < 8; i++) {
        float angle = (M_PI / 4) * i;  // 45度间隔

        GraspPoint grasp;
        grasp.center = center;
        grasp.angle = angle;
        grasp.bounding_box = cv::Rect(0, 0, region.cols, region.rows);

        candidates.push_back(grasp);
    }

    return candidates;
}

float GraspPlanner::evaluate_grasp_quality(const cv::Mat& region, const GraspPoint& grasp) {
    // 简单的抓取质量评估
    // 在实际应用中，这里应该使用更复杂的算法，如深度学习模型

    cv::Mat gray;
    cv::cvtColor(region, gray, cv::COLOR_BGR2GRAY);

    // 计算区域中心的梯度强度
    cv::Mat grad_x, grad_y;
    cv::Sobel(gray, grad_x, CV_64F, 1, 0, 3);
    cv::Sobel(gray, grad_y, CV_64F, 0, 1, 3);

    cv::Point center(region.cols / 2, region.rows / 2);
    float gradient_sum = 0.0f;

    int sample_radius = 10;
    for (int y = -sample_radius; y <= sample_radius; y++) {
        for (int x = -sample_radius; x <= sample_radius; x++) {
            int px = center.x + x;
            int py = center.y + y;

            if (px >= 0 && px < grad_x.cols && py >= 0 && py < grad_x.rows) {
                float gx = grad_x.at<double>(py, px);
                float gy = grad_y.at<double>(py, px);
                gradient_sum += std::sqrt(gx * gx + gy * gy);
            }
        }
    }

    // 归一化并返回
    float quality = std::tanh(gradient_sum / 1000.0f);
    return std::max(0.0f, std::min(1.0f, quality));
}

// ============================================================================
// RobotController 实现
// ============================================================================

RobotController::RobotController() : connected_(false), ready_(false) {}

// ============================================================================
// SimulatedRobot 实现
// ============================================================================

SimulatedRobot::SimulatedRobot()
    : RobotController(), x_(0.0f), y_(0.0f), z_(0.0f) {}

bool SimulatedRobot::connect() {
    connected_ = true;
    ready_ = true;
    std::cout << "模拟机器人已连接" << std::endl;
    return true;
}

bool SimulatedRobot::disconnect() {
    connected_ = false;
    ready_ = false;
    std::cout << "模拟机器人已断开连接" << std::endl;
    return true;
}

bool SimulatedRobot::move_to_position(float x, float y, float z) {
    if (!connected_ || !ready_) {
        std::cerr << "机器人未连接或未就绪" << std::endl;
        return false;
    }

    x_ = x;
    y_ = y;
    z_ = z;

    std::cout << "移动到位置: (" << x << ", " << y << ", " << z << ")" << std::endl;
    return true;
}

bool SimulatedRobot::execute_grasp(float width, float force) {
    if (!connected_ || !ready_) {
        std::cerr << "机器人未连接或未就绪" << std::endl;
        return false;
    }

    std::cout << "执行抓取: 宽度=" << width << ", 力度=" << force << std::endl;
    return true;
}

bool SimulatedRobot::release_grasp() {
    if (!connected_ || !ready_) {
        std::cerr << "机器人未连接或未就绪" << std::endl;
        return false;
    }

    std::cout << "释放抓取" << std::endl;
    return true;
}

bool SimulatedRobot::is_connected() const {
    return connected_;
}

bool SimulatedRobot::is_ready() const {
    return ready_;
}

void SimulatedRobot::get_current_position(float& x, float& y, float& z) const {
    x = x_;
    y = y_;
    z = z_;
}

// ============================================================================
// VisionGraspingSystem 实现
// ============================================================================

VisionGraspingSystem::VisionGraspingSystem()
    : initialized_(false), debug_mode_(false), output_window_("抓取结果") {}

VisionGraspingSystem::~VisionGraspingSystem() {
    shutdown();
}

bool VisionGraspingSystem::initialize() {
    try {
        // 创建相机
        camera_ = std::make_unique<Camera>(0);
        if (!camera_->is_open()) {
            std::cerr << "无法打开相机" << std::endl;
            return false;
        }

        // 创建检测器
        if (!detector_) {
            detector_ = std::make_unique<SimpleObjectDetector>();
        }

        // 创建抓取规划器
        grasp_planner_ = std::make_unique<GraspPlanner>();

        // 创建机器人控制器
        if (!robot_) {
            robot_ = std::make_unique<SimulatedRobot>();
        }

        // 连接机器人
        if (!robot_->connect()) {
            std::cerr << "无法连接机器人" << std::endl;
            return false;
        }

        initialized_ = true;
        std::cout << "系统初始化成功" << std::endl;
        return true;

    } catch (const std::exception& e) {
        std::cerr << "初始化错误: " << e.what() << std::endl;
        return false;
    }
}

void VisionGraspingSystem::shutdown() {
    if (robot_) {
        robot_->disconnect();
    }
    if (camera_) {
        camera_->close();
    }
    initialized_ = false;
    std::cout << "系统已关闭" << std::endl;
}

bool VisionGraspingSystem::run_grasping_sequence() {
    if (!initialized_) {
        std::cerr << "系统未初始化" << std::endl;
        return false;
    }

    try {
        // 捕获图像
        cv::Mat frame = camera_->capture_frame();
        if (frame.empty()) {
            std::cerr << "无法捕获图像" << std::endl;
            return false;
        }

        // 检测物体
        std::vector<DetectedObject> objects = detector_->detect_objects(frame);
        std::cout << "检测到 " << objects.size() << " 个物体" << std::endl;

        if (objects.empty()) {
            if (debug_mode_) {
                cv::imshow(output_window_, frame);
                cv::waitKey(1);
            }
            return true;
        }

        // 规划抓取
        std::vector<GraspPoint> all_grasps;
        for (const auto& obj : objects) {
            std::vector<GraspPoint> grasps = grasp_planner_->plan_grasps(obj, frame);
            all_grasps.insert(all_grasps.end(), grasps.begin(), grasps.end());
        }

        std::cout << "生成了 " << all_grasps.size() << " 个抓取点" << std::endl;

        // 执行最佳抓取
        if (!all_grasps.empty()) {
            const GraspPoint& best_grasp = all_grasps[0];

            // 将图像坐标转换为世界坐标（简化版本）
            float world_x = (best_grasp.center.x - frame.cols / 2) / 100.0f;
            float world_y = (best_grasp.center.y - frame.rows / 2) / 100.0f;
            float world_z = 0.1f;  // 假设工作台高度

            std::cout << "执行最佳抓取: 置信度=" << best_grasp.confidence << std::endl;

            // 移动到抓取位置
            robot_->move_to_position(world_x, world_y, world_z);

            // 执行抓取
            robot_->execute_grasp(0.05f, 50.0f);

            // 稍作等待
            cv::waitKey(100);

            // 释放抓取
            robot_->release_grasp();
        }

        // 显示调试信息
        if (debug_mode_) {
            display_debug_info(frame, objects, all_grasps);
            cv::imshow(output_window_, frame);
            cv::waitKey(1);
        }

        return true;

    } catch (const std::exception& e) {
        std::cerr << "执行抓取序列时出错: " << e.what() << std::endl;
        return false;
    }
}

void VisionGraspingSystem::set_robot_controller(std::unique_ptr<RobotController> controller) {
    robot_ = std::move(controller);
}

void VisionGraspingSystem::set_detector(std::unique_ptr<VisionDetector> detector) {
    detector_ = std::move(detector);
}

void VisionGraspingSystem::set_output_window(const std::string& window_name) {
    output_window_ = window_name;
}

void VisionGraspingSystem::display_debug_info(const cv::Mat& frame,
                                              const std::vector<DetectedObject>& objects,
                                              const std::vector<GraspPoint>& grasps) {
    cv::Mat debug_frame = frame.clone();

    // 绘制检测到的物体
    for (size_t i = 0; i < objects.size(); i++) {
        const auto& obj = objects[i];
        cv::rectangle(debug_frame, obj.bbox, cv::Scalar(0, 255, 0), 2);

        std::string label = obj.label + " (" +
                          std::to_string(static_cast<int>(obj.confidence * 100)) + "%)";
        cv::putText(debug_frame, label,
                   cv::Point(obj.bbox.x, obj.bbox.y - 10),
                   cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 2);
    }

    // 绘制抓取点
    for (size_t i = 0; i < grasps.size(); i++) {
        const auto& grasp = grasps[i];

        // 绘制抓取中心
        cv::circle(debug_frame, grasp.center, 5,
                  cv::Scalar(255, 0, 0), -1);

        // 绘制抓取方向
        float line_length = 30.0f;
        cv::Point2f direction(std::cos(grasp.angle), std::sin(grasp.angle));
        cv::Point2f start = grasp.center - direction * line_length;
        cv::Point2f end = grasp.center + direction * line_length;

        cv::line(debug_frame, start, end, cv::Scalar(0, 0, 255), 2);

        // 绘制置信度
        std::string conf_text = "G" + std::to_string(i) + ": " +
                              std::to_string(static_cast<int>(grasp.confidence * 100)) + "%";
        cv::putText(debug_frame, conf_text,
                   cv::Point(grasp.center.x + 10, grasp.center.y + 10),
                   cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 0, 255), 1);
    }
}

} // namespace robotic