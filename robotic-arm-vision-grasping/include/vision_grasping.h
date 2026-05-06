#ifndef VISION_GRASPING_H
#define VISION_GRASPING_H

#include <opencv2/opencv.hpp>
#include <string>
#include <vector>
#include <memory>

namespace robotic {

struct GraspPoint {
    cv::Point2f center;      // 抓取中心点
    float angle;             // 抓取角度 (弧度)
    float confidence;        // 置信度 [0,1]
    cv::Rect bounding_box;   // 边界框
};

struct DetectedObject {
    std::string label;       // 物体标签
    cv::Rect bbox;           // 边界框
    float confidence;        // 检测置信度
    cv::Mat mask;            // 分割掩码 (可选)
};

class Camera {
public:
    Camera(int device_id = 0);
    ~Camera();

    bool open(int device_id);
    cv::Mat capture_frame();
    void close();
    bool is_open() const { return opened_; }

private:
    cv::VideoCapture capture_;
    bool opened_;
};

class VisionDetector {
public:
    VisionDetector();
    ~VisionDetector();

    void set_parameters(double confidence_threshold = 0.5,
                       double nms_threshold = 0.4);

    std::vector<DetectedObject> detect_objects(const cv::Mat& frame);

protected:
    virtual std::vector<DetectedObject> detect_impl(const cv::Mat& frame) = 0;

    double confidence_threshold_;
    double nms_threshold_;
};

class SimpleObjectDetector : public VisionDetector {
public:
    SimpleObjectDetector();

protected:
    std::vector<DetectedObject> detect_impl(const cv::Mat& frame) override;

private:
    cv::Ptr<cv::SimpleBlobDetector> blob_detector_;
};

class GraspPlanner {
public:
    GraspPlanner();

    void set_parameters(int grasp_points_per_object = 5,
                      double min_confidence = 0.6);

    std::vector<GraspPoint> plan_grasps(const DetectedObject& object,
                                       const cv::Mat& frame);

private:
    cv::Mat preprocess_for_grasp(const cv::Mat& frame, const cv::Rect& bbox);
    std::vector<GraspPoint> generate_grasp_candidates(const cv::Mat& region);
    float evaluate_grasp_quality(const cv::Mat& region, const GraspPoint& grasp);

    int grasp_points_per_object_;
    double min_confidence_;
};

class RobotController {
public:
    RobotController();
    virtual ~RobotController() = default;

    virtual bool connect() = 0;
    virtual bool disconnect() = 0;

    virtual bool move_to_position(float x, float y, float z) = 0;
    virtual bool execute_grasp(float width, float force) = 0;
    virtual bool release_grasp() = 0;

    virtual bool is_connected() const = 0;
    virtual bool is_ready() const = 0;

protected:
    bool connected_;
    bool ready_;
};

class SimulatedRobot : public RobotController {
public:
    SimulatedRobot();

    bool connect() override;
    bool disconnect() override;

    bool move_to_position(float x, float y, float z) override;
    bool execute_grasp(float width, float force) override;
    bool release_grasp() override;

    bool is_connected() const override;
    bool is_ready() const override;

    void get_current_position(float& x, float& y, float& z) const;

private:
    float x_, y_, z_;
};

class VisionGraspingSystem {
public:
    VisionGraspingSystem();
    ~VisionGraspingSystem();

    bool initialize();
    void shutdown();

    bool run_grasping_sequence();

    void set_robot_controller(std::unique_ptr<RobotController> controller);
    void set_detector(std::unique_ptr<VisionDetector> detector);

    void enable_debug_mode(bool enable) { debug_mode_ = enable; }
    void set_output_window(const std::string& window_name);

private:
    void display_debug_info(const cv::Mat& frame,
                          const std::vector<DetectedObject>& objects,
                          const std::vector<GraspPoint>& grasps);

    std::unique_ptr<Camera> camera_;
    std::unique_ptr<VisionDetector> detector_;
    std::unique_ptr<GraspPlanner> grasp_planner_;
    std::unique_ptr<RobotController> robot_;

    bool initialized_;
    bool debug_mode_;
    std::string output_window_;
};

} // namespace robotic

#endif // VISION_GRASPING_H