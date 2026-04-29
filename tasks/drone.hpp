#ifndef DRONE_DETECTOR_HPP
#define DRONE_DETECTOR_HPP

#include <fmt/chrono.h>
#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <memory>
#include <list>
#include <vector>

#include <opencv2/opencv.hpp>

// Drone 结构体定义
struct Drone {
    int class_id;                          // 类别ID
    float confidence;                      // 置信度
    cv::Rect box;                          // 边界框
    std::vector<cv::Point2f> points;       // 四个角点（左上、右上、右下、左下）
    cv::Point2f center;                    // 中心点
    cv::Point2f center_norm;               // 归一化中心点（相对于图像尺寸）
    
    // 构造函数
    Drone() : class_id(-1), confidence(0.0f), box(), points(), center(), center_norm() {}
    
    Drone(int id, float conf, const cv::Rect& bbox, const std::vector<cv::Point2f>& pts)
        : class_id(id), confidence(conf), box(bbox), points(pts), center(), center_norm() {}
    
    Drone(const Drone& other) = default;
    Drone& operator=(const Drone& other) = default;
};

namespace deploy {
    struct PoseRes;
    struct Image;
    struct InferOption;
    template<typename T> class BaseModel;
}

namespace drone_detection {

class DroneDetector {
public:
    DroneDetector(const std::string& config_path, bool debug = false);
    
    // 主检测接口：返回无人机列表
    std::list<Drone> detect(const cv::Mat& raw_img, int frame_count);
    
private:
    // 预处理
    void preprocess(const cv::Mat& raw_img, cv::Mat& processed_img);
    
    // 后处理
    void postprocess(const deploy::PoseRes& result, std::list<Drone>& drones);
    
    // 类别映射（模型类别ID -> 无人机属性）
    int remap_class_id(int model_id);
    
    // 关键点排序
    void sort_keypoints(std::vector<cv::Point2f>& keypoints);
    
    // 筛选
    bool check_confidence(const Drone& drone) const;
    void nms_filter(std::list<Drone>& drones);
    
    // 可视化
    void draw_detections(const cv::Mat& img, const std::list<Drone>& drones, int frame_count) const;
    
    // 成员变量
    bool debug_;
    std::unique_ptr<deploy::BaseModel<deploy::PoseRes>> model_;
    std::string model_path_;
    std::string device_;
    
    double confidence_threshold_;
    double nms_threshold_;
    double min_confidence_;
    
    cv::Rect roi_;
    cv::Point2f offset_;
    bool use_roi_;
    
    std::string save_path_;
    cv::Mat tmp_img_;
};

} // namespace drone_detection

#endif // DRONE_DETECTOR_HPP
