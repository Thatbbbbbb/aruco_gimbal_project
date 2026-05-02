#pragma once

#include <list>
#include <memory>
#include <string>
#include <opencv2/opencv.hpp>
#include "drone.hpp"

// 直接包含完整的 deploy 头文件，而不是前向声明
#include "deploy/model.hpp"
#include "deploy/result.hpp"

namespace drone_detection {

class DroneDetector {
public:
    DroneDetector(const std::string& config_path, bool debug = false);
    ~DroneDetector() = default;
    
    std::list<Drone> detect(const cv::Mat& raw_img, int frame_count);

private:
    void preprocess(const cv::Mat& raw_img, cv::Mat& processed_img);
    void postprocess(const deploy::PoseRes& result, std::list<Drone>& drones);
    int remap_class_id(int model_id);
    void sort_keypoints(std::vector<cv::Point2f>& keypoints);
    void nms_filter(std::list<Drone>& drones);
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
    cv::Size source_size_;
};

} // namespace drone_detection