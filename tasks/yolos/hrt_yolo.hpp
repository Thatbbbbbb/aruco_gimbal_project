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
cv::Mat resizeAndPad(const cv::Mat& src, int dst_width, int dst_height, float& scale, cv::Point2f& offset);
class DroneDetector {
public:
    DroneDetector(const std::string& config_path, bool debug = false);
    ~DroneDetector() = default;
    
    std::list<Drone> detect(const cv::Mat& raw_img, int frame_count);

private:
    void preprocess(const cv::Mat& raw_img, cv::Mat& processed_img);
    void postprocess(const deploy::DetectRes& result, std::list<Drone>& drones);
    int remap_class_id(int model_id);
    void sort_keypoints(std::vector<cv::Point2f>& keypoints);
    void nms_filter(std::list<Drone>& drones);
    void draw_detections(const cv::Mat& img, const std::list<Drone>& drones, int frame_count) const;
    
    private:
    /**
     * @brief 从融合的 [1,5,8400] 输出直接解析检测框并做 NMS
     */
    std::list<Drone> parseOutputAndNMS(const float* output, int num_boxes);
    // 成员变量
    bool debug_;
    std::unique_ptr<deploy::BaseModel<deploy::DetectRes>> model_;
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
    int input_width_ = 640;
    int input_height_ = 640;
    float last_scale_ = 1.0f;
    float pad_left_;   // 水平填充（模型画布左边缘到有效图像的距离）
    float pad_top_;    // 垂直填充
};

} // namespace drone_detection