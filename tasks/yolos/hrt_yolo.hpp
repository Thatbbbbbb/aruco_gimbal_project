#ifndef HITCRT_YOLO_HPP
#define HITCRT_YOLO_HPP

#include <deploy/result.hpp>
#include <opencv2/opencv.hpp>
#include <list>
#include <memory>
#include <string>

#include "drone.hpp"

namespace drone_detection {

class HitcrtYOLO {
public:
    HitcrtYOLO(const std::string& config_path, bool debug = false);
    
    // 无人机检测接口
    std::list<Drone> detect_drones(const cv::Mat& raw_img, int frame_count);
    
private:
    // 预处理
    void preprocess(const cv::Mat& raw_img, cv::Mat& processed_img);
    
    // 后处理
    void postprocess(const deploy::PoseRes& result, std::list<Drone>& drones);
    
    // 类别映射（模型类别 -> 无人机类型）
    int remap_drone_class_id(int model_id);
    
    // 关键点排序（确保顺序：左上、右上、右下、左下）
    void sort_keypoints(std::vector<cv::Point2f>& keypoints);
    
    // 非极大值抑制
    void nms_filter(std::list<Drone>& drones);
    
    // 可视化
    void draw_detections(const cv::Mat& img, const std::list<Drone>& drones, 
                        int frame_count) const;
    
    // 成员变量
    bool debug_;
    std::unique_ptr<deploy::BaseModel<deploy::PoseRes>> model_;
    std::string model_path_;
    std::string device_;
    
    double confidence_threshold_;  // 置信度阈值
    double nms_threshold_;         // NMS阈值
    double min_confidence_;        // 最小置信度
    
    cv::Rect roi_;
    cv::Point2f offset_;
    bool use_roi_;
    
    std::string save_path_;
    cv::Mat tmp_img_;
};

}  // namespace auto_aim

#endif
