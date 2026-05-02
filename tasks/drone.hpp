#pragma once

#include <list>
#include <vector>
#include <opencv2/opencv.hpp>

namespace drone_detection {

// Drone 结构体定义
struct Drone {
    int class_id = 0;
    float confidence = 0.0f;
    cv::Rect box;
    std::vector<cv::Point2f> points;
    cv::Point2f center;
    cv::Point2f center_norm;
    
    Drone() = default;
    Drone(int id, float conf, const cv::Rect& b, const std::vector<cv::Point2f>& pts)
        : class_id(id), confidence(conf), box(b), points(pts) {}
};

// 前置声明
class DroneDetector;

} // namespace drone_detection