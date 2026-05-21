#include "lightbar_detector.hpp"
#include <algorithm>
#include <cmath>
#include <set>
#include <fstream>
#include "tools/yaml.hpp"

namespace drone_detection {

// ========== SingleLightbar 实现 ==========

bool LightbarDetector::loadConfig(const std::string& path) {
    try {
        YAML::Node config = YAML::LoadFile(path);
        
        if (config["lightbar"]) {
            lightbar_color_ = config["lightbar"]["color"].as<std::string>();
            
            // 读取颜色阈值
            if (config["lightbar"]["h_min"]) h_min_ = config["lightbar"]["h_min"].as<int>();
            if (config["lightbar"]["h_max"]) h_max_ = config["lightbar"]["h_max"].as<int>();
            if (config["lightbar"]["s_min"]) s_min_ = config["lightbar"]["s_min"].as<int>();
            if (config["lightbar"]["s_max"]) s_max_ = config["lightbar"]["s_max"].as<int>();
            if (config["lightbar"]["v_min"]) v_min_ = config["lightbar"]["v_min"].as<int>();
            if (config["lightbar"]["v_max"]) v_max_ = config["lightbar"]["v_max"].as<int>();
        }
        
        return true;
    } catch (...) {
        return false;
    }
}
/**
 * 在图像的指定ROI内提取指定颜色的发光点中心坐标
 * 
 * @param frame      BGR输入图像
 * @param roi        感兴趣区域（在此矩形内检测）
 * @param color      颜色名称："red" / "blue" / "purple"
 * @param h_min      色调最小值 (0~180)
 * @param h_max      色调最大值 (0~180)
 * @param s_min      饱和度最小值 (0~255)
 * @param s_max      饱和度最大值 (0~255)
 * @param v_min      明度最小值 (0~255)
 * @param v_max      明度最大值 (0~255)
 * @param min_area   有效发光点最小面积（像素）
 * @param max_area   有效发光点最大面积（像素）
 * @return           所有满足条件的发光点的中心坐标（全局图像坐标系）
 */
std::vector<cv::Point2f> extractBrightPoints(const cv::Mat& frame,
                                              const cv::Rect& roi,
                                              const std::string& color,
                                              int h_min, int h_max,
                                              int s_min, int s_max,
                                              int v_min, int v_max,
                                              int min_area = 20,
                                              int max_area = 1000)
{
    std::vector<cv::Point2f> points;
    
    // 参数有效性检查
    if (frame.empty() || roi.empty()) {
        return points;
    }
    
    // 裁剪ROI区域（防止越界）
    cv::Rect roi_clipped = roi & cv::Rect(0, 0, frame.cols, frame.rows);
    if (roi_clipped.area() <= 0) {
        return points;
    }
    
    // 1. 提取ROI子图
    cv::Mat roi_bgr = frame(roi_clipped);
    
    // 2. BGR转HSV
    cv::Mat hsv;
    cv::cvtColor(roi_bgr, hsv, cv::COLOR_BGR2HSV);
    
    // 3. 根据颜色生成掩膜
    cv::Mat mask;
    if (color == "red") {
        // 红色在HSV色相环上跨越0和180两个区域
        cv::Mat low_red, high_red;
        cv::inRange(hsv, cv::Scalar(0, s_min, v_min), cv::Scalar(std::min(10, h_max), s_max, v_max), low_red);
        cv::inRange(hsv, cv::Scalar(std::max(160, h_min), s_min, v_min), cv::Scalar(180, s_max, v_max), high_red);
        mask = low_red | high_red;
    } else {
        // 蓝色、紫色等单一区间
        cv::inRange(hsv, cv::Scalar(h_min, s_min, v_min), cv::Scalar(h_max, s_max, v_max), mask);
    }
    
    // 4. 形态学去噪（开运算+闭运算）
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3));
    cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);
    cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);
    
    // 5. 连通域分析，获取每个斑点的质心
    cv::Mat labels, stats, centroids;
    int ncomponents = cv::connectedComponentsWithStats(mask, labels, stats, centroids, 8, CV_32S);
    
    // 6. 遍历所有连通域（跳过背景label=0）
    for (int label = 1; label < ncomponents; ++label) {
        int area = stats.at<int>(label, cv::CC_STAT_AREA);
        if (area < min_area || area > max_area) {
            continue;   // 面积不符合要求，跳过
        }
        // 质心坐标（ROI局部坐标系）
        double cx = centroids.at<double>(label, 0);
        double cy = centroids.at<double>(label, 1);
        // 转换到全局图像坐标系
        points.emplace_back(static_cast<float>(cx + roi_clipped.x),
                            static_cast<float>(cy + roi_clipped.y));
    }
    
    return points;
}


} // namespace drone_detection