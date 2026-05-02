#pragma once

#include <list>
#include <vector>
#include <opencv2/opencv.hpp>
#include "drone.hpp"

namespace drone_detection {

// 单个灯条结构体
struct SingleLightbar {
    cv::RotatedRect rotated_rect;
    cv::Point2f center;
    cv::Point2f top;
    cv::Point2f bottom;
    float length;
    float width;
    float angle;
    float ratio;
    
    SingleLightbar() = default;
    SingleLightbar(const cv::RotatedRect& rect, const cv::Point2f& offset = cv::Point2f(0, 0));
};

// 水平灯条组（一排）
struct HorizontalLightbarGroup {
    std::vector<SingleLightbar> lightbars;
    cv::Point2f center;
    float y_level;           // Y 坐标（用于区分上下排）
    float confidence;
    
    HorizontalLightbarGroup() = default;
};

// 上下平行双排灯条组合
struct ParallelLightbarSet {
    HorizontalLightbarGroup top_row;     // 上排
    HorizontalLightbarGroup bottom_row;  // 下排
    cv::Point2f center;                  // 整体中心
    std::vector<cv::Point2f> corners;    // 四个角点
    float confidence;
    float top_to_bottom_distance;        // 上下排距离
    
    ParallelLightbarSet() = default;
};

// 灯条检测器
class LightbarDetector {
public:
    LightbarDetector();
    
    // 在 ROI 区域内检测灯条组合
    std::vector<ParallelLightbarSet> detect(const cv::Mat& img, const cv::Rect& roi);
    
    // 设置检测参数
    void setMinLightbarArea(double area) { min_lightbar_area_ = area; }
    void setMaxLightbarRatio(double ratio) { max_lightbar_ratio_ = ratio; }
    void setMaxGroupDistance(double distance) { max_group_distance_ = distance; }
    void setMaxTopBottomDistance(double distance) { max_top_bottom_distance_ = distance; }
    void setMinLightbarsPerRow(int min) { min_lightbars_per_row_ = min; }
    void setMaxLightbarsPerRow(int max) { max_lightbars_per_row_ = max; }
    
private:
    // 检测所有灯条
    std::vector<SingleLightbar> detectLightbars(const cv::Mat& img, const cv::Rect& roi);
    
    // 将灯条分组为行（按 Y 坐标聚类）
    std::vector<HorizontalLightbarGroup> groupByRow(const std::vector<SingleLightbar>& lightbars);
    
    // 过滤每行灯条（按 X 坐标排序，检查间距）
    HorizontalLightbarGroup filterRowGroup(const std::vector<SingleLightbar>& row_candidates);
    
    // 匹配上下排
    std::vector<ParallelLightbarSet> matchTopBottomRows(
        const std::vector<HorizontalLightbarGroup>& rows);
    
    // 参数
    double min_lightbar_area_ = 20.0;           // 最小灯条面积
    double max_lightbar_ratio_ = 6.0;           // 最大长宽比
    double max_group_distance_ = 50.0;          // 同一排灯条最大间距
    double max_top_bottom_distance_ = 100.0;    // 上下排最大距离
    double min_top_bottom_distance_ = 20.0;     // 上下排最小距离
    int min_lightbars_per_row_ = 2;             // 每排最少灯条数
    int max_lightbars_per_row_ = 3;             // 每排最多灯条数
    int brightness_threshold_ = 200;            // 亮度阈值
};

} // namespace drone_detection