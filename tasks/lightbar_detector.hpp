#pragma once

#include <vector>
#include <opencv2/opencv.hpp>

namespace drone_detection {

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

struct HorizontalLightbarGroup {
    std::vector<SingleLightbar> lightbars;
    cv::Point2f center;
    float y_level;
    float confidence;
};

struct ParallelLightbarSet {
    HorizontalLightbarGroup top_row;
    HorizontalLightbarGroup bottom_row;
    cv::Point2f center;
    std::vector<cv::Point2f> corners;
    float confidence;
    float top_to_bottom_distance;
};

class LightbarDetector {
public:
    LightbarDetector();

    bool loadConfig(const std::string& path);

    std::vector<ParallelLightbarSet> detect(const cv::Mat& img, const cv::Rect& roi);

    void setMinLightbarArea(double area) { min_lightbar_area_ = area; }
    void setMaxLightbarRatio(double ratio) { max_lightbar_ratio_ = ratio; }
    void setMaxGroupDistance(double distance) { max_group_distance_ = distance; }
    void setMaxTopBottomDistance(double distance) { max_top_bottom_distance_ = distance; }
    void setMinLightbarsPerRow(int min) { min_lightbars_per_row_ = min; }
    void setMaxLightbarsPerRow(int max) { max_lightbars_per_row_ = max; }

private:
    cv::Mat extractColor(const cv::Mat& img);
    std::vector<SingleLightbar> detectLightbars(const cv::Mat& img, const cv::Rect& roi);
    std::vector<HorizontalLightbarGroup> groupByRow(const std::vector<SingleLightbar>& lightbars);
    HorizontalLightbarGroup filterRowGroup(const std::vector<SingleLightbar>& row_candidates);
    std::vector<ParallelLightbarSet> matchTopBottomRows(const std::vector<HorizontalLightbarGroup>& rows);

    std::string lightbar_color_ = "blue";
    int h_min_ = 0, h_max_ = 180;
    int s_min_ = 0, s_max_ = 255;
    int v_min_ = 0, v_max_ = 255;

    double min_lightbar_area_ = 20.0;
    double max_lightbar_area_ = 1000.0;
    double min_lightbar_ratio_ = 3.0;
    double max_lightbar_ratio_ = 6.0;
    double max_angle_error_ = 20.0;
    double max_group_distance_ = 50.0;
    double min_top_bottom_distance_ = 20.0;
    double max_top_bottom_distance_ = 100.0;
    int min_lightbars_per_row_ = 2;
    int max_lightbars_per_row_ = 3;
};

} // namespace drone_detection