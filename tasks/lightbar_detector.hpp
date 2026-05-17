#pragma once

#include <vector>
#include <opencv2/opencv.hpp>

namespace drone_detection {

class LightbarDetector {
public:
    LightbarDetector();

    bool loadConfig(const std::string& path);
    cv::Point2f getLightbarCenter(const cv::Mat& img, const cv::Rect& roi,std::string& color); 
    

private:
    std::string lightbar_color_ = "red";
    int h_min_ = 0, h_max_ = 180;
    int s_min_ = 0, s_max_ = 255;
    int v_min_ = 0, v_max_ = 255;
};

} // namespace drone_detection