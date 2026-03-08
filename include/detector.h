#pragma once
#include <opencv2/opencv.hpp>
#include <Eigen/Geometry>
#include <list>
#include <string>
#include <vector>


namespace   auto_aim {

struct Lightbar {
    cv::RotatedRect rotated_rect;
    int id;
    int color;

    bool has_pose = false;
    cv::Mat rvec;
    cv::Mat tvec;
    Eigen::Vector3d euler_angle;

    double angle_error;
    double ratio;
    double length;
    cv::Point2f center;

    Lightbar(cv::RotatedRect rect, int _id);
};


class Detector {
public:
    Detector(const std::string& config_path);

    std::list<Lightbar> detect(const cv::Mat& bgr_img, int frame_count);

private:
    cv::Mat camera_matrix_;
    cv::Mat dist_coeffs_;
    std::vector<cv::Point3f> lightbar_3d_model_;

    double threshold_;
    double max_angle_error_;
    double min_lightbar_ratio_;
    double max_lightbar_ratio_;
    double min_lightbar_length_;

    bool debug_;
    std::string save_path_;

    bool get_lightbar_pose(const Lightbar& lightbar, cv::Mat& rvec, cv::Mat& tvec);
    int get_color(const cv::Mat& bgr_img, const std::vector<cv::Point>& contour);
    bool check_geometry(const Lightbar& lightbar) const;
};

} // namespace auto_aim