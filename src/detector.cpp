#include "detector.h"
#include "config_parser.h"
#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <algorithm>
#include <opencv2/calib3d.hpp>
namespace auto_aim {
Eigen::Vector3d rvec_to_euler(const cv::Mat& rvec) {
    cv::Mat R;
    cv::Rodrigues(rvec, R);

    Eigen::Matrix3d eigen_R;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            eigen_R(i, j) = R.at<double>(i, j);
        }
    }

    return eigen_R.eulerAngles(2, 1, 0);
}

// ===================== Lightbar 构造函数 =====================
Lightbar::Lightbar(cv::RotatedRect rect, int _id)
    : rotated_rect(rect), id(_id), color(0), has_pose(false)
{
    auto size = rect.size;
    length = std::max(size.width, size.height);
    double width = std::min(size.width, size.height);
    ratio = length / width;
    angle_error = std::abs(rect.angle - 45.0);
    center = rect.center;
}


// ===================== 构造函数 =====================
Detector::Detector(const std::string& config_path)
{
    YAML::Node yaml = YAML::LoadFile(config_path);

    camera_matrix_ = (cv::Mat_<double>(3,3) <<
        yaml["camera"]["fx"].as<double>(), 0, yaml["camera"]["cx"].as<double>(),
        0, yaml["camera"]["fy"].as<double>(), yaml["camera"]["cy"].as<double>(),
        0, 0, 1);

    std::vector<double> dist_coeffs = yaml["camera"]["distortion_coeffs"].as<std::vector<double>>();
    dist_coeffs_ = cv::Mat(dist_coeffs, CV_64F);

    threshold_ = yaml["threshold"].as<double>();
    max_angle_error_ = yaml["max_angle_error"].as<double>() / 57.3;
    min_lightbar_ratio_ = yaml["min_lightbar_ratio"].as<double>();
    max_lightbar_ratio_ = yaml["max_lightbar_ratio"].as<double>();
    min_lightbar_length_ = yaml["min_lightbar_length"].as<double>();

    double lightbar_width = yaml["lightbar"]["width"].as<double>(0.01);
    double lightbar_length = yaml["lightbar"]["length"].as<double>(0.05);
    double lightbar_thickness = yaml["lightbar"]["thickness"].as<double>(0.005);

    lightbar_3d_model_ = {
        cv::Point3f(-lightbar_width/2,  lightbar_length/2, -lightbar_thickness/2),
        cv::Point3f( lightbar_width/2,  lightbar_length/2, -lightbar_thickness/2),
        cv::Point3f( lightbar_width/2, -lightbar_length/2, -lightbar_thickness/2),
        cv::Point3f(-lightbar_width/2, -lightbar_length/2, -lightbar_thickness/2)
    };

    debug_ = yaml["debug"].as<bool>(false);
    save_path_ = yaml["save_path"].as<std::string>("patterns");
    std::filesystem::create_directory(save_path_);
}

// ===================== PnP 位姿求解 =====================
bool Detector::get_lightbar_pose(const Lightbar& lightbar, cv::Mat& rvec, cv::Mat& tvec) {
    cv::Point2f pts_2d[4];
    lightbar.rotated_rect.points(pts_2d);
    std::vector<cv::Point2f> image_points(pts_2d, pts_2d + 4);

    return cv::solvePnP(lightbar_3d_model_, image_points,
                        camera_matrix_, dist_coeffs_,
                        rvec, tvec, false, cv::SOLVEPNP_ITERATIVE);
}

// ===================== 颜色识别 =====================
int Detector::get_color(const cv::Mat& bgr_img, const std::vector<cv::Point>& contour) {
    cv::Mat mask = cv::Mat::zeros(bgr_img.size(), CV_8U);
    cv::drawContours(mask, {contour}, 0, 255, -1);

    double mean_r = cv::mean(bgr_img, mask)[2];
    double mean_b = cv::mean(bgr_img, mask)[0];

    if (mean_r > mean_b + 30) return 1;
    if (mean_b > mean_r + 30) return 2;
    return 0;
}

// ===================== 几何筛选 =====================
bool Detector::check_geometry(const Lightbar& lightbar) const {
    if (lightbar.angle_error > max_angle_error_) return false;
    if (lightbar.ratio < min_lightbar_ratio_ || lightbar.ratio > max_lightbar_ratio_) return false;
    if (lightbar.length < min_lightbar_length_) return false;
    return true;
}

// ===================== 主检测函数 =====================
std::list<Lightbar> Detector::detect(const cv::Mat& bgr_img, int frame_count)
{
    std::list<Lightbar> lightbars;

    cv::Mat gray, binary;
    cv::cvtColor(bgr_img, gray, cv::COLOR_BGR2GRAY);
    cv::threshold(gray, binary, threshold_, 255, cv::THRESH_BINARY);

    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(binary, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);

    size_t id = 0;
    for (const auto& cnt : contours) {
        if (cv::contourArea(cnt) < 10) continue;

        auto rect = cv::minAreaRect(cnt);
        Lightbar lb(rect, (int)id);

        if (!check_geometry(lb)) {
            ++id;
            continue;
        }

        lb.color = get_color(bgr_img, cnt);

        cv::Mat rvec, tvec;
        if (get_lightbar_pose(lb, rvec, tvec)) {
            lb.has_pose = true;
            lb.rvec = rvec;
            lb.tvec = tvec;
            lb.euler_angle = rvec_to_euler(rvec);
        }

        lightbars.push_back(lb);
        ++id;
    }

    return lightbars;
}

} // namespace auto_aim