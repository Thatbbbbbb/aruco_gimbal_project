/*
#include "lightbar.h"
#include <opencv2/opencv.hpp>
#include <algorithm>
#include <stdexcept>
#include <cmath>
#include <spdlog/spdlog.h>  // 可选日志，如不需要可注释

namespace lightbar_detection {

// ---------- 全局变量定义 ----------
std::string COLOR_STR = "red";
cv::Scalar DRAW_COLOR(0, 0, 255);
int FPS = 30;
int IMAGE_WIDTH = 1280, IMAGE_HEIGHT = 720;
int ARROW_BRIGHTNESS_THRESHOLD = 200;
float LOCAL_ROI_DISTANCE_RATIO = 0.8f, LOCAL_ROI_WIDTH = 150.0f;
float ARMOR_CENTER_VERTICAL_DISTANCE_THRESHOLD = 20.0f;
float GLOBAL_ROI_LENGTH_RATIO = 1.5f;
float MIN_ARROW_LIGHTLINE_AREA = 30.0f, MAX_ARROW_LIGHTLINE_AREA = 200.0f;
float MAX_ARROW_LIGHTLINE_ASPECT_RATIO = 2.5f;
int MIN_ARROW_LIGHTLINE_NUM = 3, MAX_ARROW_LIGHTLINE_NUM = 6;
float MAX_SAME_ARROW_AREA_RATIO = 2.0f;
float MIN_ARROW_ASPECT_RATIO = 1.2f, MAX_ARROW_ASPECT_RATIO = 3.0f;
float MAX_ARROW_AREA = 800.0f;
float MIN_CENTER_AREA = 20.0f, MAX_CENTER_AREA = 150.0f;
float MAX_CENTER_ASPECT_RATIO = 1.5f;
Mode MODE = Mode::SMALL;
float MIN_BULLET_SPEED = 10.0f, DEFAULT_BULLET_SPEED = 20.0f;
cv::Matx<double, 3, 1> CAMERA_TO_GIMBAL_TRANSLATION_VECTOR(0, 0, 0);
float COMPANSATE_TIME = 0.1f, COMPANSATE_PITCH = 0.0f, COMPANSATE_YAW = 0.0f;
cv::Matx<double, 3, 3> INTRINSIC_MATRIX(1,0,0,0,1,0,0,0,1);
cv::Matx<double, 5, 1> DIST_COEFFS(0,0,0,0,0);
int MIN_FIT_DATA_SIZE = 10, MAX_FIT_DATA_SIZE = 200;

// ---------- 辅助几何函数实现 ----------
double pointPointDistance(const cv::Point2f& a, const cv::Point2f& b) {
    return std::hypot(a.x - b.x, a.y - b.y);
}

double pointLineDistance(const cv::Point2f& p, const cv::Vec4f& line) {
    // line = [vx, vy, x0, y0]
    cv::Point2f dir(line[0], line[1]);
    cv::Point2f pt0(line[2], line[3]);
    return std::abs((p - pt0).cross(dir)) / std::hypot(dir.x, dir.y);
}

double pointLineDistance(const cv::Point2f& p, const cv::Point2f& a, const cv::Point2f& b) {
    cv::Point2f ab = b - a;
    cv::Point2f ap = p - a;
    return std::abs(ap.cross(ab)) / std::hypot(ab.x, ab.y);
}

bool inRange(double val, double minVal, double maxVal) {
    return val >= minVal && val <= maxVal;
}

double angle2Radian(double deg) { return deg * CV_PI / 180.0; }
double radian2Angle(double rad) { return rad * 180.0 / CV_PI; }

// ---------- Lightline 构造函数 ----------
Lightline::Lightline(const std::vector<cv::Point>& contour, const cv::Rect2f& globalRoi,
                     const cv::Rect2f& localRoi)
    : m_contour(contour), m_contourArea(cv::contourArea(contour)), m_rotatedRect(cv::minAreaRect(contour)) {
    m_width = m_rotatedRect.size.width;
    m_length = m_rotatedRect.size.height;
    if (m_width > m_length) std::swap(m_width, m_length);
    m_aspectRatio = m_length / m_width;
    m_center = m_rotatedRect.center;
    m_angle = m_rotatedRect.angle;
    m_area = m_rotatedRect.size.width * m_rotatedRect.size.height;
    std::array<cv::Point2f, 4> pts;
    m_rotatedRect.points(pts.data());
    if (m_rotatedRect.size.width > m_rotatedRect.size.height) {
        m_tl = pts[1]; m_tr = pts[2]; m_bl = pts[0]; m_br = pts[3];
    } else {
        m_tl = pts[0]; m_tr = pts[1]; m_bl = pts[3]; m_br = pts[2];
    }
    cv::Point2f offset = localRoi.tl() + globalRoi.tl();
    m_tl += offset; m_tr += offset; m_bl += offset; m_br += offset;
    m_center += offset;
    m_x = m_center.x; m_y = m_center.y;
}

// ---------- Arrow::set ----------
void Arrow::set(const std::vector<Lightline>& lightlines, const cv::Point2f& roi) {
    std::vector<cv::Point2f> arrowPoints;
    double fillArea = 0.0;
    double pointLineThresh = 0.0;
    for (const auto& l : lightlines) {
        for (const auto& pt : l.m_contour)
            arrowPoints.emplace_back(pt);
        fillArea += l.m_contourArea;
        pointLineThresh += l.m_length / static_cast<double>(lightlines.size());
    }
    cv::Vec4f line;
    cv::fitLine(arrowPoints, line, cv::DIST_L2, 0, 0.01, 0.01);
    m_contour.clear();
    for (const auto& pt : arrowPoints) {
        if (pointLineDistance(pt, line) < pointLineThresh) {
            m_contour.push_back(cv::Point(pt));
        }
    }
    m_rotatedRect = cv::minAreaRect(m_contour);
    m_center = m_rotatedRect.center + roi;
    m_length = m_rotatedRect.size.height;
    m_width = m_rotatedRect.size.width;
    if (m_length < m_width) {
        m_angle = m_rotatedRect.angle;
        std::swap(m_length, m_width);
    } else {
        m_angle = m_rotatedRect.angle + 90;
    }
    m_aspectRatio = m_length / m_width;
    m_area = m_length * m_width;
    m_fillRatio = fillArea / m_area;
}

// ---------- Detector 构造函数 ----------
Detector::Detector()
    : m_localMask(cv::Mat::zeros(IMAGE_HEIGHT, IMAGE_WIDTH, CV_8U)),
      m_globalRoi(0, 0, IMAGE_WIDTH, IMAGE_HEIGHT) {
    // 其他成员默认初始化
}

// ---------- Detector::detectArrow ----------
bool Detector::detectArrow() {
    std::vector<Lightline> lightlines;
    findArrowLightlines(m_imageArrow, lightlines, m_globalRoi);
#if SHOW_IMAGE >= 2
    for (const auto& lightline : lightlines) {
        draw(lightline, cv::Scalar(0, 255, 0)); // 绿色
    }
#endif
    if (!findArrow(m_arrow, lightlines, m_globalRoi)) {
        return false;
    }
#if SHOW_IMAGE >= 1
    draw(m_arrow.m_rotatedRect, cv::Scalar(255, 255, 255), 2);
#endif
    return true;
}

// ---------- 自由函数实现 ----------
void findArrowLightlines(const cv::Mat& binary, std::vector<Lightline>& lightlines, const cv::Rect2f& roi) {
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(binary, contours, cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE);
    for (const auto& contour : contours) {
        Lightline ll(contour, roi);
        if (!inRange(ll.m_area, MIN_ARROW_LIGHTLINE_AREA, MAX_ARROW_LIGHTLINE_AREA))
            continue;
        if (ll.m_aspectRatio > MAX_ARROW_LIGHTLINE_ASPECT_RATIO)
            continue;
        lightlines.emplace_back(std::move(ll));
    }
}

bool sameArrow(const Lightline& l1, const Lightline& l2) {
    double areaRatio = l1.m_area / l2.m_area;
    if (!inRange(areaRatio, 1.0 / MAX_SAME_ARROW_AREA_RATIO, MAX_SAME_ARROW_AREA_RATIO))
        return false;
    double distance = pointPointDistance(l1.m_rotatedRect.center, l2.m_rotatedRect.center);
    double maxDist = 1.2 * (l1.m_width + l2.m_width);
    if (distance > maxDist) return false;
    return true;
}

bool findArrow(Arrow& arrow, const std::vector<Lightline>& lightlines, const cv::Rect2f& roi) {
    std::vector<int> labels;
    cv::partition(lightlines, labels, sameArrow);
    std::vector<std::pair<int, int>> data;
    for (int label : labels) {
        auto it = std::find_if(data.begin(), data.end(),
                               [label](const std::pair<int,int>& p) { return p.first == label; });
        if (it == data.end())
            data.emplace_back(label, 1);
        else
            it->second++;
    }
    if (data.empty()) return false;
    auto maxElem = std::max_element(data.begin(), data.end(),
        [](const std::pair<int,int>& a, const std::pair<int,int>& b) { return a.second < b.second; });
    int maxLabel = maxElem->first;
    int maxNum = maxElem->second;
    if (!inRange(maxNum, MIN_ARROW_LIGHTLINE_NUM, MAX_ARROW_LIGHTLINE_NUM))
        return false;
    std::vector<int> indices;
    for (size_t i = 0; i < labels.size(); ++i)
        if (labels[i] == maxLabel) indices.push_back(static_cast<int>(i));
    std::vector<Lightline> arrowLightlines;
    for (int idx : indices)
        arrowLightlines.push_back(lightlines[idx]);
    arrow.set(arrowLightlines, roi.tl());
    if (!inRange(arrow.m_aspectRatio, MIN_ARROW_ASPECT_RATIO, MAX_ARROW_ASPECT_RATIO))
        return false;
    if (arrow.m_area > MAX_ARROW_AREA)
        return false;
    return true;
}

// ---------- loadConfig 实现 ----------
void loadConfig(const std::string& filename) {
    cv::FileStorage fs(filename, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        throw std::runtime_error("Cannot open config file: " + filename);
    }
    // color
    std::string colorStr;
    fs["color"] >> colorStr;
    std::transform(colorStr.begin(), colorStr.end(), colorStr.begin(),
                   [](unsigned char c) { return std::tolower(c); });
    if (colorStr == "red") {
        COLOR_STR = "red";
        DRAW_COLOR = cv::Scalar(0, 0, 255); // 红色
    } else if (colorStr == "blue") {
        COLOR_STR = "blue";
        DRAW_COLOR = cv::Scalar(255, 0, 0); // 蓝色
    } else {
        throw std::runtime_error("unknown color " + colorStr);
    }
    fs["fps"] >> FPS;
    auto fsImage = fs["image"];
    fsImage["width"] >> IMAGE_WIDTH;
    fsImage["height"] >> IMAGE_HEIGHT;
    auto fsDetect = fs["detect"];
    auto fsBrightness = fsDetect["brightness_threshold"][colorStr];
    fsBrightness["arrow"] >> ARROW_BRIGHTNESS_THRESHOLD;
    auto fsLocalRoi = fsDetect["local_roi"];
    fsLocalRoi["distance_ratio"] >> LOCAL_ROI_DISTANCE_RATIO;
    fsLocalRoi["width"] >> LOCAL_ROI_WIDTH;
    fsDetect["armor_center_vertical_distance_threshold"] >> ARMOR_CENTER_VERTICAL_DISTANCE_THRESHOLD;
    fsDetect["global_roi_length_ratio"] >> GLOBAL_ROI_LENGTH_RATIO;
    // arrow
    auto fsArrow = fsDetect["arrow"];
    fsArrow["lightline"]["area"]["min"] >> MIN_ARROW_LIGHTLINE_AREA;
    fsArrow["lightline"]["area"]["max"] >> MAX_ARROW_LIGHTLINE_AREA;
    fsArrow["lightline"]["aspect_ratio_max"] >> MAX_ARROW_LIGHTLINE_ASPECT_RATIO;
    fsArrow["lightline"]["num"]["min"] >> MIN_ARROW_LIGHTLINE_NUM;
    fsArrow["lightline"]["num"]["max"] >> MAX_ARROW_LIGHTLINE_NUM;
    fsArrow["same_area_ratio_max"] >> MAX_SAME_ARROW_AREA_RATIO;
    fsArrow["aspect_ratio"]["min"] >> MIN_ARROW_ASPECT_RATIO;
    fsArrow["aspect_ratio"]["max"] >> MAX_ARROW_ASPECT_RATIO;
    fsArrow["area_max"] >> MAX_ARROW_AREA;
    // centerR
    auto fsCenterR = fsDetect["centerR"];
    fsCenterR["area"]["min"] >> MIN_CENTER_AREA;
    fsCenterR["area"]["max"] >> MAX_CENTER_AREA;
    fsCenterR["aspect_ratio_max"] >> MAX_CENTER_ASPECT_RATIO;
    // mode
    std::string modeStr;
    fs["mode"] >> modeStr;
    std::transform(modeStr.begin(), modeStr.end(), modeStr.begin(),
                   [](unsigned char c) { return std::tolower(c); });
    if (modeStr == "small") MODE = Mode::SMALL;
    else if (modeStr == "big") MODE = Mode::BIG;
    else throw std::runtime_error("unknown mode " + modeStr);
    // calculate
    auto fsCal = fs["calculate"];
    fsCal["bullet_speed"]["min"] >> MIN_BULLET_SPEED;
    fsCal["bullet_speed"]["default"] >> DEFAULT_BULLET_SPEED;
    auto fsc2g = fsCal["tvec_c2g"];
    if (fsc2g.type() == cv::FileNode::SEQ) {
        int idx = 0;
        for (auto it = fsc2g.begin(); it != fsc2g.end(); ++it) {
            CAMERA_TO_GIMBAL_TRANSLATION_VECTOR(idx) = static_cast<double>(*it);
            ++idx;
        }
    }
    auto fsCompensate = fsCal["compensate"];
    fsCompensate["time"] >> COMPANSATE_TIME;
    fsCompensate["pitch"] >> COMPANSATE_PITCH;
    fsCompensate["yaw"] >> COMPANSATE_YAW;
    fsCal["intrinsic_matrix"] >> INTRINSIC_MATRIX;
    fsCal["distortion"] >> DIST_COEFFS;
    fsCal["fit_data_size"]["min"] >> MIN_FIT_DATA_SIZE;
    fsCal["fit_data_size"]["max"] >> MAX_FIT_DATA_SIZE;
    fs.release();
}

// ---------- Detector 绘图函数的空实现（可自行完善） ----------
void Detector::draw(const Lightline& lightline, const cv::Scalar& color, int thickness,
                    const cv::Rect2f& localRoi) {
    draw(lightline.m_rotatedRect, color, thickness, localRoi);
}
void Detector::draw(const cv::RotatedRect& rotatedRect, const cv::Scalar& color, int thickness,
                    const cv::Rect2f& localRoi) {
    std::array<cv::Point2f, 4> vertices;
    rotatedRect.points(vertices.data());
    draw(vertices.data(), vertices.size(), color, thickness, localRoi);
}
void Detector::draw(const cv::Rect2f& rect, const cv::Scalar& color, int thickness,
                    const cv::Rect2f& localRoi) {
    cv::Rect2f temp = rect;
    temp.x += localRoi.x + m_globalRoi.x;
    temp.y += localRoi.y + m_globalRoi.y;
    cv::rectangle(m_imageShow, temp, color, thickness);
}
void Detector::draw(const std::vector<cv::Point2f>& points, const cv::Scalar& color, int thickness,
                    const cv::Rect2f& localRoi) {
    for (size_t i = 0; i < points.size(); ++i) {
        cv::line(m_imageShow, points[i] + localRoi.tl() + m_globalRoi.tl(),
                 points[(i+1)%points.size()] + localRoi.tl() + m_globalRoi.tl(),
                 color, thickness);
    }
}
void Detector::draw(const cv::Point2f* points, size_t size, const cv::Scalar& color, int thickness,
                    const cv::Rect2f& localRoi) {
    for (size_t i = 0; i < size; ++i) {
        cv::line(m_imageShow, points[i] + localRoi.tl() + m_globalRoi.tl(),
                 points[(i+1)%size] + localRoi.tl() + m_globalRoi.tl(),
                 color, thickness);
    }
}
void Detector::visualize() const {
    cv::imshow("lightbar detection", m_imageShow);
}

} // namespace lightbar_detection
 */