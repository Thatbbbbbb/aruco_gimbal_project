#pragma once

#include <vector>
#include <opencv2/opencv.hpp>

namespace lightbar_detection {

// ---------- 全局参数变量（在 cpp 中定义） ----------
extern std::string COLOR_STR;
extern cv::Scalar DRAW_COLOR;
extern int FPS;
extern int IMAGE_WIDTH, IMAGE_HEIGHT;
extern int ARROW_BRIGHTNESS_THRESHOLD, ARMOR_BRIGHTNESS_THRESHOLD;
extern float LOCAL_ROI_DISTANCE_RATIO, LOCAL_ROI_WIDTH;
extern float ARMOR_CENTER_VERTICAL_DISTANCE_THRESHOLD;
extern float GLOBAL_ROI_LENGTH_RATIO;
extern float MIN_ARROW_LIGHTLINE_AREA, MAX_ARROW_LIGHTLINE_AREA;
extern float MAX_ARROW_LIGHTLINE_ASPECT_RATIO;
extern int MIN_ARROW_LIGHTLINE_NUM, MAX_ARROW_LIGHTLINE_NUM;
extern float MAX_SAME_ARROW_AREA_RATIO;
extern float MIN_ARROW_ASPECT_RATIO, MAX_ARROW_ASPECT_RATIO;
extern float MAX_ARROW_AREA;
extern float MIN_ARMOR_LIGHTLINE_AREA, MAX_ARMOR_LIGHTLINE_AREA;
extern float MIN_ARMOR_LIGHTLINE_CONTOUR_AREA, MAX_ARMOR_LIGHTLINE_CONTOUR_AREA;
extern float MIN_ARMOR_LIGHTLINE_ASPECT_RATIO, MAX_ARMOR_LIGHTLINE_ASPECT_RATIO;
extern float MAX_SAME_ARMOR_AREA_RATIO;
extern float MIN_SAME_ARMOR_DISTANCE, MAX_SAME_ARMOR_DISTANCE;
extern float MIN_CENTER_AREA, MAX_CENTER_AREA;
extern float MAX_CENTER_ASPECT_RATIO;

enum class Mode { SMALL, BIG };
extern Mode MODE;
extern float MIN_BULLET_SPEED, DEFAULT_BULLET_SPEED;
extern cv::Matx<double, 3, 1> CAMERA_TO_GIMBAL_TRANSLATION_VECTOR;
extern float COMPANSATE_TIME, COMPANSATE_PITCH, COMPANSATE_YAW;
extern cv::Matx<double, 3, 3> INTRINSIC_MATRIX;
extern cv::Matx<double, 5, 1> DIST_COEFFS;
extern float ARMOR_OUTSIDE_WIDTH, ARMOR_OUTSIDE_HEIGHT, ARMOR_OUTSIDE_Y;
extern float ARMOR_INSIDE_WIDTH, ARMOR_INSIDE_Y;
extern int MIN_FIT_DATA_SIZE, MAX_FIT_DATA_SIZE;

// ---------- 辅助几何函数 ----------
double pointPointDistance(const cv::Point2f& a, const cv::Point2f& b);
double pointLineDistance(const cv::Point2f& p, const cv::Vec4f& line);
double pointLineDistance(const cv::Point2f& p, const cv::Point2f& a, const cv::Point2f& b);
bool inRange(double val, double minVal, double maxVal);
double angle2Radian(double deg);
double radian2Angle(double rad);

// ---------- 灯条结构 ----------
struct Lightline {
    Lightline() = default;
    Lightline(const std::vector<cv::Point>& contour, const cv::Rect2f& globalRoi,
              const cv::Rect2f& localRoi = cv::Rect2f(0, 0, IMAGE_WIDTH, IMAGE_HEIGHT));

    std::vector<cv::Point> m_contour;
    double m_contourArea;
    double m_area;
    cv::RotatedRect m_rotatedRect;
    cv::Point2f m_tl, m_tr, m_bl, m_br;
    cv::Point2f m_center;
    double m_length, m_width;
    double m_x, m_y;
    double m_angle;
    double m_aspectRatio;
};

// ---------- 箭头结构 ----------
struct Arrow {
    Arrow() = default;
    void set(const std::vector<Lightline>& lightlines, const cv::Point2f& roi);

    std::vector<cv::Point> m_contour;
    cv::RotatedRect m_rotatedRect;
    double m_length, m_width;
    cv::Point2f m_center;
    double m_angle;
    double m_aspectRatio;
    double m_area;
    double m_fillRatio;
};

// ---------- 检测器类 ----------
class Detector {
public:
    Detector();
    bool detectArrow();   // 仅实现箭头检测，其他函数可后续扩展
    // 可视化
    void visualize() const;

private:
    cv::Mat m_imageRaw;
    cv::Mat m_imageArrow;
    cv::Mat m_imageArmor;
    cv::Mat m_imageCenter;
    cv::Mat m_imageShow;
    cv::Mat m_localMask;
    cv::Rect2f m_globalRoi;
    cv::Rect2f m_armorRoi;
    cv::Rect2f m_centerRoi;
    Arrow m_arrow;

    void draw(const Lightline& lightline, const cv::Scalar& color, int thickness = 1,
              const cv::Rect2f& localRoi = cv::Rect2f(0, 0, IMAGE_WIDTH, IMAGE_HEIGHT));
    void draw(const cv::RotatedRect& rotatedRect, const cv::Scalar& color, int thickness = 1,
              const cv::Rect2f& localRoi = cv::Rect2f(0, 0, IMAGE_WIDTH, IMAGE_HEIGHT));
    void draw(const cv::Rect2f& rect, const cv::Scalar& color, int thickness = 1,
              const cv::Rect2f& localRoi = cv::Rect2f(0, 0, IMAGE_WIDTH, IMAGE_HEIGHT));
    void draw(const std::vector<cv::Point2f>& points, const cv::Scalar& color, int thickness = 1,
              const cv::Rect2f& localRoi = cv::Rect2f(0, 0, IMAGE_WIDTH, IMAGE_HEIGHT));
    void draw(const cv::Point2f* points, size_t size, const cv::Scalar& color, int thickness = 1,
              const cv::Rect2f& localRoi = cv::Rect2f(0, 0, IMAGE_WIDTH, IMAGE_HEIGHT));
};

// ---------- 自由函数声明 ----------
void loadConfig(const std::string& filename = "lightbar.yaml");
void findArrowLightlines(const cv::Mat& binary, std::vector<Lightline>& lightlines, const cv::Rect2f& roi);
bool findArrow(Arrow& arrow, const std::vector<Lightline>& lightlines, const cv::Rect2f& roi);
bool sameArrow(const Lightline& l1, const Lightline& l2);

} // namespace lightbar_detection