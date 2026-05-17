#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <cmath>
#include <iomanip>
#include <chrono>
#include "camera.hpp"
#include "serial_port.h"
#include "engineer_kinematics.hpp"
#include <eigen3/Eigen/Dense>

using namespace cv;
using namespace std;
using namespace hitcrt::kinematics;
using namespace hitcrt::serial;

// --------------------------------------------
// 全局相机内参和畸变系数（根据实际标定填写）
// --------------------------------------------
static Mat cameraMatrix = (Mat_<double>(3,3) <<
    9991.10399893341, 0, 718.418901980155,
    0, 9991.82243368060, 558.088945484812,
    0, 0, 1);
static Mat distCoeffs = (Mat_<double>(5,1) <<
    0.209283116250229,
    0.676385453898682,
    0, 0, 0);

// --------------------------------------------
// 矩形信息结构体
// --------------------------------------------
struct RectInfo {
    RotatedRect rect;
    float longSideLen;
    float shortSideLen;
    float orientation;
    float area;
};

// --------------------------------------------
// 函数声明
// --------------------------------------------
Mat getRedMask(const Mat& hsv);
float computeLongSideOrientation(const RotatedRect& rect, float& longSideLen, float& shortSideLen);
vector<RectInfo> extractRectangles(const vector<vector<Point>>& contours, double minArea, float aspectRatioMin, float aspectRatioMax);
bool areMatchingRects(const RectInfo& r1, const RectInfo& r2, float angleTolerance, float areaTolerance, float distanceFactor);
bool findMatchingRectPair(const vector<RectInfo>& rects, pair<int, int>& pairIdx);
Rect drawGreenBoundingBox(Mat& image, const RectInfo& r1, const RectInfo& r2);
Vec3d rotationMatrixToEulerAngles(const Mat& R);
bool solvePnPForRect(const vector<Point2f>& imgPoints, float objectWidth, float objectHeight,
                     Mat& rvec, Mat& tvec);
Mat processImage(Mat image, double& pitch, double& yaw, double& distance, Mat& rvec, Mat& tvec);

// --------------------------------------------
// 颜色掩码
// --------------------------------------------
Mat getRedMask(const Mat& hsv) {
    Mat mask1, mask2;
    inRange(hsv, Scalar(0, 100, 100), Scalar(10, 255, 255), mask1);
    inRange(hsv, Scalar(160, 100, 100), Scalar(180, 255, 255), mask2);
    return mask1 | mask2;
}

// 计算矩形长边方向（0~180度）
float computeLongSideOrientation(const RotatedRect& rect, float& longSideLen, float& shortSideLen) {
    float width = rect.size.width;
    float height = rect.size.height;
    if (width >= height) {
        longSideLen = width;
        shortSideLen = height;
        float angle = rect.angle;
        if (angle < 0) angle += 180.0f;
        return angle;
    } else {
        longSideLen = height;
        shortSideLen = width;
        float angle = rect.angle + 90.0f;
        if (angle >= 180.0f) angle -= 180.0f;
        if (angle < 0) angle += 180.0f;
        return angle;
    }
}

// 提取符合条件的矩形
vector<RectInfo> extractRectangles(const vector<vector<Point>>& contours,
                                   double minArea = 500.0,
                                   float aspectRatioMin = 0.3f,
                                   float aspectRatioMax = 3.0f) {
    vector<RectInfo> rects;
    for (const auto& contour : contours) {
        RotatedRect rotRect = minAreaRect(contour);
        float area = rotRect.size.width * rotRect.size.height;
        if (area < minArea) continue;
        float longLen, shortLen;
        float orient = computeLongSideOrientation(rotRect, longLen, shortLen);
        float ratio = longLen / shortLen;
        if (ratio < aspectRatioMin || ratio > aspectRatioMax) continue;
        rects.push_back({rotRect, longLen, shortLen, orient, area});
    }
    return rects;
}

// 判断两个矩形是否匹配（平行、面积相近、距离合适）
bool areMatchingRects(const RectInfo& r1, const RectInfo& r2,
                      float angleTolerance = 10.0f,
                      float areaTolerance = 0.1f,
                      float distanceFactor = 1.5f) {
    float angleDiff = std::abs(r1.orientation - r2.orientation);
    angleDiff = std::min(angleDiff, 180.0f - angleDiff);
    if (angleDiff > angleTolerance) return false;
    float areaDiff = std::abs(r1.area - r2.area);
    float maxArea = std::max(r1.area, r2.area);
    if (areaDiff > maxArea * areaTolerance) return false;
    Point2f center1 = r1.rect.center;
    Point2f center2 = r2.rect.center;
    float distance = norm(center1 - center2);
    float maxLongLen = std::max(r1.longSideLen, r2.longSideLen);
    if (distance > maxLongLen * distanceFactor) return false;
    return true;
}

// 查找匹配的矩形对
bool findMatchingRectPair(const vector<RectInfo>& rects, pair<int, int>& pairIdx) {
    int n = rects.size();
    for (int i = 0; i < n; ++i) {
        for (int j = i + 1; j < n; ++j) {
            if (areMatchingRects(rects[i], rects[j])) {
                pairIdx = {i, j};
                return true;
            }
        }
    }
    return false;
}

// 绘制绿色外框并返回矩形
Rect drawGreenBoundingBox(Mat& image, const RectInfo& r1, const RectInfo& r2) {
    vector<Point2f> corners(8);
    Point2f corners1[4], corners2[4];
    r1.rect.points(corners1);
    r2.rect.points(corners2);
    for (int i = 0; i < 4; ++i) {
        corners[i] = corners1[i];
        corners[i + 4] = corners2[i];
    }
    Rect boundingBox = boundingRect(corners);
    rectangle(image, boundingBox, Scalar(0, 255, 0), 2);
    return boundingBox;
}

// 旋转矩阵转欧拉角（顺序: roll, pitch, yaw）
Vec3d rotationMatrixToEulerAngles(const Mat& R) {
    double sy = sqrt(R.at<double>(0,0) * R.at<double>(0,0) + R.at<double>(1,0) * R.at<double>(1,0));
    bool singular = sy < 1e-6;
    double x, y, z;
    if (!singular) {
        x = atan2(R.at<double>(2,1), R.at<double>(2,2));
        y = atan2(-R.at<double>(2,0), sy);
        z = atan2(R.at<double>(1,0), R.at<double>(0,0));
    } else {
        x = atan2(-R.at<double>(1,2), R.at<double>(1,1));
        y = atan2(-R.at<double>(2,0), sy);
        z = 0;
    }
    return Vec3d(x, y, z);
}

// PnP解算，返回旋转向量和平移向量（相机坐标系）
bool solvePnPForRect(const vector<Point2f>& imgPoints, float objectWidth, float objectHeight,
                     Mat& rvec, Mat& tvec) {
    if (imgPoints.size() != 4) {
        cerr << "需要4个角点进行PnP解算" << endl;
        return false;
    }
    vector<Point3f> objPoints;
    objPoints.push_back(Point3f(-objectWidth/2,  objectHeight/2, 0));
    objPoints.push_back(Point3f( objectWidth/2,  objectHeight/2, 0));
    objPoints.push_back(Point3f( objectWidth/2, -objectHeight/2, 0));
    objPoints.push_back(Point3f(-objectWidth/2, -objectHeight/2, 0));

    bool success = solvePnP(objPoints, imgPoints, cameraMatrix, distCoeffs,
                            rvec, tvec, false, SOLVEPNP_ITERATIVE);
    if (!success) {
        cerr << "PnP解算失败" << endl;
    }
    return success;
}

// 核心图像处理函数，返回处理后的图像，同时输出旋转向量和平移向量
Mat processImage(Mat image, double& pitch, double& yaw, double& distance, Mat& rvec, Mat& tvec) {
    if (image.empty()) {
        cerr << "输入图像为空" << endl;
        return image;
    }
    Mat hsv;
    cvtColor(image, hsv, COLOR_BGR2HSV);
    Mat mask = getRedMask(hsv);
    Mat kernel = getStructuringElement(MORPH_RECT, Size(5, 5));
    Mat dilated;
    dilate(mask, dilated, kernel);
    vector<vector<Point>> contours;
    findContours(dilated, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
    vector<RectInfo> rects = extractRectangles(contours, 500.0, 0.3f, 3.0f);
    pair<int, int> matchingPair;
    if (findMatchingRectPair(rects, matchingPair)) {
        const RectInfo& r1 = rects[matchingPair.first];
        const RectInfo& r2 = rects[matchingPair.second];
        Rect greenBox = drawGreenBoundingBox(image, r1, r2);
        // 获取绿色外接矩形的四个角点
        vector<Point2f> boxCorners(4);
        boxCorners[0] = Point2f(greenBox.x, greenBox.y);
        boxCorners[1] = Point2f(greenBox.x + greenBox.width, greenBox.y);
        boxCorners[2] = Point2f(greenBox.x + greenBox.width, greenBox.y + greenBox.height);
        boxCorners[3] = Point2f(greenBox.x, greenBox.y + greenBox.height);
        cout << "\n绿色外框角点（像素坐标）：" << endl;
        cout << "左上: " << boxCorners[0] << endl;
        cout << "右上: " << boxCorners[1] << endl;
        cout << "右下: " << boxCorners[2] << endl;
        cout << "左下: " << boxCorners[3] << endl;

        // PnP解算，目标尺寸（单位：米）请根据实际修改
        float targetWidth = 0.05f;   // 宽度5cm
        float targetHeight = 0.05f;  // 高度5cm
        if (solvePnPForRect(boxCorners, targetWidth, targetHeight, rvec, tvec)) {
            // 计算距离
            distance = norm(tvec);
            // 计算欧拉角
            Mat R;
            Rodrigues(rvec, R);
            Vec3d euler = rotationMatrixToEulerAngles(R);
            pitch = euler[1] * 180.0 / CV_PI;
            yaw   = euler[2] * 180.0 / CV_PI;
            cout << "距离: " << distance << " 米" << endl;
            cout << "Pitch (俯仰): " << fixed << setprecision(2) << pitch << " °" << endl;
            cout << "Yaw   (偏航): " << yaw << " °" << endl;
        } else {
            pitch = yaw = distance = 0;
        }
    } else {
        cout << "未找到匹配的矩形对" << endl;
        pitch = yaw = distance = 0;
        rvec = Mat::zeros(3, 1, CV_64F);
        tvec = Mat::zeros(3, 1, CV_64F);
    }
    return image;
}

// --------------------------------------------
// 主函数：读取视频，检测，逆运动学求解，串口发送
// --------------------------------------------
int main(int argc, char** argv) {
    // 1. 初始化串口
    SerialPort serial("/dev/ttyUSB0", 100);
    if (!serial.open_port()) {
        std::cerr << "Failed to init serial port!" << std::endl;
        return -1;
    }

    // 2. 初始化运动学（读取配置文件中的参数，这里用默认构造，也可以从文件读取）
    params my_params;
    // 如果你有配置文件，可以在这里读取并覆盖my_params
    engineer_kinematics kin(my_params);

    // 假设相机到机械臂基座的变换矩阵为单位阵（相机坐标系即基座坐标系）
    // 若实际有外参，请替换为标定结果
    Eigen::Isometry3d T_cam2base = Eigen::Isometry3d::Identity();

    // 3. 打开视频（也可以换成相机）
    std::string videoPath = "/opt/MVS/bin/Temp/Data/MV-CS016-10UC+DA4886227/Video_20260510164431821.avi";
    cv::VideoCapture cap(videoPath);
    if (!cap.isOpened()) {
        std::cerr << "视频打开失败！路径：" << videoPath << std::endl;
        return -1;
    }

    // 获取视频原始帧率，用于实时播放控制
    double video_fps = cap.get(cv::CAP_PROP_FPS);
    int delay_ms = int(1000.0 / video_fps);
    if (delay_ms < 1) delay_ms = 30;  // 防止异常
    std::cout << "视频帧率: " << video_fps << " fps, 延时: " << delay_ms << " ms" << std::endl;

    int frame_count = 0;
    int total_frames = 0;

    // 用于存储当前关节角（弧度），初始可给任意值，用于平滑选择解
    std::array<float, 3> current_joints = {0, 0, 0};

    while (true) {
        auto start_time = std::chrono::steady_clock::now();

        cv::Mat frame;
        cap >> frame;
        if (frame.empty()) break;

        frame_count++;
        total_frames++;

        // 处理图像，获取旋转向量、平移向量及欧拉角（用于显示）
        Mat rvec, tvec;
        double pitch_deg = 0, yaw_deg = 0, distance = 0;
        cv::Mat processed = processImage(frame, pitch_deg, yaw_deg, distance, rvec, tvec);

        if (distance > 0) {
        // 构造目标在相机坐标系下的位姿 T_cam_target
        Mat R_cv;
        Rodrigues(rvec, R_cv);
        Eigen::Matrix3d R_eigen;
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
                R_eigen(i, j) = R_cv.at<double>(i, j);
        Eigen::Vector3d t_eigen(tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2));

        // ========== 添加激光相对于相机的偏移
        t_eigen.y() += 0.07317399;   // Y方向偏移（米）
        t_eigen.z() += 0.0185156;    // Z方向偏移（米）
        // ============================================
        // ✅ 正确构造 Isometry3d
        Eigen::Isometry3d T_cam_target = Eigen::Isometry3d::Identity();
        T_cam_target.rotate(R_eigen);
        T_cam_target.translation() = t_eigen;

        // 假设相机到基座的变换也是 Isometry3d（此处用单位阵示例）
        Eigen::Isometry3d T_cam2base = Eigen::Isometry3d::Identity();

        // 变换到基座坐标系（乘积仍为 Isometry3d）
        Eigen::Isometry3d T_base_target = T_cam2base * T_cam_target;

        // 逆运动学求解
        std::vector<std::array<float, 3>> solutions;
        if (kin.inverse_kinematics(T_base_target, solutions)) {
            // ... 选择解并发送 ...
        }
    }
        // 显示图像
        cv::imshow("Red Light Bar Detection", processed);

        // 帧率控制：按视频原始帧率播放
        auto end_time = std::chrono::steady_clock::now();
        int elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
        int wait_ms = std::max(1, delay_ms - elapsed_ms);
        if (cv::waitKey(wait_ms) == 27) break;   // ESC退出

        // 每30帧打印一次处理帧数
        if (frame_count % 30 == 0) {
            std::cout << "已处理帧数: " << frame_count << std::endl;
        }
    }

    std::cout << "总共处理帧数: " << total_frames << std::endl;
    cap.release();
    cv::destroyAllWindows();
    return 0;
}