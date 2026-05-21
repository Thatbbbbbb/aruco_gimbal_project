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
#include <numeric>   // for std::iota
#include <array>     // for std::array
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
// --------------------------------------------
// 过滤共线且长距离的红色点（干扰灯带）
// 输入：候选矩形列表，图像尺寸
// 输出：过滤后的矩形列表（移除线性干扰点）
// --------------------------------------------
vector<RectInfo> filterLinearRedPoints(const vector<RectInfo>& rects, const Size& imgSize) {
    if (rects.size() < 3) return rects;  // 至少需要3个点才能形成线

    // 提取所有矩形中心点
    vector<Point2f> centers;
    for (const auto& r : rects) centers.push_back(r.rect.center);

    // 拟合直线（最小二乘）
    Vec4f line;
    fitLine(centers, line, DIST_L2, 0, 0.01, 0.01);
    Point2f dir(line[0], line[1]);
    Point2f pointOnLine(line[2], line[3]);
    float dirNorm = norm(dir);
    if (dirNorm < 1e-6) return rects;
    dir /= dirNorm;  // 单位方向向量

    // 计算每个点到直线的距离
    vector<float> distances(centers.size());
    for (size_t i = 0; i < centers.size(); ++i) {
        Point2f toPt = centers[i] - pointOnLine;
        float proj = toPt.dot(dir);
        Point2f foot = pointOnLine + dir * proj;
        distances[i] = norm(centers[i] - foot);
    }

    // 共线性判断：最大距离小于阈值（例如 5 像素）
    const float maxDistThresh = 5.0f;
    float maxDist = *max_element(distances.begin(), distances.end());
    if (maxDist > maxDistThresh) return rects;  // 不共线，保留所有

    // 计算每个点在直线上的投影坐标
    vector<float> projs(centers.size());
    for (size_t i = 0; i < centers.size(); ++i) {
        projs[i] = (centers[i] - pointOnLine).dot(dir);
    }

    // 按投影坐标排序，得到索引顺序
    vector<size_t> idx(centers.size());
    iota(idx.begin(), idx.end(), 0);
    sort(idx.begin(), idx.end(), [&](size_t a, size_t b) { return projs[a] < projs[b]; });

    // 聚类：相邻投影差小于阈值（如 10 像素）视为同一组
    const float gapThresh = 10.0f;
    vector<vector<size_t>> clusters;
    vector<size_t> currentCluster = { idx[0] };
    for (size_t i = 1; i < idx.size(); ++i) {
        float gap = projs[idx[i]] - projs[idx[i-1]];
        if (gap < gapThresh) {
            currentCluster.push_back(idx[i]);
        } else {
            if (currentCluster.size() >= 3) clusters.push_back(currentCluster);
            currentCluster = { idx[i] };
        }
    }
    if (currentCluster.size() >= 3) clusters.push_back(currentCluster);

    // 对每个聚类，判断是否属于长线性干扰
    vector<bool> toRemove(rects.size(), false);
    float imgDiag = sqrt(imgSize.width*imgSize.width + imgSize.height*imgSize.height);
    const float lengthRatio = 10.0f;   // 长度阈值：大于真目标长度10倍
    const float trueTargetLen = 0.05f; // 真目标实际长度5cm，但这里是像素，需换算？
    // 由于我们不知道实际距离，改用图像像素阈值：例如长于图像对角线的1/3或固定200像素
    const float minLinearLength = max(imgDiag * 0.33f, 200.0f);
    const float maxAvgArea = 800.0f;   // 平均面积小于此值认为是小点

    for (const auto& cluster : clusters) {
        if (cluster.size() < 3) continue;
        // 计算该组的总长度（投影极差）
        float minProj = projs[cluster[0]];
        float maxProj = projs[cluster.back()];
        float length = maxProj - minProj;
        if (length < minLinearLength) continue;

        // 计算组内矩形平均面积
        float totalArea = 0;
        for (size_t i : cluster) totalArea += rects[i].area;
        float avgArea = totalArea / cluster.size();
        if (avgArea > maxAvgArea) continue;  // 不是小点，可能是真实目标

        // 满足条件：标记这些矩形为干扰
        for (size_t i : cluster) toRemove[i] = true;
        cout << "过滤线性干扰：点数=" << cluster.size() << "，长度=" << length
             << "px，平均面积=" << avgArea << endl;
    }

    // 构建过滤后的矩形列表
    vector<RectInfo> filtered;
    for (size_t i = 0; i < rects.size(); ++i) {
        if (!toRemove[i]) filtered.push_back(rects[i]);
    }
    return filtered;
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
Mat getBlueMask(const Mat& hsv) {
    Mat mask;
    // 蓝色范围 H: 100~130，S: 100~255，V: 100~255（可根据实际情况调整）
    inRange(hsv, Scalar(100, 100, 100), Scalar(130, 255, 255), mask);
    return mask;
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
    rects = filterLinearRedPoints(rects, image.size());
    pair<int, int> matchingPair;
    if (findMatchingRectPair(rects, matchingPair)) {
        const RectInfo& r1 = rects[matchingPair.first];
        const RectInfo& r2 = rects[matchingPair.second];

        // 获取两个矩形的四个角点（顺序：顺时针或逆时针）
        Point2f corners1[4], corners2[4];
        r1.rect.points(corners1);
        r2.rect.points(corners2);

        // 确定上下矩形：根据中心点 y 坐标，较小的在上方
        bool r1_is_upper = (r1.rect.center.y < r2.rect.center.y);
        const RectInfo& upperRect = r1_is_upper ? r1 : r2;
        const RectInfo& lowerRect = r1_is_upper ? r2 : r1;
        const Point2f* upperCorners = r1_is_upper ? corners1 : corners2;
        const Point2f* lowerCorners = r1_is_upper ? corners2 : corners1;

        // 根据矩形的长边方向（orientation），决定选取哪个边的两个顶点
        // 长边方向接近竖直（例如 70°~110°）时，上下矩形分别取上边和下边的两个点
        // 长边方向接近水平（例如 -20°~20° 或 160°~180°）时，左右矩形分别取左边和右边的两个点（此处暂不实现）
        // 我们假设目标为竖直灯条，取上矩形的上边两点、下矩形的下边两点
        vector<Point2f> outerPoints(4);

        // 对于上矩形：选择 y 坐标最小的两个点（上边）
        vector<Point2f> upperPoints(upperCorners, upperCorners+4);
        sort(upperPoints.begin(), upperPoints.end(), [](const Point2f& a, const Point2f& b) {
            return a.y < b.y;
        });
        outerPoints[0] = upperPoints[0]; // 左上
        outerPoints[1] = upperPoints[1]; // 右上（实际这两个点可能是左右顺序，但之后会按多边形顺序调整）

        // 对于下矩形：选择 y 坐标最大的两个点（下边）
        vector<Point2f> lowerPoints(lowerCorners, lowerCorners+4);
        sort(lowerPoints.begin(), lowerPoints.end(), [](const Point2f& a, const Point2f& b) {
            return a.y > b.y;
        });
        outerPoints[2] = lowerPoints[0]; // 右下
        outerPoints[3] = lowerPoints[1]; // 左下

        // 调整四点顺序为顺时针或逆时针（确保四边形不自交）
        // 简单方法：按角度排序
        Point2f center = (outerPoints[0] + outerPoints[1] + outerPoints[2] + outerPoints[3]) / 4.0f;
        sort(outerPoints.begin(), outerPoints.end(), [&center](const Point2f& a, const Point2f& b) {
            return atan2(a.y - center.y, a.x - center.x) < atan2(b.y - center.y, b.x - center.x);
        });

        // 绘制绿色四边形
        for (int i = 0; i < 4; ++i) {
            line(image, outerPoints[i], outerPoints[(i+1)%4], Scalar(0, 255, 0), 2);
          
        }



            // 使用这四个角点作为 PnP 输入
            vector<Point2f> boxCorners = outerPoints;
            cout << "\n目标最外侧四个角点（像素坐标）：" << endl;
            for (int i = 0; i < 4; ++i) {
                cout << "点" << i+1 << ": " << boxCorners[i] << endl;
            }

        // PnP解算，目标尺寸（单位：米）请根据实际修改
        float targetWidth = 0.05f;   // 宽度5cm
        float targetHeight = 0.05f;  // 高度5cm
        if (solvePnPForRect(boxCorners, targetWidth, targetHeight, rvec, tvec)) {
            distance = norm(tvec);
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
// 主函数：读取摄像头，检测，逆运动学求解，串口发送
int main(int argc, char** argv) {
    // 1. 初始化串口
    SerialPort serial("/dev/ttyUSB0", 100);
    if (!serial.open_port()) {
        std::cerr << "Failed to init serial port!" << std::endl;
        return -1;
    }

    // 2. 初始化运动学
    params my_params;
    engineer_kinematics kin(my_params);
    // 相机到基座的变换（实际应标定）
    Eigen::Isometry3d T_cam2base = Eigen::Isometry3d::Identity();

    // 3. 初始化相机（使用 io::Camera）
    std::string camera_config_path = "/home/thatbbbbbb/projects/aruco_gimbal_project/config/camera.yaml";
    io::Camera camera(camera_config_path);

    // 帧率控制
    int delay_ms = 33;  // 30 fps
    std::cout << "目标帧率: 30 fps, 延时: " << delay_ms << " ms" << std::endl;

    int frame_count = 0;
    int total_frames = 0;
    std::array<float, 3> current_joints = {0, 0, 0};

    while (true) {
        auto start_time = std::chrono::steady_clock::now();

        cv::Mat frame;
        auto timestamp = std::chrono::steady_clock::now();
        camera.read(frame, timestamp);
        if (frame.empty()) {
            std::cerr << "Failed to get frame from camera" << std::endl;
            continue;
        }

        frame_count++;
        total_frames++;

        // 处理图像，获取旋转向量、平移向量及欧拉角
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

            // 添加激光相对于相机的偏移
            t_eigen.y() -= 0.0389706599;   // Y方向偏移（米）
            t_eigen.x() += 0.01288587;     // X方向偏移（米）

            Eigen::Isometry3d T_cam_target = Eigen::Isometry3d::Identity();
            T_cam_target.rotate(R_eigen);
            T_cam_target.translation() = t_eigen;

            // 变换到基座坐标系
            Eigen::Isometry3d T_base_target = T_cam2base * T_cam_target;

            // 逆运动学求解
            std::vector<std::array<float, 3>> solutions;
            // 打印相机坐标系下的目标位置
            cout << "Target in camera frame: (" << t_eigen.x() << ", " << t_eigen.y() << ", " << t_eigen.z() << ") m" << endl;
            // 打印基座坐标系下的目标位置
            cout << "Target in base frame: (" << T_base_target.translation().x() << ", "
                << T_base_target.translation().y() << ", "
                << T_base_target.translation().z() << ") m" << endl;
            if (kin.inverse_kinematics(T_base_target, solutions)) {
                // 选择与当前关节角最接近的解
                int best_idx = 0;
                float min_diff = std::numeric_limits<float>::max();
                for (size_t i = 0; i < solutions.size(); ++i) {
                    float diff = std::abs(solutions[i][0] - current_joints[0]) +
                                 std::abs(solutions[i][1] - current_joints[1]) +
                                 std::abs(solutions[i][2] - current_joints[2]);
                    if (diff < min_diff) {
                        min_diff = diff;
                        best_idx = i;
                    }
                }
                auto joints = solutions[best_idx];
                current_joints = joints;
                // 弧度转角度（注意：M_PIf 可能需要定义，或直接用 M_PI）
                std::vector<float> angles_deg = {
                    static_cast<float>(joints[0] * 180.0 / M_PI),
                    static_cast<float>(joints[1] * 180.0 / M_PI),
                    static_cast<float>(joints[2] * 180.0 / M_PI)
                };
                // 通过串口发送关节角度
                serial.send_motor_angles(angles_deg);
                std::cout << std::fixed << std::setprecision(2)
                          << "Frame " << frame_count << " | Distance=" << distance
                          << "m | Pitch=" << pitch_deg << "° | Yaw=" << yaw_deg << "°"
                          << " | Joints(deg): " << angles_deg[0] << ", " << angles_deg[1] << ", " << angles_deg[2] << std::endl;
            } else {
                std::cout << "Frame " << frame_count << " | Inverse kinematics failed" << std::endl;
            }
        }  // 结束 if (distance > 0)

        // 显示图像（无论是否检测到目标都显示）
        cv::imshow("Red Light Bar Detection", processed);

        // 帧率控制
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
    cv::destroyAllWindows();
    return 0;
}