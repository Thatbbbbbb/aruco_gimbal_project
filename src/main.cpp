#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <cmath>
#include <iomanip>
#include <chrono>
#include <filesystem>
#include "camera.hpp"
#include "serial_port.h"
#include "engineer_kinematics.hpp"
#include "drone.hpp"                     // 假设定义了 Drone 结构体
#include "yolos/hrt_yolo.hpp"            // 无人机检测器头文件
// 注意：确保上面两个头文件路径正确，或根据实际项目调整

using namespace cv;
using namespace std;
using namespace hitcrt::serial;
using namespace hitcrt::kinematics;

// ====================== 全局相机内参与畸变（根据实际标定） ======================
static Mat cameraMatrix = (Mat_<double>(3,3) <<
    9991.10399893341, 0, 718.418901980155,
    0, 9991.82243368060, 558.088945484812,
    0, 0, 1);
static Mat distCoeffs = (Mat_<double>(5,1) <<
    0.209283116250229,
    0.676385453898682,
    0, 0, 0);

// ====================== 矩形信息结构体（灯条用） ======================
struct RectInfo {
    RotatedRect rect;
    float longSideLen;
    float shortSideLen;
    float orientation;
    float area;
};

// ---------------------- 灯条检测相关函数声明 ----------------------
Mat getRedMask(const Mat& hsv);
float computeLongSideOrientation(const RotatedRect& rect, float& longSideLen, float& shortSideLen);
vector<RectInfo> extractRectangles(const vector<vector<Point>>& contours, double minArea, float aspectMin, float aspectMax);
bool areMatchingRects(const RectInfo& r1, const RectInfo& r2, float angleTol, float areaTol, float distFactor);
bool findMatchingRectPair(const vector<RectInfo>& rects, pair<int,int>& pairIdx);
Rect drawGreenBoundingBox(Mat& img, const RectInfo& r1, const RectInfo& r2);
Vec3d rotationMatrixToEulerAngles(const Mat& R);
bool solvePnPForRect(const vector<Point2f>& imgPoints, float objW, float objH, Mat& rvec, Mat& tvec);
bool detectLightbarsInROI(const Mat& roi, const Point2f& roi_offset,
                          vector<Point2f>& boxCorners, Mat& rvec, Mat& tvec,
                          double& pitch, double& yaw, double& distance);

// ---------------------- 灯条检测函数实现 ----------------------
Mat getRedMask(const Mat& hsv) {
    Mat mask1, mask2;
    inRange(hsv, Scalar(0, 100, 100), Scalar(10, 255, 255), mask1);
    inRange(hsv, Scalar(160, 100, 100), Scalar(180, 255, 255), mask2);
    return mask1 | mask2;
}

float computeLongSideOrientation(const RotatedRect& rect, float& longLen, float& shortLen) {
    float w = rect.size.width, h = rect.size.height;
    if (w >= h) {
        longLen = w; shortLen = h;
        float ang = rect.angle;
        if (ang < 0) ang += 180.0f;
        return ang;
    } else {
        longLen = h; shortLen = w;
        float ang = rect.angle + 90.0f;
        if (ang >= 180.0f) ang -= 180.0f;
        if (ang < 0) ang += 180.0f;
        return ang;
    }
}

vector<RectInfo> extractRectangles(const vector<vector<Point>>& contours,
                                   double minArea = 500.0,
                                   float aspectMin = 0.3f,
                                   float aspectMax = 3.0f) {
    vector<RectInfo> rects;
    for (const auto& c : contours) {
        RotatedRect rr = minAreaRect(c);
        float area = rr.size.area();
        if (area < minArea) continue;
        float longLen, shortLen;
        float orient = computeLongSideOrientation(rr, longLen, shortLen);
        float ratio = longLen / shortLen;
        if (ratio < aspectMin || ratio > aspectMax) continue;
        rects.push_back({rr, longLen, shortLen, orient, area});
    }
    return rects;
}

bool areMatchingRects(const RectInfo& r1, const RectInfo& r2,
                      float angleTol = 10.0f, float areaTol = 0.1f, float distFactor = 1.5f) {
    float diff = abs(r1.orientation - r2.orientation);
    diff = min(diff, 180.0f - diff);
    if (diff > angleTol) return false;
    float areaDiff = abs(r1.area - r2.area);
    float maxArea = max(r1.area, r2.area);
    if (areaDiff > maxArea * areaTol) return false;
    float dist = norm(r1.rect.center - r2.rect.center);
    float maxLen = max(r1.longSideLen, r2.longSideLen);
    if (dist > maxLen * distFactor) return false;
    return true;
}

bool findMatchingRectPair(const vector<RectInfo>& rects, pair<int,int>& pairIdx) {
    for (size_t i = 0; i < rects.size(); ++i)
        for (size_t j = i+1; j < rects.size(); ++j)
            if (areMatchingRects(rects[i], rects[j])) {
                pairIdx = {i,j};
                return true;
            }
    return false;
}

Rect drawGreenBoundingBox(Mat& img, const RectInfo& r1, const RectInfo& r2) {
    vector<Point2f> corners(8);
    Point2f pts1[4], pts2[4];
    r1.rect.points(pts1);
    r2.rect.points(pts2);
    for (int i = 0; i < 4; ++i) {
        corners[i] = pts1[i];
        corners[i+4] = pts2[i];
    }
    Rect box = boundingRect(corners);
    rectangle(img, box, Scalar(0,255,0), 2);
    return box;
}

Vec3d rotationMatrixToEulerAngles(const Mat& R) {
    double sy = sqrt(R.at<double>(0,0)*R.at<double>(0,0) + R.at<double>(1,0)*R.at<double>(1,0));
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

bool solvePnPForRect(const vector<Point2f>& imgPoints, float objW, float objH, Mat& rvec, Mat& tvec) {
    if (imgPoints.size() != 4) return false;
    vector<Point3f> objPts = {
        Point3f(-objW/2,  objH/2, 0),
        Point3f( objW/2,  objH/2, 0),
        Point3f( objW/2, -objH/2, 0),
        Point3f(-objW/2, -objH/2, 0)
    };
    return solvePnP(objPts, imgPoints, cameraMatrix, distCoeffs, rvec, tvec, false, SOLVEPNP_ITERATIVE);
}

bool detectLightbarsInROI(const Mat& roi, const Point2f& roi_offset,
                          vector<Point2f>& boxCorners, Mat& rvec, Mat& tvec,
                          double& pitch, double& yaw, double& distance) {
    if (roi.empty()) return false;
    Mat hsv; cvtColor(roi, hsv, COLOR_BGR2HSV);
    Mat mask = getRedMask(hsv);
    Mat kernel = getStructuringElement(MORPH_RECT, Size(5,5));
    Mat dilated;
    dilate(mask, dilated, kernel);
    vector<vector<Point>> contours;
    findContours(dilated, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
    vector<RectInfo> rects = extractRectangles(contours, 500.0, 0.3f, 3.0f);
    pair<int,int> pairIdx;
    if (!findMatchingRectPair(rects, pairIdx)) return false;

    const RectInfo& r1 = rects[pairIdx.first];
    const RectInfo& r2 = rects[pairIdx.second];

    // 将角点转到全图
    Point2f pts1[4], pts2[4];
    r1.rect.points(pts1);
    r2.rect.points(pts2);
    vector<Point2f> allCorners;
    for (int i=0; i<4; ++i) allCorners.push_back(pts1[i] + roi_offset);
    for (int i=0; i<4; ++i) allCorners.push_back(pts2[i] + roi_offset);
    Rect greenBox = boundingRect(allCorners);
    boxCorners.clear();
    boxCorners.push_back(Point2f(greenBox.x, greenBox.y));
    boxCorners.push_back(Point2f(greenBox.x+greenBox.width, greenBox.y));
    boxCorners.push_back(Point2f(greenBox.x+greenBox.width, greenBox.y+greenBox.height));
    boxCorners.push_back(Point2f(greenBox.x, greenBox.y+greenBox.height));

    const float targetW = 0.05f, targetH = 0.05f;   // 目标实际尺寸（米），请根据实物修改
    if (!solvePnPForRect(boxCorners, targetW, targetH, rvec, tvec)) return false;

    distance = norm(tvec);
    Mat R;
    Rodrigues(rvec, R);
    Vec3d euler = rotationMatrixToEulerAngles(R);
    pitch = euler[1] * 180.0 / CV_PI;
    yaw   = euler[2] * 180.0 / CV_PI;
    return true;
}

// ---------------------- 主函数 ----------------------
int main(int argc, char** argv) {
    // 1. 初始化串口
    SerialPort serial("/dev/ttyUSB0", 100);
    if (!serial.open_port()) {
        cerr << "Failed to open serial port!" << endl;
        return -1;
    }

    // 2. 初始化运动学（参数从配置文件加载，这里用默认构造，实际应加载 yaml）
    params kin_params;
    engineer_kinematics kin(kin_params);
    Eigen::Isometry3d T_cam2base = Eigen::Isometry3d::Identity();  // 相机到基座变换，实际应标定

    // 3. 初始化无人机检测器
    string drone_config = "/home/thatbbbbbb/projects/aruco_gimbal_project/config/drone.yaml";
    drone_detection::DroneDetector drone_detector(drone_config, false);  // 假设类已实现

    // 4. 初始化相机
    io::Camera camera("/home/thatbbbbbb/projects/aruco_gimbal_project/config/camera.yaml");

    // 5. 主循环变量
    int frame_count = 0, total_frames = 0, lightbar_detected_frames = 0;
    array<float, 3> current_joints = {0,0,0};

    cout << "===== Drone + Lightbar Detection Started =====" << endl;

    while (true) {
        cv::Mat frame;
        auto timestamp = chrono::steady_clock::now();
        camera.read(frame, timestamp);
        if (frame.empty()) break;

        frame_count++;
        total_frames++;
        cv::Mat display = frame.clone();

        // ----- 第一步：检测无人机 -----
        int target_w = 640, target_h = 640;
        float scale; Point2f offset;
        cv::Mat processed_frame = drone_detection::resizeAndPad(frame, target_w, target_h, scale, offset);
        auto drones = drone_detector.detect(processed_frame, frame_count);

        bool drone_found = !drones.empty();
        if (drone_found) {
            // 选择置信度最高的无人机
            auto best = max_element(drones.begin(), drones.end(),
                [](const auto& a, const auto& b) { return a.confidence < b.confidence; });
            cv::Rect drone_box = best->box;
            // 放大ROI为原来的3倍（面积×3，即边长×√3 ≈ 1.732）
            const float roi_scale = 1.732f;
            cv::Point2f center(drone_box.x + drone_box.width*0.5f, drone_box.y + drone_box.height*0.5f);
            int new_w = max(1, (int)(drone_box.width * roi_scale));
            int new_h = max(1, (int)(drone_box.height * roi_scale));
            cv::Rect roi(cvRound(center.x - new_w/2), cvRound(center.y - new_h/2), new_w, new_h);
            roi &= cv::Rect(0,0,frame.cols,frame.rows);
            if (!roi.empty()) {
                // 绘制无人机ROI（绿色框）
                rectangle(display, roi, Scalar(0,255,0), 2);
                putText(display, "Drone ROI", Point(roi.x, roi.y-5), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0,255,0), 1);

                // ----- 第二步：在ROI内检测灯条 -----
                cv::Mat roi_img = frame(roi);
                vector<Point2f> boxCorners;
                Mat rvec, tvec;
                double pitch_deg, yaw_deg, distance;
                if (detectLightbarsInROI(roi_img, Point2f(roi.x, roi.y), boxCorners, rvec, tvec, pitch_deg, yaw_deg, distance)) {
                    lightbar_detected_frames++;
                    // 在图像上绘制绿色外框和矩形角点（黄色）
                    for (int i=0; i<4; ++i) {
                        line(display, boxCorners[i], boxCorners[(i+1)%4], Scalar(0,255,255), 2);
                    }
                    circle(display, (boxCorners[0]+boxCorners[2])*0.5f, 5, Scalar(0,255,255), -1);
                    putText(display, "Lightbar Set", Point(roi.x, roi.y+roi.height+15), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0,255,255), 1);

                    // 添加Y、Z方向偏移（相机坐标系）
                    Eigen::Vector3d t_eigen(tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2));
                    t_eigen.y() -= 0.0389706599;   // Y偏移
                    t_eigen.x() += 0.01288587;    // X偏移

                    // 构造位姿（相机坐标系）
                    Mat R_cv; Rodrigues(rvec, R_cv);
                    Eigen::Matrix3d R_eigen;
                    for (int i=0; i<3; ++i)
                        for (int j=0; j<3; ++j)
                            R_eigen(i,j) = R_cv.at<double>(i,j);
                    Eigen::Isometry3d T_cam_target = Eigen::Isometry3d::Identity();
                    T_cam_target.rotate(R_eigen);
                    T_cam_target.translation() = t_eigen;

                    // 转到基座坐标系
                    Eigen::Isometry3d T_base_target = T_cam2base * T_cam_target;

                    // 逆运动学求解
                    vector<array<float,3>> solutions;
                    if (kin.inverse_kinematics(T_base_target, solutions)) {
                        // 选择与当前关节角最接近的解
                        int best_idx = 0;
                        float min_diff = numeric_limits<float>::max();
                        for (size_t i=0; i<solutions.size(); ++i) {
                            float diff = abs(solutions[i][0]-current_joints[0]) +
                                         abs(solutions[i][1]-current_joints[1]) +
                                         abs(solutions[i][2]-current_joints[2]);
                            if (diff < min_diff) { min_diff = diff; best_idx = i; }
                        }
                        auto joints = solutions[best_idx];
                        current_joints = joints;
                        vector<float> angles_deg = {
                            joints[0] * 180.0f / M_PIf,
                            joints[1] * 180.0f / M_PIf,
                            joints[2] * 180.0f / M_PIf
                        };
                        serial.send_motor_angles(angles_deg);
                        cout << fixed << setprecision(2)
                             << "Frame " << frame_count << " | Distance=" << distance
                             << "m | Pitch=" << pitch_deg << "° | Yaw=" << yaw_deg << "°"
                             << " | Joints(deg): " << angles_deg[0] << ", " << angles_deg[1] << ", " << angles_deg[2] << endl;
                    } else {
                        cout << "Frame " << frame_count << " | Inverse kinematics failed" << endl;
                    }
                } else {
                    cout << "Frame " << frame_count << " | No lightbar pair in ROI" << endl;
                }
            }
        } else {
            cout << "Frame " << frame_count << " | No drone detected" << endl;
        }

        // 显示辅助信息
        putText(display, cv::format("Frame: %d", frame_count), Point(10,30), FONT_HERSHEY_SIMPLEX, 0.6, Scalar(255,255,255), 2);
        putText(display, cv::format("Drone: %s", drone_found ? "YES" : "NO"), Point(10,60), FONT_HERSHEY_SIMPLEX, 0.5,
                drone_found ? Scalar(0,255,0) : Scalar(0,0,255), 1);
        putText(display, cv::format("Lightbar: %s", (lightbar_detected_frames > 0 && drone_found) ? "YES" : "NO"),
                Point(10,85), FONT_HERSHEY_SIMPLEX, 0.5,
                (lightbar_detected_frames > 0 && drone_found) ? Scalar(0,255,255) : Scalar(0,0,255), 1);

        cv::imshow("Drone + Lightbar Detection", display);
        char key = cv::waitKey(1);
        if (key == 'q' || key == 'Q') break;
    }

    // 统计
    cout << "========================================" << endl;
    cout << "Total frames: " << total_frames << endl;
    cout << "Lightbar detected frames: " << lightbar_detected_frames << endl;
    if (total_frames>0)
        cout << "Detection rate: " << (float)lightbar_detected_frames/total_frames*100 << "%" << endl;
    cout << "========================================" << endl;

    cv::destroyAllWindows();
    return 0;
}