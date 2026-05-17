#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <cmath>
#include "camera.hpp"
using namespace cv;
using namespace std;

// 矩形信息结构体，存储最小外接矩形及其长边方向角度（0~180°）
struct RectInfo {
    RotatedRect rect;
    float longSideLen;   // 长边长度
    float shortSideLen;  // 短边长度
    float orientation;   // 长边方向角度（度，范围[0,180)）
    float area;
};

/**
 * 获取红色的HSV掩码（处理红色在色环两端的特性）
 * @param hsv HSV图像
 * @return 红色区域的二值掩码
 */
Mat getRedMask(const Mat& hsv) {
    Mat mask1, mask2;
    // 红色范围1: 0~10
    inRange(hsv, Scalar(0, 100, 100), Scalar(10, 255, 255), mask1);
    // 红色范围2: 160~180
    inRange(hsv, Scalar(160, 100, 100), Scalar(180, 255, 255), mask2);
    return mask1 | mask2;
}

/**
 * 计算旋转矩形的最长边方向角度（0~180°）
 * @param rect 旋转矩形
 * @param longSideLen 输出长边长度
 * @param shortSideLen 输出短边长度
 * @return 长边方向角度（度）
 */
float computeLongSideOrientation(const RotatedRect& rect, float& longSideLen, float& shortSideLen) {
    float width = rect.size.width;
    float height = rect.size.height;
    if (width >= height) {
        longSideLen = width;
        shortSideLen = height;
        // 角度范围[-90,0)，转换为[0,180)
        float angle = rect.angle;
        if (angle < 0) angle += 180.0f;
        return angle;   // 长边与x轴夹角
    } else {
        longSideLen = height;
        shortSideLen = width;
        // 短边方向角度+90°即为长边方向
        float angle = rect.angle + 90.0f;
        if (angle >= 180.0f) angle -= 180.0f;
        if (angle < 0) angle += 180.0f;
        return angle;
    }
}

/**
 * 从轮廓中提取有效的矩形信息（面积足够大且宽高比合理）
 * @param contours 轮廓列表
 * @param minArea 最小面积阈值
 * @param aspectRatioMin 最小宽高比
 * @param aspectRatioMax 最大宽高比
 * @return 矩形信息列表
 */
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
        if (ratio < aspectRatioMin || ratio > aspectRatioMax) continue; // 不是合理的矩形

        rects.push_back({rotRect, longLen, shortLen, orient, area});
    }
    return rects;
}

/**
 * 判断两个矩形是否满足条件：
 * 1. 长边相互平行（角度差小于阈值）
 * 2. 面积近似相等（相对误差小于阈值）
 * 3. 中心距离不超过较大矩形长边的1.5倍
 */
bool areMatchingRects(const RectInfo& r1, const RectInfo& r2,
                      float angleTolerance = 10.0f,
                      float areaTolerance = 0.1f,   // 相对面积误差10%
                      float distanceFactor = 1.5f) {
    // 平行性检查
    float angleDiff = std::abs(r1.orientation - r2.orientation);
    angleDiff = std::min(angleDiff, 180.0f - angleDiff);
    if (angleDiff > angleTolerance) return false;

    // 面积相等检查
    float areaDiff = std::abs(r1.area - r2.area);
    float maxArea = std::max(r1.area, r2.area);
    if (areaDiff > maxArea * areaTolerance) return false;

    // 距离检查
    Point2f center1 = r1.rect.center;
    Point2f center2 = r2.rect.center;
    float distance = norm(center1 - center2);
    float maxLongLen = std::max(r1.longSideLen, r2.longSideLen);
    if (distance > maxLongLen * distanceFactor) return false;

    return true;
}

/**
 * 查找满足条件的一对矩形
 * @param rects 所有矩形列表
 * @param pairIdx 输出一对索引，若找到则返回true
 */
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

/**
 * 绘制包含两个矩形的最小绿色边界框（轴对齐）
 */
void drawGreenBoundingBox(Mat& image, const RectInfo& r1, const RectInfo& r2) {
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
}

/**
 * 主处理函数
 * @param image 输入彩色图像（BGR格式）
 * @return 处理后的图像（在原图上绘制结果）
 */
Mat processImage(Mat image) {
    if (image.empty()) {
        cerr << "输入图像为空" << endl;
        return image;
    }

    // 1. HSV转换并生成红色掩码
    Mat hsv;
    cvtColor(image, hsv, COLOR_BGR2HSV);
    Mat mask = getRedMask(hsv);

    // 2. 膨胀处理连接邻近光点
    Mat kernel = getStructuringElement(MORPH_RECT, Size(5, 5));
    Mat dilated;
    dilate(mask, dilated, kernel);

    // 3. 查找轮廓
    vector<vector<Point>> contours;
    findContours(dilated, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

    // 4. 提取矩形并筛选
    vector<RectInfo> rects = extractRectangles(contours, 500.0, 0.3f, 3.0f);

    // 5. 查找符合条件的一对矩形
    pair<int, int> matchingPair;
    if (findMatchingRectPair(rects, matchingPair)) {
        const RectInfo& r1 = rects[matchingPair.first];
        const RectInfo& r2 = rects[matchingPair.second];
        drawGreenBoundingBox(image, r1, r2);
        cout << "找到匹配矩形对，已绘制绿色边框" << endl;
    } else {
        cout << "未找到匹配的矩形对" << endl;
    }

    return image;
}
double getTimeSeconds() {
    auto now = std::chrono::steady_clock::now();
    auto duration = now.time_since_epoch();
    return std::chrono::duration<double>(duration).count();
}
// // 示例使用
// int main(int argc, char** argv) {
//     int frame_count = 0;
//     int total_frames = 0;
//     io::Camera camera("/home/thatbbbbbb/projects/aruco_gimbal_project/config/camera.yaml");
//     while (true) {
//         double current_time = getTimeSeconds();

//         cv::Mat frame;
//         auto timestamp = std::chrono::steady_clock::now();
//         camera.read(frame, timestamp);

//         if (!frame.empty()) {
//             frame_count++;
//             total_frames++;

//             Mat processed = processImage(frame);
//             imshow("Red Light Bar Detection", processed);

//             if (waitKey(1) == 27) break; // ESC 退出
//         } else {
//             cerr << "读取帧失败" << endl;
//             break;
//         }

//         // 帧率计算（每30帧输出一次）
//         if (frame_count % 30 == 0) {
//             double elapsed = getTimeSeconds() - current_time;
//             double fps = 1.0 / elapsed;
//             cout << "处理帧数: " << frame_count << ", FPS: " << fps << endl;
//         }
//     }

//     cout << "总共处理帧数: " << total_frames << endl;
//     return 0;
// }
int main(int argc, char** argv) {
    int frame_count = 0;
    int total_frames = 0;

    // 直接读取你的测试视频文件
    std::string videoPath = "/opt/MVS/bin/Temp/Data/MV-CS016-10UC+DA4886227/Video_20260510164431821.avi";
    cv::VideoCapture cap(videoPath);

    if (!cap.isOpened()) {
        std::cerr << "视频打开失败！路径：" << videoPath << std::endl;
        return -1;
    }

    while (true) {
        double current_time = (double)cv::getTickCount() / cv::getTickFrequency();

        cv::Mat frame;
        cap >> frame;  // 从视频读帧

        if (!frame.empty()) {
            frame_count++;
            total_frames++;

            cv::Mat processed = processImage(frame);
            cv::imshow("Red Light Bar Detection", processed);

            if (cv::waitKey(1) == 27) break; // ESC 退出
        } else {
            std::cerr << "视频读取完毕或失败" << std::endl;
            break;
        }

        // 帧率计算（每30帧输出一次）
        if (frame_count % 30 == 0) {
            double elapsed = (double)cv::getTickCount() / cv::getTickFrequency() - current_time;
            double fps = 30.0 / elapsed;
            std::cout << "处理帧数: " << frame_count << ", FPS: " << fps << std::endl;
        }
    }

    std::cout << "总共处理帧数: " << total_frames << std::endl;

    cap.release();
    cv::destroyAllWindows();
    return 0;
}