#include <opencv2/opencv.hpp>
#include <vector>
#include <algorithm>
#include <cmath>
#include <iostream>

// 灯条结构体
struct LightBar {
    cv::RotatedRect rect;       // 旋转矩形
    cv::Point2f center;         // 中心点
    float length;               // 长边长度
    float width;                // 短边长度
    float angle;                // 角度（度）
    float aspectRatio;          // 长宽比
    float area;                 // 矩形面积

    LightBar(const std::vector<cv::Point>& contour) {
        rect = cv::minAreaRect(contour);
        center = rect.center;
        float w = rect.size.width;
        float h = rect.size.height;
        if (w > h) {
            length = w;
            width = h;
            angle = rect.angle;
        } else {
            length = h;
            width = w;
            angle = rect.angle + 90.0f;
        }
        aspectRatio = length / width;
        area = length * width;
    }
};

// 全局参数（给滑动条用）
int g_brightnessThresh = 50;
int g_minLightBarArea = 20;
int g_maxLightBarArea = 100;
int g_minAspectRatio = 20;  // 扩大10倍存储
int g_maxAspectRatio = 30;
int g_maxAngleDiff = 20;
int g_maxYDistRatio = 30;
int g_maxXOffsetRatio = 30;

// 几何辅助函数
double pointDistance(const cv::Point2f& a, const cv::Point2f& b) {
    return std::hypot(a.x - b.x, a.y - b.y);
}

// 检测主函数
bool detectDoubleRowTarget(cv::Mat& image, cv::Point2f& targetCenter, cv::Rect& targetRect, cv::Mat& binaryOut) {
    // 1. 从全局滑动条读取参数
    const std::string colorMode = "red";     // "red" 或 "blue"
    const int brightnessThresh = g_brightnessThresh;
    const float minLightBarArea = (float)g_minLightBarArea;
    const float maxLightBarArea = (float)g_maxLightBarArea;
    const float minAspectRatio = (float)g_minAspectRatio / 10.0f;
    const float maxAspectRatio = (float)g_maxAspectRatio / 10.0f;
    const float maxAngleDiff = (float)g_maxAngleDiff;
    const int expectedCountPerRow = 4;
    const float maxYDistRatio = (float)g_maxYDistRatio / 100.0f;
    const float maxXOffsetRatio = (float)g_maxXOffsetRatio / 100.0f;

    // 2. 图像预处理：根据颜色提取通道并二值化
    cv::Mat gray, binary;
    if (colorMode == "red") {
        std::vector<cv::Mat> channels;
        cv::split(image, channels);
        cv::Mat red = channels[2];
        cv::Mat redMinusGreen, redMinusBlue;
        cv::subtract(red, channels[1], redMinusGreen);
        cv::subtract(red, channels[0], redMinusBlue);
        cv::Mat redDiff = cv::max(redMinusGreen, redMinusBlue);
        cv::threshold(redDiff, binary, brightnessThresh, 255, cv::THRESH_BINARY);
    } else if (colorMode == "blue") {
        std::vector<cv::Mat> channels;
        cv::split(image, channels);
        cv::Mat blue = channels[0];
        cv::Mat blueMinusRed, blueMinusGreen;
        cv::subtract(blue, channels[2], blueMinusRed);
        cv::subtract(blue, channels[1], blueMinusGreen);
        cv::Mat blueDiff = cv::max(blueMinusRed, blueMinusGreen);
        cv::threshold(blueDiff, binary, brightnessThresh, 255, cv::THRESH_BINARY);
    } else {
        std::cerr << "Unsupported color mode" << std::endl;
        return false;
    }

    // 形态学
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    cv::morphologyEx(binary, binary, cv::MORPH_OPEN, kernel);
    cv::morphologyEx(binary, binary, cv::MORPH_CLOSE, kernel);

    // 输出二值图给外部显示
    binaryOut = binary.clone();

    // 3. 提取轮廓
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(binary, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    std::vector<LightBar> lightBars;
    for (const auto& contour : contours) {
        if (contour.size() < 5) continue;
        LightBar lb(contour);
        if (lb.area < minLightBarArea || lb.area > maxLightBarArea) continue;
        if (lb.aspectRatio < minAspectRatio || lb.aspectRatio > maxAspectRatio) continue;
        lightBars.push_back(lb);
    }

    if (lightBars.size() < expectedCountPerRow * 2) {
        std::cout << "Not enough light bars: " << lightBars.size() << std::endl;
        return false;
    }

    // 4. 角度聚类
    std::sort(lightBars.begin(), lightBars.end(),
              [](const LightBar& a, const LightBar& b) { return a.angle < b.angle; });
    float bestAngle = lightBars[0].angle;
    int bestCount = 0;
    int count = 1;
    for (size_t i = 1; i < lightBars.size(); ++i) {
        if (std::abs(lightBars[i].angle - lightBars[i-1].angle) < maxAngleDiff) {
            count++;
        } else {
            if (count > bestCount) {
                bestCount = count;
                bestAngle = lightBars[i-1].angle;
            }
            count = 1;
        }
    }
    if (count > bestCount) bestAngle = lightBars.back().angle;

    std::vector<LightBar> filtered;
    for (const auto& lb : lightBars) {
        float diff = std::abs(lb.angle - bestAngle);
        if (diff > maxAngleDiff && std::abs(diff - 180) > maxAngleDiff) continue;
        filtered.push_back(lb);
    }

    if (filtered.size() < expectedCountPerRow * 2) {
        std::cout << "After angle filter: " << filtered.size() << std::endl;
        return false;
    }

    // 5. 分上下排
    std::sort(filtered.begin(), filtered.end(),
              [](const LightBar& a, const LightBar& b) { return a.center.y < b.center.y; });
    size_t splitIdx = filtered.size() / 2;
    std::vector<LightBar> topRow(filtered.begin(), filtered.begin() + splitIdx);
    std::vector<LightBar> bottomRow(filtered.begin() + splitIdx, filtered.end());

    if (std::abs((int)topRow.size() - (int)bottomRow.size()) > 1) {
        float medianY = filtered[filtered.size()/2].center.y;
        topRow.clear(); bottomRow.clear();
        for (const auto& lb : filtered) {
            if (lb.center.y < medianY) topRow.push_back(lb);
            else bottomRow.push_back(lb);
        }
    }

    if (topRow.size() != expectedCountPerRow || bottomRow.size() != expectedCountPerRow) {
        std::cout << "Row sizes: top=" << topRow.size() << ", bottom=" << bottomRow.size() << std::endl;
        return false;
    }

    // 6. 共线检查
    auto checkRow = [](const std::vector<LightBar>& row, float& minX, float& maxX) {
        if (row.empty()) return false;
        minX = row[0].center.x; maxX = row[0].center.x; float sumY = 0;
        for (const auto& lb : row) { minX = std::min(minX, lb.center.x); maxX = std::max(maxX, lb.center.x); sumY += lb.center.y; }
        float meanY = sumY / row.size(); float varY = 0;
        for (const auto& lb : row) varY += pow(lb.center.y - meanY, 2);
        varY /= row.size();
        float avgLen = 0; for (auto& l : row) avgLen += l.length; avgLen /= row.size();
        return varY < avgLen * 0.25f;
    };

    float tmin, tmax, bmin, bmax;
    if (!checkRow(topRow, tmin, tmax) || !checkRow(bottomRow, bmin, bmax)) {
        std::cout << "Rows not collinear" << std::endl;
        return false;
    }

    // 7. X排序
    std::sort(topRow.begin(), topRow.end(), [](auto& a, auto& b) { return a.center.x < b.center.x; });
    std::sort(bottomRow.begin(), bottomRow.end(), [](auto& a, auto& b) { return a.center.x < b.center.x; });

    for (size_t i=0; i<expectedCountPerRow; ++i) {
        float dx = abs(topRow[i].center.x - bottomRow[i].center.x);
        float maxAllow = maxXOffsetRatio * (topRow[i].width + bottomRow[i].width)/2;
        if (dx > maxAllow) {
            std::cout << "X offset too large " << i << std::endl;
            return false;
        }
    }

    // 8. 排距离检查
    float rowDist = abs(topRow[0].center.y - bottomRow[0].center.y);
    float expectDist = (topRow[0].length + bottomRow[0].length)/2;
    if (abs(rowDist - expectDist) / expectDist > maxYDistRatio) {
        std::cout << "Row distance mismatch" << std::endl;
        return false;
    }

    // 输出目标框
    float L = std::min(tmin, bmin);
    float R = std::max(tmax, bmax);
    float T = topRow[0].center.y - topRow[0].length/2;
    float B = bottomRow[0].center.y + bottomRow[0].length/2;
    targetRect = cv::Rect(cv::Point(L, T), cv::Point(R, B));
    targetCenter = cv::Point2f((L+R)/2, (T+B)/2);

    return true;
}

int main() {
    // 打开视频
    std::string videoPath = "/opt/MVS/bin/Temp/Data/MV-CS016-10UC+DA4886227/Video_20260510164431821.avi";
    cv::VideoCapture cap(videoPath);
    if (!cap.isOpened()) {
        std::cerr << "视频打开失败！" << std::endl;
        return -1;
    }

    // 创建窗口
    cv::namedWindow("Control", cv::WINDOW_NORMAL);
    cv::resizeWindow("Control", 400, 500);

    // 创建滑动条
    cv::createTrackbar("BrightThresh", "Control", &g_brightnessThresh, 255);
    cv::createTrackbar("MinArea", "Control", &g_minLightBarArea, 500);
    cv::createTrackbar("MaxArea", "Control", &g_maxLightBarArea, 5000);
    cv::createTrackbar("MinAspect*10", "Control", &g_minAspectRatio, 100);
    cv::createTrackbar("MaxAspect*10", "Control", &g_maxAspectRatio, 100);
    cv::createTrackbar("MaxAngleDiff", "Control", &g_maxAngleDiff, 90);
    cv::createTrackbar("YDistRatio%", "Control", &g_maxYDistRatio, 100);
    cv::createTrackbar("XOffsetRatio%", "Control", &g_maxXOffsetRatio, 100);

    cv::Mat frame;
    while (true) {
        cap >> frame;
        if (frame.empty()) break;

        cv::Point2f center;
        cv::Rect rect;
        cv::Mat binary;
        bool detected = detectDoubleRowTarget(frame, center, rect, binary);

        if (detected) {
            cv::rectangle(frame, rect, cv::Scalar(0,255,0), 2);
            cv::circle(frame, center, 4, cv::Scalar(0,0,255), -1);
        } else {
            cv::putText(frame, "No Target", {30,50}, cv::FONT_HERSHEY_SIMPLEX, 1, {0,0,255},2);
        }

        cv::imshow("Result", frame);
        if (!binary.empty()) cv::imshow("Binary", binary);

        if (cv::waitKey(1) == 27) break;
    }

    cap.release();
    cv::destroyAllWindows();
    return 0;
}