/*
#include "hrt_yolo.hpp"

int main() {
    // 初始化检测器
    drone_detection::DroneDetector detector("drone_config.yaml", true);
    
    // 打开视频或摄像头
    cv::VideoCapture cap(0);  // 或视频文件路径
    cv::Mat frame;
    int frame_count = 0;
    
    while (cap.read(frame)) {
        frame_count++;
        
        // 检测无人机
        auto drones = detector.detect(frame, frame_count);
        
        // 处理每个检测到的无人机
        for (auto& drone : drones) {
            // 1. 获取角点坐标
            std::vector<cv::Point2f> corners = drone.points;  // 已经是四个角点
            std::cout << "Drone corners:" << std::endl;
            for (size_t i = 0; i < corners.size(); ++i) {
                std::cout << "  Corner " << i << ": (" 
                          << corners[i].x << ", " << corners[i].y << ")" << std::endl;
            }
            
            // 2. 裁剪无人机区域（使用Drone类中的crop方法）
            cv::Mat cropped = drone.crop(frame, 20);
            if (!cropped.empty()) {
                cv::imshow("Cropped Drone", cropped);
            }
            
            // 3. 透视校正（摆正无人机）
            cv::Mat rectified = drone.cropAndRectify(frame, cv::Size(224, 224));
            if (!rectified.empty()) {
                cv::imshow("Rectified Drone", rectified);
            }
            
            // 4. 中心点坐标
            std::cout << "Drone center: (" << drone.center.x 
                      << ", " << drone.center.y << ")" << std::endl;
        }
        
        // 显示结果
        if (cv::waitKey(1) == 'q') break;
    }
    
    return 0;
}
*/