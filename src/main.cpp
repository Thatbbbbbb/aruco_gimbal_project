#include "aruco_detection.h"
#include "serial_port.h"
#include "gimbal_transform.h"
#include "config_parser.h"
#include <opencv2/opencv.hpp>
#include <iostream>
#include <unistd.h>

// 海康相机初始化（不变）
cv::VideoCapture initHikCamera(const ProjectConfig& config) {
    cv::VideoCapture cap(config.camera_index);
    if (!cap.isOpened()) {
        std::cerr << "Failed to open Hik camera! Index: " << config.camera_index << std::endl;
        exit(EXIT_FAILURE);
    }

    cap.set(cv::CAP_PROP_FRAME_WIDTH, config.camera_width);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, config.camera_height);
    cap.set(cv::CAP_PROP_FPS, config.camera_fps);

    return cap;
}

int main(int argc, char** argv) {
    // 1. 加载YAML配置文件
    ProjectConfig config;
    std::string yaml_path = "../config/gimbal_aruco_config.yaml";
    if (!ConfigParser::loadConfig(yaml_path, config)) {
        std::cerr << "Failed to load config file!" << std::endl;
        return -1;
    }

    // 2. 初始化Aruco检测器
    ArucoDetector aruco_detector(config.camera_matrix, config.dist_coeffs,
                                 config.aruco_dict_id, config.marker_size);

    // 3. 初始化串口（传入新配置）
    SerialPort serial;
    if (!serial.init(config.serial_port, config.serial_config)) {
        std::cerr << "Failed to init serial port!" << std::endl;
        return -1;
    }

    // 4. 初始化海康相机
    cv::VideoCapture cap = initHikCamera(config);

    // 5. 主循环
    cv::Mat frame;
    GimbalData current_gimbal, target_gimbal;
    cv::Vec3d marker_cam_coord;
    bool is_timeout;

    while (true) {
        cap >> frame;
        if (frame.empty()) {
            std::cerr << "Failed to read frame!" << std::endl;
            break;
        }

        // 6. 检测Aruco码
        bool marker_detected = aruco_detector.detectMarker(frame, marker_cam_coord);
        if (!marker_detected) {
            std::cout << "No aruco marker detected!" << std::endl;
            cv::imshow("Frame", frame);
            if (cv::waitKey(1) == 27) break;
            continue;
        }

        // 7. 接收云台当前角度（带超时+重传）
        if (!serial.receiveGimbalData(current_gimbal, is_timeout)) {
            if (is_timeout) {
                std::cerr << "Receive gimbal data timeout!" << std::endl;
            } else {
                std::cerr << "Receive gimbal data error (check frame/crc)!" << std::endl;
            }
            continue; // 接收失败则跳过发送，保证交互顺序
        }

        // 8. 坐标转换
        camToGimbalTransform(marker_cam_coord, config.gimbal_pos_in_cam, target_gimbal.yaw, target_gimbal.pitch);

        // 9. 发送目标角度给云台（带重传）
        if (!serial.sendGimbalData(target_gimbal)) {
            std::cerr << "Send gimbal data failed after retry!" << std::endl;
            continue;
        }

        // 10. 打印调试信息
        std::cout << "=====================================" << std::endl;
        std::cout << "Marker Cam Coord: " << marker_cam_coord << std::endl;
        std::cout << "Current Gimbal: Yaw=" << current_gimbal.yaw << " Pitch=" << current_gimbal.pitch << std::endl;
        std::cout << "Target Gimbal: Yaw=" << target_gimbal.yaw << " Pitch=" << target_gimbal.pitch << std::endl;

        // 11. 显示图像
        cv::imshow("Frame", frame);
        if (cv::waitKey(1) == 27) break;

        // 12. 循环延迟
        usleep(config.loop_delay_ms * 1000);
    }

    // 释放资源
    cap.release();
    cv::destroyAllWindows();
    serial.close();

    return 0;
}