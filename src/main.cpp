// #include "serial_port.h"
// #include "engineer_kinematics.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <unistd.h>
#include <chrono>
#include <cmath>
#include <thread>
#include <mutex>
#include "drone.hpp"
#include "yolos/hrt_yolo.hpp"
// #include "gimbal_coordinate.hpp"
#include "camera.hpp"
#include "tasks/lightbar_detector.hpp"  // 添加灯条检测器

// 计时工具：获取当前时间（秒）
double getTimeSeconds() {
    auto now = std::chrono::steady_clock::now();
    auto duration = now.time_since_epoch();
    return std::chrono::duration<double>(duration).count();
}

// 注释掉云台相关函数
/*
std::vector<float> convertEulerListToVector(const std::vector<std::array<float, 3>>& euler_list)
{
    std::vector<float> result;
    result.reserve(euler_list.size() * 3);

    for (const auto& arr : euler_list) {
        result.push_back(arr[0] * 180.0 / M_PI);
        result.push_back(arr[1] * 180.0 / M_PI);
        result.push_back(arr[2] * 180.0 / M_PI);
    }
    return result;
}

Eigen::Isometry3d eulerToIsometry(float roll, float pitch, float yaw)
{
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();

    Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());
    Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());

    Eigen::Quaterniond q = yawAngle * pitchAngle * rollAngle;
    T.rotate(q);
    return T;
}

std::vector<float> rad_to_deg(const std::vector<float>& rad_vec) {
    std::vector<float> deg_vec;
    deg_vec.reserve(rad_vec.size());
    for (float rad : rad_vec) {
        deg_vec.push_back(rad * 180.0f / M_PI);
    }
    return deg_vec;
}
*/

int main(int argc, char** argv) {

    // ======================
    // 注释掉串口初始化
    // ======================
    // hitcrt::serial::SerialPort serial("/dev/ttyUSB0", 100);
    // if (!serial.open_port()) {
    //     std::cerr << "Failed to init serial port!" << std::endl;
    //     return -1;
    // }

    // ======================
    // 注释掉运动学初始化
    // ======================
    // hitcrt::kinematics::params my_params;
    // hitcrt::kinematics::engineer_kinematics kin(my_params);

    // ======================
    // 1. 初始化无人机检测器
    // ======================
    std::string config_path = "/home/thatbbbbbb/projects/aruco_gimbal_project/config/drone.yaml";
    drone_detection::DroneDetector detector(config_path, false);
    
    // ======================
    // 2. 初始化灯条检测器
    // ======================
    drone_detection::LightbarDetector lightbar_detector;
    
    // ======================
    // 3. 初始化相机
    // ======================
    io::Camera camera("/home/thatbbbbbb/projects/aruco_gimbal_project/config/camera.yaml");
    
    // ======================
    // 注释掉相机标定和坐标转换
    // ======================
    // CameraConfig cam_config;
    // cam_config.load("config/camera_calibration.yaml");
    // GimbalCoordinateConverter converter(cam_config);

    // ======================
    // 云台角度变量（保留但未使用）
    // ======================
    int frame_count = 0;
    // std::vector<float> current_joint_angles(3);
    
    float pitch_deg = 0.0f;
    float roll_deg = 0.0f;
    float yaw_deg = 0.0f;
    
    double last_scan_time = getTimeSeconds();
    double scan_interval = 2.0;
    float scan_angle = 0.0f;
    
    // 目标追踪相关
    bool has_target = false;
    float target_confidence = 0.0f;
    cv::Point2f target_pixel;
    cv::Point2f target_center_norm;
    cv::Rect current_roi;
    
    // 统计变量
    int total_frames = 0;
    int lightbar_detected_frames = 0;

    std::cout << "========================================" << std::endl;
    std::cout << "Drone Detection Test Program" << std::endl;
    std::cout << "Green Box: Drone ROI" << std::endl;
    std::cout << "Red Box: Lightbar Set" << std::endl;
    std::cout << "Yellow Circle: Aim Point (Lightbar Center)" << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << "Output format: frame_id, has_target, lightbar_center_x, lightbar_center_y, confidence" << std::endl;
    std::cout << "========================================" << std::endl;

    while (true) {
        double current_time = getTimeSeconds();
        
        // ======================
        // 读取图像
        // ======================
        cv::Mat frame;
        auto timestamp = std::chrono::steady_clock::now();
        camera.read(frame, timestamp); 
        
        if (!frame.empty()) {
            frame_count++;
            total_frames++;
            cv::Mat display_frame = frame.clone();
            
            // ======================
            // 第一步：检测无人机（获取ROI）
            // ======================
            int target_w = 640, target_h = 640;
            float scale;
            cv::Point2f offset;
            cv::Mat processed_frame = drone_detection::resizeAndPad(frame, target_w, target_h, scale, offset);
            auto drones = detector.detect(processed_frame, frame_count);
            
            cv::Rect target_roi;
            bool found_lightbar = false;
            cv::Point2f aim_point;
            
            if (!drones.empty()) {
                // 选择置信度最高的无人机
                auto best_drone = std::max_element(drones.begin(), drones.end(),
                    [](const drone_detection::Drone& a, const drone_detection::Drone& b) {
                        return a.confidence < b.confidence;
                    });
                
                // 获取无人机边界框作为ROI
                target_roi = best_drone->box;
                
                // 按中心等比放大 ROI，使面积约为原来的两倍
                const float roi_scale = std::sqrt(2.0f);
                const cv::Point2f roi_center(
                    target_roi.x + target_roi.width * 0.5f,
                    target_roi.y + target_roi.height * 0.5f);
                const int scaled_width = std::max(1, static_cast<int>(std::round(target_roi.width * roi_scale)));
                const int scaled_height = std::max(1, static_cast<int>(std::round(target_roi.height * roi_scale)));
                cv::Rect scaled_roi(
                    static_cast<int>(std::round(roi_center.x - scaled_width * 0.5f)),
                    static_cast<int>(std::round(roi_center.y - scaled_height * 0.5f)),
                    scaled_width,
                    scaled_height);
                target_roi = scaled_roi & cv::Rect(0, 0, frame.cols, frame.rows);

                if (target_roi.empty()) {
                    continue;
                }
                
                // 绘制绿色ROI框（无人机检测区域）
                cv::rectangle(display_frame, target_roi, cv::Scalar(0, 255, 0), 2);
                cv::putText(display_frame, "Drone ROI", 
                            cv::Point(target_roi.x, target_roi.y - 5),
                            cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);
                
                // ======================
                // 第二步：在ROI内检测灯条组
                // ======================
              
                cv::putText(display_frame, 
                            cv::format("Drone Conf: %.2f", best_drone->confidence),
                            cv::Point(target_roi.x, target_roi.y + target_roi.height + 15),
                            cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);
                
            } else {
                // 没有检测到无人机
                std::cout << "Frame " << frame_count 
                          << " | No drone detected" << std::endl;
                
                // 扫描模式提示
                if (current_time - last_scan_time >= scan_interval) {
                    scan_angle = (scan_angle >= 30.0f) ? -30.0f : scan_angle + 10.0f;
                    last_scan_time = current_time;
                }
            }
            
            // ======================
            // 显示统计信息
            // ======================
            cv::putText(display_frame, cv::format("Frame: %d", frame_count), 
                        cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.6, 
                        cv::Scalar(255, 255, 255), 2);
            
            cv::putText(display_frame, cv::format("Drone Detected: %s", 
                        (!drones.empty()) ? "YES" : "NO"), 
                        cv::Point(10, 60), cv::FONT_HERSHEY_SIMPLEX, 0.5, 
                        (!drones.empty()) ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255), 1);
            
            cv::putText(display_frame, cv::format("Lightbar Set: %s", 
                        (found_lightbar) ? "YES" : "NO"), 
                        cv::Point(10, 85), cv::FONT_HERSHEY_SIMPLEX, 0.5, 
                        (found_lightbar) ? cv::Scalar(0, 255, 255) : cv::Scalar(0, 0, 255), 1);
            
            // 显示说明文字
            cv::putText(display_frame, "Green: Drone ROI | Red: Lightbar Set | Yellow: Aim Point", 
                        cv::Point(10, frame.rows - 10), cv::FONT_HERSHEY_SIMPLEX, 0.45, 
                        cv::Scalar(200, 200, 200), 1);
            
            // 显示图像
            cv::imshow("Drone Detection Test", display_frame);
        }
        
        // ======================
        // 键盘控制
        // ======================
        char key = cv::waitKey(1);
        switch (key) {
            case 'q':
            case 'Q':
                std::cout << "\n========================================" << std::endl;
                std::cout << "Test Summary:" << std::endl;
                std::cout << "Total Frames: " << total_frames << std::endl;
                std::cout << "Frames with Lightbar: " << lightbar_detected_frames << std::endl;
                if (total_frames > 0) {
                    std::cout << "Detection Rate: " 
                              << (float)lightbar_detected_frames / total_frames * 100 
                              << "%" << std::endl;
                }
                std::cout << "========================================" << std::endl;
                std::cout << "Exiting..." << std::endl;
                goto exit;
                break;
            case 's':
                // 保存当前帧
                {
                    cv::Mat screenshot;
                    camera.read(screenshot, timestamp);
                    std::string filename = cv::format("screenshot_%d.jpg", frame_count);
                    cv::imwrite(filename, screenshot);
                    std::cout << "Screenshot saved: " << filename << std::endl;
                }
                break;
            default:
                break;
        }

        usleep(10000); // 10ms loop
    }
    
exit:
    // serial.close_port();  // 注释掉
    cv::destroyAllWindows();
    return 0;
}