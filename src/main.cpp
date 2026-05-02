#include "serial_port.h"
#include "engineer_kinematics.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <unistd.h>
#include <chrono>
#include <thread>
#include <mutex>
#include "drone.hpp"
#include "yolos/hrt_yolo.hpp"
#include "gimbal_coordinate.hpp"
#include "camera.hpp"
#include "tasks/lightbar_detector.hpp"  // 添加灯条检测器

// 计时工具：获取当前时间（秒）
double getTimeSeconds() {
    auto now = std::chrono::steady_clock::now();
    auto duration = now.time_since_epoch();
    return std::chrono::duration<double>(duration).count();
}

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

int main(int argc, char** argv) {

    // ======================
    // 1. 初始化串口（云台）
    // ======================
    hitcrt::serial::SerialPort serial("/dev/ttyUSB0", 100);
    if (!serial.open_port()) {
        std::cerr << "Failed to init serial port!" << std::endl;
        return -1;
    }

    // ======================
    // 2. 初始化运动学
    // ======================
    hitcrt::kinematics::params my_params;
    hitcrt::kinematics::engineer_kinematics kin(my_params);

    // ======================
    // 3. 初始化无人机检测器
    // ======================
    std::string config_path = "config/drone.yaml";
    drone_detection::DroneDetector detector(config_path, false);
    
    // ======================
    // 4. 初始化灯条检测器
    // ======================
    drone_detection::LightbarDetector lightbar_detector;
    
    // ======================
    // 5. 初始化相机
    // ======================
    io::Camera camera("config/camera.yaml");
    
    // ======================
    // 6. 加载相机标定参数
    // ======================
    CameraConfig cam_config;
    cam_config.load("config/camera_calibration.yaml");
    
    // ======================
    // 7. 创建坐标转换器
    // ======================
    GimbalCoordinateConverter converter(cam_config);

    // ======================
    // 云台角度变量
    // ======================
    int frame_count = 0;
    std::vector<float> current_joint_angles(3);
    
    float pitch_deg = 0.0f;
    float roll_deg = 0.0f;
    float yaw_deg = 0.0f;
    
    double last_scan_time = getTimeSeconds();
    double scan_interval = 2.0;
    float scan_angle = 0.0f;
    
    // 目标追踪相关
    bool has_target = false;
    float target_confidence = 0.0f;
    cv::Point2f target_pixel;           // 瞄准点（灯条组中心）
    cv::Point2f target_center_norm;
    cv::Rect current_roi;               // 当前ROI区域
    
    // 目标位姿
    Eigen::Isometry3d T_desired;

    std::cout << "Starting drone tracking system..." << std::endl;
    std::cout << "Output format: frame_id, has_target, x, y, z, distance, yaw_angle, pitch_angle, confidence" << std::endl;
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
            cv::Mat display_frame = frame.clone();
            
            // ======================
            // 第一步：检测无人机（获取ROI）
            // ======================
            auto drones = detector.detect(frame, frame_count);
            
            cv::Rect target_roi;
            bool found_lightbar = false;
            cv::Point2f aim_point;  // 最终瞄准点
            
            if (!drones.empty()) {
                // 选择置信度最高的无人机
                auto best_drone = std::max_element(drones.begin(), drones.end(),
                    [](const drone_detection::Drone& a, const drone_detection::Drone& b) {
                        return a.confidence < b.confidence;
                    });
                
                // 获取无人机边界框作为ROI
                target_roi = best_drone->box;
                
                // 扩展ROI区域（给灯条检测留出边距）
                int expand = 40;
                target_roi.x = std::max(0, target_roi.x - expand);
                target_roi.y = std::max(0, target_roi.y - expand);
                target_roi.width = std::min(frame.cols - target_roi.x, target_roi.width + 2 * expand);
                target_roi.height = std::min(frame.rows - target_roi.y, target_roi.height + 2 * expand);
                
                // 绘制ROI区域
                cv::rectangle(display_frame, target_roi, cv::Scalar(0, 255, 0), 2);
                
                // ======================
                // 第二步：在ROI内检测灯条组
                // ======================
                auto lightbar_sets = lightbar_detector.detect(frame, target_roi);
                
                if (!lightbar_sets.empty()) {
                    // 选择置信度最高的灯条组
                    auto best_set = std::max_element(lightbar_sets.begin(), lightbar_sets.end(),
                        [](const drone_detection::ParallelLightbarSet& a, 
                           const drone_detection::ParallelLightbarSet& b) {
                            return a.confidence < b.confidence;
                        });
                    
                    // 瞄准点 = 灯条组中心
                    aim_point = best_set->center;
                    found_lightbar = true;
                    target_confidence = best_set->confidence;
                    
                    // ======================
                    // 可视化灯条组
                    // ======================
                    // 绘制整体中心（黄色大圆）- 瞄准点
                    cv::circle(display_frame, aim_point, 10, cv::Scalar(0, 255, 255), -1);
                    cv::putText(display_frame, "AIM", 
                                cv::Point(aim_point.x - 20, aim_point.y - 10),
                                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 255), 2);
                    
                    // 绘制四个角点（青色）
                    for (const auto& corner : best_set->corners) {
                        cv::circle(display_frame, corner, 3, cv::Scalar(255, 255, 0), -1);
                    }
                    
                    // 绘制上下排中心（红色和蓝色）
                    cv::circle(display_frame, best_set->top_row.center, 4, cv::Scalar(0, 0, 255), -1);
                    cv::circle(display_frame, best_set->bottom_row.center, 4, cv::Scalar(255, 0, 0), -1);
                    
                    // 绘制上下排连接线
                    cv::line(display_frame, best_set->top_row.center, 
                             best_set->bottom_row.center, cv::Scalar(0, 255, 0), 2);
                    
                    // 显示灯条数量信息
                    std::string info = cv::format("Top:%d Bot:%d Conf:%.2f", 
                        (int)best_set->top_row.lightbars.size(),
                        (int)best_set->bottom_row.lightbars.size(),
                        best_set->confidence);
                    cv::putText(display_frame, info, 
                                cv::Point(aim_point.x - 30, aim_point.y + 20),
                                cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255, 255, 255), 1);
                    
                } else {
                    // 没有检测到灯条组，使用无人机中心作为备选
                    aim_point = best_drone->center;
                    target_confidence = best_drone->confidence * 0.5;  // 降低置信度
                    
                    cv::circle(display_frame, aim_point, 8, cv::Scalar(0, 255, 0), -1);
                    cv::putText(display_frame, "DRONE CENTER (FALLBACK)", 
                                cv::Point(aim_point.x - 80, aim_point.y - 10),
                                cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 255, 0), 1);
                }
                
                // ======================
                // 更新追踪目标
                // ======================
                has_target = true;
                target_pixel = aim_point;
                target_center_norm = cv::Point2f(
                    target_pixel.x / frame.cols,
                    target_pixel.y / frame.rows
                );
                
                // 转换为云台坐标系下的3D坐标
                GimbalPoint3D gimbal_pos = converter.pixelToGimbal(target_pixel);
                
                // 输出瞄准点坐标
                std::cout << frame_count << ","
                          << "1" << ","
                          << gimbal_pos.x << ","
                          << gimbal_pos.y << ","
                          << gimbal_pos.z << ","
                          << gimbal_pos.distance << ","
                          << gimbal_pos.yaw_angle << ","
                          << gimbal_pos.pitch_angle << ","
                          << target_confidence 
                          << ",LIGHTBAR"  // 标记来源
                          << std::endl;
                
                // 根据瞄准点计算云台目标角度
                float target_yaw = target_center_norm.x * 60.0f - 30.0f;
                float target_pitch = target_center_norm.y * 60.0f - 30.0f;
                
                pitch_deg = std::clamp(target_pitch, -30.0f, 30.0f);
                yaw_deg = std::clamp(target_yaw, -30.0f, 30.0f);
                
                // 重置扫描时间
                last_scan_time = current_time;
                
                // 在图像上显示瞄准点坐标
                cv::putText(display_frame, 
                            cv::format("Target: (%.1f, %.1f)", target_pixel.x, target_pixel.y),
                            cv::Point(10, 60),
                            cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 255), 2);
                
            } else {
                has_target = false;
                
                // 无目标时输出
                std::cout << frame_count << ",0,0,0,0,0,0,0,0,NO_TARGET" << std::endl;
                
                // 扫描模式
                if (current_time - last_scan_time >= scan_interval) {
                    scan_angle = (scan_angle >= 30.0f) ? -30.0f : scan_angle + 10.0f;
                    yaw_deg = scan_angle;
                    last_scan_time = current_time;
                }
            }
            
            // 显示帧数和状态
            cv::putText(display_frame, cv::format("Frame: %d", frame_count), 
                        cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.6, 
                        has_target ? cv::Scalar(0, 255, 0) : cv::Scalar(255, 255, 255), 2);
            
            // 显示图像
            cv::imshow("Drone Detection", display_frame);
        }
        
        // ======================
        // 更新目标位姿（用于云台控制）
        // ======================
        T_desired = eulerToIsometry(
            roll_deg * M_PI / 180.0f,
            pitch_deg * M_PI / 180.0f,
            yaw_deg * M_PI / 180.0f
        );

        // ======================
        // 接收云台当前角度
        // ======================
        if (!serial.receive_motor_angles(current_joint_angles)) {
            // 接收失败，继续使用目标角度
        }

        // ======================
        // 逆解计算并发送云台角度
        // ======================
        std::vector<std::array<float, 3>> euler_list;
        bool success = kin.inverse_kinematics(T_desired, euler_list);

        if (success && !euler_list.empty()) {
            std::vector<float> euler_vector = convertEulerListToVector(euler_list);
            std::vector<float> target_angles(euler_vector.begin(), euler_vector.begin() + 3);

            if (!serial.send_motor_angles(target_angles)) {
                std::cerr << "Send gimbal data failed!" << std::endl;
            }
        }

        // ======================
        // 键盘控制
        // ======================
        char key = cv::waitKey(1);
        switch (key) {
            case 'q':
            case 'Q':
                std::cout << "Exiting..." << std::endl;
                goto exit;
            case 'w':
                pitch_deg += 5.0f;
                pitch_deg = std::clamp(pitch_deg, -30.0f, 30.0f);
                std::cout << "Manual pitch: " << pitch_deg << "°" << std::endl;
                break;
            case 's':
                pitch_deg -= 5.0f;
                pitch_deg = std::clamp(pitch_deg, -30.0f, 30.0f);
                std::cout << "Manual pitch: " << pitch_deg << "°" << std::endl;
                break;
            case 'a':
                yaw_deg -= 5.0f;
                yaw_deg = std::clamp(yaw_deg, -30.0f, 30.0f);
                std::cout << "Manual yaw: " << yaw_deg << "°" << std::endl;
                break;
            case 'd':
                yaw_deg += 5.0f;
                yaw_deg = std::clamp(yaw_deg, -30.0f, 30.0f);
                std::cout << "Manual yaw: " << yaw_deg << "°" << std::endl;
                break;
            case ' ':
                pitch_deg = 0;
                yaw_deg = 0;
                std::cout << "Angles reset!" << std::endl;
                break;
        }

        usleep(10000);
    }
    
exit:
    serial.close_port();
    cv::destroyAllWindows();
    return 0;
}