#include "detector.h"
#include "serial_port.h"
#include "gimbal_transform.h"
#include "config_parser.h"
#include <opencv2/opencv.hpp>
#include <iostream>
#include <unistd.h>

// 海康相机初始化（不变）
cv::VideoCapture initHikCamera(const Parser::parser::ConfigData& config) {
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

    Parser::parser::ConfigData config;
    std::string yaml_path = "../config/gimbal_aruco_config.yaml";
    if (!Parser::parser::loadConfig(yaml_path, config)) {
        std::cerr << "Failed to load config file!" << std::endl;
        return -1;
    }
    hitcrt::kinematics::EngineerKinematics kinematics(config.kinematics_params);
    GimbalTransformer gimbal_transformer(config);
    auto_aim::Detector detector(yaml_path);
    cv::VideoCapture cap(0);
    
    if (!cap.isOpened()) return -1;

    cv::Mat frame;
    int count = 0;


    // 初始化串口（传入新配置）
    hitcrt::serial::SerialPort serial(config.serial_port, config.serial_timeout);
    if (!serial.open_port()) {
        std::cerr << "Failed to init serial port!" << std::endl;
        return -1;
    }

    // 初始化海康相机
    cv::VideoCapture cap = initHikCamera(config);

    cv::Mat frame;
    int frame_count = 0; // 帧计数（传给detect函数）
    std::vector<double> current_angles(3), target_angles(3);
    cv::Vec3d marker_cam_coord;
    bool is_timeout;

    while (true) {
        cap >> frame;
        if (frame.empty()) {
            std::cerr << "Failed to read frame!" << std::endl;
            break;
        }
        

        // 
        std::list<auto_aim::Lightbar> lightbars = detector.detect(frame, frame_count);
        if (lightbars.empty()) {
            std::cout << "No aruco marker detected!" << std::endl;
            cv::imshow("Frame", frame);
            if (cv::waitKey(1) == 27) break;
            continue;
        }

        //  接收云台当前角度（带超时+重传）
      if (!serial.receive_motor_angles(current_angles) {
            if (is_timeout) {
                std::cerr << "Receive gimbal data timeout!" << std::endl;
            } else {
                std::cerr << "Receive gimbal data error (check frame/crc)!" << std::endl;
            }
            continue; 
        }

 
        Eigen::Isometry3d forward_pose = kinematics.forward_kinematics( current_angles);
   
        // 输出正解结果,此时云台角度
        Eigen::Vector3d translation = forward_pose.translation();
        std::cout << "X轴平移：" << translation.x() << std::endl;
        std::cout << "Y轴平移：" << translation.y() << std::endl;
        std::cout << "Z轴平移：" << translation.z() << std::endl;
        
        
        //这里传入一个假设的位姿target_pose
        hitcrt::kinematics::Params params; 
        hitcrt::kinematics::EngineerKinematics kinematics(params);
        Eigen::Isometry3d target_pose = Eigen::Isometry3d::Identity();  // 初始位姿（单位矩阵）
        target_pose.pretranslate(Eigen::Vector3d(0.1, 0.2, 0.3)); 
        Eigen::Vector3d gimbal_coords = gimbal_transformer.cam_to_gimbal_pose(target_pose).translation();     // 设置平移：x=0.1, y=0.2, z=0.3
        bool inv_success = kinematics.inverse_kinematics(target_pose, target_angles);
    
        // 输出逆解结果
        if (inv_success) {
            std::cout << "=== 逆运动学求解结果（电机角度，单位：弧度）===" << std::endl;
            std::cout << "电机1角度：" << target_angles[0] << " (约" << target_angles[0]*180/M_PI << "度)" << std::endl;
            std::cout << "电机2角度：" << target_angles[1] << " (约" << target_angles[1]*180/M_PI << "度)" << std::endl;
            std::cout << "电机3角度：" << target_angles[2] << " (约" << target_angles[2]*180/M_PI << "度)" << std::endl;
        } else {
            std::cerr << "逆运动学求解失败！" << std::endl;
        }

        // 发送目标角度给云台（带重传）
        if (!serial.send_motor_angles(target_angles)) {
            std::cerr << "Send gimbal data failed after retry!" << std::endl;
            continue;
        }

        std::cout << "=====================================" << std::endl;
        std::cout << "Marker Cam Coord: " << marker_cam_coord << std::endl;
        std::cout << "Current Gimbal Angles: " << current_angles[0]<< ", " << current_angles[1] << ", " << current_angles[2]<< std::endl;
        std::cout << "Target Gimbal Angles: " << target_angles[0] <<    ", " << target_angles[2] << ", " << target_angles[3]<< std::endl;


        cv::imshow("Frame", frame);
        if (cv::waitKey(1) == 27) break;

        usleep(1000);
    }

    cap.release();
    cv::destroyAllWindows();
    serial.close_port();

    return 0;
}