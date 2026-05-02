#ifndef CAMERA_CALIBRATION_HPP
#define CAMERA_CALIBRATION_HPP

#include <yaml-cpp/yaml.h>
#include <opencv2/opencv.hpp>
#include <iostream>

// 相机内参结构体
struct CameraIntrinsics {
    float fx = 800.0f;
    float fy = 800.0f;
    float cx = 640.0f;
    float cy = 360.0f;
    float image_width = 1280.0f;
    float image_height = 720.0f;
    
    // 畸变系数
    float k1 = 0.0f, k2 = 0.0f, p1 = 0.0f, p2 = 0.0f, k3 = 0.0f;
    
    // 从YAML加载
    void load(const YAML::Node& yaml) {
        if (yaml["camera_intrinsics"]) {
            auto intr = yaml["camera_intrinsics"];
            fx = intr["fx"].as<float>(fx);
            fy = intr["fy"].as<float>(fy);
            cx = intr["cx"].as<float>(cx);
            cy = intr["cy"].as<float>(cy);
            image_width = intr["image_width"].as<float>(image_width);
            image_height = intr["image_height"].as<float>(image_height);
        }
        
        if (yaml["distortion_coeffs"]) {
            auto dist = yaml["distortion_coeffs"];
            k1 = dist["k1"].as<float>(k1);
            k2 = dist["k2"].as<float>(k2);
            p1 = dist["p1"].as<float>(p1);
            p2 = dist["p2"].as<float>(p2);
            k3 = dist["k3"].as<float>(k3);
        }
    }
    
    // 获取相机矩阵 (OpenCV格式)
    cv::Mat getCameraMatrix() const {
        return (cv::Mat_<float>(3, 3) << fx, 0, cx, 0, fy, cy, 0, 0, 1);
    }
    
    // 获取畸变系数
    cv::Mat getDistortionCoeffs() const {
        return (cv::Mat_<float>(1, 5) << k1, k2, p1, p2, k3);
    }
    
    // 像素点到归一化坐标
    cv::Point2f pixelToNormalized(const cv::Point2f& pixel) const {
        return cv::Point2f(
            (pixel.x - cx) / fx,
            (pixel.y - cy) / fy
        );
    }
    
    // 归一化坐标到像素点
    cv::Point2f normalizedToPixel(const cv::Point2f& norm) const {
        return cv::Point2f(
            norm.x * fx + cx,
            norm.y * fy + cy
        );
    }
};

// 云台-相机外参结构体
struct GimbalCameraExtrinsics {
    // 平移向量 (米)
    float tx = 0.0f, ty = 0.0f, tz = 0.0f;
    
    // 旋转角度 (度)
    float roll_offset = 0.0f;
    float pitch_offset = 0.0f;
    float yaw_offset = 0.0f;
    
    // 从YAML加载
    void load(const YAML::Node& yaml) {
        if (yaml["gimbal_camera_extrinsics"]) {
            auto ext = yaml["gimbal_camera_extrinsics"];
            
            if (ext["translation"]) {
                auto trans = ext["translation"];
                tx = trans["x"].as<float>(tx);
                ty = trans["y"].as<float>(ty);
                tz = trans["z"].as<float>(tz);
            }
            
            if (ext["rotation"]) {
                auto rot = ext["rotation"];
                roll_offset = rot["roll"].as<float>(roll_offset);
                pitch_offset = rot["pitch"].as<float>(pitch_offset);
                yaw_offset = rot["yaw"].as<float>(yaw_offset);
            }
        }
    }
};

// 跟踪参数结构体
struct TrackingParams {
    float kp = 30.0f;           // 比例增益
    float ki = 0.0f;            // 积分增益
    float kd = 0.0f;            // 微分增益
    float max_angle = 30.0f;    // 最大角度
    float estimated_distance = 10.0f;  // 估计距离
    float fov_h = 60.0f;        // 水平视场角
    float fov_v = 45.0f;        // 垂直视场角
    
    void load(const YAML::Node& yaml) {
        if (yaml["tracking_params"]) {
            auto track = yaml["tracking_params"];
            kp = track["kp"].as<float>(kp);
            ki = track["ki"].as<float>(ki);
            kd = track["kd"].as<float>(kd);
            max_angle = track["max_angle"].as<float>(max_angle);
            estimated_distance = track["estimated_distance"].as<float>(estimated_distance);
            
            if (track["field_of_view"]) {
                auto fov = track["field_of_view"];
                fov_h = fov["horizontal"].as<float>(fov_h);
                fov_v = fov["vertical"].as<float>(fov_v);
            }
        }
    }
};

// 完整的相机配置
struct CameraConfig {
    CameraIntrinsics intrinsics;
    GimbalCameraExtrinsics extrinsics;
    TrackingParams tracking;
    
    void load(const std::string& config_path) {
        YAML::Node yaml = YAML::LoadFile(config_path);
        intrinsics.load(yaml);
        extrinsics.load(yaml);
        tracking.load(yaml);
        
        std::cout << "Camera config loaded from: " << config_path << std::endl;
        std::cout << "  fx=" << intrinsics.fx << ", fy=" << intrinsics.fy << std::endl;
        std::cout << "  cx=" << intrinsics.cx << ", cy=" << intrinsics.cy << std::endl;
        std::cout << "  image size: " << intrinsics.image_width << "x" << intrinsics.image_height << std::endl;
    }
};

#endif // CAMERA_CALIBRATION_HPP