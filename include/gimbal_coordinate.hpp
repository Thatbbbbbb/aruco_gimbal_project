#ifndef GIMBAL_COORDINATE_HPP
#define GIMBAL_COORDINATE_HPP

#include <yaml-cpp/yaml.h>
#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath>
#include <iostream>

// 相机内参结构体
struct CameraIntrinsics {
    float fx = 800.0f;
    float fy = 800.0f;
    float cx = 640.0f;
    float cy = 360.0f;
    float image_width = 1280.0f;
    float image_height = 720.0f;
    
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
    }
    
    // 像素坐标 -> 归一化坐标
    cv::Point2f pixelToNormalized(const cv::Point2f& pixel) const {
        return cv::Point2f(
            (pixel.x - cx) / fx,
            (pixel.y - cy) / fy
        );
    }
};

// 云台-相机外参
struct GimbalCameraExtrinsics {
    float tx = 0.0f, ty = 0.0f, tz = 0.0f;  // 平移
    float roll = 0.0f, pitch = 0.0f, yaw = 0.0f;  // 旋转 (度)
    
    void load(const YAML::Node& yaml) {
        if (yaml["gimbal_camera_extrinsics"]) {
            auto ext = yaml["gimbal_camera_extrinsics"];
            
            if (ext["translation"]) {
                auto t = ext["translation"];
                tx = t["x"].as<float>(tx);
                ty = t["y"].as<float>(ty);
                tz = t["z"].as<float>(tz);
            }
            
            if (ext["rotation"]) {
                auto r = ext["rotation"];
                roll = r["roll"].as<float>(roll);
                pitch = r["pitch"].as<float>(pitch);
                yaw = r["yaw"].as<float>(yaw);
            }
        }
    }
};

// 测距参数
struct DepthParams {
    float default_distance = 10.0f;
    bool use_depth_sensor = false;
    
    void load(const YAML::Node& yaml) {
        if (yaml["depth_estimation"]) {
            auto depth = yaml["depth_estimation"];
            default_distance = depth["default_distance"].as<float>(default_distance);
            use_depth_sensor = depth["use_depth_sensor"].as<bool>(use_depth_sensor);
        }
    }
};

// 完整配置
struct CameraConfig {
    CameraIntrinsics intrinsics;
    GimbalCameraExtrinsics extrinsics;
    DepthParams depth;
    
    void load(const std::string& config_path) {
        YAML::Node yaml = YAML::LoadFile(config_path);
        intrinsics.load(yaml);
        extrinsics.load(yaml);
        depth.load(yaml);
        
        std::cout << "Camera config loaded: " << config_path << std::endl;
    }
};

// 云台坐标系下的3D点
struct GimbalPoint3D {
    float x, y, z;      // 云台坐标系下的坐标 (米)
    float distance;     // 距离 (米)
    float yaw_angle;    // 水平角 (度)
    float pitch_angle;  // 俯仰角 (度)
    
    void print() const {
        std::cout << "Gimbal coordinates: (" << x << ", " << y << ", " << z << ") m"
                  << " | Distance: " << distance << " m"
                  << " | Angle: yaw=" << yaw_angle << "°, pitch=" << pitch_angle << "°"
                  << std::endl;
    }
};

// 坐标转换器
class GimbalCoordinateConverter {
public:
    GimbalCoordinateConverter(const CameraConfig& config) 
        : config_(config) {}
    
    // 核心函数：将无人机像素坐标转换为云台坐标系下的3D坐标
    GimbalPoint3D pixelToGimbal(const cv::Point2f& pixel, 
                                 float distance = -1.0f) {
        GimbalPoint3D result;
        
        // 1. 获取距离（如果没有提供，使用默认值）
        float z_camera = (distance > 0) ? distance : config_.depth.default_distance;
        
        // 2. 像素坐标 -> 相机坐标系
        cv::Point2f norm = config_.intrinsics.pixelToNormalized(pixel);
        
        // 相机坐标系下的一点
        Eigen::Vector3f p_camera(norm.x * z_camera, 
                                  norm.y * z_camera, 
                                  z_camera);
        
        // 3. 相机坐标系 -> 安装坐标系（考虑相机安装偏差）
        Eigen::Matrix3f R_cam_to_mount = getCameraToMountRotation();
        Eigen::Vector3f p_mount = R_cam_to_mount * p_camera;
        
        // 加上平移偏移
        p_mount += Eigen::Vector3f(config_.extrinsics.tx, 
                                    config_.extrinsics.ty, 
                                    config_.extrinsics.tz);
        
        // 4. 安装坐标系 -> 云台坐标系
        // 云台原点与安装原点重合，只是方向相同
        // 如果云台有当前角度，需要在此处转换，但这里假设云台回零位
        // 实际使用时，如果有云台当前角度，可以传入进行转换
        
        // 最终坐标
        result.x = p_mount.x();
        result.y = p_mount.y();
        result.z = p_mount.z();
        result.distance = std::sqrt(result.x * result.x + 
                                    result.y * result.y + 
                                    result.z * result.z);
        
        // 计算云台坐标系下的角度
        result.yaw_angle = std::atan2(result.x, result.z) * 180.0 / M_PI;
        result.pitch_angle = std::atan2(result.y, result.z) * 180.0 / M_PI;
        
        return result;
    }
    
    // 带云台当前角度的转换（如果需要考虑云台旋转）
    GimbalPoint3D pixelToGimbalWithGimbalAngle(const cv::Point2f& pixel,
                                                float gimbal_pitch_deg,
                                                float gimbal_yaw_deg,
                                                float distance = -1.0f) {
        GimbalPoint3D result;
        
        float z_camera = (distance > 0) ? distance : config_.depth.default_distance;
        cv::Point2f norm = config_.intrinsics.pixelToNormalized(pixel);
        
        // 相机坐标系下的点
        Eigen::Vector3f p_camera(norm.x * z_camera, norm.y * z_camera, z_camera);
        
        // 相机 -> 安装坐标系
        Eigen::Matrix3f R_cam_to_mount = getCameraToMountRotation();
        Eigen::Vector3f p_mount = R_cam_to_mount * p_camera;
        p_mount += Eigen::Vector3f(config_.extrinsics.tx, 
                                    config_.extrinsics.ty, 
                                    config_.extrinsics.tz);
        
        // 考虑云台当前角度（将点从云台坐标系转到世界坐标系）
        float pitch_rad = gimbal_pitch_deg * M_PI / 180.0f;
        float yaw_rad = gimbal_yaw_deg * M_PI / 180.0f;
        
        Eigen::Matrix3f R_gimbal;
        R_gimbal = Eigen::AngleAxisf(yaw_rad, Eigen::Vector3f::UnitZ())
                 * Eigen::AngleAxisf(pitch_rad, Eigen::Vector3f::UnitY());
        
        Eigen::Vector3f p_gimbal = R_gimbal.transpose() * p_mount;
        
        result.x = p_gimbal.x();
        result.y = p_gimbal.y();
        result.z = p_gimbal.z();
        result.distance = std::sqrt(result.x * result.x + 
                                    result.y * result.y + 
                                    result.z * result.z);
        result.yaw_angle = std::atan2(result.x, result.z) * 180.0 / M_PI;
        result.pitch_angle = std::atan2(result.y, result.z) * 180.0 / M_PI;
        
        return result;
    }
    
    // 获取配置信息
    const CameraConfig& getConfig() const { return config_; }
    
private:
    CameraConfig config_;
    
    Eigen::Matrix3f getCameraToMountRotation() const {
        float roll_rad = config_.extrinsics.roll * M_PI / 180.0f;
        float pitch_rad = config_.extrinsics.pitch * M_PI / 180.0f;
        float yaw_rad = config_.extrinsics.yaw * M_PI / 180.0f;
        
        Eigen::Matrix3f R;
        R = Eigen::AngleAxisf(yaw_rad, Eigen::Vector3f::UnitZ())
          * Eigen::AngleAxisf(pitch_rad, Eigen::Vector3f::UnitY())
          * Eigen::AngleAxisf(roll_rad, Eigen::Vector3f::UnitX());
        return R;
    }
};

#endif // GIMBAL_COORDINATE_HPP