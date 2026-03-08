#pragma once
#include<opencv2/core/core.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#include <string>
#include <vector>
#include <yaml-cpp/yaml.h>
#include "engineer_kinematics.h"
namespace Parser{
class parser  
{
public:
    struct ConfigData {
        
        // 相机内参
        double fx, fy, cx, cy;
        std::vector<double> distortion_coeffs;
        int camera_index;          // 海康相机索引
        int camera_width;          // 视频流宽度
        int camera_height;         // 视频流高度
        int camera_fps;            // 视频流帧率
        cv::Mat camera_matrix_;     // 内参矩阵
        cv::Mat dist_coeffs_;       // 畸变系数矩阵

        // 检测参数
        double threshold_;          // 二值化阈值
        double max_angle_error_;    // 最大角度误差（弧度） 
        double min_lightbar_ratio_; // 最小灯条长宽比
        double max_lightbar_ratio_; // 最大灯条长宽比
        double min_lightbar_length_; // 最小灯条长度

        
        // 相机→云台转换
        std::vector<double> cam_to_gimbal_rot;
        std::vector<double> cam_to_gimbal_trans;

        // 串口配置（波特率固化115200）
        std::string serial_port;
        int serial_timeout;

        // 运动学参数
        hitcrt::kinematics::Params kinematics_params;
    };

    static bool loadConfig(const std::string& config_path, Parser::parser::ConfigData& config);
};
}