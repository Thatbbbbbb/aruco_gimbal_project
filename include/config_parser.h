#ifndef CONFIG_PARSER_H
#define CONFIG_PARSER_H

#include <string>
#include <opencv2/core.hpp>
#include "serial_port.h"

// 汇总所有配置的结构体
struct ProjectConfig {
    // 串口配置
    std::string serial_port;
    uint32_t serial_baudrate;

    // 相机配置
    int camera_index;
    int camera_width;
    int camera_height;
    int camera_fps;
    cv::Mat camera_matrix;
    cv::Mat dist_coeffs;

    // ArUco配置
    int aruco_dict_id;
    float marker_size;

    // 云台配置
    cv::Vec3d gimbal_pos_in_cam;
    int loop_delay_ms;
};

// YAML配置解析类
class ConfigParser {
public:
    // 从YAML文件加载配置
    static bool loadConfig(const std::string& yaml_path, ProjectConfig& config);

private:
    // 辅助函数：将YAML中的字符串转换为ArUco字典ID
    static int arucoDictNameToId(const std::string& dict_name);
};

#endif