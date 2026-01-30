#include "config_parser.h"
#include <yaml-cpp/yaml.h>
#include <opencv2/aruco.hpp>
#include <iostream>

bool ConfigParser::loadConfig(const std::string& yaml_path, ProjectConfig& config) {
    try {
        // 加载YAML文件
        YAML::Node root = YAML::LoadFile(yaml_path);

        // 解析串口配置
        YAML::Node serial_node = root["serial_config"];
        config.serial_port = serial_node["port_name"].as<std::string>();
        config.serial_baudrate = serial_node["baudrate"].as<uint32_t>();

        // 解析相机配置
        YAML::Node camera_node = root["camera_config"];
        config.camera_index = camera_node["camera_index"].as<int>();
        config.camera_width = camera_node["width"].as<int>();
        config.camera_height = camera_node["height"].as<int>();
        config.camera_fps = camera_node["fps"].as<int>();

        // 解析相机内参（3x3）
        std::vector<double> cam_mat_vec = camera_node["camera_matrix"].as<std::vector<double>>();
        config.camera_matrix = cv::Mat(3, 3, CV_64F, cam_mat_vec.data()).clone();

        // 解析畸变参数（1x5）
        std::vector<double> dist_vec = camera_node["dist_coeffs"].as<std::vector<double>>();
        config.dist_coeffs = cv::Mat(1, 5, CV_64F, dist_vec.data()).clone();

        // 解析ArUco配置
        YAML::Node aruco_node = root["aruco_config"];
        std::string dict_name = aruco_node["dict_id"].as<std::string>();
        config.aruco_dict_id = arucoDictNameToId(dict_name);
        config.marker_size = aruco_node["marker_size"].as<float>();

        // 解析云台配置
        YAML::Node gimbal_node = root["gimbal_config"];
        std::vector<double> gimbal_pos_vec = gimbal_node["position_in_cam"].as<std::vector<double>>();
        config.gimbal_pos_in_cam = cv::Vec3d(gimbal_pos_vec[0], gimbal_pos_vec[1], gimbal_pos_vec[2]);
        config.loop_delay_ms = gimbal_node["loop_delay_ms"].as<int>();

        return true;
    } catch (const YAML::Exception& e) {
        std::cerr << "YAML parse error: " << e.what() << std::endl;
        return false;
    } catch (const std::exception& e) {
        std::cerr << "Config load error: " << e.what() << std::endl;
        return false;
    }
}

int ConfigParser::arucoDictNameToId(const std::string& dict_name) {
    if (dict_name == "DICT_6X6_250") return cv::aruco::DICT_6X6_250;
    if (dict_name == "DICT_4X4_100") return cv::aruco::DICT_4X4_100;
    if (dict_name == "DICT_5X5_250") return cv::aruco::DICT_5X5_250;
    // 默认返回6X6_250
    return cv::aruco::DICT_6X6_250;
}