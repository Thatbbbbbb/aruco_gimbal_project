#include "config_parser.h"
#include <stdexcept>
#include <iostream>

namespace Parser {
bool parse_config(const std::string& config_path, Parser::parser::ConfigData& config) {
    try {
        YAML::Node yaml_config = YAML::LoadFile(config_path);


        // 解析相机内参
        config.fx = yaml_config["camera"]["fx"].as<double>();
        config.fy = yaml_config["camera"]["fy"].as<double>();
        config.cx = yaml_config["camera"]["cx"].as<double>();
        config.cy = yaml_config["camera"]["cy"].as<double>();
        config.distortion_coeffs = yaml_config["camera"]["distortion_coeffs"].as<std::vector<double>>();
        config.camera_index = yaml_config["camera"]["index"].as<int>();
        config.camera_width = yaml_config["camera"]["capture_width"].as<int>();
        config.camera_height = yaml_config["camera"]["capture_height"].as<int>();
        config.camera_fps = yaml_config["camera"]["capture_fps"].as<int>();
        config.threshold_ = yaml_config["threshold"].as<double>();
        config.max_angle_error_ = yaml_config["max_angle_error"].as<double>() / 57.3;
        config.min_lightbar_ratio_ = yaml_config["min_lightbar_ratio"].as<double>();
        config.max_lightbar_ratio_ = yaml_config["max_lightbar_ratio"].as<double>();
        config.min_lightbar_length_ = yaml_config["min_lightbar_length"].as<double>();

        // 加载相机参数（从配置文件读取，需提前标定）
       
        config.camera_matrix_ = (cv::Mat_<double>(3,3) << 
            config.fx, 0, config.cx,
            0, config.fy, config.cy,
            0, 0, 1);
        // 解析相机→云台转换
        config.cam_to_gimbal_rot = yaml_config["transform"]["camera_to_gimbal_rot"].as<std::vector<double>>();
        config.cam_to_gimbal_trans = yaml_config["transform"]["camera_to_gimbal_trans"].as<std::vector<double>>();

        config.serial_port = yaml_config["serial"]["port"].as<std::string>();
        config.serial_timeout = yaml_config["serial"]["timeout"].as<int>();

        config.kinematics_params.delta = yaml_config["kinematics"]["delta"].as<double>();
        config.kinematics_params.yita1 = yaml_config["kinematics"]["yita1"].as<double>();
        config.kinematics_params.yita2 = yaml_config["kinematics"]["yita2"].as<double>();
        config.kinematics_params.yita3 = yaml_config["kinematics"]["yita3"].as<double>();
        config.kinematics_params.arfa1 = yaml_config["kinematics"]["arfa1"].as<double>();
        config.kinematics_params.arfa2 = yaml_config["kinematics"]["arfa2"].as<double>();

        std::cout << "配置文件解析成功！" << std::endl;
        return true;
    } catch (const YAML::BadFile& e) {
        std::cerr << "配置文件不存在: " << config_path << " 错误: " << e.what() << std::endl;
        return false;
    } catch (const YAML::Exception& e) {
        std::cerr << "配置文件解析错误: " << e.what() << std::endl;
        return false;
    } catch (const std::exception& e) {
        std::cerr << "解析配置时出错: " << e.what() << std::endl;
        return false;
    }
}
}