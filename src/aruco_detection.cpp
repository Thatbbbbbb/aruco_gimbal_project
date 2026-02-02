bool ConfigParser::loadConfig(const std::string& yaml_path, ProjectConfig& config) {
    try {
        // 加载YAML文件
        YAML::Node root = YAML::LoadFile(yaml_path);

        // 解析串口配置（升级）
        YAML::Node serial_node = root["serial_config"];
        config.serial_port = serial_node["port_name"].as<std::string>();
        config.serial_config.baudrate = serial_node["baudrate"].as<uint32_t>();
        // 新增容错参数（默认值兜底）
        config.serial_config.retry_times = serial_node["retry_times"].as<int>(3);
        config.serial_config.timeout_ms = serial_node["timeout_ms"].as<int>(1000);
        config.serial_config.crc_check = serial_node["crc_check"].as<bool>(true);

        // 其他配置解析（不变）
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