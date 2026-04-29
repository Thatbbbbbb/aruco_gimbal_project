#include "serial_port.h"
#include "engineer_kinematics.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <unistd.h>
#include <chrono>  // 用于计时

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
    hitcrt::serial::SerialPort serial("/dev/ttyUSB0", 100);
    if (!serial.open_port()) {
        std::cerr << "Failed to init serial port!" << std::endl;
        return -1;
    }

    hitcrt::kinematics::params my_params;
    hitcrt::kinematics::engineer_kinematics kin(my_params);

    int frame_count = 0;
    std::vector<float> current_joint_angles(3);
    bool is_timeout = false;


    // ======================
    // 自动 pitch 递增变量
    // ======================
    float pitch_deg = 0.0f;        // 从 0 度开始
    double last_time = getTimeSeconds();
    const double increment_interval = 0.1;  // 每 1 秒加 1 度
    const float max_pitch = 30.0f;   // 最大角度（防止云台撞墙）

    float roll_deg = 0.0f;
    float yaw_deg = 0.0f;

    Eigen::Isometry3d T_desired;

    while (true) {
        double current_time = getTimeSeconds();

        // ======================
        // 核心：每秒 +1° pitch
        // ======================
        if (current_time - last_time >= increment_interval) {
            pitch_deg += 1.0f;
            if (pitch_deg > max_pitch) {  // 超过最大值就归零（循环）
                pitch_deg = 0.0f;
            }
            last_time = current_time;

            std::cout << "✅ 当前 pitch: " << pitch_deg << " °" << std::endl;
        }

        // 重新计算目标位姿
        T_desired = eulerToIsometry(
            roll_deg * M_PI / 180.0f,
            pitch_deg * M_PI / 180.0f,
            yaw_deg * M_PI / 180.0f
        );

        // 接收云台角度
        if (!serial.receive_motor_angles(current_joint_angles)) {
            std::cerr << "Receive gimbal data failed!" << std::endl;
            continue;
        }

        // 逆解
        std::vector<std::array<float, 3>> euler_list;
        bool success = kin.inverse_kinematics(T_desired, euler_list);

        if (success && !euler_list.empty()) {
            std::vector<float> euler_vector = convertEulerListToVector(euler_list);
            std::vector<float> target_angles(euler_vector.begin(), euler_vector.begin() + 3);

            if (!serial.send_motor_angles(target_angles)) {
                std::cerr << "Send gimbal data failed!" << std::endl;
            }
        }

        usleep(10000); // 10ms loop
    }

    return 0;
}