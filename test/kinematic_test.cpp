#include <iostream>
#include <cmath>
#include <array>
#include <vector>
#include <string>
#include <chrono>
#include <thread>
#include "serial_port.h"
#include "engineer_kinematics.hpp"
#include <eigen3/Eigen/Dense>

using namespace std;
using namespace hitcrt::serial;
using namespace hitcrt::kinematics;

// 角度转弧度
inline float deg2rad(float deg) {
    return deg * M_PI / 180.0f;
}

// 弧度转角度
inline float rad2deg(float rad) {
    return rad * 180.0f / M_PI;
}

// 通过欧拉角（roll, pitch, yaw，单位：弧度）构造 Eigen::Isometry3d
// 旋转顺序：ZYX (yaw -> pitch -> roll)
Eigen::Isometry3d eulerToIsometry(float roll_rad, float pitch_rad, float yaw_rad) {
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    Eigen::Matrix3d R;
    R = Eigen::AngleAxisd(yaw_rad, Eigen::Vector3d::UnitZ())
      * Eigen::AngleAxisd(pitch_rad, Eigen::Vector3d::UnitY())
      * Eigen::AngleAxisd(roll_rad, Eigen::Vector3d::UnitX());
    T.rotate(R);
    T.translation() = Eigen::Vector3d(0, 0, 0);
    return T;
}

int main(int argc, char** argv) {
    // 解析命令行参数，支持两种模式
    bool direct_mode = false;
    bool loop_mode = false;
    vector<float> direct_angles;   // 直接发送的角度（度）
    float pitch_deg = 0, yaw_deg = 0, roll_deg = 0;

    // 检查是否为直接发送模式
    if (argc >= 2 && string(argv[1]) == "--direct") {
        direct_mode = true;
        if (argc < 5) {
            cerr << "直接发送模式需要三个角度参数: --direct <angle1> <angle2> <angle3>" << endl;
            return -1;
        }
        for (int i = 0; i < 3; ++i) {
            direct_angles.push_back(stof(argv[2 + i]));
        }
        if (argc >= 6 && string(argv[5]) == "--loop") {
            loop_mode = true;
        }
        cout << "直接发送模式，目标角度 (deg): "
             << direct_angles[0] << ", " << direct_angles[1] << ", " << direct_angles[2] << endl;
    } else {
        // 普通模式：需要三个欧拉角参数
        if (argc != 4) {
            cerr << "Usage (逆解模式): " << argv[0] << " <pitch_deg> <yaw_deg> <roll_deg>" << endl;
            cerr << "Usage (直接发送): " << argv[0] << " --direct <angle1> <angle2> <angle3> [--loop]" << endl;
            return -1;
        }
        pitch_deg = stof(argv[1]);
        yaw_deg   = stof(argv[2]);
        roll_deg  = stof(argv[3]);
        cout << "逆解模式，目标欧拉角 (deg): pitch=" << pitch_deg 
             << ", yaw=" << yaw_deg << ", roll=" << roll_deg << endl;
    }

    // 1. 初始化串口
    SerialPort serial("/dev/ttyUSB0", 100);
    if (!serial.open_port()) {
        cerr << "Failed to open serial port!" << endl;
        return -1;
    }
    cout << "串口打开成功" << endl;

    // 2. 如果是直接发送模式且不循环，发送后直接退出
    if (direct_mode && !loop_mode) {
        if (serial.send_motor_angles(direct_angles)) {
            cout << "关节角度已发送" << endl;
        } else {
            cerr << "发送失败" << endl;
        }
        serial.close_port();
        return 0;
    }

    // 3. 如果是直接发送模式且循环，进入循环发送（按 q 退出）
    if (direct_mode && loop_mode) {
        cout << "循环发送模式，按 'q' 退出..." << endl;
        while (true) {
            this_thread::sleep_for(chrono::milliseconds(100)); // 每100ms发送一次
            serial.send_motor_angles(direct_angles);
            cout << "发送角度: " << direct_angles[0] << ", " << direct_angles[1] << ", " << direct_angles[2] << endl;
            // 使用 cv::waitKey 需要 opencv，这里用简单的 getchar 模拟，或者直接睡眠
            // 为了不增加依赖，使用 std::this_thread::sleep_for 和简单输入检测
            char key;
            cout << "输入 'q' 退出，其他键继续: ";
            cin >> key;
            if (key == 'q' || key == 'Q') break;
        }
        serial.close_port();
        return 0;
    }

    // 4. 以下为普通逆解模式
    // 初始化运动学（使用默认参数）
    params my_params;
    engineer_kinematics kin(my_params);

    // 构造目标位姿（只使用旋转部分）
    float pitch_rad = deg2rad(pitch_deg);
    float yaw_rad   = deg2rad(yaw_deg);
    float roll_rad  = deg2rad(roll_deg);
    Eigen::Isometry3d T_base_target = eulerToIsometry(roll_rad, pitch_rad, yaw_rad);

    // 逆运动学求解
    vector<array<float, 3>> solutions;
    if (!kin.inverse_kinematics(T_base_target, solutions)) {
        cerr << "逆运动学无解！请检查目标姿态是否在可达范围内。" << endl;
        serial.close_port();
        return -1;
    }

    // 选择第一个有效解
    auto joints = solutions[0];
    vector<float> angles_deg = {
        rad2deg(joints[0]),
        rad2deg(joints[1]),
        rad2deg(joints[2])
    };


    cout << "解算出的关节角 (deg, 0~360): "
         << angles_deg[0] << ", "
         << angles_deg[1] << ", "
         << angles_deg[2] << endl;

    // 发送关节角度
    if (serial.send_motor_angles(angles_deg)) {
        cout << "关节角度已发送" << endl;
    } else {
        cerr << "发送失败" << endl;
    }

    // 等待发送完成
    this_thread::sleep_for(chrono::milliseconds(100));

    serial.close_port();
    return 0;
}