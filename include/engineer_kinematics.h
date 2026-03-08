#pragma once
#include <vector>
#include <cmath>
#include <array>
#include <iostream>
#include <eigen3/Eigen/Dense>

// 提前声明串口相关（避免循环依赖）
namespace hitcrt {
namespace serial {
    struct MotorAngles {
        double angle1;
        double angle2;
        double angle3;
    };
}
}

namespace hitcrt {
namespace kinematics {

struct Params {
    double delta = M_PI_2;       // 初始状态下在上平台投影点的夹角
    double yita1 = -M_PI_2;      // 初始W轴角度
    double yita2 = 30.0/180.0*M_PI;
    double yita3 = 150.0/180.0*M_PI;
    double arfa1 = 54.0/180.0*M_PI;  // 连杆参量
    double arfa2 = M_PI_2;
};

class EngineerKinematics {
private:
    double delta_ = M_PI_2;                  // 初始状态下在上平台投影点的夹角
    double yita_[3];                         // 初始状态下上平台三个投影点对应的角度
    double arfa1_;
    double arfa2_;

    // 从位姿T中解出三个向量V1/V2/V3
    inline bool get_3v_fromT(const Eigen::Isometry3d &T, Eigen::Vector3d &vector_V1,
                             Eigen::Vector3d &vector_V2, Eigen::Vector3d &vector_V3);
    
    // 从单个向量V反解电机角度
    inline bool solve_oneangle(const Eigen::Vector3d &vector_V, const size_t& index, double& angle);

public:
    // 构造函数（传入自定义参数）
    EngineerKinematics(const Params& params_temp);
    
    // 析构函数
    ~EngineerKinematics() = default;

    // 正运动学：电机角度→位姿T
    Eigen::Isometry3d forward_kinematics(const std::vector<double>& joints);
    
    // 逆运动学：位姿T→电机角度
    bool inverse_kinematics(const Eigen::Isometry3d &T, std::vector<double>& res);
};

// 补充运动学核心函数的实现（基础版，可根据实际云台模型调整）
inline bool EngineerKinematics::get_3v_fromT(const Eigen::Isometry3d &T, Eigen::Vector3d &v1, Eigen::Vector3d &v2, Eigen::Vector3d &v3) {
    // 示例实现：基于初始角度生成三个向量
    v1 = T * Eigen::Vector3d(cos(yita_[0]), sin(yita_[0]), 0);
    v2 = T * Eigen::Vector3d(cos(yita_[1]), sin(yita_[1]), 0);
    v3 = T * Eigen::Vector3d(cos(yita_[2]), sin(yita_[2]), 0);
    return true;
}

inline bool EngineerKinematics::solve_oneangle(const Eigen::Vector3d &v, const size_t& index, double& angle) {
    // 示例实现：简化的逆解（需根据实际云台机械结构修改）
    angle = atan2(v.y(), v.x()) + yita_[index];
    return true;
}

EngineerKinematics::EngineerKinematics(const Params& params_temp) {
    delta_ = params_temp.delta;
    yita_[0] = params_temp.yita1;
    yita_[1] = params_temp.yita2;
    yita_[2] = params_temp.yita3;
    arfa1_ = params_temp.arfa1;
    arfa2_ = params_temp.arfa2;
}

Eigen::Isometry3d EngineerKinematics::forward_kinematics(const std::vector<double>& joints) {
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    // 示例实现：简化正解（需根据实际云台模型修改）
    T.pretranslate(Eigen::Vector3d(joints[0], joints[1], joints[2]));
    return T;
}

bool EngineerKinematics::inverse_kinematics(const Eigen::Isometry3d &T, std::vector<double>& res) {
    if (res.size() != 3) res.resize(3);
    Eigen::Vector3d v1, v2, v3;
    if (!get_3v_fromT(T, v1, v2, v3)) return false;
    if (!solve_oneangle(v1, 0, res[0])) return false;
    if (!solve_oneangle(v2, 1, res[1])) return false;
    if (!solve_oneangle(v3, 2, res[2])) return false;
    return true;
}

} // namespace kinematics
} // namespace hitcrt

