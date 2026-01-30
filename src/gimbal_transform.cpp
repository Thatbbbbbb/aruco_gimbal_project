#include "gimbal_transform.h"
#include <cmath>

// 移除硬编码的全局变量，改为从外部传入（通过配置文件）
void camToGimbalTransform(const cv::Vec3d& marker_cam, const cv::Vec3d& gimbal_pos, float& target_yaw, float& target_pitch) {
    // 计算marker相对于云台的坐标：云台坐标 = 相机坐标 - 云台在相机中的坐标
    double x = marker_cam[0] - gimbal_pos[0];
    double y = marker_cam[1] - gimbal_pos[1];
    double z = marker_cam[2] - gimbal_pos[2];

    // 计算偏航角yaw：水平方向角度，atan2(y, x)
    target_yaw = atan2(y, x) * 180 / M_PI;

    // 计算俯仰角pitch：垂直方向角度，atan2(-z, sqrt(x²+y²)) 负号根据坐标系方向调整
    double horizontal_dist = sqrt(x*x + y*y);
    target_pitch = atan2(-z, horizontal_dist) * 180 / M_PI;
}