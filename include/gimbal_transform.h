#pragma once
#ifndef GIMBAL_TRANSFORM_H
#define GIMBAL_TRANSFORM_H

#include <opencv2/core.hpp>

// 相机坐标系下云台的位置，可在此修改
const extern cv::Vec3d GIMBAL_POS_IN_CAM; // (x,y,z) 单位m

// 将aruco的相机坐标转换为云台坐标，并计算所需的瞄准角度
void camToGimbalTransform(const cv::Vec3d& marker_cam, const cv::Vec3d& gimbal_pos, float& target_yaw, float& target_pitch);

#endif