/*
#pragma once
#include "gimbal_transform.h"
#include <iostream>
#include <stdexcept>

GimbalTransformer::GimbalTransformer(const Parser::parser::ConfigData& config) {

    if (config.cam_to_gimbal_rot.size() != 9) {
        throw std::runtime_error("旋转矩阵参数数量错误（需9个），当前数量：" + std::to_string(config.cam_to_gimbal_rot.size()));
    }
    cam_to_gimbal_rot << config.cam_to_gimbal_rot[0], config.cam_to_gimbal_rot[1], config.cam_to_gimbal_rot[2],
                         config.cam_to_gimbal_rot[3], config.cam_to_gimbal_rot[4], config.cam_to_gimbal_rot[5],
                         config.cam_to_gimbal_rot[6], config.cam_to_gimbal_rot[7], config.cam_to_gimbal_rot[8];

    if (config.cam_to_gimbal_trans.size() != 3) {
        throw std::runtime_error("平移向量参数数量错误（需3个），当前数量：" + std::to_string(config.cam_to_gimbal_trans.size()));
    }
    cam_to_gimbal_trans << config.cam_to_gimbal_trans[0],
                           config.cam_to_gimbal_trans[1],
                           config.cam_to_gimbal_trans[2];

    std::cout << "云台坐标转换器初始化完成！" << std::endl;
}

Eigen::Isometry3d GimbalTransformer::cam_to_gimbal_pose(const Eigen::Isometry3d& cam_pose) {
    // 构造相机→云台的变换矩阵
    Eigen::Isometry3d cam2gimbal = Eigen::Isometry3d::Identity();
    cam2gimbal.rotate(cam_to_gimbal_rot);
    cam2gimbal.translation() = cam_to_gimbal_trans;

    // 云台位姿 = 相机→云台变换 * 相机位姿
    Eigen::Isometry3d gimbal_pose = cam2gimbal * cam_pose;

    std::cout << "相机位姿转换为云台位姿完成" << std::endl;
    return gimbal_pose;
}
    */
