#pragma once

#include <Eigen/Dense>
#include "config_parser.h"


class GimbalTransformer {
private:
    Eigen::Matrix3d cam_to_gimbal_rot;  
    Eigen::Vector3d cam_to_gimbal_trans;
public:

    GimbalTransformer(const Parser::parser::ConfigData& config);

    Eigen::Isometry3d cam_to_gimbal_pose(const Eigen::Isometry3d& cam_pose);
};