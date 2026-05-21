#include <eigen3/Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <vector>

struct Laser
{
  cv::Point2f center;
  std::vector<cv::Point2f> points;

  Eigen::Vector3d xyz_in_gimbal;  // 单位：m
  Eigen::Vector3d xyz_in_world;   // 单位：m
};