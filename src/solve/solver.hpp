#ifndef SOLVER_HPP
#define SOLVER_HPP

#include <eigen3/Eigen/Dense>  // 必须在opencv2/core/eigen.hpp上面
#include <eigen3/Eigen/Geometry>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

#include "laser.hpp"

class Solver
{
public:
  explicit Solver(const std::string & config_path);

  void set_gimbal2world(double pitch, double yaw, double roll);

  Eigen::Isometry3d solve(Laser & laser);

  std::vector<cv::Point2f> world2pixel(const std::vector<cv::Point3f> & worldPoints);

private:
  double origin2laser_;
  cv::Mat camera_matrix_;
  cv::Mat distort_coeffs_;
  Eigen::Matrix3d R_camera2gimbal_mat_;
  Eigen::Vector3d t_camera2gimbal_vec_;

  Eigen::Matrix4d _laser2camera_;
  Eigen::Matrix4d _camera2gimbal_;
  Eigen::Matrix4d _gimbal2world_;

  // 坐标转换函数
  Eigen::Matrix4d laser2Camera(const std::vector<cv::Point3f> &object_points, const std::vector<cv::Point2f> &cameraPoints);
  Eigen::Matrix4d camera2Gimbal(); 
  Eigen::Matrix4d gimbal2world();


};

#endif  // SOLVER_HPP