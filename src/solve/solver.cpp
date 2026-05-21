#include "solver.hpp"

#include <yaml-cpp/yaml.h>

#include <vector>

#include "tools/logger.hpp"
#include "tools/math_tools.hpp"

const std::vector<cv::Point3f> LaserPoints ={
  {0, 50.0 / 2, 72.0 / 2},
  {0, -50.0 / 2, 72.0 / 2},
  {0, -50.0 / 2, -72.0 / 2},
  {0, 50.0 / 2, -72.0 / 2}};

Solver::Solver(const std::string & config_path)
{
  auto yaml = YAML::LoadFile(config_path);

  auto R_camera2gimbal_data = yaml["R_camera2gimbal"].as<std::vector<double>>();
  auto t_camera2gimbal_data = yaml["t_camera2gimbal"].as<std::vector<double>>();
  R_camera2gimbal_mat_ = Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(R_camera2gimbal_data.data());
  t_camera2gimbal_vec_ = Eigen::Map<const Eigen::Vector3d>(t_camera2gimbal_data.data());

  auto camera_matrix_data = yaml["camera_matrix"].as<std::vector<double>>();
  auto distort_coeffs_data = yaml["distort_coeffs"].as<std::vector<double>>();
  Eigen::Matrix<double, 3, 3, Eigen::RowMajor> camera_matrix(camera_matrix_data.data());
  Eigen::Matrix<double, 1, 5> distort_coeffs(distort_coeffs_data.data());
  cv::eigen2cv(camera_matrix, camera_matrix_);
  cv::eigen2cv(distort_coeffs, distort_coeffs_);
  _camera2gimbal_ = camera2Gimbal();
  _gimbal2world_ = Eigen::Matrix4d::Identity();

  if (yaml["origin2laser"]) {
    origin2laser_ = yaml["origin2laser"].as<double>();
  } else {
    origin2laser_ = 42.7e-3;
  }
}

//solvePnP（获得姿态及反解云台）
Eigen::Isometry3d Solver::solve(Laser & laser)
{
  _laser2camera_ = laser2Camera(LaserPoints, laser.points);
  _camera2gimbal_ = camera2Gimbal();
  _gimbal2world_ = gimbal2world();

  Eigen::Vector3d xyz_in_camera;
  xyz_in_camera = _laser2camera_.topRightCorner<3, 1>();
  laser.xyz_in_gimbal = (_camera2gimbal_ * xyz_in_camera.homogeneous()).hnormalized();
  laser.xyz_in_world = (_gimbal2world_ * laser.xyz_in_gimbal.homogeneous()).hnormalized();

  // 获取世界坐标系下的坐标
  double Tx = laser.xyz_in_world.x();
  double Ty = laser.xyz_in_world.y();
  double Tz = laser.xyz_in_world.z();

  // 根据推导解算yaw和pitch角
  // 目标在(0, -1, 0)方向上，先绕Z轴旋转yaw角
  double yaw = std::atan2(Tx, -Ty);

  double d_xy = std::sqrt(Tx * Tx + Ty * Ty);
  double L2 = Tz * Tz + d_xy * d_xy;
  double D2 = origin2laser_ * origin2laser_;
  
  double pitch = 0.0;
  // 绕X轴旋转pitch角，要求射线上通过目标点且Z轴偏置为origin2laser_
  if (L2 >= D2) {
    double sq = std::sqrt(L2 - D2);
    // 利用三角方程展开反解pitch
    double v = (Tz * origin2laser_ + d_xy * sq) / L2; // cos(pitch)
    double u = (d_xy * origin2laser_ - Tz * sq) / L2; // sin(pitch)
    pitch = std::atan2(u, v);
  }

  // 构造齐次矩阵(只管旋转，平移设为0)
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.rotate(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitX()));

  return T;
}

/**
 * @brief 激光模块坐标系转相机坐标系
 * @param[in] object_points 装甲板坐标系坐标
 * @param[in] cameraPoints  相机坐标系坐标
 * @return cv::Mat
 */
Eigen::Matrix4d Solver::laser2Camera(const std::vector<cv::Point3f> &object_points,
                                          const std::vector<cv::Point2f> &cameraPoints) {
    cv::Mat rVec, tVec, rMat;
    cv::solvePnP(object_points, cameraPoints, camera_matrix_, distort_coeffs_, rVec, tVec, false,
                 cv::SOLVEPNP_ITERATIVE);
    cv::Rodrigues(rVec, rMat);
    
    // 将OpenCV矩阵转换为Eigen矩阵
    Eigen::Matrix3d R;
    Eigen::Vector3d t;
    
    cv::cv2eigen(rMat, R);
    cv::cv2eigen(tVec, t);
    
    // 构建齐次变换矩阵
    Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
    transform.topLeftCorner<3, 3>() = R;
    transform.topRightCorner<3, 1>() = t;

    return transform;
}
/**
 * @brief 相机坐标系转云台坐标系
 * @param[in] r             旋转参数
 * @param[in] t             平移参数
 * @return cv::Mat
 */
Eigen::Matrix4d Solver::camera2Gimbal() {
    Eigen::Matrix4d c2g = Eigen::Matrix4d::Identity();
    c2g.topLeftCorner<3, 3>() = R_camera2gimbal_mat_;
    c2g.topRightCorner<3, 1>() = t_camera2gimbal_vec_;
    return c2g;
}

/**
 * @brief 云台坐标系转世界/机器人坐标系,！！！注意传入欧拉角为弧度制
 * @param[in] pitch
 * @param[in] yaw
 * @param[in] roll
 * @return cv::Mat
 */
void Solver::set_gimbal2world(double pitch, double yaw, double roll) {
    // pitch轴
    Eigen::Matrix4d matY;
    matY << 
     cos(pitch), 0,  sin(pitch), 0, 
              0, 1,           0, 0,
    -sin(pitch), 0,  cos(pitch), 0,
              0, 0,           0, 1;
    // yaw轴
    Eigen::Matrix4d matZ;
    matZ << 
     cos(yaw),-sin(yaw), 0, 0,
     sin(yaw), cos(yaw), 0, 0, 
            0,        0, 1, 0, 
            0,        0, 0, 1;
    // roll轴
    Eigen::Matrix4d matX;
    matX << 
    1,         0,         0, 0,
    0 ,cos(roll),-sin(roll), 0,
    0, sin(roll), cos(roll), 0,
    0,         0,         0, 1;
    _gimbal2world_ =  matZ * matY * matX;
}
Eigen::Matrix4d Solver::gimbal2world() { return _gimbal2world_; }
/**
 * @brief 世界坐标转像素坐标
 * @param[in] worldPoints
 * @return std::vector<cv::Point2f>
 */
std::vector<cv::Point2f> Solver::world2pixel(const std::vector<cv::Point3f> & worldPoints)
{
  Eigen::Matrix4d world2camera = camera2Gimbal().inverse() * _gimbal2world_.inverse();
  Eigen::Matrix3d R_world2camera = world2camera.topLeftCorner<3, 3>();
  Eigen::Vector3d t_world2camera = world2camera.topRightCorner<3, 1>();

  cv::Mat R_cv;
  cv::Vec3d rvec;
  cv::Vec3d tvec(t_world2camera[0], t_world2camera[1], t_world2camera[2]);
  cv::eigen2cv(R_world2camera, R_cv);
  cv::Rodrigues(R_cv, rvec);

  std::vector<cv::Point3f> valid_world_points;
  for (const auto & world_point : worldPoints) {
    Eigen::Vector3d world_point_eigen(world_point.x, world_point.y, world_point.z);
    Eigen::Vector3d camera_point = R_world2camera * world_point_eigen + t_world2camera;

    if (camera_point.z() > 0) {
      valid_world_points.push_back(world_point);
    }
  }
  // 如果没有有效点，返回空vector
  if (valid_world_points.empty()) {
    return std::vector<cv::Point2f>();
  }
  std::vector<cv::Point2f> pixelPoints;
  cv::projectPoints(valid_world_points, rvec, tvec, camera_matrix_, distort_coeffs_, pixelPoints);
  return pixelPoints;
}
