#ifndef ARUCO_DETECTION_H
#define ARUCO_DETECTION_H

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>

class ArucoDetector {
public:
    // 构造函数：新增marker_size参数，从配置文件传入
    ArucoDetector(const cv::Mat& camera_matrix, const cv::Mat& dist_coeffs, 
                 int aruco_dict_id = cv::aruco::DICT_6X6_250, float marker_size = 0.1f);

    // 检测函数：输入图像，输出aruco中心的相机坐标(x,y,z)
    bool detectMarker(const cv::Mat& frame, cv::Vec3d& marker_cam_coord);

private:
    cv::Ptr<cv::aruco::Dictionary> aruco_dict_;
    cv::Ptr<cv::aruco::DetectorParameters> detector_params_;
    cv::Mat camera_matrix_;
    cv::Mat dist_coeffs_;
    float marker_size_; // 从配置文件加载，不再硬编码
};

#endif