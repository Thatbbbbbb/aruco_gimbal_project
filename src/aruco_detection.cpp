#include "aruco_detection.h"

// 构造函数新增marker_size参数
ArucoDetector::ArucoDetector(const cv::Mat& camera_matrix, const cv::Mat& dist_coeffs, 
                             int aruco_dict_id, float marker_size) {
    camera_matrix_ = camera_matrix;
    dist_coeffs_ = dist_coeffs;
    aruco_dict_ = cv::aruco::getPredefinedDictionary(aruco_dict_id);
    detector_params_ = cv::aruco::DetectorParameters::create();
    marker_size_ = marker_size; // 从配置文件传入，不再硬编码
}

bool ArucoDetector::detectMarker(const cv::Mat& frame, cv::Vec3d& marker_cam_coord) {
    std::vector<int> marker_ids;
    std::vector<std::vector<cv::Point2f>> marker_corners, rejected_candidates;

    // 检测aruco码
    cv::aruco::detectMarkers(frame, aruco_dict_, marker_corners, marker_ids, detector_params_, rejected_candidates);

    if (marker_ids.empty()) {
        return false;
    }

    // 估计位姿
    std::vector<cv::Vec3d> rvecs, tvecs;
    cv::aruco::estimatePoseSingleMarkers(marker_corners, marker_size_, camera_matrix_, dist_coeffs_, rvecs, tvecs);

    // tvecs即为marker在相机坐标系下的平移向量（x,y,z）
    marker_cam_coord = tvecs[0];
    return true;
}