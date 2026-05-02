#include "hrt_yolo.hpp"
#include "deploy/model.hpp"
#include "deploy/option.hpp"
#include "deploy/result.hpp"
#include "tools/logger.hpp"
#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <algorithm>  // 添加 for std::sort
#include <cmath>
#include <fmt/format.h>  // 如果使用 fmt::format
namespace drone_detection {

// 构造函数实现（只保留一份）
DroneDetector::DroneDetector(const std::string& config_path, bool debug)
    : debug_(debug)
{
    auto yaml = YAML::LoadFile(config_path);
    
    // 加载模型路径
    if (yaml["drone_model_path"]) {
        model_path_ = yaml["drone_model_path"].as<std::string>();
    } else {
        tools::logger()->error("Config 'drone_model_path' not found in yaml!");
        throw std::runtime_error("Model path not configured");
    }
    
    // 加载参数
    device_ = yaml["device"].as<std::string>("cuda");
    confidence_threshold_ = yaml["confidence_threshold"].as<double>(0.5);
    nms_threshold_ = yaml["nms_threshold"].as<double>(0.45);
    min_confidence_ = yaml["min_confidence"].as<double>(0.3);
    
    // 加载输入尺寸
    int input_width = yaml["input_width"].as<int>(640);
    int input_height = yaml["input_height"].as<int>(640);
    
    // 加载ROI配置
    if (yaml["use_roi"] && yaml["use_roi"].as<bool>()) {
        use_roi_ = true;
        int x = yaml["roi"]["x"].as<int>(0);
        int y = yaml["roi"]["y"].as<int>(0);
        int width = yaml["roi"]["width"].as<int>(-1);
        int height = yaml["roi"]["height"].as<int>(-1);
        roi_ = cv::Rect(x, y, width, height);
    } else {
        use_roi_ = false;
    }
    
    // 创建保存目录
    save_path_ = "drone_imgs";
    std::filesystem::create_directory(save_path_);
    
    // 配置TensorRT
    deploy::InferOption option;
    option.device_id = 0;
    option.enableSwapRB();  // OpenCV BGR -> RGB
    option.setInputDimensions(input_width, input_height);
    
    try {
        tools::logger()->info("Loading TensorRT model from: {}", model_path_);
        model_ = std::make_unique<deploy::BaseModel<deploy::PoseRes>>(model_path_, option);
        tools::logger()->info("DroneDetector initialized successfully. Input size: {}x{}", input_width, input_height);
    } catch (const std::exception& e) {
        tools::logger()->error("Failed to initialize model: {}", e.what());
        throw;
    }
}

void DroneDetector::preprocess(const cv::Mat& raw_img, cv::Mat& processed_img) {
    offset_ = cv::Point2f(0.0f, 0.0f);

    if (raw_img.empty()) {
        processed_img.release();
        return;
    }

    if (use_roi_) {
        cv::Rect image_bounds(0, 0, raw_img.cols, raw_img.rows);
        cv::Rect valid_roi = roi_ & image_bounds;
        if (valid_roi.area() > 0) {
            processed_img = raw_img(valid_roi).clone();
            offset_ = cv::Point2f(static_cast<float>(valid_roi.x), static_cast<float>(valid_roi.y));
            return;
        }
    }

    processed_img = raw_img.clone();
}

int DroneDetector::remap_class_id(int model_id) {
    return model_id;
}

void DroneDetector::sort_keypoints(std::vector<cv::Point2f>& keypoints) {
    if (keypoints.size() < 2) {
        return;
    }

    cv::Point2f center(0.0f, 0.0f);
    for (const auto& point : keypoints) {
        center.x += point.x;
        center.y += point.y;
    }
    center.x /= static_cast<float>(keypoints.size());
    center.y /= static_cast<float>(keypoints.size());

    std::sort(keypoints.begin(), keypoints.end(), [&center](const cv::Point2f& lhs, const cv::Point2f& rhs) {
        float lhs_angle = std::atan2(lhs.y - center.y, lhs.x - center.x);
        float rhs_angle = std::atan2(rhs.y - center.y, rhs.x - center.x);
        return lhs_angle < rhs_angle;
    });

    auto top_left_it = std::min_element(keypoints.begin(), keypoints.end(), [](const cv::Point2f& lhs, const cv::Point2f& rhs) {
        float lhs_score = lhs.x + lhs.y;
        float rhs_score = rhs.x + rhs.y;
        if (lhs_score == rhs_score) {
            return lhs.y < rhs.y;
        }
        return lhs_score < rhs_score;
    });

    if (top_left_it != keypoints.end()) {
        std::rotate(keypoints.begin(), top_left_it, keypoints.end());
    }
}

void DroneDetector::postprocess(const deploy::PoseRes& result, std::list<Drone>& drones) {
    drones.clear();

    int count = std::min({result.num,
                          static_cast<int>(result.boxes.size()),
                          static_cast<int>(result.classes.size()),
                          static_cast<int>(result.scores.size()),
                          static_cast<int>(result.kpts.size())});

    if (count <= 0 || source_size_.width <= 0 || source_size_.height <= 0) {
        return;
    }

    for (int i = 0; i < count; ++i) {
        float confidence = result.scores[i];
        if (confidence < confidence_threshold_ || confidence < min_confidence_) {
            continue;
        }

        const auto& box = result.boxes[i];
        std::vector<cv::Point2f> keypoints;
        keypoints.reserve(result.kpts[i].size());

        for (const auto& kp : result.kpts[i]) {
            cv::Point2f point(kp.x + offset_.x, kp.y + offset_.y);
            keypoints.push_back(point);
        }

        if (keypoints.size() >= 2) {
            sort_keypoints(keypoints);
        }

        if (keypoints.empty()) {
            float left = box.left + offset_.x;
            float top = box.top + offset_.y;
            float right = box.right + offset_.x;
            float bottom = box.bottom + offset_.y;
            keypoints.emplace_back(left, top);
            keypoints.emplace_back(right, top);
            keypoints.emplace_back(right, bottom);
            keypoints.emplace_back(left, bottom);
        }

        float x1 = std::min(box.left, box.right) + offset_.x;
        float y1 = std::min(box.top, box.bottom) + offset_.y;
        float x2 = std::max(box.left, box.right) + offset_.x;
        float y2 = std::max(box.top, box.bottom) + offset_.y;

        int left = std::clamp(static_cast<int>(std::round(x1)), 0, std::max(0, source_size_.width - 1));
        int top = std::clamp(static_cast<int>(std::round(y1)), 0, std::max(0, source_size_.height - 1));
        int right = std::clamp(static_cast<int>(std::round(x2)), 0, std::max(0, source_size_.width - 1));
        int bottom = std::clamp(static_cast<int>(std::round(y2)), 0, std::max(0, source_size_.height - 1));

        if (right <= left || bottom <= top) {
            continue;
        }

        cv::Rect rect(left, top, right - left, bottom - top);
        Drone drone(remap_class_id(result.classes[i]), confidence, rect, keypoints);
        drone.center = cv::Point2f((rect.x + rect.width * 0.5f), (rect.y + rect.height * 0.5f));
        drone.center_norm = cv::Point2f(drone.center.x / static_cast<float>(source_size_.width),
                                        drone.center.y / static_cast<float>(source_size_.height));
        drones.emplace_back(std::move(drone));
    }
}

void DroneDetector::nms_filter(std::list<Drone>& drones) {
    if (drones.size() < 2) {
        return;
    }

    drones.sort([](const Drone& lhs, const Drone& rhs) {
        return lhs.confidence > rhs.confidence;
    });

    std::list<Drone> filtered;
    for (const auto& candidate : drones) {
        bool keep = true;
        for (const auto& kept : filtered) {
            cv::Rect2f intersection = candidate.box & kept.box;
            float intersection_area = intersection.area();
            float union_area = static_cast<float>(candidate.box.area() + kept.box.area() - intersection_area);
            float iou = union_area > 0.0f ? intersection_area / union_area : 0.0f;
            if (iou > static_cast<float>(nms_threshold_)) {
                keep = false;
                break;
            }
        }
        if (keep) {
            filtered.emplace_back(candidate);
        }
    }

    drones.swap(filtered);
}

void DroneDetector::draw_detections(const cv::Mat& img, const std::list<Drone>& drones, int frame_count) const {
    if (img.empty()) {
        return;
    }

    cv::Mat vis = img.clone();
    for (const auto& drone : drones) {
        cv::Scalar color(0, 255, 0);
        cv::rectangle(vis, drone.box, color, 2);

        if (drone.points.size() >= 2) {
            std::vector<cv::Point> polygon;
            polygon.reserve(drone.points.size());
            for (const auto& point : drone.points) {
                polygon.emplace_back(cv::Point(cvRound(point.x), cvRound(point.y)));
            }
            const cv::Point* pts = polygon.data();
            int npts = static_cast<int>(polygon.size());
            cv::polylines(vis, &pts, &npts, 1, true, color, 2);
        }

        cv::circle(vis, drone.center, 3, cv::Scalar(0, 0, 255), -1);

        std::string label = fmt::format("cls:{} conf:{:.2f}", drone.class_id, drone.confidence);
        cv::putText(vis, label, cv::Point(drone.box.x, std::max(0, drone.box.y - 6)),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv::LINE_AA);
    }

    if (!save_path_.empty()) {
        std::filesystem::create_directories(save_path_);
        std::string file_path = save_path_ + "/frame_" + std::to_string(frame_count) + ".jpg";
        cv::imwrite(file_path, vis);
    }

    cv::imshow("Drone Detection Debug", vis);
}

std::list<Drone> DroneDetector::detect(const cv::Mat& raw_img, int frame_count) {
    std::list<Drone> drones;
    if (raw_img.empty() || !model_) {
        return drones;
    }

    source_size_ = raw_img.size();

    cv::Mat processed_img;
    preprocess(raw_img, processed_img);
    if (processed_img.empty()) {
        return drones;
    }

    deploy::Image input_image(processed_img.data, processed_img.cols, processed_img.rows);
    deploy::PoseRes result = model_->predict(input_image);

    postprocess(result, drones);
    nms_filter(drones);

    if (debug_) {
        draw_detections(raw_img, drones, frame_count);
    }

    return drones;
}

} // namespace drone_detection