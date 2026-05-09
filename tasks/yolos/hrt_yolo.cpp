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
/**
 * @brief 将图像缩放并填充到目标尺寸 (保持宽高比，图像放在左上角，其余填充黑色)
 * @param src       输入图像
 * @param dst_width  目标宽度
 * @param dst_height 目标高度
 * @param scale      输出缩放比例 (原图到缩放后图像的比例)
 * @param offset     输出偏移量 (原图左上角在目标图像中的位置，通常为 (0,0))
 * @return           处理后的图像 (尺寸 dst_width x dst_height)
 */
cv::Mat resizeAndPad(const cv::Mat& src, int dst_width, int dst_height, float& scale, cv::Point2f& offset) {
    cv::Mat result = cv::Mat::zeros(dst_height, dst_width, src.type());
    if (src.empty()) {
        return result;
    }

    int src_w = src.cols;
    int src_h = src.rows;

    // 计算缩放比例 (等比例，取较小值以保证完整显示)
    float scale_x = static_cast<float>(dst_width) / static_cast<float>(src_w);
    float scale_y = static_cast<float>(dst_height) / static_cast<float>(src_h);
    scale = std::min(scale_x, scale_y);

    // 计算缩放后的实际尺寸
    int new_w = static_cast<int>(std::round(src_w * scale));
    int new_h = static_cast<int>(std::round(src_h * scale));
    new_w = std::max(1, new_w);
    new_h = std::max(1, new_h);

    // 缩放图像
    cv::Mat resized;
    cv::resize(src, resized, cv::Size(new_w, new_h));

    // 放置到目标图像的左上角 (0,0)
    offset = cv::Point2f(0.0f, 0.0f);
    resized.copyTo(result(cv::Rect(0, 0, new_w, new_h)));

    return result;
}
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
    input_width_ = input_width;
    input_height_ = input_height;
    
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
    option.setInputDimensions(input_width_, input_height_);
    
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
    last_scale_ = 1.0f;
    float pad_left_ = 0.0f;   // 新增：记录水平填充（单位：像素）
    float pad_top_ = 0.0f;    // 新增：记录垂直填充

    if (raw_img.empty()) {
        processed_img.release();
        return;
    }

    cv::Mat roi_img;
    if (use_roi_) {
        cv::Rect image_bounds(0, 0, raw_img.cols, raw_img.rows);
        cv::Rect valid_roi = roi_ & image_bounds;
        if (valid_roi.area() > 0) {
            roi_img = raw_img(valid_roi).clone();
            offset_ = cv::Point2f(static_cast<float>(valid_roi.x), static_cast<float>(valid_roi.y));
        } else {
            roi_img = raw_img.clone();
            offset_ = cv::Point2f(0.0f, 0.0f);
        }
    } else {
        roi_img = raw_img.clone();
        offset_ = cv::Point2f(0.0f, 0.0f);
    }

    int src_w = roi_img.cols;
    int src_h = roi_img.rows;
    if (src_w <= 0 || src_h <= 0) {
        processed_img.release();
        return;
    }

    // 计算缩放因子（等比例，使图像完全适应画布）
    float scale_x = static_cast<float>(input_width_) / static_cast<float>(src_w);
    float scale_y = static_cast<float>(input_height_) / static_cast<float>(src_h);
    float scale = std::min(scale_x, scale_y);
    last_scale_ = scale;

    int new_w = std::max(1, static_cast<int>(std::round(src_w * scale)));
    int new_h = std::max(1, static_cast<int>(std::round(src_h * scale)));

    // 计算填充偏移（使图像居中）
    pad_left_ = (input_width_ - new_w) / 2.0f;
    pad_top_  = (input_height_ - new_h) / 2.0f;

    // 创建画布并填充黑色（或灰色，通常用114）
    processed_img = cv::Mat::zeros(input_height_, input_width_, roi_img.type());
    // 如果想要灰色填充，可以使用 cv::Scalar(114,114,114) 代替 zeros

    cv::Mat resized;
    cv::resize(roi_img, resized, cv::Size(new_w, new_h));
    // 放到居中位置
    resized.copyTo(processed_img(cv::Rect(static_cast<int>(pad_left_),
                                          static_cast<int>(pad_top_),
                                          new_w, new_h)));
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
            // result 的坐标基于模型输入（已缩放放在左上角），映射回原图需除以缩放并加上 roi 偏移
            cv::Point2f point(kp.x / last_scale_ + offset_.x, kp.y / last_scale_ + offset_.y);
            keypoints.push_back(point);
        }

        if (keypoints.size() >= 2) {
            sort_keypoints(keypoints);
        }

        if (keypoints.empty()) {
            float left = box.left / last_scale_ + offset_.x;
            float top = box.top / last_scale_ + offset_.y;
            float right = box.right / last_scale_ + offset_.x;
            float bottom = box.bottom / last_scale_ + offset_.y;
            keypoints.emplace_back(left, top);
            keypoints.emplace_back(right, top);
            keypoints.emplace_back(right, bottom);
            keypoints.emplace_back(left, bottom);
        }

        float x1 = std::min(box.left, box.right) / last_scale_ + offset_.x;
        float y1 = std::min(box.top, box.bottom) / last_scale_ + offset_.y;
        float x2 = std::max(box.left, box.right) / last_scale_ + offset_.x;
        float y2 = std::max(box.top, box.bottom) / last_scale_ + offset_.y;

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

std::list<Drone> DroneDetector::parseOutputAndNMS(const float* output, int num_boxes) {
    std::list<Drone> drones;
    if (!output || num_boxes <= 0) return drones;

    // 输出形状：NCHW，C=5 (x,y,w,h,obj)
    const float* x_ptr   = output + 0 * num_boxes;
    const float* y_ptr   = output + 1 * num_boxes;
    const float* w_ptr   = output + 2 * num_boxes;
    const float* h_ptr   = output + 3 * num_boxes;
    const float* obj_ptr = output + 4 * num_boxes;

    // 置信度阈值（同时考虑 min_confidence_ 兜底）
    float thresh = std::min(confidence_threshold_, min_confidence_);

    // 第一遍：收集所有通过阈值的候选框
    struct Candidate {
        float left, top, right, bottom;
        float confidence;
    };
    std::vector<Candidate> candidates;
    candidates.reserve(num_boxes);

    for (int i = 0; i < num_boxes; ++i) {
        float obj = obj_ptr[i];
        if (obj < thresh) continue;   // 置信度过低，跳过

        float x = x_ptr[i];
        float y = y_ptr[i];
        float w = w_ptr[i];
        float h = h_ptr[i];

        // 模型输出是 640×640 输入空间下的绝对坐标，需要映射回原图
        // 映射公式：原图坐标 = 模型坐标 / scale + ROI偏移
        float left   = (x - 0.5f * w) / last_scale_ + offset_.x;
        float top    = (y - 0.5f * h) / last_scale_ + offset_.y;
        float right  = (x + 0.5f * w) / last_scale_ + offset_.x;
        float bottom = (y + 0.5f * h) / last_scale_ + offset_.y;

        // 钳位到图像尺寸内
        left   = std::clamp(left,   0.0f, static_cast<float>(source_size_.width  - 1));
        top    = std::clamp(top,    0.0f, static_cast<float>(source_size_.height - 1));
        right  = std::clamp(right,  0.0f, static_cast<float>(source_size_.width  - 1));
        bottom = std::clamp(bottom, 0.0f, static_cast<float>(source_size_.height - 1));

        if (right <= left || bottom <= top) continue;

        candidates.push_back({left, top, right, bottom, obj});
    }

    // NMS（按置信度降序）
    std::sort(candidates.begin(), candidates.end(),
              [](const Candidate& a, const Candidate& b) {
                  return a.confidence > b.confidence;
              });

    std::vector<Candidate> kept;
    for (const auto& c : candidates) {
        bool keep = true;
        for (const auto& k : kept) {
            float inter_left   = std::max(c.left,   k.left);
            float inter_top    = std::max(c.top,    k.top);
            float inter_right  = std::min(c.right,  k.right);
            float inter_bottom = std::min(c.bottom, k.bottom);
            if (inter_left < inter_right && inter_top < inter_bottom) {
                float inter_area = (inter_right - inter_left) * (inter_bottom - inter_top);
                float area_c = (c.right - c.left) * (c.bottom - c.top);
                float area_k = (k.right - k.left) * (k.bottom - k.top);
                float iou = inter_area / (area_c + area_k - inter_area);
                if (iou > nms_threshold_) {
                    keep = false;
                    break;
                }
            }
        }
        if (keep) kept.push_back(c);
    }

    // 转换为 Drone 对象（没有真实关键点，用包围框四角替代）
    for (const auto& c : kept) {
        cv::Rect rect(cv::Point(cvRound(c.left), cvRound(c.top)),
                      cv::Point(cvRound(c.right), cvRound(c.bottom)));
        std::vector<cv::Point2f> pts = {
            cv::Point2f(c.left, c.top),
            cv::Point2f(c.right, c.top),
            cv::Point2f(c.right, c.bottom),
            cv::Point2f(c.left, c.bottom)
        };
        Drone drone(0, c.confidence, rect, pts);   // class_id暂用0
        drone.center = cv::Point2f(rect.x + rect.width * 0.5f,
                                   rect.y + rect.height * 0.5f);
        drone.center_norm = cv::Point2f(drone.center.x / source_size_.width,
                                        drone.center.y / source_size_.height);
        drones.push_back(drone);
    }
    return drones;
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

    // ----- 手动推理与解析融合输出 -----
    model_->getBackend()->infer(std::vector<deploy::Image>{input_image});

    auto& out_tensor = model_->getBackend()->tensor_infos[1];
    float* output_data = static_cast<float*>(out_tensor.buffer->host());
    int num_boxes = out_tensor.shape.d[2];  // 8400

    drones = parseOutputAndNMS(output_data, num_boxes);
    // ---------------------------------

    if (debug_) {
        draw_detections(raw_img, drones, frame_count);
    }

    return drones;
}
} // namespace drone_detection