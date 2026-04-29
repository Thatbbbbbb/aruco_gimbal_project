#include "drone.hpp"
#include "hrt_yolo.hpp"
#include "deploy/model.hpp"
#include "deploy/option.hpp"
#include "deploy/result.hpp"
#include "tools/logger.hpp"

namespace drone_detection {

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
    
    try {
        tools::logger()->info("Loading TensorRT Drone model from: {}", model_path_);
        model_ = std::make_unique<deploy::BaseModel<deploy::PoseRes>>(model_path_, option);
        tools::logger()->info("DroneDetector initialized successfully.");
    } catch (const std::exception& e) {
        tools::logger()->error("Failed to initialize model: {}", e.what());
        throw;
    }
}

std::list<Drone> DroneDetector::detect(const cv::Mat& raw_img, int frame_count)
{
    if (!model_ || raw_img.empty()) {
        return std::list<Drone>();
    }
    
    tmp_img_ = raw_img;
    cv::Mat processed_img;
    preprocess(raw_img, processed_img);
    
    // 推理
    deploy::Image img(processed_img.data, processed_img.cols, processed_img.rows);
    auto result = model_->predict(img);
    
    std::list<Drone> drones;
    postprocess(result, drones);
    
    // NMS过滤
    nms_filter(drones);
    
    // 计算归一化坐标
    for (auto& drone : drones) {
        drone.center_norm = cv::Point2f(
            drone.center.x / raw_img.cols,
            drone.center.y / raw_img.rows
        );
    }
    
    if (debug_) {
        draw_detections(raw_img, drones, frame_count);
    }
    
    return drones;
}

void DroneDetector::preprocess(const cv::Mat& raw_img, cv::Mat& processed_img)
{
    if (use_roi_) {
        // 修正ROI尺寸
        cv::Rect safe_roi = roi_;
        if (roi_.width == -1) safe_roi.width = raw_img.cols;
        if (roi_.height == -1) safe_roi.height = raw_img.rows;
        
        // 确保ROI在图像范围内
        safe_roi = safe_roi & cv::Rect(0, 0, raw_img.cols, raw_img.rows);
        
        // 裁剪并克隆（确保内存连续）
        processed_img = raw_img(safe_roi).clone();
        offset_ = cv::Point2f(safe_roi.x, safe_roi.y);
    } else {
        processed_img = raw_img;
        offset_ = cv::Point2f(0, 0);
    }
}

void DroneDetector::postprocess(const deploy::PoseRes& result, std::list<Drone>& drones)
{
    for (int i = 0; i < result.num; ++i) {
        float score = result.scores[i];
        if (score < confidence_threshold_) continue;
        
        // 映射类别ID
        int class_id = remap_class_id(result.classes[i]);
        if (class_id < 0) continue;
        
        // 获取边界框（已在processed_img坐标系下）
        const auto& box = result.boxes[i];
        cv::Rect bbox(
            static_cast<int>(box.left),
            static_cast<int>(box.top),
            static_cast<int>(box.right - box.left),
            static_cast<int>(box.bottom - box.top)
        );
        
        // 获取关键点（无人机四个角点）
        std::vector<cv::Point2f> keypoints;
        const auto& kpts = result.kpts[i];
        if (kpts.size() >= 4) {
            for (size_t k = 0; k < 4 && k < kpts.size(); ++k) {
                keypoints.emplace_back(kpts[k].x, kpts[k].y);
            }
        } else {
            // 如果没有足够的关键点，使用边界框的四个角
            keypoints = {
                cv::Point2f(bbox.x, bbox.y),
                cv::Point2f(bbox.x + bbox.width, bbox.y),
                cv::Point2f(bbox.x + bbox.width, bbox.y + bbox.height),
                cv::Point2f(bbox.x, bbox.y + bbox.height)
            };
        }
        
        // 排序关键点
        sort_keypoints(keypoints);
        
        // 转换到原图坐标系（如果使用了ROI）
        if (use_roi_) {
            for (auto& pt : keypoints) {
                pt += offset_;
            }
            bbox.x += offset_.x;
            bbox.y += offset_.y;
        }
        
        // 计算中心点（使用关键点中心或边界框中心）
        cv::Point2f center(0, 0);
        for (const auto& pt : keypoints) {
            center += pt;
        }
        center.x /= static_cast<float>(keypoints.size());
        center.y /= static_cast<float>(keypoints.size());
        
        // 创建Drone对象
        Drone drone(
            class_id,           // 类别ID
            score,              // 置信度
            bbox,               // 边界框
            keypoints           // 四个角点
        );
        drone.center = center;
        
        drones.push_back(drone);
    }
}

int DroneDetector::remap_class_id(int model_id)
{
    // 根据你的训练数据定义映射
    // 示例：假设模型输出 0-无人机，1-无人机类型1等
    switch(model_id) {
        case 0: return 0;   // 普通无人机
        case 1: return 1;   // 小型无人机
        case 2: return 2;   // 大型无人机
        // 添加更多映射...
        default: return -1;  // 未知类别
    }
}

void DroneDetector::sort_keypoints(std::vector<cv::Point2f>& keypoints)
{
    if (keypoints.size() != 4) return;
    
    // 按Y坐标排序，区分上下
    std::sort(keypoints.begin(), keypoints.end(),
        [](const cv::Point2f& a, const cv::Point2f& b) {
            return a.y < b.y;
        });
    
    std::vector<cv::Point2f> top = {keypoints[0], keypoints[1]};
    std::vector<cv::Point2f> bottom = {keypoints[2], keypoints[3]};
    
    // 按X坐标排序
    std::sort(top.begin(), top.end(),
        [](const cv::Point2f& a, const cv::Point2f& b) {
            return a.x < b.x;
        });
    std::sort(bottom.begin(), bottom.end(),
        [](const cv::Point2f& a, const cv::Point2f& b) {
            return a.x < b.x;
        });
    
    // 最终顺序：左上、右上、右下、左下
    keypoints[0] = top[0];      // 左上
    keypoints[1] = top[1];      // 右上
    keypoints[2] = bottom[1];   // 右下
    keypoints[3] = bottom[0];   // 左下
}

bool DroneDetector::check_confidence(const Drone& drone) const
{
    return drone.confidence > min_confidence_;
}

void DroneDetector::nms_filter(std::list<Drone>& drones)
{
    if (drones.empty()) return;
    
    // 转换为vector便于处理
    std::vector<Drone> drone_vec(drones.begin(), drones.end());
    
    // 按置信度降序排序
    std::sort(drone_vec.begin(), drone_vec.end(),
        [](const Drone& a, const Drone& b) {
            return a.confidence > b.confidence;
        });
    
    std::vector<bool> suppressed(drone_vec.size(), false);
    
    for (size_t i = 0; i < drone_vec.size(); ++i) {
        if (suppressed[i]) continue;
        
        for (size_t j = i + 1; j < drone_vec.size(); ++j) {
            if (suppressed[j]) continue;
            
            // 计算IoU
            const cv::Rect& rect_i = drone_vec[i].box;
            const cv::Rect& rect_j = drone_vec[j].box;
            
            int intersect_x1 = std::max(rect_i.x, rect_j.x);
            int intersect_y1 = std::max(rect_i.y, rect_j.y);
            int intersect_x2 = std::min(rect_i.x + rect_i.width, rect_j.x + rect_j.width);
            int intersect_y2 = std::min(rect_i.y + rect_i.height, rect_j.y + rect_j.height);
            
            int intersection = std::max(0, intersect_x2 - intersect_x1) *
                             std::max(0, intersect_y2 - intersect_y1);
            int area_i = rect_i.width * rect_i.height;
            int area_j = rect_j.width * rect_j.height;
            int union_area = area_i + area_j - intersection;
            
            double iou = union_area > 0 ? 
                        static_cast<double>(intersection) / union_area : 0.0;
            
            if (iou > nms_threshold_) {
                suppressed[j] = true;
            }
        }
    }
    
    // 重建结果
    drones.clear();
    for (size_t i = 0; i < drone_vec.size(); ++i) {
        if (!suppressed[i]) {
            drones.push_back(drone_vec[i]);
        }
    }
}

void DroneDetector::draw_detections(
    const cv::Mat& img, const std::list<Drone>& drones, int frame_count) const
{
    cv::Mat vis = img.clone();
    
    // 简单绘制（需要根据实际 tools 库调整）
    cv::putText(vis, fmt::format("Frame: {}", frame_count), cv::Point(10, 30), 
                cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 255), 2);
    
    for (const auto& drone : drones) {
        // 绘制边界框
        cv::rectangle(vis, drone.box, cv::Scalar(0, 255, 0), 2);
        
        // 绘制角点
        for (const auto& pt : drone.points) {
            cv::circle(vis, pt, 5, cv::Scalar(0, 0, 255), -1);
        }
        
        // 绘制中心
        cv::circle(vis, drone.center, 3, cv::Scalar(255, 0, 0), -1);
        
        // 显示信息
        auto info = fmt::format("ID:{} Conf:{:.2f}", drone.class_id, drone.confidence);
        cv::putText(vis, info, cv::Point(static_cast<int>(drone.center.x), static_cast<int>(drone.center.y)),
                   cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);
        
        // 绘制角点编号
        const char* labels[] = {"TL", "TR", "BR", "BL"};  // Top-Left, Top-Right, Bottom-Right, Bottom-Left
        for (size_t i = 0; i < drone.points.size() && i < 4; ++i) {
            cv::putText(vis, labels[i], 
                       cv::Point(static_cast<int>(drone.points[i].x + 5), static_cast<int>(drone.points[i].y - 5)),
                       cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255, 255, 0), 1);
        }
    }
    
    // 绘制ROI区域
    if (use_roi_) {
        cv::rectangle(vis, roi_, cv::Scalar(255, 255, 0), 2);
    }
    
    // 调整大小显示
    cv::resize(vis, vis, cv::Size(), 0.8, 0.8);
    cv::imshow("Drone Detection", vis);
    cv::waitKey(1);
}

} // namespace drone_detection
