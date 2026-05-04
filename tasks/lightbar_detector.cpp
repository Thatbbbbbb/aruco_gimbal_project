#include "lightbar_detector.hpp"
#include <algorithm>
#include <cmath>
#include <set>
#include <fstream>
#include "tools/yaml.hpp"

namespace drone_detection {

// ========== SingleLightbar 实现 ==========
SingleLightbar::SingleLightbar(const cv::RotatedRect& rect, const cv::Point2f& offset) {
    rotated_rect = rect;
    center = rect.center + offset;
    
    // 获取四个角点并排序
    std::vector<cv::Point2f> corners(4);
    rect.points(&corners[0]);
    for (auto& pt : corners) {
        pt += offset;
    }
    std::sort(corners.begin(), corners.end(), 
        [](const cv::Point2f& a, const cv::Point2f& b) {
            return a.y < b.y;
        });
    
    top = (corners[0] + corners[1]) / 2;
    bottom = (corners[2] + corners[3]) / 2;
    
    auto top2bottom = bottom - top;
    length = cv::norm(top2bottom);
    width = cv::norm(corners[0] - corners[1]);
    angle = std::atan2(top2bottom.y, top2bottom.x);
    ratio = length / width;
}

// ========== LightbarDetector 实现 ==========
LightbarDetector::LightbarDetector() {
}

bool LightbarDetector::loadConfig(const std::string& path) {
    try {
        YAML::Node config = YAML::LoadFile(path);
        
        if (config["lightbar"]) {
            lightbar_color_ = config["lightbar"]["color"].as<std::string>();
            
            // 读取颜色阈值
            if (config["lightbar"]["h_min"]) h_min_ = config["lightbar"]["h_min"].as<int>();
            if (config["lightbar"]["h_max"]) h_max_ = config["lightbar"]["h_max"].as<int>();
            if (config["lightbar"]["s_min"]) s_min_ = config["lightbar"]["s_min"].as<int>();
            if (config["lightbar"]["s_max"]) s_max_ = config["lightbar"]["s_max"].as<int>();
            if (config["lightbar"]["v_min"]) v_min_ = config["lightbar"]["v_min"].as<int>();
            if (config["lightbar"]["v_max"]) v_max_ = config["lightbar"]["v_max"].as<int>();
        }
        
        if (config["detect"]) {
            if (config["detect"]["min_area"]) min_lightbar_area_ = config["detect"]["min_area"].as<double>();
            if (config["detect"]["max_area"]) max_lightbar_area_ = config["detect"]["max_area"].as<double>();
            if (config["detect"]["min_ratio"]) min_lightbar_ratio_ = config["detect"]["min_ratio"].as<double>();
            if (config["detect"]["max_ratio"]) max_lightbar_ratio_ = config["detect"]["max_ratio"].as<double>();
            if (config["detect"]["max_angle_error"]) max_angle_error_ = config["detect"]["max_angle_error"].as<double>();
            if (config["detect"]["max_group_distance"]) max_group_distance_ = config["detect"]["max_group_distance"].as<double>();
            if (config["detect"]["min_row_lightbars"]) min_lightbars_per_row_ = config["detect"]["min_row_lightbars"].as<int>();
            if (config["detect"]["max_row_lightbars"]) max_lightbars_per_row_ = config["detect"]["max_row_lightbars"].as<int>();
            if (config["detect"]["min_top_bottom_distance"]) min_top_bottom_distance_ = config["detect"]["min_top_bottom_distance"].as<double>();
            if (config["detect"]["max_top_bottom_distance"]) max_top_bottom_distance_ = config["detect"]["max_top_bottom_distance"].as<double>();
        }
        
        return true;
    } catch (...) {
        return false;
    }
}

cv::Mat LightbarDetector::extractColor(const cv::Mat& img) {
    cv::Mat hsv, mask;
    cv::cvtColor(img, hsv, cv::COLOR_BGR2HSV);

    // 使用配置文件读取的阈值，不再写死！
    cv::inRange(hsv, cv::Scalar(h_min_, s_min_, v_min_),
                     cv::Scalar(h_max_, s_max_, v_max_), mask);

    // 红色特殊处理
    if (lightbar_color_ == "red") {
        cv::Mat mask2;
        cv::inRange(hsv, cv::Scalar(160, s_min_, v_min_),
                         cv::Scalar(180, s_max_, v_max_), mask2);
        mask |= mask2;
    }

    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);
    cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);

    return mask;
}

std::vector<SingleLightbar> LightbarDetector::detectLightbars(const cv::Mat& img, const cv::Rect& roi) {
    std::vector<SingleLightbar> lightbars;
    
    if (roi.x < 0 || roi.y < 0 || roi.x + roi.width > img.cols || roi.y + roi.height > img.rows) {
        return lightbars;
    }
    
    cv::Mat roi_img = img(roi);
    if (roi_img.empty() || roi_img.rows < 10 || roi_img.cols < 10) return lightbars;
    
    cv::Mat binary = extractColor(roi_img);
    
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(binary, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    
    for (const auto& contour : contours) {
        double area = cv::contourArea(contour);
        if (area < min_lightbar_area_) continue;
        if (area > max_lightbar_area_) continue;  // 最大面积过滤
        
        cv::RotatedRect rect = cv::minAreaRect(contour);
        
        float w = rect.size.width;
        float h = rect.size.height;
        float len = std::max(w, h);
        float wid = std::min(w, h);
        float ratio = len / wid;
        
        if (ratio < min_lightbar_ratio_) continue;   // 最小长宽比
        if (ratio > max_lightbar_ratio_) continue;   // 最大长宽比

        // 角度过滤
        float angle = 0;
        if (w < h) angle = rect.angle;
        else angle = rect.angle + 90;
        float angle_deg = std::abs(angle);
        float angle_error = std::min(angle_deg, 180.0f - angle_deg);
        if (angle_error > max_angle_error_) continue;
        
        lightbars.emplace_back(rect, cv::Point2f(roi.x, roi.y));
    }
    
    return lightbars;
}

std::vector<HorizontalLightbarGroup> LightbarDetector::groupByRow(
    const std::vector<SingleLightbar>& lightbars) {
    
    std::vector<HorizontalLightbarGroup> rows;
    
    if (lightbars.empty()) return rows;
    
    std::vector<SingleLightbar> sorted = lightbars;
    std::sort(sorted.begin(), sorted.end(),
        [](const SingleLightbar& a, const SingleLightbar& b) {
            return a.center.y < b.center.y;
        });
    
    std::vector<std::vector<SingleLightbar>> row_clusters;
    float y_threshold = 15.0f;
    
    for (const auto& lb : sorted) {
        bool added = false;
        for (auto& cluster : row_clusters) {
            if (!cluster.empty()) {
                float avg_y = 0;
                for (const auto& c : cluster) {
                    avg_y += c.center.y;
                }
                avg_y /= cluster.size();
                
                if (std::abs(lb.center.y - avg_y) < y_threshold) {
                    cluster.push_back(lb);
                    added = true;
                    break;
                }
            }
        }
        if (!added) {
            row_clusters.push_back({lb});
        }
    }
    
    for (const auto& cluster : row_clusters) {
        if (cluster.size() >= min_lightbars_per_row_ && 
            cluster.size() <= max_lightbars_per_row_) {
            rows.push_back(filterRowGroup(cluster));
        }
    }
    
    return rows;
}

HorizontalLightbarGroup LightbarDetector::filterRowGroup(
    const std::vector<SingleLightbar>& row_candidates) {
    
    HorizontalLightbarGroup group;
    
    if (row_candidates.empty()) return group;
    
    std::vector<SingleLightbar> sorted = row_candidates;
    std::sort(sorted.begin(), sorted.end(),
        [](const SingleLightbar& a, const SingleLightbar& b) {
            return a.center.x < a.center.x;
        });
    
    std::vector<SingleLightbar> filtered;
    for (size_t i = 0; i < sorted.size(); ++i) {
        if (filtered.empty()) {
            filtered.push_back(sorted[i]);
            continue;
        }
        
        float distance = std::abs(sorted[i].center.x - filtered.back().center.x);
        if (distance < max_group_distance_) {
            filtered.push_back(sorted[i]);
        } else if (filtered.size() >= min_lightbars_per_row_) {
            break;
        } else {
            filtered.clear();
            filtered.push_back(sorted[i]);
        }
    }
    
    if (filtered.size() < min_lightbars_per_row_ || 
        filtered.size() > max_lightbars_per_row_) {
        return group;
    }
    
    group.lightbars = filtered;
    
    cv::Point2f center(0, 0);
    float y_level = 0;
    int valid_count = 0;
    for (const auto& lb : filtered) {
        center += lb.center;
        y_level += lb.center.y;
        valid_count++;
    }
    if (valid_count > 0) {
        center.x /= valid_count;
        center.y /= valid_count;
        y_level /= valid_count;
    }
    group.center = center;
    group.y_level = y_level;
    
    if (max_lightbars_per_row_ == 3) {
        if (filtered.size() == 3) group.confidence = 1.0f;
        else if (filtered.size() == 2) group.confidence = 0.7f;
        else group.confidence = 0.5f;
    } else {
        group.confidence = 1.0f;
    }
    
    return group;
}

std::vector<ParallelLightbarSet> LightbarDetector::matchTopBottomRows(
    const std::vector<HorizontalLightbarGroup>& rows) {
    
    std::vector<ParallelLightbarSet> results;
    
    if (rows.size() < 2) return results;
    
    std::vector<HorizontalLightbarGroup> sorted_rows = rows;
    std::sort(sorted_rows.begin(), sorted_rows.end(),
        [](const HorizontalLightbarGroup& a, const HorizontalLightbarGroup& b) {
            return a.y_level < b.y_level;
        });
    
    for (size_t i = 0; i < sorted_rows.size() - 1; ++i) {
        for (size_t j = i + 1; j < sorted_rows.size(); ++j) {
            const auto& top = sorted_rows[i];
            const auto& bottom = sorted_rows[j];
            
            float vertical_distance = bottom.y_level - top.y_level;
            if (vertical_distance < min_top_bottom_distance_ || 
                vertical_distance > max_top_bottom_distance_) {
                continue;
            }
            
            int top_count = top.lightbars.size();
            int bottom_count = bottom.lightbars.size();
            
            if (std::abs(top_count - bottom_count) > 1) continue;
            
            bool aligned = true;
            float max_x_diff = 20.0f;
            
            for (size_t k = 0; k < std::min(top.lightbars.size(), bottom.lightbars.size()); ++k) {
                float x_diff = std::abs(top.lightbars[k].center.x - bottom.lightbars[k].center.x);
                if (x_diff > max_x_diff) {
                    aligned = false;
                    break;
                }
            }
            
            if (!aligned) continue;
            
            ParallelLightbarSet set;
            set.top_row = top;
            set.bottom_row = bottom;
            set.top_to_bottom_distance = vertical_distance;
            set.center = (top.center + bottom.center) / 2;
            
            float min_x = std::min(top.lightbars.front().center.x, bottom.lightbars.front().center.x);
            float max_x = std::max(top.lightbars.back().center.x, bottom.lightbars.back().center.x);
            float min_y = top.y_level - top.lightbars.front().length / 2;
            float max_y = bottom.y_level + bottom.lightbars.front().length / 2;
            
            set.corners = {
                cv::Point2f(min_x, min_y),
                cv::Point2f(max_x, min_y),
                cv::Point2f(max_x, max_y),
                cv::Point2f(min_x, max_y)
            };
            
            set.confidence = (top.confidence + bottom.confidence) / 2;
            
            if (top_count == 3 && bottom_count == 3) set.confidence *= 1.0f;
            else if (top_count == 2 && bottom_count == 2) set.confidence *= 0.8f;
            else set.confidence *= 0.6f;
            
            results.push_back(set);
        }
    }
    
    std::vector<ParallelLightbarSet> final_results;
    std::vector<bool> suppressed(results.size(), false);
    
    for (size_t i = 0; i < results.size(); ++i) {
        if (suppressed[i]) continue;
        final_results.push_back(results[i]);
        
        for (size_t j = i + 1; j < results.size(); ++j) {
            if (suppressed[j]) continue;
            
            float distance = cv::norm(results[i].center - results[j].center);
            if (distance < 30.0f) {
                suppressed[j] = true;
            }
        }
    }
    
    return final_results;
}

std::vector<ParallelLightbarSet> LightbarDetector::detect(const cv::Mat& img, const cv::Rect& roi) {
    auto lightbars = detectLightbars(img, roi);
    if (lightbars.empty()) return {};
    auto rows = groupByRow(lightbars);
    if (rows.empty()) return {};
    return matchTopBottomRows(rows);
}

} // namespace drone_detection