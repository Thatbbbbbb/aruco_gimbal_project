#include "lightbar_detector.hpp"
#include <algorithm>
#include <cmath>
#include <set>

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

std::vector<SingleLightbar> LightbarDetector::detectLightbars(const cv::Mat& img, const cv::Rect& roi) {
    std::vector<SingleLightbar> lightbars;
    
    // 检查 ROI 有效性
    if (roi.x < 0 || roi.y < 0 || roi.x + roi.width > img.cols || roi.y + roi.height > img.rows) {
        return lightbars;
    }
    
    // 提取 ROI 区域
    cv::Mat roi_img = img(roi);
    if (roi_img.empty() || roi_img.rows < 10 || roi_img.cols < 10) return lightbars;
    
    // 转换为灰度图
    cv::Mat gray;
    cv::cvtColor(roi_img, gray, cv::COLOR_BGR2GRAY);
    
    // 二值化（提取高亮区域）
    cv::Mat binary;
    cv::threshold(gray, binary, brightness_threshold_, 255, cv::THRESH_BINARY);
    
    // 形态学操作去除噪点
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    cv::morphologyEx(binary, binary, cv::MORPH_OPEN, kernel);
    cv::morphologyEx(binary, binary, cv::MORPH_CLOSE, kernel);
    
    // 查找轮廓
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(binary, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    
    for (const auto& contour : contours) {
        double area = cv::contourArea(contour);
        if (area < min_lightbar_area_) continue;
        
        // 拟合旋转矩形
        cv::RotatedRect rect = cv::minAreaRect(contour);
        
        // 检查长宽比（灯条应该是细长的）
        float width = std::min(rect.size.width, rect.size.height);
        float height = std::max(rect.size.width, rect.size.height);
        float ratio = height / width;
        
        if (ratio > max_lightbar_ratio_) continue;
        
        // 创建灯条并调整坐标到原图
        lightbars.emplace_back(rect, cv::Point2f(roi.x, roi.y));
    }
    
    return lightbars;
}

std::vector<HorizontalLightbarGroup> LightbarDetector::groupByRow(
    const std::vector<SingleLightbar>& lightbars) {
    
    if (lightbars.empty()) return {};
    
    // 按 Y 坐标排序
    std::vector<SingleLightbar> sorted = lightbars;
    std::sort(sorted.begin(), sorted.end(),
        [](const SingleLightbar& a, const SingleLightbar& b) {
            return a.center.y < b.center.y;
        });
    
    // 聚类分组（Y 坐标相近的为同一行）
    std::vector<std::vector<SingleLightbar>> row_clusters;
    float y_threshold = 15.0f;  // Y 坐标差异阈值
    
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
    
    // 过滤掉灯条数量不足的行
    std::vector<HorizontalLightbarGroup> rows;
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
    
    // 按 X 坐标排序
    std::vector<SingleLightbar> sorted = row_candidates;
    std::sort(sorted.begin(), sorted.end(),
        [](const SingleLightbar& a, const SingleLightbar& b) {
            return a.center.x < b.center.x;
        });
    
    // 检查灯条之间的间距
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
            // 如果间距太大，考虑作为新组
            break;
        } else {
            filtered.clear();
            filtered.push_back(sorted[i]);
        }
    }
    
    // 检查灯条数量是否符合要求
    if (filtered.size() < min_lightbars_per_row_ || 
        filtered.size() > max_lightbars_per_row_) {
        return group;
    }
    
    group.lightbars = filtered;
    
    // 计算组中心
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
    
    // 计算置信度（基于灯条数量一致性）
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
    
    // 按 Y 坐标排序
    std::vector<HorizontalLightbarGroup> sorted_rows = rows;
    std::sort(sorted_rows.begin(), sorted_rows.end(),
        [](const HorizontalLightbarGroup& a, const HorizontalLightbarGroup& b) {
            return a.y_level < b.y_level;
        });
    
    // 匹配上排和下排
    for (size_t i = 0; i < sorted_rows.size() - 1; ++i) {
        for (size_t j = i + 1; j < sorted_rows.size(); ++j) {
            const auto& top = sorted_rows[i];
            const auto& bottom = sorted_rows[j];
            
            // 检查上下排距离
            float vertical_distance = bottom.y_level - top.y_level;
            if (vertical_distance < min_top_bottom_distance_ || 
                vertical_distance > max_top_bottom_distance_) {
                continue;
            }
            
            // 检查灯条数量一致性（2x2, 2x3, 3x3）
            int top_count = top.lightbars.size();
            int bottom_count = bottom.lightbars.size();
            
            if (std::abs(top_count - bottom_count) > 1) continue;
            
            // 检查水平对齐（每对灯条的 X 坐标应该相近）
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
            
            // 创建组合
            ParallelLightbarSet set;
            set.top_row = top;
            set.bottom_row = bottom;
            set.top_to_bottom_distance = vertical_distance;
            
            // 计算整体中心
            set.center = (top.center + bottom.center) / 2;
            
            // 计算四个角点
            float min_x = std::min(top.lightbars.front().center.x, bottom.lightbars.front().center.x);
            float max_x = std::max(top.lightbars.back().center.x, bottom.lightbars.back().center.x);
            float min_y = top.y_level - top.lightbars.front().length / 2;
            float max_y = bottom.y_level + bottom.lightbars.front().length / 2;
            
            set.corners = {
                cv::Point2f(min_x, min_y),  // 左上
                cv::Point2f(max_x, min_y),  // 右上
                cv::Point2f(max_x, max_y),  // 右下
                cv::Point2f(min_x, max_y)   // 左下
            };
            
            // 计算整体置信度
            set.confidence = (top.confidence + bottom.confidence) / 2;
            
            // 根据灯条数量调整置信度
            if (top_count == 3 && bottom_count == 3) set.confidence *= 1.0f;
            else if (top_count == 2 && bottom_count == 2) set.confidence *= 0.8f;
            else set.confidence *= 0.6f;
            
            results.push_back(set);
        }
    }
    
    // 非极大值抑制（NMS）
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
    // 1. 检测所有灯条
    auto lightbars = detectLightbars(img, roi);
    
    if (lightbars.empty()) return {};
    
    // 2. 按行分组
    auto rows = groupByRow(lightbars);
    
    if (rows.size() < 2) return {};
    
    // 3. 匹配上下排
    auto results = matchTopBottomRows(rows);
    
    return results;
}

} // namespace drone_detection