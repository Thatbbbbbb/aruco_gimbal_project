
#include <cstdint>
#include <cstring>
#include <sstream>
#include <vector>

#include "model.hpp"
#include "result.hpp"

namespace deploy {

template <typename ResultType>
std::unique_ptr<BaseModel<ResultType>> BaseModel<ResultType>::clone() const {
    auto clone_model              = std::make_unique<BaseModel<ResultType>>();
    clone_model->backend_         = backend_->clone();  // < 克隆 TrtBackend
    clone_model->infer_gpu_trace_ = std::make_unique<GpuTimer>(clone_model->backend_->stream);
    clone_model->infer_cpu_trace_ = std::make_unique<CpuTimer>();
    return clone_model;
}

template <typename ResultType>
std::vector<ResultType> BaseModel<ResultType>::predict(const std::vector<Image>& images) {
    if (backend_->option.enable_performance_report) {
        total_request_ += (backend_->dynamic ? images.size() : backend_->max_shape.x);
        infer_cpu_trace_->start();
        infer_gpu_trace_->start();
    }

    backend_->infer(images);  // 调用推理方法

    // 预分配结果空间
    std::vector<ResultType> results(images.size());
    for (auto idx = 0u; idx < images.size(); ++idx) {
        results[idx] = postProcess(idx);
    }

    if (backend_->option.enable_performance_report) {
        infer_gpu_trace_->stop();
        infer_cpu_trace_->stop();
    }

    return results;
}

template <typename ResultType>
ResultType BaseModel<ResultType>::predict(const Image& image) {
    return predict(std::vector<Image>{image}).front();
}

template <typename ResultType>
int BaseModel<ResultType>::batch_size() const {
    return backend_->max_shape.x;
}

template <typename ResultType>
std::tuple<std::string, std::string, std::string> BaseModel<ResultType>::performanceReport() {
    if (backend_->option.enable_performance_report) {
        float const       throughput = total_request_ / infer_cpu_trace_->totalMilliseconds() * 1000;
        std::stringstream ss;

        // 构建吞吐量字符串
        ss << "Throughput: " << throughput << " qps";
        std::string throughputStr = ss.str();
        ss.str("");  // 清空 stringstream

        auto percentiles = std::vector<float>{90, 95, 99};

        auto getLatencyStr = [&](const auto& trace, const std::string& device) {
            auto result = getPerformanceResult(trace->milliseconds(), {0.90, 0.95, 0.99});
            ss << device << " Latency: min = " << result.min << " ms, max = " << result.max << " ms, mean = " << result.mean << " ms, median = " << result.median << " ms";
            for (int32_t i = 0, n = percentiles.size(); i < n; ++i) {
                ss << ", percentile(" << percentiles[i] << "%) = " << result.percentiles[i] << " ms";
            }
            std::string output = ss.str();
            ss.str("");  // 清空 stringstream
            return output;
        };

        std::string cpuLatencyStr = getLatencyStr(infer_cpu_trace_, "CPU");
        std::string gpuLatencyStr = getLatencyStr(infer_gpu_trace_, "GPU");

        total_request_ = 0;
        infer_cpu_trace_->reset();
        infer_gpu_trace_->reset();

        return std::make_tuple(throughputStr, cpuLatencyStr, gpuLatencyStr);
    } else {
        // 性能报告未启用时返回空字符串
        return std::make_tuple("", "", "");
    }
}

// ClassifyModel 的后处理方法实现
template <>
ClassifyRes BaseModel<ClassifyRes>::postProcess(int idx) {
    auto&  tensor_info = backend_->tensor_infos[1];
    float* topk        = static_cast<float*>(tensor_info.buffer->host()) + idx * tensor_info.shape.d[1] * tensor_info.shape.d[2];

    ClassifyRes result;
    result.num = tensor_info.shape.d[1];
    result.scores.reserve(result.num);
    result.classes.reserve(result.num);

    for (int i = 0; i < result.num; ++i) {
        result.scores.push_back(topk[i * tensor_info.shape.d[2]]);
        result.classes.push_back(topk[i * tensor_info.shape.d[2] + 1]);
    }

    return result;
}

// DetectModel 的后处理方法实现
template <>
DetectRes BaseModel<DetectRes>::postProcess(int idx) {
    auto& num_tensor   = backend_->tensor_infos[1];
    auto& box_tensor   = backend_->tensor_infos[2];
    auto& score_tensor = backend_->tensor_infos[3];
    auto& class_tensor = backend_->tensor_infos[4];

    int    num     = static_cast<int*>(num_tensor.buffer->host())[idx];
    float* boxes   = static_cast<float*>(box_tensor.buffer->host()) + idx * box_tensor.shape.d[1] * box_tensor.shape.d[2];
    float* scores  = static_cast<float*>(score_tensor.buffer->host()) + idx * score_tensor.shape.d[1];
    int*   classes = static_cast<int*>(class_tensor.buffer->host()) + idx * class_tensor.shape.d[1];

    DetectRes result;
    result.num   = num;
    int box_size = box_tensor.shape.d[2];

    auto& affine_transform = backend_->option.input_shape.has_value()
                                 ? backend_->affine_transforms.front()
                                 : backend_->affine_transforms[idx];

    result.boxes.reserve(num);
    result.scores.reserve(num);
    result.classes.reserve(num);

    for (int i = 0; i < num; ++i) {
        int   base_index = i * box_size;
        float left = boxes[base_index], top = boxes[base_index + 1];
        float right = boxes[base_index + 2], bottom = boxes[base_index + 3];

        affine_transform.applyTransform(left, top, &left, &top);
        affine_transform.applyTransform(right, bottom, &right, &bottom);

        result.boxes.emplace_back(Box{left, top, right, bottom});
        result.scores.push_back(scores[i]);
        result.classes.push_back(classes[i]);
    }

    return result;
}

// OBBModel 的后处理方法实现
template <>
OBBRes BaseModel<OBBRes>::postProcess(int idx) {
    auto& num_tensor   = backend_->tensor_infos[1];
    auto& box_tensor   = backend_->tensor_infos[2];
    auto& score_tensor = backend_->tensor_infos[3];
    auto& class_tensor = backend_->tensor_infos[4];

    int    num     = static_cast<int*>(num_tensor.buffer->host())[idx];
    float* boxes   = static_cast<float*>(box_tensor.buffer->host()) + idx * box_tensor.shape.d[1] * box_tensor.shape.d[2];
    float* scores  = static_cast<float*>(score_tensor.buffer->host()) + idx * score_tensor.shape.d[1];
    int*   classes = static_cast<int*>(class_tensor.buffer->host()) + idx * class_tensor.shape.d[1];

    OBBRes result;
    result.num   = num;
    int box_size = box_tensor.shape.d[2];

    auto& affine_transform = backend_->option.input_shape.has_value()
                                 ? backend_->affine_transforms.front()
                                 : backend_->affine_transforms[idx];

    result.boxes.reserve(num);
    result.scores.reserve(num);
    result.classes.reserve(num);

    for (int i = 0; i < num; ++i) {
        int   base_index = i * box_size;
        float left = boxes[base_index], top = boxes[base_index + 1];
        float right = boxes[base_index + 2], bottom = boxes[base_index + 3];
        float theta = boxes[base_index + 4];

        affine_transform.applyTransform(left, top, &left, &top);
        affine_transform.applyTransform(right, bottom, &right, &bottom);

        result.boxes.emplace_back(RotatedBox{left, top, right, bottom, theta});
        result.scores.push_back(scores[i]);
        result.classes.push_back(classes[i]);
    }

    return result;
}

// SegmentModel 的后处理方法实现
template <>
SegmentRes BaseModel<SegmentRes>::postProcess(int idx) {
    auto& num_tensor   = backend_->tensor_infos[1];
    auto& box_tensor   = backend_->tensor_infos[2];
    auto& score_tensor = backend_->tensor_infos[3];
    auto& class_tensor = backend_->tensor_infos[4];
    auto& mask_tensor  = backend_->tensor_infos[5];
    int   mask_height  = mask_tensor.shape.d[2];
    int   mask_width   = mask_tensor.shape.d[3];

    int      num     = static_cast<int*>(num_tensor.buffer->host())[idx];
    float*   boxes   = static_cast<float*>(box_tensor.buffer->host()) + idx * box_tensor.shape.d[1] * box_tensor.shape.d[2];
    float*   scores  = static_cast<float*>(score_tensor.buffer->host()) + idx * score_tensor.shape.d[1];
    int*     classes = static_cast<int*>(class_tensor.buffer->host()) + idx * class_tensor.shape.d[1];
    uint8_t* masks   = static_cast<uint8_t*>(mask_tensor.buffer->host()) + idx * mask_tensor.shape.d[1] * mask_height * mask_width;

    SegmentRes result;
    result.num   = num;
    int box_size = box_tensor.shape.d[2];

    auto& affine_transform = backend_->option.input_shape.has_value()
                                 ? backend_->affine_transforms.front()
                                 : backend_->affine_transforms[idx];

    result.boxes.reserve(num);
    result.scores.reserve(num);
    result.classes.reserve(num);
    result.masks.reserve(num);

    for (int i = 0; i < num; ++i) {
        int   base_index = i * box_size;
        float left = boxes[base_index], top = boxes[base_index + 1];
        float right = boxes[base_index + 2], bottom = boxes[base_index + 3];

        affine_transform.applyTransform(left, top, &left, &top);
        affine_transform.applyTransform(right, bottom, &right, &bottom);

        result.boxes.emplace_back(Box{left, top, right, bottom});
        result.scores.push_back(scores[i]);
        result.classes.push_back(classes[i]);

        Mask mask(mask_width - 2 * affine_transform.dst_offset_x, mask_height - 2 * affine_transform.dst_offset_y);

        // Crop the mask's edge area, applying offset to adjust the position
        int start_idx = i * mask_height * mask_width;
        int src_idx   = start_idx + affine_transform.dst_offset_y * mask_width + affine_transform.dst_offset_x;
        for (int y = 0; y < mask.height; ++y) {
            std::memcpy(&mask.data[y * mask.width], masks + src_idx, mask.width);
            src_idx += mask_width;
        }

        result.masks.emplace_back(std::move(mask));
    }

    return result;
}

// PoseModel 的后处理方法实现
template <>
PoseRes BaseModel<PoseRes>::postProcess(int idx) {
    // 诊断：首次推理时打印所有张量信息，帮助定位 engine 布局不匹配问题
    static bool first_call = true;
    if (first_call) {
        first_call = false;
        std::cerr << "[DEBUG] postProcess<PoseRes>: backend_->tensor_infos has "
                  << backend_->tensor_infos.size() << " tensors:" << std::endl;
        for (size_t i = 0; i < backend_->tensor_infos.size(); ++i) {
            auto& t = backend_->tensor_infos[i];
            std::cerr << "  [" << i << "] name=\"" << t.name
                      << "\" input=" << t.input
                      << " shape=[";
            for (int d = 0; d < t.shape.nbDims; ++d) {
                std::cerr << t.shape.d[d];
                if (d + 1 < t.shape.nbDims) std::cerr << ",";
            }
            std::cerr << "] host=" << t.buffer->host()
                      << " device=" << t.buffer->device()
                      << " size=" << t.buffer->size() << std::endl;
        }
    }

    // 防御性检查：输出张量数量校验
    if (backend_->tensor_infos.size() < 6) {
        std::cerr << "[ERROR] postProcess<PoseRes> expects at least 6 tensors (1 input + 5 outputs), "
                  << "but only " << backend_->tensor_infos.size() << " found. "
                  << "Your TensorRT engine may be a standard YOLO detect model with a single combined output, "
                  << "not compatible with PoseRes format which requires separate num/boxes/scores/classes/kpts tensors."
                  << std::endl;
        return PoseRes{};
    }

    auto& num_tensor   = backend_->tensor_infos[1];
    auto& box_tensor   = backend_->tensor_infos[2];
    auto& score_tensor = backend_->tensor_infos[3];
    auto& class_tensor = backend_->tensor_infos[4];
    auto& kpt_tensor   = backend_->tensor_infos[5];

    // 防御性检查：确保各张量的 host() 非空
    if (!num_tensor.buffer->host() || !box_tensor.buffer->host() ||
        !score_tensor.buffer->host() || !class_tensor.buffer->host() ||
        !kpt_tensor.buffer->host()) {
        std::cerr << "[ERROR] postProcess<PoseRes>: one or more output tensors have nullptr host buffer. "
                  << "num_host=" << num_tensor.buffer->host() << " name=\"" << num_tensor.name << "\" input=" << num_tensor.input
                  << ", box_host=" << box_tensor.buffer->host() << " name=\"" << box_tensor.name << "\" input=" << box_tensor.input
                  << ", score_host=" << score_tensor.buffer->host() << " name=\"" << score_tensor.name << "\" input=" << score_tensor.input
                  << ", class_host=" << class_tensor.buffer->host() << " name=\"" << class_tensor.name << "\" input=" << class_tensor.input
                  << ", kpt_host=" << kpt_tensor.buffer->host() << " name=\"" << kpt_tensor.name << "\" input=" << kpt_tensor.input
                  << std::endl;
        return PoseRes{};
    }

    int   nkpt         = kpt_tensor.shape.d[2];
    int   ndim         = kpt_tensor.shape.d[3];

    int    num     = static_cast<int*>(num_tensor.buffer->host())[idx];
    float* boxes   = static_cast<float*>(box_tensor.buffer->host()) + idx * box_tensor.shape.d[1] * box_tensor.shape.d[2];
    float* scores  = static_cast<float*>(score_tensor.buffer->host()) + idx * score_tensor.shape.d[1];
    int*   classes = static_cast<int*>(class_tensor.buffer->host()) + idx * class_tensor.shape.d[1];
    float* kpts    = static_cast<float*>(kpt_tensor.buffer->host()) + idx * kpt_tensor.shape.d[1] * nkpt * ndim;

    PoseRes result;
    result.num   = num;
    int box_size = box_tensor.shape.d[2];

    auto& affine_transform = backend_->option.input_shape.has_value()
                                 ? backend_->affine_transforms.front()
                                 : backend_->affine_transforms[idx];

    result.boxes.reserve(num);
    result.scores.reserve(num);
    result.classes.reserve(num);
    result.kpts.reserve(num);

    for (int i = 0; i < num; ++i) {
        int   base_index = i * box_size;
        float left = boxes[base_index], top = boxes[base_index + 1];
        float right = boxes[base_index + 2], bottom = boxes[base_index + 3];

        affine_transform.applyTransform(left, top, &left, &top);
        affine_transform.applyTransform(right, bottom, &right, &bottom);

        result.boxes.emplace_back(Box{left, top, right, bottom});
        result.scores.push_back(scores[i]);
        result.classes.push_back(classes[i]);

        std::vector<KeyPoint> keypoints;
        for (int j = 0; j < nkpt; ++j) {
            float x = kpts[i * nkpt * ndim + j * ndim];
            float y = kpts[i * nkpt * ndim + j * ndim + 1];
            affine_transform.applyTransform(x, y, &x, &y);
            keypoints.emplace_back((ndim == 2) ? KeyPoint(x, y) : KeyPoint(x, y, kpts[i * nkpt * ndim + j * ndim + 2]));
        }
        result.kpts.emplace_back(std::move(keypoints));
    }

    return result;
}

}  // namespace deploy
