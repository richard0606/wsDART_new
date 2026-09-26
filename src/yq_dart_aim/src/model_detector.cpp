#include "yq_dart_aim/model_detector.hpp"

#include <algorithm>
#include <cmath>
#include <exception>

#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>

namespace yq_dart_aim {

namespace {
// 保留的最大检测数（与原模型内置 TopK 的 K 一致）
constexpr int kMaxDetections = 30;
// 原始输出布局: [x1,y1,x2,y2] [cls x12] [color x4] [kpt x8]
constexpr int kBoxChannels = 4;
constexpr int kColorChannels = 4;
constexpr int kKeypointChannels = 8;
constexpr int kRawChannels =
    kBoxChannels + model_classes::kNumClasses + kColorChannels + kKeypointChannels;

rclcpp::Logger modelLogger() { return rclcpp::get_logger("model_detector"); }

int64_t nowMs() {
    return std::chrono::duration_cast<std::chrono::milliseconds>(
               std::chrono::steady_clock::now().time_since_epoch())
        .count();
}
}  // namespace

ModelDetector::ModelDetector() = default;

ModelDetector::~ModelDetector() {
    unloadModel();
}

void ModelDetector::unloadModel() {
    stop_.store(true);
    new_frame_ready_.store(true);  // 唤醒推理线程以便退出
    if (infer_thread_.joinable()) {
        infer_thread_.join();
    }
    backendUnload(params_.backend);
    model_loaded_ = false;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        latest_frame_ = cv::Mat();
        latest_results_.clear();
    }
    last_result_ms_.store(0);
    infer_fail_count_.store(0);
}

bool ModelDetector::loadModel(const ModelParams& params) {
    unloadModel();  // 支持运行时换后端/换模型：先彻底卸载上一个

    params_ = params;

    if (params_.model_path.empty()) {
        return false;
    }
    if (!backendAvailable(params_.backend)) {
        RCLCPP_ERROR(modelLogger(), "推理后端 %s 未编译进本程序（缺依赖或编译时被禁用）",
                     backendName(params_.backend));
        return false;
    }

    try {
        if (!backendLoad(params_.backend, params_.model_path, kRawChannels)) {
            model_loaded_ = false;
            return false;
        }
    } catch (const std::exception&) {
        // 模型路径非法/文件损坏/格式不支持等一律视为加载失败，
        // 不能把异常抛到节点构造函数里（会直接终止进程）
        model_loaded_ = false;
        return false;
    }

    model_loaded_ = true;
    max_result_age_ms_.store(std::max(0, params_.max_result_age_ms));
    stop_.store(false);
    infer_thread_ = std::thread(&ModelDetector::inferenceLoop, this);
    RCLCPP_INFO(modelLogger(), "推理后端 = %s, 模型 = %s", backendName(params_.backend),
                params_.model_path.c_str());
    return true;
}

bool ModelDetector::isReady() const {
    return model_loaded_;
}

void ModelDetector::submit(const cv::Mat& image) {
    if (!model_loaded_) return;
    std::lock_guard<std::mutex> lock(mutex_);
    latest_frame_ = image.clone();
    new_frame_ready_.store(true);
}

std::vector<TargetInfo> ModelDetector::getResults() {
    std::lock_guard<std::mutex> lock(mutex_);
    const int max_age_ms = max_result_age_ms_.load();
    if (max_age_ms > 0 && !latest_results_.empty()) {
        const double age_ms = resultAgeMs();
        if (age_ms > static_cast<double>(max_age_ms)) {
            static rclcpp::Clock steady_clock(RCL_STEADY_TIME);
            RCLCPP_WARN_THROTTLE(modelLogger(), steady_clock, 2000,
                                 "模型结果过期: age=%.0fms > %dms，丢弃", age_ms, max_age_ms);
            return {};
        }
    }
    return latest_results_;
}

double ModelDetector::resultAgeMs() const {
    const int64_t last = last_result_ms_.load();
    if (last == 0) return -1.0;
    return static_cast<double>(nowMs() - last);
}

void ModelDetector::setMaxResultAgeMs(int ms) {
    const int clamped = std::max(0, ms);
    max_result_age_ms_.store(clamped);
    const std::string text = (clamped > 0) ? (std::to_string(clamped) + " ms") : std::string("不限制");
    RCLCPP_INFO(modelLogger(), "模型结果有效期 = %s", text.c_str());
}

// ==================== 预处理 ====================
// 按模型输入的宽高比做中心裁剪（不填充），再缩放。
//
// 为什么不用上游 README 里的 letterbox：这是有意选的「用视野换精度」，不是遗漏。
//   本机链路：相机 1280x872 -> 中心裁剪 1280x720 -> 4:3 裁剪 960x720 -> 缩放 768x576
//   letterbox：1280x720 -> 等比缩到 768x432 -> 上下补灰到 768x576
// 差异：
//   - 中心裁剪缩放系数 0.8（目标更大更清楚，置信度更高），代价是左右各丢 160px，
//     即只保留画面中间 75% 的宽度；目标偏到左右两侧会直接看不到
//   - letterbox 保留 100% 宽度，但缩放只有 0.6，目标更小、置信度更低
// 结论：中心区域精度优先，视野换精度。要改 Letterbox 前先想清楚这个取舍。
//
// 注意：有效瞄准区域是 960x720（不是整个 1280x872），标定偏移量按这个区域算。
void ModelDetector::preprocess(const cv::Mat& frame, cv::Mat& out, cv::Rect& crop) const {
    const double want_aspect = static_cast<double>(params_.input_width) / params_.input_height;
    const double have_aspect = static_cast<double>(frame.cols) / frame.rows;

    int cw = 0;
    int ch = 0;
    if (have_aspect > want_aspect) {
        ch = frame.rows;                                              // 图比模型宽 -> 裁左右
        cw = static_cast<int>(std::lround(frame.rows * want_aspect));
    } else {
        cw = frame.cols;                                              // 图比模型高 -> 裁上下
        ch = static_cast<int>(std::lround(frame.cols / want_aspect));
    }
    cw = std::min(cw, frame.cols);
    ch = std::min(ch, frame.rows);
    crop = cv::Rect((frame.cols - cw) / 2, (frame.rows - ch) / 2, cw, ch);

    cv::resize(frame(crop), out, cv::Size(params_.input_width, params_.input_height), 0, 0,
               cv::INTER_LINEAR);
}

// ==================== 解码 ====================
// 输入 (N, 28)：[x1,y1,x2,y2, cls x12, color x4, kpt x8]（坐标在模型输入图坐标系）
// 输出：图像坐标系下的检测结果（映射回 frame，即 dart_aim_node 的裁剪图，
//       这样 HSV 那套按裁剪图标定的偏移量和图像中心完全不受影响）
std::vector<TargetInfo> ModelDetector::decode(const RawOutput& raw, const cv::Rect& crop,
                                              const cv::Size& frame_size) const {
    std::vector<TargetInfo> results;
    if (raw.data == nullptr || raw.n <= 0 || raw.c < kRawChannels) {
        return results;
    }

    const int cls_off = kBoxChannels;
    const int color_off = cls_off + model_classes::kNumClasses;

    // 1) 每个候选框取最大类别分数作为置信度
    std::vector<float> conf(static_cast<size_t>(raw.n));
    std::vector<int> cls(static_cast<size_t>(raw.n));
    for (int a = 0; a < raw.n; ++a) {
        const float* p = raw.data + static_cast<size_t>(a) * raw.row_stride;
        float best = p[cls_off];
        int best_i = 0;
        for (int k = 1; k < model_classes::kNumClasses; ++k) {
            const float v = p[cls_off + k];
            if (v > best) {
                best = v;
                best_i = k;
            }
        }
        conf[static_cast<size_t>(a)] = best;
        cls[static_cast<size_t>(a)] = best_i;
    }

    // 2) 置信度过滤 + 取前 kMaxDetections
    std::vector<int> order;
    order.reserve(static_cast<size_t>(raw.n));
    for (int a = 0; a < raw.n; ++a) {
        if (conf[static_cast<size_t>(a)] >= params_.conf_threshold) {
            order.push_back(a);
        }
    }
    if (order.empty()) {
        return results;
    }
    const size_t keep = std::min<size_t>(order.size(), kMaxDetections);
    std::partial_sort(order.begin(), order.begin() + static_cast<ptrdiff_t>(keep), order.end(),
                      [&conf](int a, int b) { return conf[a] > conf[b]; });
    order.resize(keep);

    // 3) 组装 + 坐标映射回原图
    const float sx = static_cast<float>(crop.width) / params_.input_width;
    const float sy = static_cast<float>(crop.height) / params_.input_height;
    const float max_x = static_cast<float>(frame_size.width - 1);
    const float max_y = static_cast<float>(frame_size.height - 1);

    results.reserve(keep);
    for (int a : order) {
        const float* p = raw.data + static_cast<size_t>(a) * raw.row_stride;
        const float x1 = p[0] * sx + crop.x;
        const float y1 = p[1] * sy + crop.y;
        const float x2 = p[2] * sx + crop.x;
        const float y2 = p[3] * sy + crop.y;
        if (x2 <= x1 || y2 <= y1) continue;

        TargetInfo info;
        info.center = cv::Point2f(std::clamp((x1 + x2) * 0.5f, 0.0f, max_x),
                                  std::clamp((y1 + y2) * 0.5f, 0.0f, max_y));
        info.bbox = cv::Rect(cv::Point(static_cast<int>(x1), static_cast<int>(y1)),
                             cv::Point(static_cast<int>(x2), static_cast<int>(y2))) &
                    cv::Rect(0, 0, frame_size.width, frame_size.height);
        info.area = static_cast<double>(info.bbox.area());
        info.class_id = cls[static_cast<size_t>(a)];
        info.confidence = conf[static_cast<size_t>(a)];
        (void)color_off;  // 颜色分支暂不使用（比赛只区分目标种类）
        results.push_back(info);
    }

    return results;
}

// ==================== 推理线程 ====================
void ModelDetector::inferenceLoop() {
    while (!stop_.load()) {
        while (!new_frame_ready_.load() && !stop_.load()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        if (stop_.load()) break;

        cv::Mat frame;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            frame = latest_frame_;
            new_frame_ready_.store(false);
        }
        if (frame.empty()) continue;

        try {
            cv::Mat input_img;
            cv::Rect crop;
            preprocess(frame, input_img, crop);

            RawOutput raw;
            if (!backendInfer(params_.backend, input_img, raw)) {
                const uint32_t fails = infer_fail_count_.fetch_add(1) + 1;
                if (fails == 1 || fails % 20 == 0) {
                    static rclcpp::Clock steady_clock(RCL_STEADY_TIME);
                    RCLCPP_WARN_THROTTLE(modelLogger(), steady_clock, 5000,
                                         "模型推理连续失败 %u 次", fails);
                }
                continue;  // 推理失败，保留上一次结果
            }

            auto results = decode(raw, crop, frame.size());
            {
                std::lock_guard<std::mutex> lock(mutex_);
                latest_results_ = std::move(results);
            }
            last_result_ms_.store(nowMs());
            result_seq_.fetch_add(1);
            infer_fail_count_.store(0);
        } catch (const std::exception&) {
            // 推理异常，保留上一次结果
        }
    }
}

}  // namespace yq_dart_aim
