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
// NMS 前的候选池上限（与「后处理与板端部署-提示词.md」里的 top-k 300 对齐）
constexpr int kPreNmsTopK = 300;
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

// 两个框的 IoU（同类 NMS 用）
//
// 注意：这里刻意用「框」而不是「4 关键点四边形」做 IoU。实测（2026-10-16，
// 实况帧复算）量化模型的关键点在邻近 anchor 之间抖动很大（角点 451↔464、
// 232↔244 这样跳），同一目标的候选框 IoU 稳定在 0.78~1.0，但关键点四边形
// 包围框的 IoU 会在 0.47~1.0 之间乱跳 —— 用四边形做 NMS 会漏压同类重复框
// （实况调试图上出现两个同类别框的来源）。模型内置的 NonMaxSuppression
// 输入也是框，用框 IoU 既稳又与内置行为一致。
float rectIoU(const cv::Rect2f& a, const cv::Rect2f& b) {
    const float inter = (a & b).area();
    if (inter <= 0.0f) return 0.0f;
    const float uni = a.area() + b.area() - inter;
    return uni > 0.0f ? inter / uni : 0.0f;
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
// Letterbox：等比缩放 + 灰色(114)补边，与模型训练、量化标定的分布一致。
//
// 历史取舍记录：旧版本用的是「4:3 中心裁剪」（视野换精度）。换 CoordAtt 模型后，
// 用同一批标定图对比过两种预处理（float 模型与 .bin 都测了）：
//   - letterbox 在边界样本上置信度明显更高（例：outpost_c2 某帧 float 0.77 vs
//     裁剪 0.42；量化后 0.39 vs 0.22，后者已掉出 0.25 阈值）；
//   - .bin 的量化标定图本身就是 letterbox 预处理，推理保持一致才能对齐分布。
// 所以改为 letterbox。坐标反解用 LetterboxMap（减 pad 除 scale），检测结果仍然
// 映射回裁剪图坐标系，下游偏移表/图像中心逻辑不受影响。
void ModelDetector::preprocess(const cv::Mat& frame, cv::Mat& out, LetterboxMap& map) const {
    const double scale =
        std::min(static_cast<double>(params_.input_width) / frame.cols,
                 static_cast<double>(params_.input_height) / frame.rows);
    const int nw = std::max(1, static_cast<int>(std::lround(frame.cols * scale)));
    const int nh = std::max(1, static_cast<int>(std::lround(frame.rows * scale)));
    const int pad_x = (params_.input_width - nw) / 2;
    const int pad_y = (params_.input_height - nh) / 2;

    out = cv::Mat(params_.input_height, params_.input_width, frame.type(),
                  cv::Scalar(114, 114, 114));
    cv::Mat resized;
    cv::resize(frame, resized, cv::Size(nw, nh), 0, 0, cv::INTER_LINEAR);
    resized.copyTo(out(cv::Rect(pad_x, pad_y, nw, nh)));

    map.scale = static_cast<float>(scale);
    map.pad_x = static_cast<float>(pad_x);
    map.pad_y = static_cast<float>(pad_y);
}

// ==================== 解码 ====================
// 输入 (N, 28)：[x1,y1,x2,y2, cls x12, color x4, kpt x8]（坐标在模型输入图坐标系）
// 流程：argmax -> 阈值过滤 -> TopK-300 -> 同类框 IoU NMS -> 坐标反映射
// 输出：图像坐标系下的检测结果（映射回 frame，即 dart_aim_node 的裁剪图，
//       这样 HSV 那套按裁剪图标定的偏移量和图像中心完全不受影响）
std::vector<TargetInfo> ModelDetector::decode(const RawOutput& raw, const LetterboxMap& map,
                                              const cv::Size& frame_size) const {
    std::vector<TargetInfo> results;
    if (raw.data == nullptr || raw.n <= 0 || raw.c < kRawChannels) {
        return results;
    }

    const int cls_off = kBoxChannels;

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

    // 2) 置信度过滤 + 按置信度降序取前 kPreNmsTopK 作为 NMS 候选池
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
    const size_t pool = std::min<size_t>(order.size(), kPreNmsTopK);
    std::partial_sort(order.begin(), order.begin() + static_cast<ptrdiff_t>(pool), order.end(),
                      [&conf](int a, int b) { return conf[a] > conf[b]; });
    order.resize(pool);

    // 3) 同类内框 IoU NMS
    //    （与模型内置 NMS 同款：按框的 IoU 抑制，阈值默认 0.7；仅同类之间。
    //      详见上面 rectIoU 的注释：不用关键点四边形，量化后关键点抖动会漏压）
    std::vector<cv::Rect2f> quads(order.size());
    for (size_t i = 0; i < order.size(); ++i) {
        const float* p = raw.data + static_cast<size_t>(order[i]) * raw.row_stride;
        quads[i] = cv::Rect2f(p[0], p[1], p[2] - p[0], p[3] - p[1]);
    }
    std::vector<size_t> kept_pos;
    kept_pos.reserve(std::min<size_t>(order.size(), kMaxDetections));
    for (size_t i = 0; i < order.size() && kept_pos.size() < static_cast<size_t>(kMaxDetections);
         ++i) {
        bool suppressed = false;
        for (size_t j : kept_pos) {
            if (cls[static_cast<size_t>(order[i])] == cls[static_cast<size_t>(order[j])] &&
                rectIoU(quads[i], quads[j]) > params_.nms_threshold) {
                suppressed = true;
                break;
            }
        }
        if (!suppressed) {
            kept_pos.push_back(i);
        }
    }

    // 4) 组装 + 坐标反映射回原图（letterbox 反解：减 pad 除 scale）
    const float inv_scale = (map.scale > 0.0f) ? (1.0f / map.scale) : 1.0f;
    const float max_x = static_cast<float>(frame_size.width - 1);
    const float max_y = static_cast<float>(frame_size.height - 1);

    results.reserve(kept_pos.size());
    for (size_t idx : kept_pos) {
        const int a = order[idx];
        const float* p = raw.data + static_cast<size_t>(a) * raw.row_stride;
        const float x1 = (p[0] - map.pad_x) * inv_scale;
        const float y1 = (p[1] - map.pad_y) * inv_scale;
        const float x2 = (p[2] - map.pad_x) * inv_scale;
        const float y2 = (p[3] - map.pad_y) * inv_scale;
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
            LetterboxMap map;
            preprocess(frame, input_img, map);

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

            auto results = decode(raw, map, frame.size());
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
