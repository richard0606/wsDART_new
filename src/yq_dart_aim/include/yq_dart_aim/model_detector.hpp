#pragma once

#include <vector>
#include <string>
#include <thread>
#include <mutex>
#include <atomic>
#include <opencv2/core.hpp>
#include <opencv2/dnn.hpp>
#include "common/types.hpp"
#include "detector_base.hpp"

namespace yq_dart_aim {

// 模型推理参数
struct ModelParams {
    std::string model_path;          // 模型文件路径
    float conf_threshold = 0.5f;     // 置信度阈值
    float nms_threshold = 0.45f;     // NMS 阈值
    int input_width = 640;           // 模型输入宽度
    int input_height = 640;          // 模型输入高度
};

// 模型检测器（异步推理）
class ModelDetector : public DetectorBase {
public:
    ModelDetector();
    ~ModelDetector() override;

    // DetectorBase 接口
    void submit(const cv::Mat& image) override;
    std::vector<TargetInfo> getResults() override;
    bool isReady() const override;
    std::string name() const override { return "model"; }

    // 加载模型
    bool loadModel(const ModelParams& params);

    // 设置类别名称（用于日志/调试）
    void setClassNames(const std::vector<std::string>& names);

private:
    void inferenceLoop();

    // 后处理（子类可覆盖以适配不同模型输出格式）
    std::vector<TargetInfo> postprocess(const cv::Mat& output, int img_w, int img_h);

    // NMS
    std::vector<TargetInfo> nms(std::vector<TargetInfo>& detections, float threshold);

    // 模型推理引擎（通用接口，后续可替换为 BPU 等后端）
    cv::dnn::Net net_;

    ModelParams params_;
    std::vector<std::string> class_names_;

    // 异步推理
    std::thread infer_thread_;
    std::mutex mutex_;
    std::atomic<bool> stop_{false};
    std::atomic<bool> new_frame_ready_{false};
    cv::Mat latest_frame_;
    std::vector<TargetInfo> latest_results_;

    bool model_loaded_ = false;
};

}  // namespace yq_dart_aim
