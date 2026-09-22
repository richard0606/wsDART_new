#pragma once

#include <vector>
#include <string>
#include <thread>
#include <mutex>
#include <atomic>
#include <opencv2/core.hpp>
#include "common/types.hpp"
#include "detector_base.hpp"
#include "model_backend.hpp"

namespace yq_dart_aim {

// ==================== 模型规格 ====================
// 对应 AT_NN_Detector 的 praysky_c2psa_e2e 系列（YOLO26n-Pose 装甲板检测）
//   输入: images, 1x3xHxW, RGB, 归一化 [0,1]
//   输出: (1, N, 28) = [x1,y1,x2,y2, cls x12, color x4, kpt x8]
// 注意：模型自带 NMS/TopK 后处理，但要部署到 RDK X5 BPU 必须把它切掉
// （TopK/GatherElements 等算子 BPU 不支持），切点即 /model.23/Transpose_output_0，
// 剩下对 (1,N,28) 的解码由本类在 CPU 上完成，实测与模型原生输出逐位一致。
struct ModelParams {
    std::string model_path;          // BPU: .bin；开发机(cv::dnn): .onnx（切好的版本）
    float conf_threshold = 0.25f;    // 置信度阈值
    int input_width = 768;           // 模型输入宽（576x768 型号）
    int input_height = 576;          // 模型输入高
};

// ==================== 类别表 ====================
// 来源：AT_NN_Detector/README.md（s=装甲尺寸, o=目标类型）
namespace model_classes {
    constexpr int kNumClasses = 12;
    inline const char* const kNames[kNumClasses] = {
        "s0_o0", "s0_o2", "s0_o3", "s0_o4", "s0_o5", "s0_o6",
        "s1_o0", "s1_o1", "s1_o3", "s1_o4", "s1_o5", "s1_o7",
    };

    // 飞镖任务只关心两类的映射：下位机的"打哪个目标" -> 模型类别 id
    //   1 = 前哨站 -> s0_o6 (id 5，前哨站只有小装甲)
    //   2 = 基地   -> s1_o7 (id 11，基地只有大装甲)
    constexpr int kOutpostClassId = 5;
    constexpr int kBaseClassId = 11;

    // 返回 -1 表示该目标号不需要瞄准（0=无目标 或 非法值）
    inline int classIdForTarget(int enemy) {
        if (enemy == 1) return kOutpostClassId;
        if (enemy == 2) return kBaseClassId;
        return -1;
    }
}  // namespace model_classes

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

    // 加载模型（BPU 的 .bin 或开发机的 .onnx）
    bool loadModel(const ModelParams& params);

    // 下位机目标号 -> 模型类别 id（-1 = 不瞄准）
    static int classIdForTarget(int enemy) { return model_classes::classIdForTarget(enemy); }

    // ── 以下两个是纯函数，公开出来便于离线单测（不依赖后端与线程）──

    // 预处理：按模型输入宽高比做中心裁剪（不填充）-> resize 到模型输入尺寸
    // crop 返回裁剪区域在原图中的位置，供解码时把坐标映射回去
    void preprocess(const cv::Mat& frame, cv::Mat& out, cv::Rect& crop) const;

    // 解码：对 (N,28) 原始张量做 argmax/TopK-30/组装，坐标映射回 frame_size 坐标系
    std::vector<TargetInfo> decode(const RawOutput& raw, const cv::Rect& crop,
                                   const cv::Size& frame_size) const;

private:
    void inferenceLoop();

    ModelParams params_;

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
