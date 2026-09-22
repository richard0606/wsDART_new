#pragma once

#include <string>
#include <vector>

#include <opencv2/core.hpp>

namespace yq_dart_aim {

// 推理后端统一接口
//
// 目前有两种实现：
//   - BPU 后端（WITH_BPU 打开时）：RDK X5 上跑 .bin，见 model_detector_bpu.cpp
//   - cv::dnn 后端（默认）：开发机上跑 .onnx，方便无板子时验证预处理/后处理
//
// 两个后端都只负责"输入预处理好的 blob -> 输出原始张量"，解码在 ModelDetector 里做，
// 保证换后端后处理逻辑完全一致。

// 模型原始输出张量（尚未解码）
//
// 布局：(1, N, C)，C = 28 = [x1,y1,x2,y2, 12类分数, 4颜色分数, 8关键点]，
// 坐标已经是 letterbox/crop 输入图坐标系下的 xyxy，无需再做 anchor 解码
struct RawOutput {
    const float* data = nullptr;  // 首行首元素
    int n = 0;                    // anchor 数（576x768 下为 9072）
    int c = 0;                    // 每 anchor 通道数（28）
    int row_stride = 0;           // 相邻 anchor 之间跨多少个 float（BPU 对齐后可能 > c）
};

// 后端初始化，成功返回 true
// raw_channels: 原始输出最后一维的通道数（本模型族固定 28），
//               用于在 (1,N,C) / (1,C,N) 两种布局之间消歧
bool backendLoad(const std::string& model_path, int raw_channels);

// 后端释放
void backendUnload();

// 执行一次推理。
// bgr_image: 已经按模型输入宽高裁剪+缩放好的 BGR 图（CV_8UC3），
//            由各后端自行转成模型需要的形式（RGB / NV12 / int8 featuremap 等）。
// 成功时 raw 指向后端内部缓冲（生命周期到下一次 infer），返回 true。
bool backendInfer(const cv::Mat& bgr_image, RawOutput& raw);

}  // namespace yq_dart_aim
