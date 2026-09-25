#pragma once

#include <string>
#include <vector>

#include <opencv2/core.hpp>

namespace yq_dart_aim {

// 推理后端
// 编译期由 CMake 决定哪些后端可用（DART_ENABLE_BPU / ORT / DNN，默认自动探测），
// 运行期由 infer_backend 参数在已编译进来的后端之间切换
enum class InferBackend {
    kCpu,  // ONNX Runtime，CPU 推理，跑切好的 .onnx
    kBpu,  // hobot_dnn，RDK X5 BPU 加速，跑量化好的 .bin
    kDnn,  // OpenCV DNN，跑 .onnx（备选，比 ONNX Runtime 慢）
};

// 该后端是否被编译进本程序（依赖没找到时为 false）
bool backendAvailable(InferBackend backend);

// 后端名字（用于日志）
const char* backendName(InferBackend backend);

// 解析 infer_backend 参数：cpu/ort -> kCpu，bpu/hobot_dnn -> kBpu，dnn/opencv -> kDnn
bool parseInferBackend(const std::string& text, InferBackend& out);

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

// 各后端的实际实现，由 model_backend_{bpu,ort,dnn}.cpp 提供。
// 只有被 CMake 编进来的后端才有定义，调用前必须先用 backendAvailable() 判断。
namespace impl {
namespace bpu {
bool bpuLoad(const std::string& model_path, int raw_channels);
void bpuUnload();
bool bpuInfer(const cv::Mat& bgr_image, RawOutput& raw);
}  // namespace bpu

namespace ort {
bool ortLoad(const std::string& model_path, int raw_channels);
void ortUnload();
bool ortInfer(const cv::Mat& bgr_image, RawOutput& raw);
}  // namespace ort

namespace dnn {
bool dnnLoad(const std::string& model_path, int raw_channels);
void dnnUnload();
bool dnnInfer(const cv::Mat& bgr_image, RawOutput& raw);
}  // namespace dnn
}  // namespace impl

// 后端分发表：按后端分发到具体实现
//
// raw_channels: 原始输出最后一维的通道数（本模型族固定 28），
//               用于在 (1,N,C) / (1,C,N) 两种布局之间消歧
bool backendLoad(InferBackend backend, const std::string& model_path, int raw_channels);
void backendUnload(InferBackend backend);
bool backendInfer(InferBackend backend, const cv::Mat& bgr_image, RawOutput& raw);

}  // namespace yq_dart_aim
