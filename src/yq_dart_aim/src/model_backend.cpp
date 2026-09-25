// 推理后端分发：运行期按 infer_backend 参数在已编译进来的后端之间切换。
//
// 每个后端的实现在独立源文件里（model_backend_bpu / _ort / _dnn.cpp），
// 是否编译由 CMake 探测依赖后决定；没编进来的后端 backendAvailable() 返回 false，
// 运行期选中它会明确报错并让节点退回 HSV，而不是加载失败后静默用错后端。
#include "yq_dart_aim/model_backend.hpp"

#include <algorithm>
#include <cctype>

namespace yq_dart_aim {

bool backendAvailable(InferBackend backend) {
    switch (backend) {
        case InferBackend::kCpu:
#ifdef DART_HAVE_ORT
            return true;
#else
            return false;
#endif
        case InferBackend::kBpu:
#ifdef DART_HAVE_BPU
            return true;
#else
            return false;
#endif
        case InferBackend::kDnn:
#ifdef DART_HAVE_DNN
            return true;
#else
            return false;
#endif
    }
    return false;
}

const char* backendName(InferBackend backend) {
    switch (backend) {
        case InferBackend::kCpu: return "CPU(ONNX Runtime)";
        case InferBackend::kBpu: return "BPU(hobot_dnn)";
        case InferBackend::kDnn: return "DNN(OpenCV)";
    }
    return "?";
}

bool parseInferBackend(const std::string& text, InferBackend& out) {
    std::string lower;
    lower.reserve(text.size());
    for (char c : text) {
        lower.push_back(static_cast<char>(std::tolower(static_cast<unsigned char>(c))));
    }
    if (lower == "cpu" || lower == "ort" || lower == "onnxruntime") {
        out = InferBackend::kCpu;
        return true;
    }
    if (lower == "bpu" || lower == "hobot_dnn" || lower == "hobot") {
        out = InferBackend::kBpu;
        return true;
    }
    if (lower == "dnn" || lower == "opencv") {
        out = InferBackend::kDnn;
        return true;
    }
    return false;
}

bool backendLoad(InferBackend backend, const std::string& model_path, int raw_channels) {
    if (!backendAvailable(backend)) return false;
    switch (backend) {
        case InferBackend::kCpu: return impl::ort::ortLoad(model_path, raw_channels);
        case InferBackend::kBpu: return impl::bpu::bpuLoad(model_path, raw_channels);
        case InferBackend::kDnn: return impl::dnn::dnnLoad(model_path, raw_channels);
    }
    return false;
}

void backendUnload(InferBackend backend) {
    if (!backendAvailable(backend)) return;
    switch (backend) {
        case InferBackend::kCpu: impl::ort::ortUnload(); break;
        case InferBackend::kBpu: impl::bpu::bpuUnload(); break;
        case InferBackend::kDnn: impl::dnn::dnnUnload(); break;
    }
}

bool backendInfer(InferBackend backend, const cv::Mat& bgr_image, RawOutput& raw) {
    if (!backendAvailable(backend)) return false;
    switch (backend) {
        case InferBackend::kCpu: return impl::ort::ortInfer(bgr_image, raw);
        case InferBackend::kBpu: return impl::bpu::bpuInfer(bgr_image, raw);
        case InferBackend::kDnn: return impl::dnn::dnnInfer(bgr_image, raw);
    }
    return false;
}

}  // namespace yq_dart_aim
