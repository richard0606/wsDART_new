// 推理后端分发：运行期按 infer_backend 参数在已编译进来的后端之间切换。
//
// 每个后端的实现在独立源文件里（model_backend_bpu / _ort / _dnn / _openvino.cpp），
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
        case InferBackend::kOpenvino:
#ifdef DART_HAVE_OPENVINO
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
        case InferBackend::kOpenvino: return "OpenVINO(Intel)";
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
    if (lower == "openvino" || lower == "ov") {
        out = InferBackend::kOpenvino;
        return true;
    }
    return false;
}

// 自动选择后端：先看编译期默认（DART_DEFAULT_INFER_BACKEND），
// 该后端不可用时按「板端优先」顺序回退：BPU -> OpenVINO -> ORT -> DNN。
// 这样同一份 params.yaml 在 RDK X5 和 x86 上都能启动。
bool chooseDefaultBackend(InferBackend& out) {
#ifdef DART_DEFAULT_INFER_BACKEND
    InferBackend preferred{};
    if (parseInferBackend(DART_DEFAULT_INFER_BACKEND, preferred) && backendAvailable(preferred)) {
        out = preferred;
        return true;
    }
#endif
    constexpr InferBackend kFallbackOrder[] = {InferBackend::kBpu, InferBackend::kOpenvino,
                                               InferBackend::kCpu, InferBackend::kDnn};
    for (InferBackend backend : kFallbackOrder) {
        if (backendAvailable(backend)) {
            out = backend;
            return true;
        }
    }
    return false;
}

bool parseInferBackendOrAuto(const std::string& text, InferBackend& out) {
    std::string lower;
    lower.reserve(text.size());
    for (char c : text) {
        lower.push_back(static_cast<char>(std::tolower(static_cast<unsigned char>(c))));
    }
    if (lower.empty() || lower == "auto" || lower == "default") {
        return chooseDefaultBackend(out);
    }
    return parseInferBackend(text, out);
}

// 每个 case 都必须用编译开关包住：后端实现文件没编进来时，
// 直接写 impl::xxx 调用会在链接期报 undefined reference（x86 无 hobot_dnn 即如此）。
bool backendLoad(InferBackend backend, const std::string& model_path, int raw_channels) {
    if (!backendAvailable(backend)) return false;
    switch (backend) {
        case InferBackend::kCpu:
#ifdef DART_HAVE_ORT
            return impl::ort::ortLoad(model_path, raw_channels);
#else
            return false;
#endif
        case InferBackend::kBpu:
#ifdef DART_HAVE_BPU
            return impl::bpu::bpuLoad(model_path, raw_channels);
#else
            return false;
#endif
        case InferBackend::kDnn:
#ifdef DART_HAVE_DNN
            return impl::dnn::dnnLoad(model_path, raw_channels);
#else
            return false;
#endif
        case InferBackend::kOpenvino:
#ifdef DART_HAVE_OPENVINO
            return impl::ov_impl::ovLoad(model_path, raw_channels);
#else
            return false;
#endif
    }
    return false;
}

void backendUnload(InferBackend backend) {
    if (!backendAvailable(backend)) return;
    switch (backend) {
        case InferBackend::kCpu:
#ifdef DART_HAVE_ORT
            impl::ort::ortUnload();
#endif
            break;
        case InferBackend::kBpu:
#ifdef DART_HAVE_BPU
            impl::bpu::bpuUnload();
#endif
            break;
        case InferBackend::kDnn:
#ifdef DART_HAVE_DNN
            impl::dnn::dnnUnload();
#endif
            break;
        case InferBackend::kOpenvino:
#ifdef DART_HAVE_OPENVINO
            impl::ov_impl::ovUnload();
#endif
            break;
    }
}

bool backendInfer(InferBackend backend, const cv::Mat& bgr_image, RawOutput& raw) {
    if (!backendAvailable(backend)) return false;
    switch (backend) {
        case InferBackend::kCpu:
#ifdef DART_HAVE_ORT
            return impl::ort::ortInfer(bgr_image, raw);
#else
            return false;
#endif
        case InferBackend::kBpu:
#ifdef DART_HAVE_BPU
            return impl::bpu::bpuInfer(bgr_image, raw);
#else
            return false;
#endif
        case InferBackend::kDnn:
#ifdef DART_HAVE_DNN
            return impl::dnn::dnnInfer(bgr_image, raw);
#else
            return false;
#endif
        case InferBackend::kOpenvino:
#ifdef DART_HAVE_OPENVINO
            return impl::ov_impl::ovInfer(bgr_image, raw);
#else
            return false;
#endif
    }
    return false;
}

}  // namespace yq_dart_aim
