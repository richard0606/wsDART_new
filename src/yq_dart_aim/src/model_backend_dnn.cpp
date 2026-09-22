// 推理后端：OpenCV DNN（开发机/无 BPU 时使用，跑切好的 .onnx）
//
// 用途：在没有 RDK X5 的机器上验证预处理、解码、坐标映射是否正确，
//       也让整个包在这些机器上能正常编译运行。
#include "yq_dart_aim/model_backend.hpp"

#include <algorithm>
#include <mutex>
#include <vector>

#include <opencv2/dnn.hpp>

namespace yq_dart_aim {
namespace {

cv::dnn::Net g_net;
std::mutex g_mutex;
std::vector<float> g_buf;  // 保存一份连续拷贝，保证 RawOutput.data 在下一次 infer 前有效
int g_channels = 0;
bool g_loaded = false;

}  // namespace

bool backendLoad(const std::string& model_path, int raw_channels) {
    try {
        g_net = cv::dnn::readNetFromONNX(model_path);
        g_net.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
        g_net.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
    } catch (const std::exception&) {
        g_loaded = false;
        return false;
    }
    g_channels = raw_channels;
    g_loaded = true;
    return true;
}

void backendUnload() {
    std::lock_guard<std::mutex> lock(g_mutex);
    g_net = cv::dnn::Net();
    g_buf.clear();
    g_loaded = false;
}

bool backendInfer(const cv::Mat& bgr_image, RawOutput& raw) {
    if (!g_loaded || bgr_image.empty()) return false;

    std::lock_guard<std::mutex> lock(g_mutex);
    cv::Mat out;
    try {
        // 模型要求 RGB + [0,1] + NCHW；图已经在 ModelDetector::preprocess 里
        // 裁好并缩放到模型输入尺寸，这里只做颜色/归一化/排布
        cv::Mat blob;
        cv::dnn::blobFromImage(bgr_image, blob, 1.0 / 255.0,
                               cv::Size(bgr_image.cols, bgr_image.rows),
                               cv::Scalar(), /*swapRB=*/true, /*crop=*/false, CV_32F);
        g_net.setInput(blob);
        out = g_net.forward();
    } catch (const std::exception&) {
        return false;
    }
    if (out.empty() || out.dims != 3) return false;

    const int c = g_channels;
    const size_t c_size = static_cast<size_t>(c);

    if (out.size[2] == c) {
        // (1, N, C) 已经是我们要的布局，直接拷成连续内存
        const int n = out.size[1];
        g_buf.assign(out.ptr<float>(), out.ptr<float>() + static_cast<size_t>(n) * c_size);
        raw.data = g_buf.data();
        raw.n = n;
        raw.c = c;
        raw.row_stride = c;
        return true;
    }
    if (out.size[1] == c) {
        // (1, C, N) 需要转置成 (N, C)
        const int n = out.size[2];
        g_buf.resize(static_cast<size_t>(n) * c_size);
        const float* src = out.ptr<float>();
        for (int a = 0; a < n; ++a) {
            for (int k = 0; k < c; ++k) {
                g_buf[static_cast<size_t>(a) * c_size + k] = src[static_cast<size_t>(k) * n + a];
            }
        }
        raw.data = g_buf.data();
        raw.n = n;
        raw.c = c;
        raw.row_stride = c;
        return true;
    }
    return false;
}

}  // namespace yq_dart_aim
