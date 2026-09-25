// 推理后端：ONNX Runtime（CPU 加速方案）
//
// 与 BPU 后端并列，编译期由 CMake 探测 onnxruntime 决定是否编进来（DART_ENABLE_ORT），
// 运行期由 infer_backend 参数选择：
//     infer_backend: CPU   -> 本文件（跑切好后的 .onnx）
//     infer_backend: BPU   -> model_backend_bpu.cpp（跑 .bin，RDK X5 部署用）
//
// 为什么需要这个后端：ROS humble / RDK X5 自带的 OpenCV 4.5.4 的 ONNX importer
// 不支持本模型的 ArgMax 节点，cv::dnn 直接加载失败（实测），所以 CPU 侧用
// ONNX Runtime 才跑得动。模型文件用「切掉后处理」的那份 .onnx。
//
// 依赖：ONNX Runtime C++ 头文件与库（官方 release 包解压后的 include/ 与 lib/），
//       编译时用 -DORT_ROOT=/path/to/onnxruntime 指定。

#include "yq_dart_aim/model_backend.hpp"

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <opencv2/dnn.hpp>  // blobFromImage
#include <rclcpp/rclcpp.hpp>

#include <onnxruntime_cxx_api.h>

namespace yq_dart_aim {
namespace {

inline rclcpp::Logger logger() { return rclcpp::get_logger("model_backend_ort"); }

std::unique_ptr<Ort::Env> g_env;
std::unique_ptr<Ort::Session> g_session;
std::unique_ptr<Ort::MemoryInfo> g_mem_info;
std::vector<std::string> g_in_names;
std::vector<std::string> g_out_names;
std::vector<const char*> g_in_names_c;
std::vector<const char*> g_out_names_c;
std::mutex g_mutex;
std::vector<float> g_out_buf;
int g_channels = 28;
bool g_loaded = false;

std::string shapeToString(const std::vector<int64_t>& s) {
    std::string out;
    for (size_t i = 0; i < s.size(); ++i) {
        out += std::to_string(s[i]);
        if (i + 1 < s.size()) out += ",";
    }
    return out;
}

}  // namespace

namespace impl {
namespace ort {

bool ortLoad(const std::string& model_path, int raw_channels) {
    g_channels = raw_channels;
    try {
        g_env = std::make_unique<Ort::Env>(ORT_LOGGING_LEVEL_WARNING, "dart_aim_model");

        Ort::SessionOptions opts;
        opts.SetIntraOpNumThreads(4);
        opts.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
        g_session = std::make_unique<Ort::Session>(*g_env, model_path.c_str(), opts);

        Ort::AllocatorWithDefaultOptions alloc;
        g_in_names.clear();
        g_out_names.clear();
        for (size_t i = 0; i < g_session->GetInputCount(); ++i) {
            auto n = g_session->GetInputNameAllocated(i, alloc);
            g_in_names.emplace_back(n.get());
        }
        for (size_t i = 0; i < g_session->GetOutputCount(); ++i) {
            auto n = g_session->GetOutputNameAllocated(i, alloc);
            g_out_names.emplace_back(n.get());
        }
        g_in_names_c.clear();
        g_out_names_c.clear();
        for (const auto& n : g_in_names) g_in_names_c.push_back(n.c_str());
        for (const auto& n : g_out_names) g_out_names_c.push_back(n.c_str());

        if (g_in_names.empty() || g_out_names.empty()) {
            RCLCPP_ERROR(logger(), "model has no input/output: %s", model_path.c_str());
            ortUnload();
            return false;
        }

        g_mem_info = std::make_unique<Ort::MemoryInfo>(
            Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault));

        // 打印真实输入/输出形状，便于上板核对
        // 注意：ORT 的 TypeInfo / TensorTypeAndShapeInfo 是非拥有视图，
        // 必须先把 TypeInfo 存下来再取 shape，否则链式调用会读到已析构对象
        for (size_t i = 0; i < g_in_names.size(); ++i) {
            auto type_info = g_session->GetInputTypeInfo(i);
            auto tensor_info = type_info.GetTensorTypeAndShapeInfo();
            RCLCPP_INFO(logger(), "input [%s] shape=[%s]", g_in_names[i].c_str(),
                        shapeToString(tensor_info.GetShape()).c_str());
        }
        for (size_t i = 0; i < g_out_names.size(); ++i) {
            auto type_info = g_session->GetOutputTypeInfo(i);
            auto tensor_info = type_info.GetTensorTypeAndShapeInfo();
            RCLCPP_INFO(logger(), "output[%s] shape=[%s]", g_out_names[i].c_str(),
                        shapeToString(tensor_info.GetShape()).c_str());
        }

        g_loaded = true;
        RCLCPP_INFO(logger(), "ONNX Runtime model loaded: %s", model_path.c_str());
        return true;
    } catch (const Ort::Exception& e) {
        RCLCPP_ERROR(logger(), "ONNX Runtime load failed: %s", e.what());
        g_loaded = false;
        return false;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(logger(), "load failed: %s", e.what());
        g_loaded = false;
        return false;
    }
}

void ortUnload() {
    std::lock_guard<std::mutex> lock(g_mutex);
    g_session.reset();
    g_mem_info.reset();
    g_env.reset();
    g_in_names.clear();
    g_out_names.clear();
    g_in_names_c.clear();
    g_out_names_c.clear();
    g_out_buf.clear();
    g_loaded = false;
}

bool ortInfer(const cv::Mat& bgr_image, RawOutput& raw) {
    if (!g_loaded || bgr_image.empty()) return false;

    std::lock_guard<std::mutex> lock(g_mutex);
    try {
        // 输入: RGB + [0,1] + NCHW float32（与模型训练时一致）
        cv::Mat blob;
        cv::dnn::blobFromImage(bgr_image, blob, 1.0 / 255.0,
                               cv::Size(bgr_image.cols, bgr_image.rows),
                               cv::Scalar(), /*swapRB=*/true, /*crop=*/false, CV_32F);

        const std::vector<int64_t> in_shape = {1, 3, bgr_image.rows, bgr_image.cols};
        size_t in_count = 1;
        for (int64_t d : in_shape) in_count *= static_cast<size_t>(d);

        auto in_tensor = Ort::Value::CreateTensor<float>(
            *g_mem_info, reinterpret_cast<float*>(blob.data), in_count, in_shape.data(),
            in_shape.size());

        auto outputs = g_session->Run(Ort::RunOptions{nullptr}, g_in_names_c.data(), &in_tensor, 1,
                                      g_out_names_c.data(), 1);
        if (outputs.empty()) return false;

        const auto oshape = outputs[0].GetTensorTypeAndShapeInfo().GetShape();
        if (oshape.size() != 3) return false;

        const float* data = outputs[0].GetTensorData<float>();
        const int64_t d1 = oshape[1];
        const int64_t d2 = oshape[2];
        int n = 0;
        bool transposed = false;
        if (d2 == g_channels) {
            n = static_cast<int>(d1);  // (1, N, C)
        } else if (d1 == g_channels) {
            n = static_cast<int>(d2);  // (1, C, N)
            transposed = true;
        } else {
            return false;
        }

        g_out_buf.resize(static_cast<size_t>(n) * g_channels);
        for (int a = 0; a < n; ++a) {
            for (int k = 0; k < g_channels; ++k) {
                g_out_buf[static_cast<size_t>(a) * g_channels + k] =
                    transposed ? data[static_cast<size_t>(k) * n + a]
                               : data[static_cast<size_t>(a) * g_channels + k];
            }
        }

        raw.data = g_out_buf.data();
        raw.n = n;
        raw.c = g_channels;
        raw.row_stride = g_channels;
        return true;
    } catch (const Ort::Exception& e) {
        RCLCPP_ERROR_ONCE(logger(), "ONNX Runtime inference failed: %s", e.what());
        return false;
    } catch (const std::exception& e) {
        RCLCPP_ERROR_ONCE(logger(), "inference failed: %s", e.what());
        return false;
    }
}

}  // namespace ort
}  // namespace impl

}  // namespace yq_dart_aim
