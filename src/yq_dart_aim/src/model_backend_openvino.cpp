// 推理后端：OpenVINO（Intel CPU / iGPU 加速）
//
// 与 ORT 后端并列，跑同一份「切掉后处理」的 .onnx，输入/输出约定完全一致：
//   输入 [1,3,H,W] float32 RGB [0,1]
//   输出 [1,N,C] 或 [1,C,N]，C = 28 = [xyxy, 12 类, 4 颜色, 8 关键点]
// 因此两者可以共用同一套后处理，运行期通过 infer_backend 参数切换、对比实测。
//
// 设备选择：默认 CPU，可用环境变量 DART_OV_DEVICE 覆盖（CPU / GPU / AUTO）。
// 只有 CMake 探测到 OpenVINO 时本文件才会被编译（DART_HAVE_OPENVINO）。

#include "yq_dart_aim/model_backend.hpp"

#include <cstdlib>
#include <cstring>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <opencv2/dnn.hpp>  // blobFromImage
#include <rclcpp/rclcpp.hpp>

#include <openvino/openvino.hpp>

namespace yq_dart_aim {
namespace {

inline rclcpp::Logger logger() { return rclcpp::get_logger("model_backend_openvino"); }

std::unique_ptr<ov::Core> g_core;
std::shared_ptr<ov::Model> g_model;
ov::CompiledModel g_compiled;
ov::InferRequest g_request;
std::mutex g_mutex;
std::vector<float> g_out_buf;
int g_channels = 28;
int g_in_h = 0;
int g_in_w = 0;
bool g_loaded = false;

// 设备选择：默认 CPU。
// 实测 i7-1260P：GPU 单帧 10.1ms 比 CPU 4 线程 16.9ms 快，但端到端反而更慢
// （主回调 12Hz vs 27Hz）——每次推理要在内存和 iGPU 之间拷 5.3MB 输入 + 1MB 输出，
// 共享内存带宽与 DDS 大图传输互相挤占。需要时用 DART_OV_DEVICE=GPU 显式切换。
const char* deviceName() {
    const char* d = std::getenv("DART_OV_DEVICE");
    return (d != nullptr && *d != '\0') ? d : "CPU";
}

// 推理线程数：默认 4。
// 实测 i7-1260P（4P + 8E）：4 线程 16.7ms/帧，6/8 线程反而 29~34ms
// （多出的线程被调度到 E 核拖慢），12/16 线程回落到 21ms。
// 4 线程正好吃满 4 个 P 核，是这块 CPU 的最优点。可用 DART_OV_THREADS 覆盖。
int inferenceThreads() {
    const char* s = std::getenv("DART_OV_THREADS");
    if (s == nullptr || *s == '\0') return 4;
    const int n = std::atoi(s);
    return n > 0 ? n : 4;
}

std::string shapeToString(const ov::Shape& s) {
    std::string out;
    for (size_t i = 0; i < s.size(); ++i) {
        out += std::to_string(s[i]);
        if (i + 1 < s.size()) out += ",";
    }
    return out;
}

}  // namespace

namespace impl {
namespace ov_impl {

bool ovLoad(const std::string& model_path, int raw_channels) {
    g_channels = raw_channels;
    try {
        if (!g_core) {
            g_core = std::make_unique<ov::Core>();
        }
        std::string device = deviceName();

        g_model = g_core->read_model(model_path);
        auto compileWith = [&](const std::string& dev) {
            if (dev.rfind("CPU", 0) == 0) {
                const int threads = inferenceThreads();
                g_core->set_property(dev, ov::inference_num_threads(threads));
                RCLCPP_INFO(logger(), "OpenVINO CPU 推理线程数 = %d", threads);
            }
            g_compiled = g_core->compile_model(g_model, dev);
        };
        try {
            compileWith(device);
        } catch (const ov::Exception& e) {
            if (device == "CPU") throw;
            RCLCPP_WARN(logger(), "OpenVINO device=%s 不可用（%s），回退 CPU",
                        device.c_str(), e.what());
            device = "CPU";
            compileWith(device);
        }
        g_request = g_compiled.create_infer_request();

        const ov::Shape in_shape = g_request.get_input_tensor().get_shape();
        if (in_shape.size() != 4 || in_shape[0] != 1) {
            RCLCPP_ERROR(logger(), "模型输入不是 4 维 NCHW: [%s]",
                         shapeToString(in_shape).c_str());
            ovUnload();
            return false;
        }
        g_in_h = static_cast<int>(in_shape[2]);
        g_in_w = static_cast<int>(in_shape[3]);

        RCLCPP_INFO(logger(), "OpenVINO device=%s input shape=[%s]", device.c_str(),
                    shapeToString(in_shape).c_str());
        RCLCPP_INFO(logger(), "OpenVINO output shape=[%s]",
                    shapeToString(g_request.get_output_tensor().get_shape()).c_str());

        g_loaded = true;
        RCLCPP_INFO(logger(), "OpenVINO model loaded: %s", model_path.c_str());
        return true;
    } catch (const ov::Exception& e) {
        RCLCPP_ERROR(logger(), "OpenVINO load failed: %s", e.what());
        g_loaded = false;
        return false;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(logger(), "load failed: %s", e.what());
        g_loaded = false;
        return false;
    }
}

void ovUnload() {
    std::lock_guard<std::mutex> lock(g_mutex);
    // 释放顺序必须自内向外：InferRequest -> CompiledModel -> Model -> Core
    g_request = ov::InferRequest();
    g_compiled = ov::CompiledModel();
    g_model.reset();
    g_core.reset();
    g_out_buf.clear();
    g_loaded = false;
}

bool ovInfer(const cv::Mat& bgr_image, RawOutput& raw) {
    if (!g_loaded || bgr_image.empty()) return false;

    std::lock_guard<std::mutex> lock(g_mutex);
    try {
        if (bgr_image.cols != g_in_w || bgr_image.rows != g_in_h) {
            RCLCPP_ERROR_ONCE(logger(), "输入尺寸 %dx%d 与模型 %dx%d 不符（应与 ORT 后端一样先做 letterbox）",
                              bgr_image.cols, bgr_image.rows, g_in_w, g_in_h);
            return false;
        }

        // 输入: RGB + [0,1] + NCHW float32（与模型训练时一致）
        cv::Mat blob;
        cv::dnn::blobFromImage(bgr_image, blob, 1.0 / 255.0,
                               cv::Size(g_in_w, g_in_h),
                               cv::Scalar(), /*swapRB=*/true, /*crop=*/false, CV_32F);

        ov::Tensor input = g_request.get_input_tensor();
        const size_t n_elems = blob.total();
        if (input.get_size() < n_elems) {
            RCLCPP_ERROR_ONCE(logger(), "模型输入元素数 %zu 小于预处理后的 %zu",
                              input.get_size(), n_elems);
            return false;
        }
        std::memcpy(input.data<float>(), blob.ptr<float>(), n_elems * sizeof(float));

        g_request.infer();

        ov::Tensor output = g_request.get_output_tensor();
        const ov::Shape oshape = output.get_shape();
        if (oshape.size() != 3) return false;

        const float* data = output.data<float>();
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
    } catch (const ov::Exception& e) {
        RCLCPP_ERROR_ONCE(logger(), "OpenVINO inference failed: %s", e.what());
        return false;
    } catch (const std::exception& e) {
        RCLCPP_ERROR_ONCE(logger(), "inference failed: %s", e.what());
        return false;
    }
}

}  // namespace ov_impl
}  // namespace impl

}  // namespace yq_dart_aim
