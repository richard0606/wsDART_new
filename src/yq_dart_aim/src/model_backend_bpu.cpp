// 推理后端：RDK X5 BPU（hobot_dnn / hb_dnn）
//
// 只在 -DWITH_BPU=ON 时编译（见 CMakeLists.txt）。
//
// ⚠️ 本文件尚未在 RDK X5 上编译/运行验证过（开发机没装 DDK）。首次上板请先看加载日志，
//    它会打印输入/输出的 tensorType / validShape / alignedShape / stride / scale，
//    据此核对下面这些假设（哪条不对改哪里，逻辑都集中在 backendInfer）：
//
//   1) 头文件路径假定为 <hobot/dnn/hb_dnn.h> + <hobot/dnn/hb_sys.h>
//   2) 模型是「切掉后处理」的那 439 个节点，输出 (1, 9072, 28)：
//        [x1,y1,x2,y2, cls x12, color x4, kpt x8]
//      切点见 AT_NN_Detector 分析：/model.23/Transpose_output_0
//   3) 输入按 hb_mapper 的 input_type_rt 分两类，本文件都实现了：
//        nv12      : BPU 内部做 YUV->RGB（推荐，最快）
//        featuremap: 直接填 RGB 特征图（int8 或 float32）
//   4) 量化输出（int8/int16）在这里统一反量化成 float32 再交给上层解码，
//      使解码逻辑与后端无关
//   5) hbDNNInfer 的输出缓冲需要调用方先用 hbSysAllocCachedMem 分配好，
//      并把张量数组地址传进去（与官方 sample 一致）

#include "yq_dart_aim/model_backend.hpp"

#include <algorithm>
#include <cstring>
#include <mutex>
#include <string>
#include <vector>

#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>

#include <hobot/dnn/hb_dnn.h>
#include <hobot/dnn/hb_sys.h>

namespace yq_dart_aim {
namespace {

inline rclcpp::Logger logger() { return rclcpp::get_logger("model_backend_bpu"); }

// 只打一次的错误日志（避免每帧刷屏）
#define BPU_ERR_ONCE(...)                                  \
    do {                                                   \
        static bool logged_ = false;                       \
        if (!logged_) {                                    \
            logged_ = true;                                \
            RCLCPP_ERROR(logger(), __VA_ARGS__);           \
        }                                                  \
    } while (0)

hbDNNPackedHandle_t g_packed = nullptr;
hbDNNHandle_t g_model = nullptr;
std::mutex g_mutex;
int g_channels = 28;
bool g_loaded = false;

int g_in_w = 0;
int g_in_h = 0;
int g_in_nd = 0;
int g_in_y_stride = 0;  // 字节
int g_in_type = 0;      // HB_DNN_IMG_TYPE_* 或 HB_DNN_TENSOR_TYPE_*
bool g_in_is_nv12 = false;
bool g_in_is_nv12_sep = false;
float g_in_scale = 1.0f;

hbDNNTensor g_input{};
std::vector<hbDNNTensor> g_outputs;
std::vector<float> g_out_buf;

const char* typeName(int32_t t) {
    switch (t) {
        case HB_DNN_TENSOR_TYPE_F32: return "F32";
        case HB_DNN_TENSOR_TYPE_S32: return "S32";
        case HB_DNN_TENSOR_TYPE_U32: return "U32";
        case HB_DNN_TENSOR_TYPE_F16: return "F16";
        case HB_DNN_TENSOR_TYPE_S16: return "S16";
        case HB_DNN_TENSOR_TYPE_U16: return "U16";
        case HB_DNN_TENSOR_TYPE_S8:  return "S8";
        case HB_DNN_TENSOR_TYPE_U8:  return "U8";
        case HB_DNN_IMG_TYPE_Y:      return "IMG_Y";
        case HB_DNN_IMG_TYPE_NV12:   return "IMG_NV12";
        case HB_DNN_IMG_TYPE_NV12_SEPARATE: return "IMG_NV12_SEPARATE";
        case HB_DNN_IMG_TYPE_YUV444: return "IMG_YUV444";
        case HB_DNN_IMG_TYPE_RGB:    return "IMG_RGB";
        case HB_DNN_IMG_TYPE_BGR:    return "IMG_BGR";
        default: return "?";
    }
}

size_t elemSize(int32_t t) {
    switch (t) {
        case HB_DNN_TENSOR_TYPE_F32:
        case HB_DNN_TENSOR_TYPE_S32:
        case HB_DNN_TENSOR_TYPE_U32:
            return 4;
        case HB_DNN_TENSOR_TYPE_F16:
        case HB_DNN_TENSOR_TYPE_S16:
        case HB_DNN_TENSOR_TYPE_U16:
            return 2;
        default:
            return 1;
    }
}

void logTensor(const char* tag, const hbDNNTensorProperties& p) {
    std::string shape, aligned, stride;
    for (int i = 0; i < p.numDimensions; ++i) {
        shape += std::to_string(p.validShape[i]);
        aligned += std::to_string(p.alignedShape[i]);
        stride += std::to_string(p.stride[i]);
        if (i + 1 < p.numDimensions) { shape += ","; aligned += ","; stride += ","; }
    }
    RCLCPP_INFO(logger(), "%s type=%s valid=[%s] aligned=[%s] stride=[%s] scale=%f", tag,
                typeName(p.tensorType), shape.c_str(), aligned.c_str(), stride.c_str(),
                (p.scale.scaleData != nullptr) ? p.scale.scaleData[0] : 1.0f);
}

// 反量化单个元素
inline float dequant(int32_t t, float scale, const void* base, size_t idx) {
    switch (t) {
        case HB_DNN_TENSOR_TYPE_F32: return static_cast<const float*>(base)[idx];
        case HB_DNN_TENSOR_TYPE_S32: return static_cast<float>(static_cast<const int32_t*>(base)[idx]) * scale;
        case HB_DNN_TENSOR_TYPE_S16: return static_cast<float>(static_cast<const int16_t*>(base)[idx]) * scale;
        case HB_DNN_TENSOR_TYPE_U16: return static_cast<float>(static_cast<const uint16_t*>(base)[idx]) * scale;
        case HB_DNN_TENSOR_TYPE_S8:  return static_cast<float>(static_cast<const int8_t*>(base)[idx]) * scale;
        case HB_DNN_TENSOR_TYPE_U8:  return static_cast<float>(static_cast<const uint8_t*>(base)[idx]) * scale;
        default: return 0.0f;
    }
}

}  // namespace

bool backendLoad(const std::string& model_path, int raw_channels) {
    g_channels = raw_channels;

    const char* files[1] = {model_path.c_str()};
    if (hbDNNInitializeFromFiles(&g_packed, files, 1) != 0 || g_packed == nullptr) {
        RCLCPP_ERROR(logger(), "hbDNNInitializeFromFiles failed: %s", model_path.c_str());
        g_packed = nullptr;
        return false;
    }

    char const** names = nullptr;
    int32_t name_count = 0;
    if (hbDNNGetModelNameList(&names, &name_count, g_packed) != 0 || name_count < 1) {
        RCLCPP_ERROR(logger(), "hbDNNGetModelNameList failed");
        hbDNNRelease(&g_packed);
        g_packed = nullptr;
        return false;
    }
    if (hbDNNGetModelHandle(&g_model, g_packed, names[0]) != 0) {
        RCLCPP_ERROR(logger(), "hbDNNGetModelHandle failed");
        hbDNNRelease(&g_packed);
        g_packed = nullptr;
        return false;
    }

    // ── 输入 ──
    int32_t in_count = 0;
    hbDNNGetInputCount(&in_count, g_model);
    if (in_count != 1) {
        RCLCPP_ERROR(logger(), "expect 1 input, got %d", in_count);
        backendUnload();
        return false;
    }
    hbDNNTensorProperties in_props{};
    hbDNNGetInputTensorProperties(&in_props, g_model, 0);
    logTensor("input ", in_props);
    if (in_props.scale.scaleData != nullptr) g_in_scale = in_props.scale.scaleData[0];

    g_in_nd = in_props.numDimensions;
    g_in_type = in_props.tensorType;
    if (g_in_nd < 2) {
        RCLCPP_ERROR(logger(), "bad input dims: %d", g_in_nd);
        backendUnload();
        return false;
    }
    g_in_h = in_props.validShape[g_in_nd - 2];
    g_in_w = in_props.validShape[g_in_nd - 1];
    g_in_y_stride = in_props.stride[g_in_nd - 2];
    if (g_in_y_stride <= 0) g_in_y_stride = g_in_w;
    g_in_is_nv12 = (g_in_type == HB_DNN_IMG_TYPE_NV12 ||
                    g_in_type == HB_DNN_IMG_TYPE_NV12_SEPARATE);
    g_in_is_nv12_sep = (g_in_type == HB_DNN_IMG_TYPE_NV12_SEPARATE);

    if (g_in_w <= 0 || g_in_h <= 0) {
        RCLCPP_ERROR(logger(), "bad input shape %dx%d", g_in_w, g_in_h);
        backendUnload();
        return false;
    }

    // 输入缓冲
    const int y_bytes = g_in_y_stride * g_in_h;
    const int uv_bytes = g_in_y_stride * (g_in_h / 2);
    if (g_in_is_nv12_sep) {
        if (hbSysAllocCachedMem(&g_input.sysMem[0], y_bytes) != 0 ||
            hbSysAllocCachedMem(&g_input.sysMem[1], uv_bytes) != 0) {
            RCLCPP_ERROR(logger(), "alloc NV12 input failed");
            backendUnload();
            return false;
        }
    } else if (g_in_is_nv12) {
        if (hbSysAllocCachedMem(&g_input.sysMem[0], y_bytes + uv_bytes) != 0) {
            RCLCPP_ERROR(logger(), "alloc NV12 input failed");
            backendUnload();
            return false;
        }
    } else {
        size_t total = static_cast<size_t>(g_in_w) * g_in_h * 3 * elemSize(g_in_type);
        if (hbSysAllocCachedMem(&g_input.sysMem[0], static_cast<uint32_t>(total)) != 0) {
            RCLCPP_ERROR(logger(), "alloc input failed (%zu bytes)", total);
            backendUnload();
            return false;
        }
    }
    g_input.properties = in_props;

    // ── 输出（按 alignedShape 预分配，hbDNNInfer 需要现成的张量数组）──
    int32_t out_count = 0;
    hbDNNGetOutputCount(&out_count, g_model);
    if (out_count != 1) {
        RCLCPP_ERROR(logger(), "expect 1 output, got %d", out_count);
        backendUnload();
        return false;
    }
    g_outputs.resize(static_cast<size_t>(out_count));
    for (int i = 0; i < out_count; ++i) {
        hbDNNTensorProperties p{};
        hbDNNGetOutputTensorProperties(&p, g_model, i);
        logTensor("output", p);
        size_t elems = 1;
        for (int d = 0; d < p.numDimensions; ++d) elems *= static_cast<size_t>(p.alignedShape[d]);
        const size_t bytes = elems * elemSize(p.tensorType);
        if (hbSysAllocCachedMem(&g_outputs[i].sysMem[0], static_cast<uint32_t>(bytes)) != 0) {
            RCLCPP_ERROR(logger(), "alloc output failed (%zu bytes)", bytes);
            backendUnload();
            return false;
        }
        g_outputs[i].properties = p;
    }

    g_loaded = true;
    RCLCPP_INFO(logger(), "BPU model loaded: %dx%d %s", g_in_w, g_in_h,
                g_in_is_nv12 ? "nv12" : "featuremap");
    return true;
}

void backendUnload() {
    std::lock_guard<std::mutex> lock(g_mutex);
    for (auto& out : g_outputs) {
        if (out.sysMem[0].vir_addr != nullptr) hbSysFreeMem(&out.sysMem[0]);
    }
    g_outputs.clear();
    if (g_input.sysMem[0].vir_addr != nullptr) hbSysFreeMem(&g_input.sysMem[0]);
    if (g_input.sysMem[1].vir_addr != nullptr) hbSysFreeMem(&g_input.sysMem[1]);
    g_input = hbDNNTensor{};
    g_out_buf.clear();
    if (g_packed != nullptr) hbDNNRelease(&g_packed);
    g_packed = nullptr;
    g_model = nullptr;
    g_loaded = false;
}

bool backendInfer(const cv::Mat& bgr_image, RawOutput& raw) {
    if (!g_loaded || bgr_image.empty()) return false;
    if (bgr_image.cols != g_in_w || bgr_image.rows != g_in_h) {
        BPU_ERR_ONCE("input size mismatch: got %dx%d, model wants %dx%d",
                     bgr_image.cols, bgr_image.rows, g_in_w, g_in_h);
        return false;
    }

    std::lock_guard<std::mutex> lock(g_mutex);
    const int y_bytes = g_in_y_stride * g_in_h;
    const int uv_bytes = g_in_y_stride * (g_in_h / 2);

    // ── 填输入 ──
    if (g_in_is_nv12) {
        cv::Mat i420;
        cv::cvtColor(bgr_image, i420, cv::COLOR_BGR2YUV_I420);
        const uint8_t* src_y = i420.data;
        const uint8_t* src_u = src_y + static_cast<size_t>(g_in_w) * g_in_h;
        const uint8_t* src_v = src_u + static_cast<size_t>(g_in_w) * g_in_h / 4;

        uint8_t* dst_y = static_cast<uint8_t*>(g_input.sysMem[0].vir_addr);
        uint8_t* dst_uv = g_in_is_nv12_sep
                              ? static_cast<uint8_t*>(g_input.sysMem[1].vir_addr)
                              : dst_y + y_bytes;
        (void)uv_bytes;
        for (int r = 0; r < g_in_h; ++r) {
            std::memcpy(dst_y + static_cast<size_t>(r) * g_in_y_stride,
                        src_y + static_cast<size_t>(r) * g_in_w, static_cast<size_t>(g_in_w));
        }
        for (int r = 0; r < g_in_h / 2; ++r) {
            uint8_t* d = dst_uv + static_cast<size_t>(r) * g_in_y_stride;
            const uint8_t* u = src_u + static_cast<size_t>(r) * (g_in_w / 2);
            const uint8_t* v = src_v + static_cast<size_t>(r) * (g_in_w / 2);
            for (int c = 0; c < g_in_w / 2; ++c) {
                d[2 * c] = u[c];
                d[2 * c + 1] = v[c];
            }
        }
        hbSysFlushMem(&g_input.sysMem[0], HB_SYS_MEM_CACHE_CLEAN);
        if (g_in_is_nv12_sep) hbSysFlushMem(&g_input.sysMem[1], HB_SYS_MEM_CACHE_CLEAN);
    } else {
        // featuremap: RGB, NCHW, 按 tensorType 转 float / int8
        cv::Mat rgb;
        cv::cvtColor(bgr_image, rgb, cv::COLOR_BGR2RGB);
        const size_t plane = static_cast<size_t>(g_in_w) * g_in_h;
        void* dst = g_input.sysMem[0].vir_addr;
        for (int ch = 0; ch < 3; ++ch) {
            const uint8_t* src = rgb.data + plane * static_cast<size_t>(ch);
            if (g_in_type == HB_DNN_TENSOR_TYPE_F32) {
                float* d = static_cast<float*>(dst) + plane * static_cast<size_t>(ch);
                for (size_t i = 0; i < plane; ++i) d[i] = src[i] / 255.0f;
            } else {
                int8_t* d = static_cast<int8_t*>(dst) + plane * static_cast<size_t>(ch);
                for (size_t i = 0; i < plane; ++i) {
                    const float v = src[i] / 255.0f;
                    int q = static_cast<int>(v / g_in_scale + 0.5f);
                    q = std::max(-128, std::min(127, q));
                    d[i] = static_cast<int8_t>(q);
                }
            }
        }
        hbSysFlushMem(&g_input.sysMem[0], HB_SYS_MEM_CACHE_CLEAN);
    }

    // ── 推理 ──
    hbDNNTaskHandle_t task = nullptr;
    hbDNNTensor* outputs = g_outputs.data();
    if (hbDNNInfer(&task, &outputs, &g_input, g_model) != 0) {
        BPU_ERR_ONCE("hbDNNInfer failed");
        return false;
    }
    if (hbDNNWaitTaskDone(task, 1000) != 0) {
        BPU_ERR_ONCE("hbDNNWaitTaskDone timeout");
        hbDNNReleaseTask(task);
        return false;
    }
    hbDNNReleaseTask(task);

    // ── 反量化成连续 (N, C) float ──
    hbDNNTensor& out = outputs[0];
    hbSysFlushMem(&out.sysMem[0], HB_SYS_MEM_CACHE_INVALIDATE);
    const int ond = out.properties.numDimensions;
    if (ond < 2) return false;

    const int last = out.properties.validShape[ond - 1];
    const int second_last = out.properties.validShape[ond - 2];
    int n = 0;
    bool transposed = false;
    if (last == g_channels) {
        n = second_last;  // (..., N, C)
    } else if (second_last == g_channels) {
        n = last;         // (..., C, N)
        transposed = true;
    } else {
        BPU_ERR_ONCE("unexpected output shape: last=%d second_last=%d (want C=%d)",
                     last, second_last, g_channels);
        return false;
    }

    const size_t es = elemSize(out.properties.tensorType);
    const int stride_last = static_cast<int>(out.properties.stride[ond - 1] / es);
    const int stride_row = static_cast<int>(out.properties.stride[ond - 2] / es);
    const float scale = (out.properties.scale.scaleData != nullptr)
                            ? out.properties.scale.scaleData[0] : 1.0f;
    const void* base = out.sysMem[0].vir_addr;

    g_out_buf.resize(static_cast<size_t>(n) * g_channels);
    for (int a = 0; a < n; ++a) {
        for (int k = 0; k < g_channels; ++k) {
            const size_t idx = transposed
                ? static_cast<size_t>(k) * stride_row + static_cast<size_t>(a) * stride_last
                : static_cast<size_t>(a) * stride_row + static_cast<size_t>(k) * stride_last;
            g_out_buf[static_cast<size_t>(a) * g_channels + k] =
                dequant(out.properties.tensorType, scale, base, idx);
        }
    }

    raw.data = g_out_buf.data();
    raw.n = n;
    raw.c = g_channels;
    raw.row_stride = g_channels;
    return true;
}

}  // namespace yq_dart_aim
