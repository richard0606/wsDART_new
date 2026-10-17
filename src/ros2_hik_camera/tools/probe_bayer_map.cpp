// 生成「SDK 像素类型 -> OpenCV 去马赛克码」映射表，供 hik_camera_node.cpp 的
// opencvConvFor() 核对。改那张表之前请重跑本工具。
//
// 编译：
//   g++ -O2 -o probe_bayer_map probe_bayer_map.cpp \
//       -I../hikSDK/include $(pkg-config --cflags opencv4) \
//       -L../hikSDK/lib/arm64 -lMvCameraControl $(pkg-config --libs opencv4)
// 运行：
//   LD_LIBRARY_PATH=../hikSDK/lib/arm64 ./probe_bayer_map
//
// 方法：合成一张 2x2 四格各填不同数值的「探针图」，同一张图分别喂给 SDK 的
// MV_CC_ConvertPixelType 与各 OpenCV Bayer 码，读出 2x2 四格各自被判定成 B/G/R
// 中的哪一个；排列完全一致的即为正确映射。
// 不依赖现场光照，也不依赖任何命名规范的记忆。
#include "MvCameraControl.h"
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <chrono>
#include <cstdio>
#include <string>
#include <thread>
#include <vector>

namespace {

struct BayerType { unsigned value; const char* name; };

const BayerType kTypes[] = {
    {0x01080008, "BayerGR8"}, {0x01080009, "BayerRG8"},
    {0x0108000A, "BayerGB8"}, {0x0108000B, "BayerBG8"},
};
const int kCodes[] = {cv::COLOR_BayerGR2BGR, cv::COLOR_BayerRG2BGR,
                      cv::COLOR_BayerGB2BGR, cv::COLOR_BayerBG2BGR};
const char* kCodeNames[] = {"COLOR_BayerGR2BGR", "COLOR_BayerRG2BGR",
                            "COLOR_BayerGB2BGR", "COLOR_BayerBG2BGR"};

// 探针四格取值，顺序：左上 / 右上 / 左下 / 右下。差异大，便于判定归属通道
const int kProbe[4] = {10, 200, 60, 240};

// 读出 BGR 图在 2x2 四个位置上各自被判成哪个通道，拼成 4 字符串
std::string readLayout(const cv::Mat& img) {
    std::string layout;
    for (int k = 0; k < 4; ++k) {
        const cv::Vec3b c = img.at<cv::Vec3b>(24 + k / 2, 24 + k % 2);
        const int candidate[3] = {c[0], c[1], c[2]};
        int best_dist = 1 << 30;
        char best = '?';
        for (int i = 0; i < 3; ++i) {
            const int d = candidate[i] > kProbe[k] ? candidate[i] - kProbe[k]
                                                    : kProbe[k] - candidate[i];
            if (d < best_dist) { best_dist = d; best = "BGR"[i]; }
        }
        layout += best;
    }
    return layout;
}

}  // namespace

int main() {
    MV_CC_DEVICE_INFO_LIST list{};
    MV_CC_EnumDevices(MV_USB_DEVICE, &list);
    if (list.nDeviceNum == 0 || list.pDeviceInfo[0] == nullptr) {
        std::printf("没找到 USB 相机\n");
        return 1;
    }

    void* h = nullptr;
    if (MV_OK != MV_CC_CreateHandle(&h, list.pDeviceInfo[0])) {
        std::printf("CreateHandle 失败\n");
        return 1;
    }
    int ret = 0;
    for (int i = 0; i < 30 && MV_OK != (ret = MV_CC_OpenDevice(h)); ++i) {
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
    }
    if (MV_OK != ret) {
        std::printf("OpenDevice 失败 (%#x)，相机可能被占用\n", ret);
        MV_CC_DestroyHandle(&h);
        return 1;
    }

    const int W = 64, H = 64;
    std::vector<unsigned char> probe(static_cast<size_t>(W) * H);
    for (int y = 0; y < H; ++y) {
        for (int x = 0; x < W; ++x) {
            probe[static_cast<size_t>(y) * W + x] =
                static_cast<unsigned char>(kProbe[(y % 2) * 2 + (x % 2)]);
        }
    }
    const cv::Mat bayer(H, W, CV_8UC1, probe.data());

    std::printf("OpenCV 各码的 2x2 排列（顺序：左上/右上/左下/右下）:\n");
    std::string cv_layout[4];
    for (int i = 0; i < 4; ++i) {
        cv::Mat out;
        cv::demosaicing(bayer, out, kCodes[i]);
        cv_layout[i] = readLayout(out);
        std::printf("  %-22s %s\n", kCodeNames[i], cv_layout[i].c_str());
    }

    std::printf("\nSDK 像素类型 -> 应使用的 OpenCV 码:\n");
    for (const auto& t : kTypes) {
        cv::Mat out(H, W, CV_8UC3);
        MV_CC_PIXEL_CONVERT_PARAM p{};
        p.nWidth = W;
        p.nHeight = H;
        p.pSrcData = probe.data();
        p.nSrcDataLen = probe.size();
        p.enSrcPixelType = static_cast<MvGvspPixelType>(t.value);
        p.enDstPixelType = PixelType_Gvsp_BGR8_Packed;
        p.pDstBuffer = out.data;
        p.nDstBufferSize = out.total() * 3;
        if (MV_OK != MV_CC_ConvertPixelType(h, &p)) {
            std::printf("  %-12s SDK 转换失败\n", t.name);
            continue;
        }
        const std::string layout = readLayout(out);
        std::printf("  %-12s 排列=%s -> ", t.name, layout.c_str());
        bool matched = false;
        for (int i = 0; i < 4; ++i) {
            if (cv_layout[i] == layout) {
                std::printf("%s\n", kCodeNames[i]);
                matched = true;
                break;
            }
        }
        if (!matched) {
            std::printf("!! 无匹配：SDK 排列与任一 OpenCV 码都不同\n");
        }
    }

    MV_CC_CloseDevice(h);
    MV_CC_DestroyHandle(&h);
    return 0;
}