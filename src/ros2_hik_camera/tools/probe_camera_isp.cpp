// 实验：能不能让相机自己输出 BGR8（把去马赛克+上色还给相机 ISP）
//
// 背景：Hikrobot MV-CU013-A0UC 默认只输出 BayerGB8，SDK 在 X5 的 CPU 上做
// Bayer->BGR8 去马赛克，那个节点一度占 68% CPU。
// 导出的 GenICam XML 显示 BGR8Packed 的可用条件是 Is_Just_Bayer8_Ava，
// 而 Is_Just_Bayer8_Ava = (INQ && ADCBitDepth==0) ? 0 : 1。
// 也就是说 ADCBitDepth 这个寄存器是开关。这里逐个试，看能不能打开 BGR8。
#include "MvCameraControl.h"
#include <cstdio>
#include <string>
#include <vector>

static const unsigned int BGR8 = 0x02180015;  // PixelType_Gvsp_BGR8_Packed
static const unsigned int RGB8 = 0x02180014;

static void dumpFormats(void *h, const char *tag) {
    MVCC_ENUMVALUE pf{};
    if (MV_OK != MV_CC_GetPixelFormat(h, &pf)) { std::printf("  %s: GetPixelFormat 失败\n", tag); return; }
    std::printf("  %s: cur=0x%08x  共%u项: ", tag, pf.nCurValue, pf.nSupportedNum);
    bool has_bgr = false;
    for (unsigned i = 0; i < pf.nSupportedNum; ++i) {
        std::printf("0x%08x ", pf.nSupportValue[i]);
        if (pf.nSupportValue[i] == BGR8) has_bgr = true;
    }
    std::printf("\n     -> BGR8 %s\n", has_bgr ? "已出现！" : "仍然没有");
}

static int trySetDword(void *h, const char *key, unsigned int val) {
    return MV_CC_SetEnumValue(h, key, val);
}

int main() {
    MV_CC_DEVICE_INFO_LIST list{};
    MV_CC_EnumDevices(MV_USB_DEVICE, &list);
    if (list.nDeviceNum == 0 || list.pDeviceInfo[0] == nullptr) { std::printf("没找到相机\n"); return 1; }

    void *h = nullptr;
    if (MV_OK != MV_CC_CreateHandle(&h, list.pDeviceInfo[0])) { std::printf("CreateHandle 失败\n"); return 1; }
    if (MV_OK != MV_CC_OpenDevice(h)) { std::printf("OpenDevice 失败\n"); return 1; }
    std::printf("相机: %s\n\n", list.pDeviceInfo[0]->SpecialInfo.stUsb3VInfo.chModelName);

    std::printf("== 初始状态 ==\n");
    MVCC_INTVALUE depth_i{};
    if (MV_OK == MV_CC_GetIntValue(h, "ADCBitDepth", &depth_i)) {
        std::printf("  ADCBitDepth(Int) = %u  范围[%u..%u]\n",
                    depth_i.nCurValue, depth_i.nMin, depth_i.nMax);
    } else {
        std::printf("  ADCBitDepth 不是 Int 节点（GetIntValue 失败）\n");
    }
    MVCC_ENUMVALUE depth_e{};
    if (MV_OK == MV_CC_GetEnumValue(h, "ADCBitDepth", &depth_e)) {
        std::printf("  ADCBitDepth(Enum) cur=0x%x 支持%u项:", depth_e.nCurValue, depth_e.nSupportedNum);
        for (unsigned i = 0; i < depth_e.nSupportedNum && i < 8; ++i)
            std::printf(" %u", depth_e.nSupportValue[i]);
        std::printf("\n");
    }
    dumpFormats(h, "初始");

    // 逐个试 ADCBitDepth，看哪个能让 BGR8 出现
    std::printf("\n== 试 ADCBitDepth 各取值 ==\n");
    for (unsigned v : {8u, 10u, 12u, 14u, 16u}) {
        const int r = trySetDword(h, "ADCBitDepth", v);
        if (MV_OK != r) {
            MVCC_INTVALUE iv{};
            (void)iv;
            const int r2 = MV_CC_SetIntValue(h, "ADCBitDepth", static_cast<int>(v));
            std::printf("  设为 %u: Enum 失败(%#x), Int 返回 %#x\n", v, r, r2);
            if (MV_OK != r2) continue;
        }
        MVCC_INTVALUE now{};
        unsigned shown = v;
        if (MV_OK == MV_CC_GetIntValue(h, "ADCBitDepth", &now)) shown = now.nCurValue;
        std::printf("  设为 %u (读回 %u):\n", v, shown);
        dumpFormats(h, "  格式列表");
    }

    // 不改 ADCBitDepth，直接硬试设 BGR8
    std::printf("\n== 直接硬试设 BGR8/RGB8 ==\n");
    int r = MV_CC_SetPixelFormat(h, BGR8);
    std::printf("  SetPixelFormat(BGR8) = %#x\n", r);
    dumpFormats(h, "  设后");
    r = MV_CC_SetPixelFormat(h, RGB8);
    std::printf("  SetPixelFormat(RGB8) = %#x\n", r);
    dumpFormats(h, "  设后");

    MV_CC_CloseDevice(h);
    MV_CC_DestroyHandle(&h);
    return 0;
}
