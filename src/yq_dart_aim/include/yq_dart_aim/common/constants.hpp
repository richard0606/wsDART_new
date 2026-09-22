#pragma once

#include <chrono>
#include <cstdint>

namespace yq_dart_aim {

// 串口协议常量
constexpr uint8_t SERIAL_HEADER = 0xAA;
constexpr uint8_t SERIAL_TAIL = 0x55;

// 串口接收超时（秒）
constexpr auto SERIAL_RX_TIMEOUT = std::chrono::seconds(1);

// 目标/飞镖 ID 范围
constexpr int MAX_TARGET_ID = 2;
constexpr int MAX_DART_ID = 3;

// 默认参数值（必须与 config/params.yaml 保持一致：yaml 是唯一权威来源，
// 这里的值只在 yaml 没加载成功时兜底，不一致会导致静默用另一套参数跑）
namespace defaults {
    // HSV 阈值
    constexpr int H_MIN = 35;
    constexpr int S_MIN = 75;
    constexpr int V_MIN = 50;
    constexpr int H_MAX = 80;
    constexpr int S_MAX = 255;
    constexpr int V_MAX = 255;

    // 偏移
    constexpr double OFFSET_X = 35.0;
    constexpr double OFFSET_Y = 0.0;

    // 串口
    constexpr int BAUD_RATE = 115200;
    constexpr bool USE_SERIAL = true;

    // 图像裁剪
    constexpr int CROP_WIDTH = 1280;
    constexpr int CROP_HEIGHT = 720;

    // 形态学
    constexpr int MORPH_KERNEL_SIZE = 3;
    constexpr int MORPH_DILATE_KERNEL_SIZE = 5;

    // 目标筛选
    constexpr double MIN_AREA = 4.0;
    constexpr double MAX_AREA = 130.0;

    // 坐标计算
    constexpr double P_ERR = 8.0;
    constexpr float AIM_THRESHOLD = 1.5f;   // 连续帧瞄准判定阈值（像素）
    constexpr float AIM_DEAD_ZONE = 0.5f;   // 已瞄准后的死区（像素）

    // 调试输出
    constexpr bool PUBLISH_DEBUG_IMAGE = true;
    constexpr bool PUBLISH_COMPRESSED_MASK = true;
}

}  // namespace yq_dart_aim
