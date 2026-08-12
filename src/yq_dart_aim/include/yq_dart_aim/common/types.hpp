#pragma once

#include <opencv2/core.hpp>
#include <array>
#include <cstdint>

namespace yq_dart_aim {

// 检测到的目标信息
struct TargetInfo {
    cv::Point2f center;  // 目标质心（像素坐标）
    double area;         // 轮廓面积
    double score;        // 圆形匹配评分（越小越圆）
};

// 向下位机发送的数据包（16字节）
#pragma pack(push, 1)
struct DartAimPacket {
    float err_of_pix;       // X方向像素误差
    float keep_1;           // 保留字段
    float aim_information;  // 瞄准信息 (1.0=已瞄准, 0.0=未瞄准)
    float keep_3;           // 保留字段
};
#pragma pack(pop)

// 从下位机接收的数据包
#pragma pack(push, 1)
struct VisionSendPacket {
    int enemy;            // 目标类型: 0=无目标, 1=前哨站, 2=基地
    int cur;              // 当前飞镖编号: 0,1,2,3
    float encoder_angle;  // 编码器角度
};
#pragma pack(pop)

// 偏移查找表: [目标类型][飞镖编号] -> offset_x
using OffsetLookup = std::array<std::array<double, 4>, 3>;

}  // namespace yq_dart_aim
