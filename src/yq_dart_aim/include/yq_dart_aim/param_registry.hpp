#pragma once

#include <string>
#include <vector>

namespace yq_dart_aim {

// 可调参数的元数据
//
// 这是"哪些参数能调、叫什么、什么范围、中文含义"的**唯一数据源**。
// 串口调参（ParamTuner）用它做校验和给 PC 端回传说明，PC 端只负责显示，
// 不再像原 web_tuner 那样在 Python 侧抄一份默认值（抄的那份早就和
// params.yaml 脱节了：曝光写 300、实际 50000）。
//
// 注意：这里只写"元数据"，默认值仍然只存在于 config/params.yaml，
// 两者不一致时以 yaml 为准（见 constants.hpp 顶部说明）。
enum class ParamType { kInt, kDouble, kBool, kString };

// 参数归属节点：dart_aim_node 本体，或 hik_camera（跨节点，走 ROS 服务代理）
enum class ParamOwner { kDartAim, kHikCamera };

struct ParamMeta {
    const char* name;       // ROS 参数名
    ParamType type;
    double min;             // 校验范围（kBool/kString 忽略）
    double max;
    const char* label;      // 中文显示名，给非技术人员看
    const char* group;      // 分组 id
    const char* desc;       // 一句话说明（怎么调）
    ParamOwner owner;
};

// 全部可调参数。顺序即 PC 端菜单里的显示顺序（按 group 分段）。
const std::vector<ParamMeta>& paramRegistry();

// 按名字查；找不到返回 nullptr
const ParamMeta* findParam(const std::string& name);

// 分组 id -> 中文名
const char* paramGroupLabel(const char* group_id);

}  // namespace yq_dart_aim
