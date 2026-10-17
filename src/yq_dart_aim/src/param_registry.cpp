#include "yq_dart_aim/param_registry.hpp"

#include <cstring>

namespace yq_dart_aim {

namespace {

// 分组 id 与中文名
struct GroupMeta {
    const char* id;
    const char* label;
};

const GroupMeta kGroups[] = {
    {"hsv", "HSV 颜色阈值"},
    {"morphology", "形态学处理"},
    {"target", "目标筛选"},
    {"coordinate", "瞄准与偏移"},
    {"camera", "相机参数"},
    {"model", "模型识别"},
    {"system", "系统与调试"},
};

#define DART ParamOwner::kDartAim
#define CAM ParamOwner::kHikCamera

const std::vector<ParamMeta> kParams = {
    // ── HSV 阈值（传统模式用；模型模式下这些不参与检测）──────────────
    {"h_min", ParamType::kInt, 0, 180, "色相下限", "hsv", "增大可排除更多偏黄色区域", DART},
    {"h_max", ParamType::kInt, 0, 180, "色相上限", "hsv", "减小可排除更多偏绿色区域", DART},
    {"s_min", ParamType::kInt, 0, 255, "饱和度下限", "hsv", "增大可排除灰色/白色区域", DART},
    {"s_max", ParamType::kInt, 0, 255, "饱和度上限", "hsv", "一般保持 255", DART},
    {"v_min", ParamType::kInt, 0, 255, "明度下限", "hsv", "增大可排除暗色区域", DART},
    {"v_max", ParamType::kInt, 0, 255, "明度上限", "hsv", "减小可排除过曝区域", DART},

    // ── 形态学 ──────────────────────────────────────────────────
    {"morph_kernel_size", ParamType::kInt, 1, 21, "开运算核大小", "morphology",
     "去除小噪点，必须是奇数", DART},
    {"morph_dilate_kernel_size", ParamType::kInt, 1, 21, "膨胀核大小", "morphology",
     "连接断裂区域，必须是奇数", DART},

    // ── 目标筛选 ────────────────────────────────────────────────
    {"max_area", ParamType::kDouble, 1, 1000, "最大面积阈值", "target",
     "排除大面积背景干扰", DART},

    // ── 瞄准与偏移 ──────────────────────────────────────────────
    {"p_err", ParamType::kDouble, 0.1, 50, "误差缩放系数", "coordinate",
     "增大更灵敏（适合远距离）", DART},
    {"offset_y", ParamType::kDouble, -100, 100, "上下偏移(像素)", "coordinate",
     "正值瞄准点下移", DART},
    {"offset_0_0", ParamType::kDouble, -100, 100, "未指定目标·第0镖", "coordinate", "", DART},
    {"offset_0_1", ParamType::kDouble, -100, 100, "未指定目标·第1镖", "coordinate", "", DART},
    {"offset_0_2", ParamType::kDouble, -100, 100, "未指定目标·第2镖", "coordinate", "", DART},
    {"offset_0_3", ParamType::kDouble, -100, 100, "未指定目标·第3镖", "coordinate", "", DART},
    {"offset_1_0", ParamType::kDouble, -100, 100, "前哨站·第0镖", "coordinate", "", DART},
    {"offset_1_1", ParamType::kDouble, -100, 100, "前哨站·第1镖", "coordinate", "", DART},
    {"offset_1_2", ParamType::kDouble, -100, 100, "前哨站·第2镖", "coordinate", "", DART},
    {"offset_1_3", ParamType::kDouble, -100, 100, "前哨站·第3镖", "coordinate", "", DART},
    {"offset_2_0", ParamType::kDouble, -100, 100, "基地·第0镖", "coordinate", "", DART},
    {"offset_2_1", ParamType::kDouble, -100, 100, "基地·第1镖", "coordinate", "", DART},
    {"offset_2_2", ParamType::kDouble, -100, 100, "基地·第2镖", "coordinate", "", DART},
    {"offset_2_3", ParamType::kDouble, -100, 100, "基地·第3镖", "coordinate", "", DART},

    // ── 相机（跨节点：实际由 hik_camera 节点持有）────────────────
    {"exposure_time", ParamType::kInt, 100, 200000, "曝光时间(微秒)", "camera",
     "增大画面更亮；越大帧率越低", CAM},
    {"gain", ParamType::kDouble, 0, 32, "增益", "camera",
     "增大更亮但噪点变多", CAM},

    // ── 模型识别 ────────────────────────────────────────────────
    {"use_model", ParamType::kBool, 0, 1, "启用模型识别", "model",
     "开=模型识别，关=传统HSV", DART},
    {"conf_threshold", ParamType::kDouble, 0.01, 1.0, "置信度阈值", "model",
     "调低检更多但可能误检", DART},
    {"model_nms_threshold", ParamType::kDouble, 0.1, 1.0, "同类框NMS阈值(IoU)", "model",
     "同类装甲板框重叠超过该值被抑制；0.7=与模型内置NMS一致", DART},
    {"model_input_width", ParamType::kInt, 64, 2048, "模型输入宽", "model",
     "必须与模型一致", DART},
    {"model_input_height", ParamType::kInt, 64, 2048, "模型输入高", "model",
     "必须与模型一致", DART},
    {"model_default_target", ParamType::kInt, 0, 2, "无串口提示时打哪个", "model",
     "1=前哨站 2=基地 0=不瞄准", DART},
    {"model_result_max_age_ms", ParamType::kInt, 0, 5000, "模型结果有效期(ms)", "model",
     "0=不限制；BPU可设200左右防卡死", DART},

    // ── 系统与调试 ──────────────────────────────────────────────
    {"crop_width", ParamType::kInt, 320, 1920, "处理宽度(像素)", "system",
     "画面中部截取的宽度", DART},
    {"crop_height", ParamType::kInt, 240, 1080, "处理高度(像素)", "system",
     "画面中部截取的高度", DART},
    {"use_serial", ParamType::kBool, 0, 1, "启用串口(接下位机)", "system",
     "比赛必须开", DART},
    {"publish_debug_image", ParamType::kBool, 0, 1, "发布调试图像", "system",
     "关掉可省性能", DART},
    {"publish_compressed_mask", ParamType::kBool, 0, 1, "发布压缩mask", "system",
     "仅传统模式有意义", DART},
    {"enable_record", ParamType::kBool, 0, 1, "开启录制话题", "system",
     "仅录像时开", DART},
};

#undef DART
#undef CAM

}  // namespace

const std::vector<ParamMeta>& paramRegistry() {
    return kParams;
}

const ParamMeta* findParam(const std::string& name) {
    for (const auto& p : kParams) {
        if (name == p.name) return &p;
    }
    return nullptr;
}

const char* paramGroupLabel(const char* group_id) {
    for (const auto& g : kGroups) {
        if (std::strcmp(g.id, group_id) == 0) return g.label;
    }
    return group_id;
}

}  // namespace yq_dart_aim
