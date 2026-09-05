"""
参数定义（Schema）—— 所有可调参数的元数据

每个参数定义包含：
  - name:     ROS2 参数名
  - type:     数据类型 (int / float / bool / str)
  - min/max:  有效范围
  - step:     微调步长
  - default:  默认值
  - label:    中文显示名
  - group:    所属分组
  - desc:     调节说明（给机械组看的）
"""

from dataclasses import dataclass, field
from typing import Any


@dataclass
class ParamDef:
    name: str
    type: str          # "int" | "float" | "bool" | "str"
    min: float = 0
    max: float = 100
    step: float = 1
    default: Any = 0
    label: str = ""
    group: str = ""
    desc: str = ""
    target_node: str = "dart_aim_node"   # 所属 ROS2 节点名


# ============================================================
# 参数分组定义
# ============================================================

PARAM_GROUPS = [
    {
        "id": "hsv",
        "label": "HSV 颜色阈值",
        "desc": "目标颜色筛选，调节后实时生效",
        "icon": "🎨",
    },
    {
        "id": "morphology",
        "label": "形态学处理",
        "desc": "去噪和连接断裂区域",
        "icon": "🔧",
    },
    {
        "id": "target",
        "label": "目标筛选",
        "desc": "面积和形状过滤",
        "icon": "🎯",
    },
    {
        "id": "coordinate",
        "label": "坐标偏移",
        "desc": "瞄准点偏移补偿",
        "icon": "📐",
    },
    {
        "id": "camera",
        "label": "相机参数",
        "desc": "曝光和增益（hik_camera 节点）",
        "icon": "📷",
    },
    {
        "id": "model",
        "label": "模型检测",
        "desc": "ONNX 模型推理参数",
        "icon": "🤖",
    },
    {
        "id": "system",
        "label": "系统设置",
        "desc": "串口、裁剪、调试输出",
        "icon": "⚙️",
    },
]

# ============================================================
# 全部参数定义
# ============================================================

ALL_PARAMS: list[ParamDef] = [
    # ── HSV 颜色阈值 ──────────────────────────────────
    ParamDef("h_min", "int",    0, 180, 1,   35,  "色相下限",   "hsv", "增大 → 排除更多黄色/橙色区域"),
    ParamDef("h_max", "int",    0, 180, 1,   80,  "色相上限",   "hsv", "减小 → 排除更多绿色/蓝色区域"),
    ParamDef("s_min", "int",    0, 255, 1,   70,  "饱和度下限", "hsv", "增大 → 排除灰色/白色区域"),
    ParamDef("s_max", "int",    0, 255, 1,   255, "饱和度上限", "hsv", "一般保持 255"),
    ParamDef("v_min", "int",    0, 255, 1,   50,  "明度下限",   "hsv", "增大 → 排除暗色区域"),
    ParamDef("v_max", "int",    0, 255, 1,   255, "明度上限",   "hsv", "减小 → 排除过曝区域"),

    # ── 形态学处理 ────────────────────────────────────
    ParamDef("morph_kernel_size",         "int", 1, 21, 2, 3, "开运算核大小",   "morphology", "去除小噪点，必须为奇数"),
    ParamDef("morph_dilate_kernel_size",  "int", 1, 21, 2, 5, "膨胀核大小",     "morphology", "连接断裂区域，必须为奇数"),

    # ── 目标筛选 ──────────────────────────────────────
    ParamDef("max_area",    "float", 1, 1000, 1,   130.0, "最大面积阈值", "target", "排除大面积背景干扰"),
    ParamDef("circle_mask", "bool",  0, 1,    1,   True,  "圆形匹配",     "target", "启用更严格的圆形筛选"),

    # ── 坐标偏移 ──────────────────────────────────────
    ParamDef("p_err",    "float", 0.1, 50,  0.1, 8.0,  "误差缩放系数", "coordinate", "增大 → 响应更灵敏（远距离）"),
    ParamDef("offset_y", "float", -100, 100, 0.5, 0.0, "Y方向偏移",    "coordinate", "正值 → 瞄准点下移"),

    # 偏移查找表：offset_[target][dart]
    ParamDef("offset_0_0", "float", -100, 100, 0.5, 35.0, "无目标·飞镖0",  "coordinate", "无目标时的默认X偏移"),
    ParamDef("offset_0_1", "float", -100, 100, 0.5, 35.0, "无目标·飞镖1",  "coordinate", ""),
    ParamDef("offset_0_2", "float", -100, 100, 0.5, 35.0, "无目标·飞镖2",  "coordinate", ""),
    ParamDef("offset_0_3", "float", -100, 100, 0.5, 35.0, "无目标·飞镖3",  "coordinate", ""),
    ParamDef("offset_1_0", "float", -100, 100, 0.5, 35.0, "前哨站·飞镖0",  "coordinate", "前哨站X偏移"),
    ParamDef("offset_1_1", "float", -100, 100, 0.5, 35.0, "前哨站·飞镖1",  "coordinate", ""),
    ParamDef("offset_1_2", "float", -100, 100, 0.5, 35.0, "前哨站·飞镖2",  "coordinate", ""),
    ParamDef("offset_1_3", "float", -100, 100, 0.5, 35.0, "前哨站·飞镖3",  "coordinate", ""),
    ParamDef("offset_2_0", "float", -100, 100, 0.5, 12.0, "基地·飞镖0",    "coordinate", "基地X偏移"),
    ParamDef("offset_2_1", "float", -100, 100, 0.5, 29.0, "基地·飞镖1",    "coordinate", ""),
    ParamDef("offset_2_2", "float", -100, 100, 0.5, 34.0, "基地·飞镖2",    "coordinate", ""),
    ParamDef("offset_2_3", "float", -100, 100, 0.5, 30.0, "基地·飞镖3",    "coordinate", ""),

    # ── 相机参数 ──────────────────────────────────────
    ParamDef("exposure_time", "int",   1, 100000, 100, 300,  "曝光时间(μs)", "camera", "增大 → 画面更亮", target_node="hik_camera"),
    ParamDef("gain",          "float", 0, 100,    0.1, 0.0,  "增益",         "camera", "增大 → 画面更亮但噪点增多", target_node="hik_camera"),

    # ── 模型检测 ──────────────────────────────────────
    ParamDef("use_model",      "bool",  0, 1,   1,   False, "启用模型检测", "model", "HSV模式关闭，模型模式开启"),
    ParamDef("conf_threshold", "float", 0, 1,   0.01, 0.5,  "置信度阈值",   "model", "降低 → 检测更多但可能误检"),
    ParamDef("nms_threshold",  "float", 0, 1,   0.01, 0.45, "NMS阈值",      "model", "降低 → 去重更严格"),

    # ── 系统设置 ──────────────────────────────────────
    ParamDef("crop_width",               "int", 320, 1920, 16,  1024, "裁剪宽度",     "system", "处理区域宽度"),
    ParamDef("crop_height",              "int", 240, 1080, 16,  480,  "裁剪高度",     "system", "处理区域高度"),
    ParamDef("use_serial",               "bool", 0, 1,     1,   True, "启用串口",     "system", "比赛模式开启，调试模式关闭"),
    ParamDef("publish_debug_image",      "bool", 0, 1,     1,   True, "发布调试图像", "system", "关闭可节省性能"),
    ParamDef("publish_compressed",       "bool", 0, 1,     1,   True, "发布压缩图像", "system", "JPEG格式"),
    ParamDef("publish_compressed_mask",  "bool", 0, 1,     1,   True, "发布压缩mask", "system", "PNG无损格式"),
]


# ============================================================
# 辅助函数
# ============================================================

def get_params_by_group(group_id: str) -> list[ParamDef]:
    """获取指定分组的所有参数"""
    return [p for p in ALL_PARAMS if p.group == group_id]


def get_param_by_name(name: str) -> ParamDef | None:
    """按名称查找参数定义"""
    for p in ALL_PARAMS:
        if p.name == name:
            return p
    return None


def get_schema_dict() -> dict:
    """生成前端所需的完整 schema JSON"""
    groups = []
    for g in PARAM_GROUPS:
        params = get_params_by_group(g["id"])
        groups.append({
            "id": g["id"],
            "label": g["label"],
            "desc": g["desc"],
            "icon": g["icon"],
            "params": [
                {
                    "name": p.name,
                    "type": p.type,
                    "min": p.min,
                    "max": p.max,
                    "step": p.step,
                    "default": p.default,
                    "label": p.label,
                    "desc": p.desc,
                    "node": p.target_node,
                }
                for p in params
            ],
        })
    return {"groups": groups}
