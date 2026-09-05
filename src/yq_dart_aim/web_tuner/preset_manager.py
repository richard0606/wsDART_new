"""
预设配置管理器

管理 config/presets/ 目录下的预设配置文件。
支持：列出预设、加载预设、保存当前为预设、删除预设。
"""

import os
import yaml
from pathlib import Path
from typing import Any


class PresetManager:
    """预设配置管理"""

    def __init__(self, presets_dir: str | None = None):
        if presets_dir is None:
            # 默认在 web_tuner 同级的 config/presets/ 目录
            base = Path(__file__).parent.parent / "config" / "presets"
            presets_dir = str(base)

        self._dir = Path(presets_dir)
        self._dir.mkdir(parents=True, exist_ok=True)

    def list_presets(self) -> list[dict[str, str]]:
        """列出所有预设，返回 [{name, label, time}]"""
        presets = []
        for f in sorted(self._dir.glob("*.yaml")):
            try:
                with open(f, "r", encoding="utf-8") as fp:
                    data = yaml.safe_load(fp) or {}
                presets.append({
                    "name": f.stem,
                    "label": data.get("_label", f.stem),
                    "desc": data.get("_desc", ""),
                    "time": data.get("_time", ""),
                })
            except Exception:
                presets.append({"name": f.stem, "label": f.stem, "desc": "", "time": ""})
        return presets

    def load_preset(self, name: str) -> dict[str, Any] | None:
        """加载指定预设，返回参数字典"""
        path = self._dir / f"{name}.yaml"
        if not path.exists():
            return None
        try:
            with open(path, "r", encoding="utf-8") as fp:
                data = yaml.safe_load(fp) or {}
            # 去掉元数据字段
            return {k: v for k, v in data.items() if not k.startswith("_")}
        except Exception:
            return None

    def save_preset(self, name: str, params: dict[str, Any],
                    label: str = "", desc: str = "") -> bool:
        """将当前参数保存为预设"""
        from datetime import datetime
        path = self._dir / f"{name}.yaml"
        data = {
            "_label": label or name,
            "_desc": desc,
            "_time": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
        }
        data.update(params)
        try:
            with open(path, "w", encoding="utf-8") as fp:
                yaml.dump(data, fp, default_flow_style=False, allow_unicode=True)
            return True
        except Exception:
            return False

    def delete_preset(self, name: str) -> bool:
        """删除指定预设"""
        path = self._dir / f"{name}.yaml"
        if not path.exists():
            return False
        try:
            path.unlink()
            return True
        except Exception:
            return False

    def ensure_defaults(self):
        """确保内置预设存在"""
        # 比赛模式预设
        if not (self._dir / "match_day.yaml").exists():
            self.save_preset("match_day", {
                "h_min": 35, "h_max": 80,
                "s_min": 70, "s_max": 255,
                "v_min": 50, "v_max": 255,
                "morph_kernel_size": 3,
                "morph_dilate_kernel_size": 5,
                "max_area": 130.0,
                "circle_mask": True,
                "p_err": 8.0,
                "offset_y": 0.0,
                "use_serial": True,
                "publish_debug_image": True,
            }, label="比赛模式", desc="正式比赛参数配置")

        # 调试模式预设
        if not (self._dir / "debug.yaml").exists():
            self.save_preset("debug", {
                "use_serial": False,
                "publish_debug_image": True,
                "publish_compressed": True,
                "publish_compressed_mask": True,
            }, label="调试模式", desc="关闭串口，开启全部调试输出")
