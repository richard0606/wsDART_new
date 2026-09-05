#!/usr/bin/env python3
"""飞镖调参系统 - 启动入口

用法:
  python3 server.py [--host 0.0.0.0] [--port 8080] [--no-image]
  python3 -m web_tuner.server  (从 web_tuner 目录的上级运行)
"""

import sys
import os

# 确保 web_tuner 的父目录在 Python 路径中（支持直接运行）
_parent = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _parent not in sys.path:
    sys.path.insert(0, _parent)

# 现在可以用绝对导入
from web_tuner.param_schema import ALL_PARAMS, PARAM_GROUPS, get_schema_dict, get_param_by_name
from web_tuner.param_manager import ParamManager
from web_tuner.preset_manager import PresetManager

import argparse
import base64
import threading
import time
from typing import Any

from flask import Flask, jsonify, request, send_from_directory
from flask_socketio import SocketIO, emit

# ── 可选：ROS2 图像订阅 ──────────────────────────────
try:
    import rclpy
    from sensor_msgs.msg import CompressedImage
    _HAS_ROS2 = True
except ImportError:
    _HAS_ROS2 = False


app = Flask(__name__, static_folder="static", static_url_path="")
socketio = SocketIO(app, cors_allowed_origins="*", async_mode="threading")

# 全局实例
param_mgr: ParamManager | None = None
preset_mgr: PresetManager | None = None
_image_sub_node = None
_latest_images: dict[str, str] = {}   # topic -> base64
_image_lock = threading.Lock()


# ============================================================
# REST API
# ============================================================

@app.route("/")
def index():
    return send_from_directory(app.static_folder, "index.html")


@app.route("/api/schema")
def api_schema():
    """获取参数 schema（分组、类型、范围等）"""
    return jsonify(get_schema_dict())


@app.route("/api/params")
def api_get_params():
    """获取所有参数的当前值"""
    result = {}
    for p in ALL_PARAMS:
        val = param_mgr.get_cached_value(p.target_node, p.name)
        if val is None:
            val = p.default
        result[p.name] = val
    return jsonify(result)


@app.route("/api/params", methods=["POST"])
def api_set_params():
    """批量设置参数"""
    data = request.get_json(force=True)
    results = {}
    # 按节点分组
    by_node: dict[str, dict[str, Any]] = {}
    for name, value in data.items():
        pd = get_param_by_name(name)
        if pd is None:
            results[name] = False
            continue
        node = pd.target_node
        if node not in by_node:
            by_node[node] = {}
        by_node[node][name] = value

    for node, params in by_node.items():
        r = param_mgr.set_params_batch(node, params)
        results.update(r)

    return jsonify(results)


@app.route("/api/params/save", methods=["POST"])
def api_save_params():
    """将当前参数保存到 params.yaml"""
    import yaml
    from pathlib import Path

    # 读取当前参数值
    current = {}
    for p in ALL_PARAMS:
        if p.target_node == "dart_aim_node":
            val = param_mgr.get_cached_value(p.target_node, p.name)
            if val is not None:
                current[p.name] = val

    # 读取现有 yaml
    yaml_path = Path(__file__).parent.parent / "config" / "params.yaml"
    try:
        with open(yaml_path, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f) or {}
    except Exception:
        data = {}

    # 更新参数
    if "dart_aim_node" not in data:
        data["dart_aim_node"] = {}
    if "ros__parameters" not in data["dart_aim_node"]:
        data["dart_aim_node"]["ros__parameters"] = {}

    data["dart_aim_node"]["ros__parameters"].update(current)

    # 写回
    try:
        with open(yaml_path, "w", encoding="utf-8") as f:
            yaml.dump(data, f, default_flow_style=False, allow_unicode=True)
        return jsonify({"success": True})
    except Exception as e:
        return jsonify({"success": False, "error": str(e)})


# ── 预设管理 ──────────────────────────────────────────

@app.route("/api/presets")
def api_list_presets():
    return jsonify(preset_mgr.list_presets())


@app.route("/api/presets/load", methods=["POST"])
def api_load_preset():
    data = request.get_json(force=True)
    name = data.get("name", "")
    params = preset_mgr.load_preset(name)
    if params is None:
        return jsonify({"success": False, "error": "预设不存在"})

    # 应用参数
    by_node: dict[str, dict[str, Any]] = {}
    for k, v in params.items():
        pd = get_param_by_name(k)
        if pd:
            node = pd.target_node
            if node not in by_node:
                by_node[node] = {}
            by_node[node][k] = v

    results = {}
    for node, p in by_node.items():
        r = param_mgr.set_params_batch(node, p)
        results.update(r)

    return jsonify({"success": True, "applied": results})


@app.route("/api/presets/save", methods=["POST"])
def api_save_preset():
    data = request.get_json(force=True)
    name = data.get("name", "")
    label = data.get("label", "")
    desc = data.get("desc", "")

    if not name:
        return jsonify({"success": False, "error": "名称不能为空"})

    # 收集当前参数
    current = {}
    for p in ALL_PARAMS:
        val = param_mgr.get_cached_value(p.target_node, p.name)
        if val is not None:
            current[p.name] = val

    ok = preset_mgr.save_preset(name, current, label, desc)
    return jsonify({"success": ok})


@app.route("/api/presets/delete", methods=["POST"])
def api_delete_preset():
    data = request.get_json(force=True)
    name = data.get("name", "")
    ok = preset_mgr.delete_preset(name)
    return jsonify({"success": ok})


# ============================================================
# WebSocket 事件
# ============================================================

@socketio.on("connect")
def on_connect():
    emit("connected", {"status": "ok"})


@socketio.on("set_param")
def on_set_param(data):
    """客户端设置单个参数"""
    name = data.get("name")
    value = data.get("value")
    pd = get_param_by_name(name)
    if pd is None:
        emit("param_error", {"name": name, "error": "未知参数"})
        return

    ok = param_mgr.set_param(pd.target_node, name, value)
    if ok:
        emit("param_changed", {"name": name, "value": value}, broadcast=True)
    else:
        emit("param_error", {"name": name, "error": "设置失败"})


@socketio.on("get_param")
def on_get_param(data):
    """客户端请求单个参数值"""
    name = data.get("name")
    pd = get_param_by_name(name)
    if pd is None:
        return
    val = param_mgr.get_param(pd.target_node, name)
    if val is None:
        val = pd.default
    emit("param_value", {"name": name, "value": val})


@socketio.on("subscribe_image")
def on_subscribe_image(data):
    """客户端订阅图像流"""
    topic = data.get("topic", "/dart_debug/image_compressed")
    with _image_lock:
        frame = _latest_images.get(topic)
    if frame:
        emit("image_frame", {"topic": topic, "data": frame})


# ============================================================
# ROS2 图像订阅（可选）
# ============================================================

def _start_image_subscriber(topics: list[str]):
    """启动 ROS2 图像订阅线程"""
    if not _HAS_ROS2:
        return

    def _run():
        global _image_sub_node
        try:
            rclpy.init(args=None)
        except RuntimeError:
            pass

        _image_sub_node = rclpy.create_node("_web_tuner_image_sub")
        executor = rclpy.executors.MultiThreadedExecutor()
        executor.add_node(_image_sub_node)

        def make_callback(topic):
            def cb(msg: CompressedImage):
                b64 = base64.b64encode(msg.data).decode("ascii")
                with _image_lock:
                    _latest_images[topic] = b64
                socketio.emit("image_frame", {"topic": topic, "data": b64})
            return cb

        for topic in topics:
            _image_sub_node.create_subscription(
                CompressedImage, topic, make_callback(topic), 1
            )

        while rclpy.ok():
            executor.spin_once(timeout_sec=0.05)

    t = threading.Thread(target=_run, daemon=True)
    t.start()


# ============================================================
# 启动入口
# ============================================================

def main():
    parser = argparse.ArgumentParser(description="飞镖调参 Web 服务器")
    parser.add_argument("--host", default="0.0.0.0", help="监听地址")
    parser.add_argument("--port", type=int, default=8080, help="监听端口")
    parser.add_argument("--presets-dir", default=None, help="预设目录路径")
    parser.add_argument("--no-image", action="store_true", help="不订阅图像流")
    args = parser.parse_args()

    global param_mgr, preset_mgr

    # 初始化预设管理器
    preset_mgr = PresetManager(args.presets_dir)
    preset_mgr.ensure_defaults()

    # 初始化参数管理器
    def on_changed(node, name, value):
        socketio.emit("param_changed", {"name": name, "value": value})

    param_mgr = ParamManager(on_param_changed=on_changed)
    param_mgr.start()

    # 等待 ROS2 节点就绪
    time.sleep(1.0)

    # 预读所有参数到缓存
    print("正在读取参数...")
    for p in ALL_PARAMS:
        val = param_mgr.get_param(p.target_node, p.name)
        if val is not None:
            print(f"  {p.name} = {val}")
    print("参数读取完成")

    # 启动图像订阅
    if not args.no_image:
        _start_image_subscriber([
            "/dart_debug/image_compressed",
            "/dart_debug/mask_compressed",
        ])

    # 启动 Web 服务器
    print(f"\n{'='*50}")
    print(f"  飞镖调参系统已启动")
    print(f"  访问地址: http://<本机IP>:{args.port}")
    print(f"{'='*50}\n")

    socketio.run(app, host=args.host, port=args.port, allow_unsafe_werkzeug=True)


if __name__ == "__main__":
    main()
