#!/usr/bin/env python3
"""标定/校验集采集工具 —— 从海康相机节点订阅图像并存盘

为什么要用这种方式：存下来的图就是**比赛时那套曝光/增益/分辨率**下相机输出的原始帧，
直接拿去喂 hb_mapper 做量化标定最合适（不要用视频抽帧、也不要用 dart_aim_node 的裁剪图）。

用法
----
1) 先起相机节点（只起相机，别起 dart_aim_node）：
     ros2 launch hik_camera hik_camera.launch.py
2) 再跑本脚本：
     python3 tools/collect_dataset.py                      # 有屏幕：s 存一张 / a 自动 / q 退出
     python3 tools/collect_dataset.py --interval 0.5       # 无屏幕(ssh)：每 0.5s 自动存一张，Ctrl-C 结束
     python3 tools/collect_dataset.py --out ~/calib --max 300

    常用参数：
       --out DIR        输出目录（默认 ./dataset_<时间戳>/）
       --topic NAME     图像话题（默认 /image_raw）
       --interval SEC   自动模式间隔，0=纯手动（默认 1.0）
       --max N          存够 N 张自动退出（默认 0=不限）
       --format jpg|png 默认 jpg
       --quality N      jpg 质量（默认 95）
       --dup-threshold  与上一张的平均像素差小于该值就跳过（默认 1.5，避免站着不动存一堆重复图）
       --keep-dups      不跳过重复帧
       --timeout SEC    等第一帧的超时（默认 10）

按键（有屏幕时）
----------------
  s / 空格   立即存一张
  a          切换自动模式（按 --interval 的间隔自动存）
  d          切换"跳过重复帧"
  q / ESC    退出并写 meta.json
"""

from __future__ import annotations

import argparse
import datetime
import json
import os
import signal
import sys
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


def msg_to_bgr(msg: Image) -> np.ndarray:
    """sensor_msgs/Image -> BGR ndarray

    手写转换而不用 cv_bridge：cv_bridge 的 python 扩展是按 numpy 1.x 编译的，
    在装了 numpy 2.x 的机器上会直接抛 `_ARRAY_API not found`。这里只依赖 numpy+opencv，
    开发机和板子上都能跑。
    """
    enc = msg.encoding
    if enc in ("bgr8", "rgb8"):
        ch, dt = 3, np.uint8
    elif enc == "bgra8" or enc == "rgba8":
        ch, dt = 4, np.uint8
    elif enc == "mono8":
        ch, dt = 1, np.uint8
    elif enc == "mono16":
        ch, dt = 1, np.uint16
    else:
        raise ValueError(f"暂不支持的编码: {enc}")

    raw = np.frombuffer(msg.data, dtype=dt)
    row_elems = msg.step // np.dtype(dt).itemsize
    if raw.size < msg.height * row_elems:
        raise ValueError("图像数据长度不足")
    img = raw.reshape(msg.height, row_elems)[:, : msg.width * ch]

    if ch == 1:
        img = img.reshape(msg.height, msg.width)
        return cv2.cvtColor(img, cv2.COLOR_GRAY2BGR) if dt == np.uint8 else img
    img = img.reshape(msg.height, msg.width, ch)
    if enc == "rgb8":
        img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
    elif enc == "rgba8":
        img = cv2.cvtColor(img, cv2.COLOR_RGBA2BGR)
    elif enc == "bgra8":
        img = cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)
    return np.ascontiguousarray(img)


WINDOW = "dataset collector"


class ImageCollector(Node):
    def __init__(self, topic: str):
        super().__init__("dataset_collector")
        self.latest = None
        self.rx = 0
        self.last_encoding = ""
        self.sub = self.create_subscription(Image, topic, self._cb, qos_profile_sensor_data)

    def _cb(self, msg: Image):
        try:
            self.latest = msg_to_bgr(msg)
            self.last_encoding = msg.encoding
            self.rx += 1
        except Exception as e:  # 编码不支持等
            self.get_logger().warning(f"图像转换失败: {e}", throttle_duration_sec=5.0)


def thumb(bgr: np.ndarray) -> np.ndarray:
    """缩略灰度图，用于判断两帧是否几乎一样"""
    g = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
    return cv2.resize(g, (64, 36), interpolation=cv2.INTER_AREA).astype(np.int16)


def draw_overlay(bgr: np.ndarray, saved: int, rx: int, auto: bool, skip_dup: bool,
                 dup: bool, fps: float, out_dir: str) -> np.ndarray:
    vis = bgr.copy()
    h = vis.shape[0]
    lines = [
        f"saved {saved}   rx {rx}   {fps:4.1f} fps",
        f"[{'AUTO' if auto else 'MANUAL'}]  dup-skip {'on' if skip_dup else 'off'}"
        + ("   << DUP" if dup else ""),
        "s save | a auto | d dup | q quit",
    ]
    for i, t in enumerate(lines):
        y = 24 + i * 26
        cv2.putText(vis, t, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 4, cv2.LINE_AA)
        cv2.putText(vis, t, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 1, cv2.LINE_AA)
    cv2.putText(vis, out_dir, (10, h - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 3, cv2.LINE_AA)
    cv2.putText(vis, out_dir, (10, h - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
    return vis


def main():
    ap = argparse.ArgumentParser(description="标定/校验集采集（订阅海康相机图像）")
    ap.add_argument("--out", default=None, help="输出目录")
    ap.add_argument("--topic", default="/image_raw")
    ap.add_argument("--interval", type=float, default=1.0, help="自动模式间隔秒，0=纯手动")
    ap.add_argument("--max", type=int, default=0, help="最多存多少张，0=不限")
    ap.add_argument("--format", choices=["jpg", "png"], default="jpg")
    ap.add_argument("--quality", type=int, default=95, help="jpg 质量")
    ap.add_argument("--dup-threshold", type=float, default=1.5,
                    help="与上一张的平均像素差低于此值视为重复（0=不判断）")
    ap.add_argument("--keep-dups", action="store_true", help="不跳过重复帧")
    ap.add_argument("--timeout", type=float, default=10.0, help="等第一帧的超时秒数")
    ap.add_argument("--no-window", action="store_true", help="强制不开预览窗口")
    args = ap.parse_args()

    out_dir = args.out or f"dataset_{datetime.datetime.now():%Y%m%d_%H%M%S}"
    out_dir = os.path.abspath(os.path.expanduser(out_dir))
    os.makedirs(out_dir, exist_ok=True)

    rclpy.init()
    node = ImageCollector(args.topic)

    # 等第一帧
    t0 = time.time()
    while node.latest is None and time.time() - t0 < args.timeout:
        rclpy.spin_once(node, timeout_sec=0.1)
    if node.latest is None:
        print(f"❌ {args.timeout:.0f}s 内没收到 {args.topic} 的图像。"
              f"\n   先确认相机节点起来了：ros2 launch hik_camera hik_camera.launch.py"
              f"\n   再确认话题有数据：ros2 topic hz {args.topic}", file=sys.stderr)
        node.destroy_node()
        rclpy.shutdown()
        return 1

    h, w = node.latest.shape[:2]
    has_display = not args.no_window and bool(os.environ.get("DISPLAY"))
    if has_display:
        # DISPLAY 有值不代表 X 真能连上（ssh 透传/无权限时 imshow 会直接抛异常），
        # 所以这里先试开一个窗口，失败就自动降级成无界面模式
        try:
            cv2.namedWindow(WINDOW, cv2.WINDOW_NORMAL)
            cv2.imshow(WINDOW, node.latest)
            cv2.waitKey(1)
        except cv2.error as e:
            print(f"⚠️ 打不开预览窗口（{str(e).splitlines()[0][:80]}），转为无界面自动模式")
            has_display = False
    auto = not has_display          # 没屏幕 -> 默认自动模式
    skip_dup = not args.keep_dups
    interval = args.interval

    print(f"图像 {w}x{h} ({node.last_encoding})  话题 {args.topic}  输出 {out_dir}")
    print(f"模式: {'自动 %.2fs' % interval if auto else '手动'}  重复帧跳过: {skip_dup}")
    print("按键: s 存一张 / a 自动 / d 重复过滤 / q 退出")
    if not has_display:
        print("（无显示环境：自动模式运行，Ctrl-C 结束）")

    saved, skipped_dup = 0, 0
    prev_thumb = None
    last_auto = time.time()
    t_start = time.time()
    path = ""

    # 收到 SIGTERM（timeout / systemd stop）也要正常收尾，否则 meta.json 写不出来
    stop_flag = {"v": False}

    def _on_term(_signum, _frame):
        stop_flag["v"] = True

    signal.signal(signal.SIGTERM, _on_term)

    try:
        while rclpy.ok():
            if stop_flag["v"]:
                print("\n收到 SIGTERM，结束采集")
                break
            rclpy.spin_once(node, timeout_sec=0.01)
            frame = node.latest
            if frame is None:
                continue
            if frame.shape[0] != h or frame.shape[1] != w:
                h, w = frame.shape[:2]

            fps = node.rx / max(1e-6, time.time() - t_start)
            is_dup = False
            now = time.time()
            want_save = False

            if auto and interval > 0 and now - last_auto >= interval:
                want_save = True
                last_auto = now

            if has_display:
                disp = frame
                if disp.shape[1] > 1280:
                    s = 1280.0 / disp.shape[1]
                    disp = cv2.resize(disp, None, fx=s, fy=s)
                cv2.imshow(WINDOW, draw_overlay(disp, saved, node.rx, auto,
                                                skip_dup, False, fps, out_dir))
                key = cv2.waitKey(1) & 0xFF
                if key in (ord("q"), 27):
                    break
                elif key in (ord("s"), ord(" ")):
                    want_save = True
                    last_auto = now
                elif key == ord("a"):
                    auto = not auto
                    last_auto = now
                elif key == ord("d"):
                    skip_dup = not skip_dup

            if not want_save:
                continue

            # 重复帧过滤：和上一张太像就跳过，避免站着不动存一堆一样的图
            if skip_dup and prev_thumb is not None:
                d = float(np.abs(thumb(frame) - prev_thumb).mean())
                if d < args.dup_threshold:
                    is_dup = True
                    skipped_dup += 1
                    continue

            suffix = "jpg" if args.format == "jpg" else "png"
            path = os.path.join(out_dir, f"img_{saved + 1:04d}.{suffix}")
            if args.format == "jpg":
                ok = cv2.imwrite(path, frame, [cv2.IMWRITE_JPEG_QUALITY, args.quality])
            else:
                ok = cv2.imwrite(path, frame, [cv2.IMWRITE_PNG_COMPRESSION, 3])
            if not ok:
                print(f"⚠️ 写失败: {path}", file=sys.stderr)
                continue

            saved += 1
            prev_thumb = thumb(frame)
            if saved % 10 == 0 or args.max and saved >= args.max:
                print(f"  已存 {saved} 张（跳过重复 {skipped_dup} 张）")
            if args.max and saved >= args.max:
                print(f"已达到 --max {args.max}，结束")
                break
    except KeyboardInterrupt:
        print("\n收到 Ctrl-C，结束采集")
    finally:
        if has_display:
            cv2.destroyAllWindows()

    # 记录一份元信息，便于回溯这批数据是怎么采的
    meta = {
        "out_dir": out_dir,
        "topic": args.topic,
        "image_size": [w, h],
        "encoding": node.last_encoding,
        "format": args.format,
        "jpeg_quality": args.quality if args.format == "jpg" else None,
        "saved": saved,
        "skipped_duplicate": skipped_dup,
        "frames_received": node.rx,
        "dup_threshold": None if not skip_dup else args.dup_threshold,
        "auto_interval": interval if auto else None,
        "started_at": datetime.datetime.fromtimestamp(t_start).isoformat(timespec="seconds"),
        "finished_at": datetime.datetime.now().isoformat(timespec="seconds"),
        "camera_config": "见 src/ros2_hik_camera/config/camera_params.yaml（曝光/增益/像素格式）",
    }
    with open(os.path.join(out_dir, "meta.json"), "w", encoding="utf-8") as f:
        json.dump(meta, f, ensure_ascii=False, indent=2)

    node.destroy_node()
    rclpy.shutdown()
    print(f"\n共存 {saved} 张（收到 {node.rx} 帧，跳过重复 {skipped_dup} 张）-> {out_dir}")
    if saved == 0:
        print("⚠️ 一张都没存。手动模式需要按 s；无屏幕时用 --interval 开启自动模式。")
    elif saved < 100:
        print(f"⚠️ 只有 {saved} 张，hb_mapper 标定建议 100~300 张，再多采一些。")
    return 0


if __name__ == "__main__":
    sys.exit(main())
