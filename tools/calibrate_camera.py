#!/usr/bin/env python3
"""相机内参标定工具 —— 从相机话题采图 + 棋盘格标定 + 生成 ROS camera_info.yaml

标定板：黑白棋盘格（默认按 14x9 内角点、20mm 格长），请先确认板上"内角点"的个数：
  - 若板上格子的数量是 14 列 x 9 行 -> 内角点是 13x8，用 --cols 13 --rows 8
  - 若厂家标注的 14x9 指内角点        -> 直接用默认值 --cols 14 --rows 9

用法
----
1) 起相机节点（只起相机，别起 dart_aim_node，避免白白吃 CPU）：
     ros2 launch hik_camera hik_camera.launch.py
2) 有屏幕（推荐，能看着取景）：
     python3 tools/calibrate_camera.py
   快捷键: s 存一张 / a 自动连拍 / d 切换显示检测结果 / c 开始标定 / q 退出
3) 无屏幕(ssh)：
     python3 tools/calibrate_camera.py --interval 1.0
     python3 tools/calibrate_camera.py --min-samples 20
4) 标定完成后自动写入 src/ros2_hik_camera/config/camera_info.yaml（原文件备份为 .bak）

取景要求（直接影响标定质量）
----
  - 棋盘格要平整，打印建议贴在硬质平板上；格子四角不能翘
  - 覆盖整个画面：中心、四角、四边都要有，姿态要有平/斜/远近变化
  - 建议 25~35 张，姿态越多样越准；每张只动一个角度
  - 避免棋盘格反光过曝（画面整体发白会把白格和黑格糊在一起）

常用参数
--------
  --cols N --rows N   内角点列数/行数（注意是内角点，不是格子数）
  --square MM         格长（毫米），默认 20
  --out DIR           采样图片输出目录，默认 ./calib_<时间戳>/
  --min-samples N     少于 N 张不标定，默认 15
  --interval SEC      自动模式间隔，0=纯手动
  --target FILE       标定结果写到这个 camera_info.yaml（默认项目里的那个）
  --no-write          只算不写文件
  --fisheye           额外用鱼眼模型标一次并对比误差（镜头是鱼眼时用）
"""

from __future__ import annotations

import argparse
import datetime
import glob
import json
import os
import shutil
import sys
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

WINDOW = "calibrate"


# ---------------------------------------------------------------- 图像解码
def msg_to_gray(msg: Image) -> np.ndarray:
    """sensor_msgs/Image -> 灰度图。手写转换，不依赖 cv_bridge 的 numpy ABI。"""
    if msg.encoding not in ("bgr8", "rgb8", "mono8"):
        raise ValueError(f"暂不支持的编码: {msg.encoding}")
    ch = 1 if msg.encoding == "mono8" else 3
    dt = np.uint8
    row_elems = msg.step // np.dtype(dt).itemsize
    raw = np.frombuffer(msg.data, dtype=dt)
    if raw.size < msg.height * row_elems:
        raise ValueError("图像数据长度不足")
    img = raw.reshape(msg.height, row_elems)[:, : msg.width * ch]
    if ch == 3:
        img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    return np.ascontiguousarray(img)


def detect_corners(gray: np.ndarray, cols: int, rows: int):
    """找棋盘格角点，失败返回 None

    优先用 findChessboardCornersSB（opencv>=4.7）：大棋盘格/大分辨率下比经典算法稳得多，
    对光照不均也更好。检测不到再退回经典算法。
    """
    sb = getattr(cv2, "findChessboardCornersSB", None)
    if sb is not None:
        flags_sb = cv2.CALIB_CB_EXHAUSTIVE | cv2.CALIB_CB_ACCURACY
        try:
            found, corners = sb(gray, (cols, rows), flags=flags_sb)
            if found:
                return corners
        except cv2.error:
            pass

    flags = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE
    found, corners = cv2.findChessboardCorners(gray, (cols, rows), flags=flags)
    if not found:
        found, corners = cv2.findChessboardCorners(
            gray, (cols, rows), flags=flags | cv2.CALIB_CB_FAST_CHECK)
    if not found:
        return None
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 40, 1e-4)
    return cv2.cornerSubPix(gray, corners, (7, 7), (-1, -1), criteria)


class Collector(Node):
    def __init__(self):
        super().__init__("camera_calibrator")
        self.latest = None
        self.rx = 0
        self.create_subscription(Image, "/image_raw", self.cb, qos_profile_sensor_data)

    def cb(self, msg):
        try:
            self.latest = msg_to_gray(msg)
            self.rx += 1
        except Exception:
            pass


# ---------------------------------------------------------------- 标定
def calibrate(images, cols, rows, square, fisheye=False):
    """返回 (rms, cameraMatrix, distCoeffs, rvecs, tvecs)"""
    objp = np.zeros((rows * cols, 3), np.float32)
    objp[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2) * square

    objpoints, imgpoints = [], []
    size = None
    for path in images:
        img = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
        if img is None:
            continue
        size = img.shape[::-1]
        corners = detect_corners(img, cols, rows)
        if corners is None:
            print(f"  跳过（未检出角点）: {os.path.basename(path)}")
            continue
        objpoints.append(objp)
        imgpoints.append(corners)

    if len(objpoints) < 3:
        return None

    if fisheye:
        flags = cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC | cv2.fisheye.CALIB_FIX_SKEW
        ret = cv2.fisheye.calibrate(
            objpoints, imgpoints, size, np.zeros((3, 3)), np.zeros((4, 1)), flags=flags)
        rms, K, dist, rvecs, tvecs = ret[0], ret[1], ret[2], ret[3], ret[4]
        dist = np.asarray(dist).reshape(4, 1)
    else:
        # OpenCV 4.x 返回 6 个值(多一个 reprojectionError)，5.x 返回 5 个，取前 5 个即可
        ret = cv2.calibrateCamera(objpoints, imgpoints, size, None, None)
        rms, K, dist, rvecs, tvecs = ret[:5]

    # 逐张重投影误差（比全局 rms 更能看出哪些姿态有问题）
    per_image = []
    for i in range(len(objpoints)):
        proj, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], K, dist)
        err = np.linalg.norm(proj.reshape(-1, 2) - imgpoints[i].reshape(-1, 2), axis=1)
        per_image.append(float(err.mean()))

    return dict(rms=rms, K=K, dist=np.asarray(dist).reshape(-1), size=size,
                used=len(objpoints), per_image=per_image, fisheye=fisheye)


def to_yaml_dict(cal, camera_name: str) -> dict:
    K, dist, (w, h) = cal["K"], cal["dist"], cal["size"]
    if cal["fisheye"]:
        model, coeffs = "equidistant", [float(x) for x in dist[:4]]
    else:
        model = "plumb_bob"
        coeffs = [float(x) for x in dist[:5]]
    return {
        "image_width": int(w),
        "image_height": int(h),
        "camera_name": camera_name,
        "camera_matrix": {"rows": 3, "cols": 3,
                          "data": [float(v) for v in K.reshape(-1)]},
        "distortion_model": model,
        "distortion_coefficients": {"rows": 1, "cols": len(coeffs), "data": coeffs},
        "rectification_matrix": {"rows": 3, "cols": 3,
                                 "data": [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]},
        "projection_matrix": {"rows": 3, "cols": 4,
                              "data": [float(v) for v in K.reshape(-1)] + [0.0]
                                      + [0.0] * 5 + [1.0, 0.0]},
    }


def dump_yaml(path, data, camera_name):
    """按 camera_info_manager 认识的格式写 yaml（内嵌列表用 flow 风格）"""
    def fmt(v):
        return "[" + ", ".join(f"{float(x):.9f}" for x in v) + "]"

    lines = [
        f"image_width: {data['image_width']}",
        f"image_height: {data['image_height']}",
        f"camera_name: {data['camera_name']}",
        f"camera_matrix:",
        f"  rows: 3",
        f"  cols: 3",
        f"  data: {fmt(data['camera_matrix']['data'])}",
        f"distortion_model: {data['distortion_model']}",
        f"distortion_coefficients:",
        f"  rows: 1",
        f"  cols: {len(data['distortion_coefficients']['data'])}",
        f"  data: {fmt(data['distortion_coefficients']['data'])}",
        f"rectification_matrix:",
        f"  rows: 3",
        f"  cols: 3",
        f"  data: {fmt(data['rectification_matrix']['data'])}",
        f"projection_matrix:",
        f"  rows: 3",
        f"  cols: 4",
        f"  data: {fmt(data['projection_matrix']['data'])}",
    ]
    with open(path, "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")


def main():
    ap = argparse.ArgumentParser(description="相机内参标定（棋盘格）")
    ap.add_argument("--cols", type=int, default=14, help="内角点列数")
    ap.add_argument("--rows", type=int, default=9, help="内角点行数")
    ap.add_argument("--square", type=float, default=20.0, help="格长 mm")
    ap.add_argument("--out", default=None, help="采样输出目录")
    ap.add_argument("--min-samples", type=int, default=15)
    ap.add_argument("--interval", type=float, default=0.0, help="自动模式间隔，0=手动")
    ap.add_argument("--camera-name", default="narrow_stereo")
    ap.add_argument("--target", default=None, help="camera_info.yaml 输出路径")
    ap.add_argument("--no-write", action="store_true")
    ap.add_argument("--fisheye", action="store_true", help="额外用鱼眼模型对比")
    ap.add_argument("--dup-threshold", type=float, default=2.0,
                    help="与上一张平均像素差小于该值就跳过（避免连拍重复）")
    args = ap.parse_args()

    out_dir = args.out or ("calib_" + datetime.datetime.now().strftime("%Y%m%d_%H%M%S"))
    os.makedirs(out_dir, exist_ok=True)

    target = args.target or os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        "src", "ros2_hik_camera", "config", "camera_info.yaml")

    print(f"棋盘格内角点: {args.cols}x{args.rows}, 格长 {args.square}mm")
    print(f"采样目录: {out_dir}")
    print(f"按键: s=存一张 a=自动 d=显示检测 c=标定 q=退出\n")

    rclpy.init()
    node = Collector()

    saved, seq, auto, show, last_thumb = [], 0, args.interval > 0, True, None
    last_auto = 0.0
    last_save_t = 0.0
    print("等待 /image_raw ...")
    t0 = time.time()
    while rclpy.ok() and time.time() - t0 < 15 and node.latest is None:
        rclpy.spin_once(node, timeout_sec=0.2)
    if node.latest is None:
        print("ERROR: 15 秒内没收到图像，确认相机节点在跑（ros2 launch hik_camera hik_camera.launch.py）")
        rclpy.shutdown()
        return 1

    cv2.namedWindow(WINDOW, cv2.WINDOW_NORMAL)

    def thumb(gray):
        h, w = gray.shape
        scale = 960.0 / max(w, 1)
        return cv2.resize(gray, (int(w * scale), int(h * scale)))

    def try_save(gray) -> bool:
        nonlocal last_thumb, last_save_t, seq, last_save_t
        corners = detect_corners(gray, args.cols, args.rows)
        vis = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
        if corners is not None:
            cv2.drawChessboardCorners(vis, (args.cols, args.rows), corners, 1)
        small = thumb(vis)
        cv2.putText(small, f"saved={len(saved)}  rx={node.rx}", (8, 22),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        status = "OK" if corners is not None else "NO CORNER"
        cv2.putText(small, f"{args.cols}x{args.rows} {status}", (8, 48),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                    (0, 255, 0) if corners is not None else (0, 0, 255), 2)
        # 过曝提示
        sat = float((gray > 250).mean())
        cv2.putText(small, f"saturated={sat*100:.1f}%", (8, 74),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 200, 255), 1)
        cv2.imshow(WINDOW, small)

        if corners is None:
            return False
        th = cv2.resize(gray, (64, 36), interpolation=cv2.INTER_AREA).astype(np.int16)
        if last_thumb is not None and np.abs(th - last_thumb).mean() < args.dup_threshold:
            return False
        if time.time() - last_save_t < 0.3:
            return False

        seq += 1
        path = os.path.join(out_dir, f"calib_{seq:03d}.png")
        cv2.imwrite(path, gray)
        saved.append(path)
        last_thumb = th
        last_save_t = time.time()
        print(f"  已存 {len(saved):3d}  {os.path.basename(path)}")
        return True

    def do_calibrate():
        if len(saved) < args.min_samples:
            print(f"样本不足（{len(saved)} < {args.min_samples}），再拍几张")
            return
        print(f"\n开始标定，{len(saved)} 张样本 ...")
        for fisheye in ([False, True] if args.fisheye else [False]):
            cal = calibrate(saved, args.cols, args.rows, args.square, fisheye=fisheye)
            if cal is None:
                print("  标定失败")
                continue
            tag = "鱼眼" if fisheye else "针孔"
            print(f"\n[{tag}] 使用 {cal['used']} 张, 图像 {cal['size'][0]}x{cal['size'][1]}")
            print(f"[{tag}] 重投影 RMS = {cal['rms']:.3f} px  平均 = "
                  f"{np.mean(cal['per_image']):.3f} px  最差 = {max(cal['per_image']):.3f} px")
            print(f"[{tag}] fx={cal['K'][0,0]:.2f} fy={cal['K'][1,1]:.2f} "
                  f"cx={cal['K'][0,2]:.2f} cy={cal['K'][1,2]:.2f}")
            print(f"[{tag}] 畸变 = {np.array2string(cal['dist'], precision=5)}")
            if not fisheye and not args.no_write:
                data = to_yaml_dict(cal, args.camera_name)
                if os.path.exists(target):
                    shutil.copy(target, target + ".bak")
                    print(f"  已备份原文件 -> {target}.bak")
                dump_yaml(target, data, args.camera_name)
                print(f"  已写入 {target}")
                with open(os.path.join(out_dir, "calib_result.json"), "w") as f:
                    json.dump({k: (v.tolist() if isinstance(v, np.ndarray) else v)
                               for k, v in data.items()}, f, indent=2)

    # 无显示环境（ssh）时不开窗口
    headless = not os.environ.get("DISPLAY")
    if headless and args.interval <= 0:
        print("ERROR: 无 DISPLAY 且未指定 --interval，无法交互取景")
        return 1
    if not headless:
        cv2.namedWindow(WINDOW, cv2.WINDOW_NORMAL)

    saved, seq, auto, show, last_thumb = [], 0, args.interval > 0, not headless, None
    last_auto = 0.0
    last_save_t = 0.0
    print("等待 /image_raw ...")
    t0 = time.time()
    while rclpy.ok() and time.time() - t0 < 15 and node.latest is None:
        rclpy.spin_once(node, timeout_sec=0.2)
    if node.latest is None:
        print("ERROR: 15 秒内没收到图像，确认相机节点在跑（ros2 launch hik_camera hik_camera.launch.py）")
        rclpy.shutdown()
        return 1

    def thumb(gray):
        h, w = gray.shape
        scale = 960.0 / max(w, 1)
        return cv2.resize(gray, (int(w * scale), int(h * scale)))

    def try_save(gray) -> bool:
        nonlocal last_thumb, last_save_t, seq
        corners = detect_corners(gray, args.cols, args.rows)
        th = cv2.resize(gray, (64, 36), interpolation=cv2.INTER_AREA).astype(np.int16)
        if last_thumb is not None and np.abs(th - last_thumb).mean() < args.dup_threshold:
            return False
        if time.time() - last_save_t < 0.3:
            return False
        if corners is None:
            return False
        seq += 1
        path = os.path.join(out_dir, f"calib_{seq:03d}.png")
        cv2.imwrite(path, gray)
        saved.append(path)
        last_thumb = th
        last_save_t = time.time()
        sat = float((gray > 250).mean())
        print(f"  已存 {len(saved):3d}  {os.path.basename(path)}  过曝{sat*100:.1f}%")
        return True

    def do_calibrate():
        if len(saved) < args.min_samples:
            print(f"样本不足（{len(saved)} < {args.min_samples}），再拍几张")
            return
        print(f"\n开始标定，{len(saved)} 张样本 ...")
        for fisheye in ([False, True] if args.fisheye else [False]):
            cal = calibrate(saved, args.cols, args.rows, args.square, fisheye=fisheye)
            if cal is None:
                print("  标定失败")
                continue
            tag = "鱼眼" if fisheye else "针孔"
            print(f"\n[{tag}] 使用 {cal['used']} 张, 图像 {cal['size'][0]}x{cal['size'][1]}")
            print(f"[{tag}] 重投影 RMS = {cal['rms']:.3f} px  平均 = "
                  f"{np.mean(cal['per_image']):.3f} px  最差 = {max(cal['per_image']):.3f} px")
            print(f"[{tag}] fx={cal['K'][0,0]:.2f} fy={cal['K'][1,1]:.2f} "
                  f"cx={cal['K'][0,2]:.2f} cy={cal['K'][1,2]:.2f}")
            print(f"[{tag}] 畸变 = {np.array2string(cal['dist'], precision=5)}")
            if not fisheye and not args.no_write:
                data = to_yaml_dict(cal, args.camera_name)
                if os.path.exists(target):
                    shutil.copy(target, target + ".bak")
                    print(f"  已备份原文件 -> {target}.bak")
                dump_yaml(target, data, args.camera_name)
                print(f"  已写入 {target}")
                with open(os.path.join(out_dir, "calib_result.json"), "w") as f:
                    json.dump({k: (v.tolist() if isinstance(v, np.ndarray) else v)
                               for k, v in data.items()}, f, indent=2)

    print("开始取景，按 s 存图（先确认画面里能看到角点框）")
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.05)
        if node.latest is None:
            continue

        if show:
            t = thumb(node.latest)
            corners = detect_corners(node.latest, args.cols, args.rows)
            if corners is not None:
                t = cv2.cvtColor(t, cv2.COLOR_GRAY2BGR)
                cv2.drawChessboardCorners(t, (args.cols, args.rows),
                                          corners * (t.shape[1] / node.latest.shape[1]), 1)
            cv2.putText(t, f"saved={len(saved)} rx={node.rx}", (8, 22),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            cv2.putText(t, f"{args.cols}x{args.rows} corner={'OK' if corners is not None else 'NO'}",
                        (8, 48), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                        (0, 255, 0) if corners is not None else (0, 0, 255), 2)
            cv2.putText(t, "s=save a=auto c=calib q=quit", (8, 74),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
            cv2.imshow(WINDOW, t)

        key = cv2.waitKey(30) & 0xFF if not headless else 255
        if key == ord("q"):
            break
        elif key == ord("a"):
            auto = not auto
            args.interval = args.interval or 1.0
            last_auto = 0.0
            print(f"自动模式: {'开' if auto else '关'}")
        elif key == ord("c"):
            do_calibrate()
        elif key == ord("d"):
            show = not show
        elif key == ord("s") and not auto:
            try_save(node.latest)

        if auto and time.time() - last_auto >= args.interval:
            last_auto = time.time()
            try_save(node.latest)
            if len(saved) >= args.min_samples:
                break

    print(f"\n共采集 {len(saved)} 张 -> {out_dir}")
    if len(saved) >= args.min_samples:
        do_calibrate()
    else:
        print("样本不足，未标定。重新运行继续采集。")

    cv2.destroyAllWindows()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
