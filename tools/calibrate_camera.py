#!/usr/bin/env python3
"""相机内参标定工具 —— 棋盘格标定，生成 ROS camera_info.yaml

标定板：黑白棋盘格，默认按 **14x9 内角点 + 20mm 格长**（对应 15x10 个格子）。
  注意区分"格子数"和"内角点数"：15x10 个格子 -> 14x9 个内角点。

两种模式
--------
1) 实时取景（板子上跑，需要 ROS + 相机节点在跑）：
     python3 tools/calibrate_camera.py
   快捷键: s=存一张 a=自动连拍 c=开始标定 d=开关预览 q=退出

2) 离线标定（任何装了 python+opencv 的机器都行，不需要 ROS）：
   把在 PC 上拍好的图片拷到一个文件夹，然后
     python3 tools/calibrate_camera.py --from-dir /path/to/images
   图片格式 jpg/png 都行，会自动跳过检测不到角点的。

为什么不用 ROS 自带的 cameracalibrator
------------------------------------
cv_bridge 是按 numpy 1.x 编译的，本机 numpy 是 2.x，直接报 _ARRAY_API not found。
本脚本自己用 numpy 解图像（不碰 cv_bridge），并在 calib_* 目录留下可复查的结果。

取景要求（决定标定质量）
------------------------
  - 棋盘格要平整：打印后贴在硬质平板上，格子四角不能翘
  - 覆盖整个画面：中心、四角、四边，姿态要有平/斜/远近变化
  - 建议 25~35 张，每张只动一个角度
  - 避免白格过曝（像素顶到 255），否则黑白边界糊掉、角点检测失败

常用参数
--------
  --cols N --rows N   内角点列数/行数（默认 14 9）
  --square MM         格长毫米（默认 20）
  --from-dir DIR      离线模式：读这个目录里的图片做标定
  --out DIR           实时模式的采样输出目录（默认 ./calib_<时间戳>/）
  --min-samples N     少于 N 张不标定（默认 15）
  --interval SEC      自动连拍间隔，0=手动
  --target FILE       camera_info.yaml 输出路径（默认项目 src/ros2_hik_camera/config/）
  --no-write          只算不写文件
  --fisheye           额外用鱼眼模型标一次并对比误差
  --dup-threshold N   与上一张平均像素差小于 N 就跳过（默认 2.0，防连拍重复）
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

WINDOW = "calibrate"
IMG_EXT = (".png", ".jpg", ".jpeg", ".bmp", ".tif", ".tiff")


# ================================================================= 图像与角点
def msg_to_gray(msg) -> np.ndarray:
    """sensor_msgs/Image -> 灰度图。

    手写转换而不用 cv_bridge：cv_bridge 的扩展是按 numpy 1.x 编译的，
    在 numpy 2.x 的机器上会抛 _ARRAY_API not found。
    """
    enc = msg.encoding
    if enc in ("bgr8", "rgb8"):
        ch, dt = 3, np.uint8
    elif enc == "mono8":
        ch, dt = 1, np.uint8
    elif enc in ("bgra8", "rgba8"):
        ch, dt = 4, np.uint8
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
        if dt == np.uint16:
            img = (img / 257).astype(np.uint8)
        return np.ascontiguousarray(img)
    if enc == "rgb8":
        img = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
    elif enc == "rgba8":
        img = cv2.cvtColor(img, cv2.COLOR_RGBA2GRAY)
    elif enc == "bgra8":
        img = cv2.cvtColor(img, cv2.COLOR_BGRA2GRAY)
    return np.ascontiguousarray(img)


def detect_corners(gray: np.ndarray, cols: int, rows: int):
    """找棋盘格角点，失败返回 None。

    优先用 findChessboardCornersSB（opencv>=4.7）：大棋盘格/高分辨率下比经典算法稳得多，
    对光照不均也更友好；检测不到再退回经典算法 + 亚像素 refine。
    """
    sb = getattr(cv2, "findChessboardCornersSB", None)
    if sb is not None:
        try:
            found, corners = sb(gray, (cols, rows),
                                flags=cv2.CALIB_CB_EXHAUSTIVE | cv2.CALIB_CB_ACCURACY)
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


# ================================================================= 标定
def calibrate(images, cols, rows, square, fisheye=False):
    """images: 图片路径列表。返回标定结果 dict，样本不足 3 张返回 None。"""
    objp = np.zeros((rows * cols, 3), np.float32)
    objp[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2) * float(square)

    objpoints, imgpoints, skipped = [], [], []
    size = None
    for path in images:
        img = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
        if img is None:
            skipped.append((os.path.basename(path), "读取失败"))
            continue
        if size is None:
            size = img.shape[::-1]
        elif img.shape[::-1] != size:
            skipped.append((os.path.basename(path), f"尺寸不一致 {img.shape[::-1]}"))
            continue
        corners = detect_corners(img, cols, rows)
        if corners is None:
            skipped.append((os.path.basename(path), "未检出角点"))
            continue
        objpoints.append(objp)
        imgpoints.append(corners)

    for name, why in skipped:
        print(f"  跳过 {name}: {why}")
    if len(objpoints) < 3:
        return None

    if fisheye:
        flags = cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC | cv2.fisheye.CALIB_FIX_SKEW
        ret = cv2.fisheye.calibrate(objpoints, imgpoints, size, np.zeros((3, 3)),
                                    np.zeros((4, 1)), flags=flags)
        rms, K, dist, rvecs, tvecs = ret[0], ret[1], ret[2], ret[3], ret[4]
        dist = np.asarray(dist).reshape(4, 1)
    else:
        # OpenCV 4.x 返回 6 个值（多一个 reprojectionError），5.x 返回 5 个，取前 5 个
        ret = cv2.calibrateCamera(objpoints, imgpoints, size, None, None)
        rms, K, dist, rvecs, tvecs = ret[:5]

    per_image = []
    for i in range(len(objpoints)):
        proj, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], K, dist)
        err = np.linalg.norm(proj.reshape(-1, 2) - imgpoints[i].reshape(-1, 2), axis=1)
        per_image.append(float(err.mean()))

    return dict(rms=float(rms), K=K, dist=np.asarray(dist).reshape(-1), size=size,
                used=len(objpoints), per_image=per_image, fisheye=fisheye,
                samples=[os.path.basename(p) for p in images[:0]])


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
        "camera_matrix": {"rows": 3, "cols": 3, "data": [float(v) for v in K.reshape(-1)]},
        "distortion_model": model,
        "distortion_coefficients": {"rows": 1, "cols": len(coeffs), "data": coeffs},
        "rectification_matrix": {"rows": 3, "cols": 3,
                                 "data": [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]},
        "projection_matrix": {"rows": 3, "cols": 4,
                              "data": [float(v) for v in K.reshape(-1)] + [0.0]
                                      + [0.0] * 5 + [1.0, 0.0]},
    }


def dump_yaml(path, data) -> None:
    """按 camera_info_manager 认识的格式写 yaml（内嵌数组用 flow 风格）"""
    def fmt(v):
        return "[" + ", ".join(f"{float(x):.9f}" for x in v) + "]"

    lines = [
        f"image_width: {data['image_width']}",
        f"image_height: {data['image_height']}",
        f"camera_name: {data['camera_name']}",
        "camera_matrix:",
        "  rows: 3",
        "  cols: 3",
        f"  data: {fmt(data['camera_matrix']['data'])}",
        f"distortion_model: {data['distortion_model']}",
        "distortion_coefficients:",
        "  rows: 1",
        f"  cols: {len(data['distortion_coefficients']['data'])}",
        f"  data: {fmt(data['distortion_coefficients']['data'])}",
        "rectification_matrix:",
        "  rows: 3",
        "  cols: 3",
        f"  data: {fmt(data['rectification_matrix']['data'])}",
        "projection_matrix:",
        "  rows: 3",
        "  cols: 4",
        f"  data: {fmt(data['projection_matrix']['data'])}",
    ]
    with open(path, "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")


def default_target() -> str:
    return os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                        "src", "ros2_hik_camera", "config", "camera_info.yaml")


def report(cal, tag: str) -> None:
    print(f"\n[{tag}] 使用 {cal['used']} 张, 图像 {cal['size'][0]}x{cal['size'][1]}")
    print(f"[{tag}] 重投影 RMS = {cal['rms']:.3f} px   平均 = "
          f"{np.mean(cal['per_image']):.3f} px   最差 = {max(cal['per_image']):.3f} px")
    print(f"[{tag}] fx={cal['K'][0,0]:.2f} fy={cal['K'][1,1]:.2f} "
          f"cx={cal['K'][0,2]:.2f} cy={cal['K'][1,2]:.2f}")
    print(f"[{tag}] 畸变 = {np.array2string(cal['dist'], precision=6)}")
    if cal["rms"] > 1.5:
        print(f"[{tag}] ⚠ RMS 偏大：检查板子是否平整、样本姿态是否够多样、有无过曝")
    elif cal["rms"] > 1.0:
        print(f"[{tag}] 注意：RMS 略大，建议补拍不同角度/位置")


def list_images(directory: str):
    files = []
    for ext in IMG_EXT:
        files.extend(glob.glob(os.path.join(directory, f"*{ext}")))
        files.extend(glob.glob(os.path.join(directory, f"*{ext.upper()}")))
    return sorted(set(files))


# ================================================================= 离线模式
def run_offline(args) -> int:
    files = list_images(args.from_dir)
    if not files:
        print(f"ERROR: {args.from_dir} 里没有找到图片（支持 {', '.join(IMG_EXT)}）")
        return 1
    print(f"棋盘格内角点: {args.cols}x{args.rows}, 格长 {args.square}mm")
    print(f"离线标定: {args.from_dir} 共 {len(files)} 张\n")

    for fisheye in ([False, True] if args.fisheye else [False]):
        cal = calibrate(files, args.cols, args.rows, args.square, fisheye=fisheye)
        if cal is None:
            print("标定失败：可用样本不足 3 张")
            return 1
        report(cal, "鱼眼" if fisheye else "针孔")
        if not fisheye:
            data = to_yaml_dict(cal, args.camera_name)
            if not args.no_write:
                target = args.target or default_target()
                if os.path.exists(target):
                    shutil.copy(target, target + ".bak")
                    print(f"\n  已备份原文件 -> {target}.bak")
                os.makedirs(os.path.dirname(os.path.abspath(target)), exist_ok=True)
                dump_yaml(target, data)
                print(f"  已写入 {target}")
            with open(os.path.join(os.path.dirname(os.path.abspath(args.from_dir)),
                                   "calib_result.json"), "w") as f:
                json.dump({k: (v.tolist() if isinstance(v, np.ndarray) else v)
                           for k, v in data.items()}, f, indent=2)
    return 0


# ================================================================= 实时取景模式
class LiveGrab:
    """订阅 /image_raw（需要 ROS）。懒加载 rclpy，离线模式不依赖它。"""

    def __init__(self):
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import Image

        self._rclpy = rclpy
        self.msg_type = Image
        qos = qos_profile_sensor_data
        outer = self

        class _N(Node):
            def __init__(self):
                super().__init__("camera_calibrator")
                self.latest = None
                self.rx = 0
                self.create_subscription(outer.msg_type, "/image_raw", self.cb, qos)

            def cb(self, msg):
                try:
                    self.latest = msg_to_gray(msg)
                    self.rx += 1
                except Exception:
                    pass

        rclpy.init()
        self._node = _N()

    def spin(self, timeout):
        self._rclpy.spin_once(self._node, timeout_sec=timeout)

    def ok(self):
        return self._rclpy.ok()

    def shutdown(self):
        try:
            self._rclpy.shutdown()
        except Exception:
            pass

    @property
    def latest(self):
        return self._node.latest

    @property
    def rx(self):
        return self._node.rx


def run_live(args) -> int:
    out_dir = args.out or ("calib_" + datetime.datetime.now().strftime("%Y%m%d_%H%M%S"))
    os.makedirs(out_dir, exist_ok=True)
    print(f"棋盘格内角点: {args.cols}x{args.rows}, 格长 {args.square}mm")
    print(f"采样目录: {out_dir}")
    print("按键: s=存一张 a=自动 c=标定 d=预览 q=退出\n")

    headless = not os.environ.get("DISPLAY")
    if headless and args.interval <= 0:
        print("ERROR: 无 DISPLAY 且没给 --interval，无法交互取景")
        return 1

    grab = LiveGrab()
    print("等待 /image_raw ...")
    t0 = time.time()
    while grab.ok() and time.time() - t0 < 15 and grab.latest is None:
        grab.spin(0.2)
    if grab.latest is None:
        print("ERROR: 15 秒内没收到图像，确认相机节点在跑"
              "（ros2 launch hik_camera hik_camera.launch.py）")
        grab.shutdown()
        return 1

    if not headless:
        cv2.namedWindow(WINDOW, cv2.WINDOW_NORMAL)

    state = dict(saved=[], seq=0, auto=args.interval > 0, show=not headless,
                 last_thumb=None, last_auto=0.0, last_save=0.0)

    def thumb(gray):
        h, w = gray.shape
        s = 960.0 / max(w, 1)
        return cv2.resize(gray, (int(w * s), int(h * s)))

    def try_save(gray) -> bool:
        if detect_corners(gray, args.cols, args.rows) is None:
            return False
        th = cv2.resize(gray, (64, 36), interpolation=cv2.INTER_AREA).astype(np.int16)
        if state["last_thumb"] is not None and \
                np.abs(th - state["last_thumb"]).mean() < args.dup_threshold:
            return False
        if time.time() - state["last_save"] < 0.3:
            return False
        state["seq"] += 1
        path = os.path.join(out_dir, f"calib_{state['seq']:03d}.png")
        cv2.imwrite(path, gray)
        state["saved"].append(path)
        state["last_thumb"] = th
        state["last_save"] = time.time()
        print(f"  已存 {len(state['saved']):3d}  {os.path.basename(path)}  "
              f"过曝{float((gray > 250).mean())*100:.1f}%")
        return True

    def do_calibrate() -> None:
        saved = state["saved"]
        if len(saved) < args.min_samples:
            print(f"样本不足（{len(saved)} < {args.min_samples}），再拍几张")
            return
        print(f"\n开始标定，{len(saved)} 张样本 ...")
        for fisheye in ([False, True] if args.fisheye else [False]):
            cal = calibrate(saved, args.cols, args.rows, args.square, fisheye=fisheye)
            if cal is None:
                print("  标定失败")
                continue
            report(cal, "鱼眼" if fisheye else "针孔")
            if not fisheye and not args.no_write:
                target = args.target or default_target()
                data = to_yaml_dict(cal, args.camera_name)
                if os.path.exists(target):
                    shutil.copy(target, target + ".bak")
                    print(f"  已备份原文件 -> {target}.bak")
                dump_yaml(target, data)
                print(f"  已写入 {target}")
                with open(os.path.join(out_dir, "calib_result.json"), "w") as f:
                    json.dump({k: (v.tolist() if isinstance(v, np.ndarray) else v)
                               for k, v in data.items()}, f, indent=2)

    print("开始取景，按 s 存图（先确认画面里 corner=OK）")
    while grab.ok():
        grab.spin(0.05)
        if grab.latest is None:
            continue

        if state["show"]:
            t = thumb(grab.latest)
            corners = detect_corners(grab.latest, args.cols, args.rows)
            if corners is not None:
                t = cv2.cvtColor(t, cv2.COLOR_GRAY2BGR)
                cv2.drawChessboardCorners(t, (args.cols, args.rows),
                                          corners * (t.shape[1] / grab.latest.shape[1]), 1)
            ok = corners is not None
            cv2.putText(t, f"saved={len(state['saved'])} rx={grab.rx}", (8, 22),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            cv2.putText(t, f"{args.cols}x{args.rows} corner={'OK' if ok else 'NO'}", (8, 48),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0) if ok else (0, 0, 255), 2)
            cv2.putText(t, f"saturated={float((grab.latest>250).mean())*100:.1f}%", (8, 74),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 200, 255), 1)
            cv2.putText(t, "s=save a=auto c=calib q=quit", (8, 100),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
            cv2.imshow(WINDOW, t)

        key = cv2.waitKey(30) & 0xFF if not headless else 255
        if key == ord("q"):
            break
        elif key == ord("a"):
            state["auto"] = not state["auto"]
            args.interval = args.interval or 1.0
            state["last_auto"] = 0.0
            print(f"自动模式: {'开' if state['auto'] else '关'}")
        elif key == ord("c"):
            do_calibrate()
        elif key == ord("d"):
            state["show"] = not state["show"]
        elif key == ord("s") and not state["auto"]:
            try_save(grab.latest)

        if state["auto"] and time.time() - state["last_auto"] >= args.interval:
            state["last_auto"] = time.time()
            try_save(grab.latest)
            if len(state["saved"]) >= args.min_samples:
                break

    print(f"\n共采集 {len(state['saved'])} 张 -> {out_dir}")
    if len(state["saved"]) >= args.min_samples:
        do_calibrate()
    else:
        print("样本不足，未标定。重新运行可继续采集（采样图在上面的目录里）。")

    if not headless:
        cv2.destroyAllWindows()
    grab.shutdown()
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(
        description="相机内参标定（棋盘格）",
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--cols", type=int, default=14, help="内角点列数（格子数-1）")
    ap.add_argument("--rows", type=int, default=9, help="内角点行数（格子数-1）")
    ap.add_argument("--square", type=float, default=20.0, help="格长 mm")
    ap.add_argument("--from-dir", default=None,
                    help="离线模式：直接用这个目录里的图片标定（不需要 ROS）")
    ap.add_argument("--out", default=None, help="实时模式采样输出目录")
    ap.add_argument("--min-samples", type=int, default=15, help="少于 N 张不标定")
    ap.add_argument("--interval", type=float, default=0.0, help="自动连拍间隔，0=手动")
    ap.add_argument("--camera-name", default="narrow_stereo")
    ap.add_argument("--target", default=None, help="camera_info.yaml 输出路径")
    ap.add_argument("--no-write", action="store_true", help="只算不写文件")
    ap.add_argument("--fisheye", action="store_true", help="额外用鱼眼模型标一次对比")
    ap.add_argument("--dup-threshold", type=float, default=2.0,
                    help="与上一张平均像素差小于该值就跳过（默认 2.0）")
    args = ap.parse_args()

    if args.from_dir:
        return run_offline(args)
    return run_live(args)


if __name__ == "__main__":
    sys.exit(main())
