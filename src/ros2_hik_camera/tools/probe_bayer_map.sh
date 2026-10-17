#!/usr/bin/env bash
# 生成「SDK 像素类型 -> OpenCV 去马赛克码」映射表，用来核对
# src/hik_camera_node.cpp 里的 opencvConvFor()。
#
# 背景：OpenCV 与 GenICam 的 Bayer 命名规则不一致，名字相同并不代表排列相同。
# 按名字硬对应会让 R/B 通道整体互换（红蓝颠倒），HSV 阈值会全部失效。
#
# 用法： bash tools/probe_bayer_map.sh
set -euo pipefail

cd "$(dirname "$0")"
OUT="${TMPDIR:-/tmp}/probe_bayer_map"

g++ -O2 -o "$OUT" probe_bayer_map.cpp \
    -I../hikSDK/include $(pkg-config --cflags opencv4) \
    -L../hikSDK/lib/arm64 -lMvCameraControl $(pkg-config --libs opencv4) -lpthread

LD_LIBRARY_PATH="../hikSDK/lib/arm64${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}" "$OUT"