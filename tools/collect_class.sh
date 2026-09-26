#!/bin/bash
# 量化标定集采集：一类一条命令
#
#   ./tools/collect_class.sh outpost_c1        # 前哨站 + 颜色1，采 50 张
#   ./tools/collect_class.sh base_c2 --max 60  # 换参数
#
# 前置：相机节点在跑（ros2 launch hik_camera hik_camera.launch.py）
# 采集时用浏览器开 ws://<板子IP>:8765 看 /image_raw，对着画面摆角度和距离。
set -e

NAME=${1:?用法: collect_class.sh <类名> [额外参数...]   例: collect_class.sh outpost_c1}
shift || true

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUT="$ROOT/calib_quant/$NAME"
case "$NAME" in
  outpost_c1|outpost_c2) LABEL="前哨站" ;;
  base_c1|base_c2)       LABEL="基地" ;;
  *)                     LABEL="$NAME" ;;
esac
MAX=100
FMT=png
INTERVAL=0.8

while [[ $# -gt 0 ]]; do
  case "$1" in
    --max) MAX="$2"; shift 2;;
    --interval) INTERVAL="$2"; shift 2;;
    --format) FMT="$2"; shift 2;;
    --label) LABEL="$2"; shift 2;;
    *) echo "忽略未知参数: $1"; shift;;
  esac
done

if [[ -d "$OUT" ]] && [[ -n "$(ls -A "$OUT" 2>/dev/null)" ]]; then
  echo "⚠️ 目录已存在且非空: $OUT"
  echo "   续采请先清空或换目录（避免和旧数据混在一起）："
  echo "     rm -rf $OUT"
  read -r -p "   仍要继续？(y/N) " a
  [[ "$a" == "y" || "$a" == "Y" ]] || exit 1
fi

echo "=== 采集 $NAME -> $OUT"
echo "    格式 $FMT / 最多 $MAX 张 / 每 $INTERVAL 秒尝试一张（重复帧自动跳过）"
echo
echo "  ⚠️ 采集要点（之前踩过的坑）："
echo "     1) 相机要固定不动、保持正立 —— 转相机/转画面会让模型乱判（实测同一物体"
echo "        一会儿判前哨站一会儿判基地，还有一半检不出）"
echo "     2) 要动的是【目标】：距离 近~远、画面位置 中心~边缘、目标自身左右转"
echo "     3) 每张之间动一点，重复帧会被自动跳过"
echo "     4) 目标占画面 3%~12% 比较稳（太小检不出，太大偏离训练分布）"
echo
source /opt/ros/humble/setup.bash
python3 "$ROOT/tools/collect_dataset.py" \
  --out "$OUT" --max "$MAX" --format "$FMT" --interval "$INTERVAL" --label "$LABEL" "$@"

N=$(find "$OUT" -maxdepth 1 -type f \( -name "*.png" -o -name "*.jpg" \) | wc -l)
SZ=$(du -sh "$OUT" | cut -f1)
echo
echo "=== $NAME 采到 $N 张，占用 $SZ"
[[ "$N" -lt "$MAX" ]] && echo "⚠️ 少于目标 $MAX 张，不够就再跑一次这条命令（会提示续采）"

# 用切好的模型过一遍，确认这批图里确实是想要的类别、且模型能检出
echo
echo "=== 模型校验（每类检出率 / 平均置信度）"
python3 - "$OUT" "$NAME" <<'PY'
import sys, glob, os
import numpy as np, cv2
out, name = sys.argv[1], sys.argv[2]
files = sorted(glob.glob(os.path.join(out, "*.png")) + glob.glob(os.path.join(out, "*.jpg")))
if not files:
    sys.exit("没有图片可校验")
try:
    import onnxruntime as ort
except ImportError:
    sys.exit("没装 onnxruntime，跳过模型校验（pip3 install onnxruntime）")
M = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(out))),
                 "src/yq_dart_aim/model/praysky_dart_576x768_cut.onnx")
s = ort.InferenceSession(M, providers=["CPUExecutionProvider"])
NAMES = ["s0_o0","s0_o2","s0_o3","s0_o4","s0_o5","s0_o6(前哨站)",
         "s1_o0","s1_o1","s1_o3","s1_o4","s1_o5","s1_o7(基地)"]
hit, confs, sat = 0, [], 0.0
hist = {}
for f in files:
    img = cv2.imread(f)
    if img is None:
        continue
    h, w = img.shape[:2]
    a = 768 / 576
    if w / h > a:
        cw, ch = int(round(h * a)), h
    else:
        cw, ch = w, int(round(w / a))
    x0, y0 = (w - cw) // 2, (h - ch) // 2
    small = cv2.resize(img[y0:y0+ch, x0:x0+cw], (768, 576))
    xin = cv2.cvtColor(small, cv2.COLOR_BGR2RGB).astype(np.float32).transpose(2, 0, 1)[None] / 255.
    o = s.run(None, {s.get_inputs()[0].name: xin})[0][0]
    sc = o[:, 4:16].max(1)
    i = int(sc.argmax())
    c = float(sc[i])
    cn = NAMES[int(o[i, 4:16].argmax())]
    hist[cn] = hist.get(cn, 0) + 1
    if c >= 0.25:
        hit += 1
        confs.append(c)
    sat += float((img > 250).mean())
print(f"  {name}: {len(files)} 张, 检出(>=0.25) {hit} 张 "
      f"({hit*100//max(1,len(files))}%), 平均置信度 "
      f"{(sum(confs)/len(confs) if confs else 0):.3f}")
print(f"  类别分布: {hist}")
print(f"  平均过曝像素占比: {sat/len(files)*100:.1f}%")
print("  注：模型类别会随姿态飘，物理身份以 --label 为准；这里只用来发现"
      "「整体检不出」的问题")
if hit * 2 < len(files):
    print("  ⚠️ 过半图检不出：检查目标是不是太小/太远、相机有没有被转动、"
          "或者放错了目标（正常应该 >70%）")
PY
