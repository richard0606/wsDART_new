# RDK X5 BPU 部署指南

> 适用：`dart_aim_node` 的**模型识别模式**走 RDK X5 的 BPU 加速。
> 模型来源：`AT_NN_Detector`（PraySky 的 YOLO26n-Pose 装甲板检测，AGPL-3.0）。
> 本文覆盖：**从 ONNX 到板子上跑起来**的完整流程 + 所有已知坑。

---

## 0. 三条链路的分工

| 阶段 | 在哪做 | 产出 | 谁做 |
|---|---|---|---|
| ① 切模型（去掉内置后处理） | 任意有 Python 的机器 | `*_cut.onnx` | 谁都能做，5 分钟 |
| ② 量化编译 | **x86 电脑 + docker 工具链** | `*.bin` | 需要拉 1.8GB 镜像，最耗时 |
| ③ 编译运行 | **RDK X5 板子** | 识别结果 | 上板那一步 |
| ④ 标定偏移量 | 赛场实测 | `params.yaml` 里的 `offset_*` | 实弹标定，谁都替代不了 |

> 日常"改代码"**不需要**重做 ①②：模型不变时 `.bin` 可以直接复用。

---

## 1. 为什么必须"切模型"

原始 ONNX 共 454 个节点，其中：

- **前 442 个（97%）是 BPU 友好算子**（Conv / Sigmoid / Mul / Concat / Add / Reshape / Transpose / MatMul / MaxPool / Softmax / Resize …）
- **最后 12 个不行**：`TopK → GatherElements ×4 → Unsqueeze/Expand/Tile/Cast/Concat`，是把 9072 个候选压成 30 条结果的后处理

**切点：`/model.23/Transpose_output_0`**，切完的输出是

```
(1, 9072, 28) = [x1,y1,x2,y2, cls×12, color×4, kpt×8]
```

box 已经是解码好的 xyxy，关键点也已解码。剩下这段后处理由 **CPU 侧代码完成**（`model_detector.cpp` 里的 `decode()`，约 20 行）：

```
逐 anchor 取 12 类分数的 max/argmax  →  TopK-30  →  按 18 列组装  →  坐标映射回裁剪图
```

> 这个拆分已经实测验证过：CPU 后处理的输出与模型内置后处理**逐位一致（误差 0）**。

---

## 2. 电脑上：切 ONNX

```python
import onnx
from onnx.utils import extract_model

extract_model(
    'praysky_c2psa_e2e_0228_576x768.onnx',      # 原模型
    'praysky_dart_576x768_cut.onnx',            # 切完的模型（下面要拿去量化）
    ['images'],                                  # 输入名不变
    ['/model.23/Transpose_output_0'])            # 切点
```

**校验切对了**（必须满足，否则说明模型换版本了，下面全不成立）：

```python
m = onnx.load('praysky_dart_576x768_cut.onnx')
print(len(m.graph.node))                                        # 期望 439
print([(o.name, [d.dim_value for d in o.type.tensor_type.shape.dim]) for o in m.graph.output])
# 期望 [('/model.23/Transpose_output_0', [1, 9072, 28])]
```

---

## 3. 电脑上：docker 工具链量化编译出 .bin

### 3.1 拉镜像

```bash
docker pull openexplorer/ai_toolchain_ubuntu_20_x5_cpu:v1.2.8     # 约 1.8GB（压缩）
```

### 3.2 目录结构

```
horizon_work/
├── praysky_dart_576x768_cut.onnx     # 第 2 步的产物
├── calib/                            # 标定图 100~300 张
│   ├── 0001.jpg ...
└── dart_576x768.yaml                 # 下面这份配置
```

### 3.3 hb_mapper 配置（`dart_576x768.yaml`）

```yaml
model_parameters:
  onnx_model: 'praysky_dart_576x768_cut.onnx'
  march: 'bayes-e'                    # RDK X5 是 Bayes-e 架构
  output_model_file_prefix: 'praysky_dart_576x768'
  working_dir: './model_output'

input_parameters:
  input_name: 'images'
  input_type_rt: 'nv12'               # 推荐：YUV 转换在 BPU 内部做，最快
  input_layout_rt: 'NCHW'
  input_type_train: 'rgb'
  input_layout_train: 'NCHW'
  input_shape: '1x3x576x768'
  norm_type: 'data_scale'
  scale_value: 0.00392156862745098    # = 1/255

calibration_parameters:
  cal_data_dir: './calib'
  cal_data_type: 'float32'
  calibration_type: 'default'         # 精度不够时改 'mix' 并把下面的节点配成 int16

compiler_parameters:
  compile_mode: 'latency'
  optimize_level: 'O3'
  debug: false
  core_num: 1
```

### 3.4 执行

```bash
docker run --rm -it -v /path/to/horizon_work:/work \
  openexplorer/ai_toolchain_ubuntu_20_x5_cpu:v1.2.8 bash

# 容器内：
cd /work
hb_mapper checker --model-type onnx --march bayes-e --model praysky_dart_576x768_cut.onnx
hb_mapper makertbin --config dart_576x768.yaml --model-type onnx
```

产物：`model_output/praysky_dart_576x768.bin`

### 3.5 标定图要求（**直接决定精度**）

- **100~300 张**，覆盖实际比赛的光照/距离/角度；最好就是飞镖赛场用同一台相机拍的
- ❌ 不要用 `AT_NN_Detector/video/` 里那两个视频抽帧：一个是导播直播合成画面、一个是旧系统的 Foxglove 录屏，都不是有效样本
- 尺寸随意（工具链会自己处理），但内容要是真实的装甲板画面

### 3.6 已知量化风险

| 风险点 | 说明 | 缓解 |
|---|---|---|
| **Softmax ×2 + MatMul ×4** | PSA 注意力里的 Softmax 对量化不友好，模型作者本人也警告过 | 精度不达标时，把含 Softmax/MatMul 的节点在 `node_info` 里单独配 `quant_type: int16` |
| **Resize ×2** | BPU 对 Resize 有模式/倍数限制 | `hb_mapper checker` 会报，若报错再单独处理 |

**精度验证**（量化后一定要做）：用同一张图，比对 `.bin` 与 `.onnx` 的输出。若类别/框明显变差，就回去调 int16 节点配置。

---

## 4. 板子上：编译 + 运行

### 4.1 拷到板子

- 整个工作区源码（或者直接 `git pull`）
- **`praysky_dart_576x768.bin`**（建议放 `src/yq_dart_aim/model/`，注意 `.gitignore` 忽略了 `*.bin`，不会被提交）

### 4.2 编译（**必须指定后端**）

```bash
source /opt/ros/humble/setup.bash
cd /root/dart_ws_on
colcon build --cmake-args -DDART_INFER_BACKEND=BPU
```

> 不指定的话默认是 `ORT`（ONNX Runtime），板子上没装 onnxruntime 会**编译报错**并提示怎么改——这是故意的，避免静默用错后端。

### 4.3 改参数（`src/yq_dart_aim/config/params.yaml`）

```yaml
use_model: true
model_path: /root/dart_ws_on/src/yq_dart_aim/model/praysky_dart_576x768.bin
model_input_width: 768
model_input_height: 576
conf_threshold: 0.25      # 先 0.25，误检多就往上调
```

> ⚠️ `params.yaml` 是唯一权威参数来源。`dart_aim_node` 代码里的默认值只是兜底，
> 两者已对齐，但**改参数请改 yaml**。

### 4.4 第一次运行：**必看这段日志**

```bash
ros2 launch yq_dart_aim start.launch.py
```

BPU 后端加载时会打印（`[model_backend_bpu]`）：

```
input type=IMG_NV12 valid=[1,3,576,768] aligned=[...] stride=[...] scale=...
output type=S16 valid=[1,9072,28] aligned=[...] stride=[...] scale=...
BPU model loaded: 768x576 nv12
```

**这是设计好的自检输出**——因为 `model_backend_bpu.cpp` 是在没有板子的情况下写的，
文件头列了 5 条假设，用这段日志逐条核对：

| # | 假设 | 日志里怎么看 | 不对时改哪 |
|---|---|---|---|
| 1 | 头文件路径是 `<hobot/dnn/hb_dnn.h>` | 编译期就该过 | `CMakeLists.txt` 里注释掉的 `target_include_directories` |
| 2 | 输入是 `nv12` 或 `featuremap` | `input type=` 是 `IMG_NV12*` 还是 `S8/F32` | 两种都实现了，日志对得上就不用改 |
| 3 | 输出最后一维是 28 | `output valid=[1,9072,28]` | 若不是 28，说明切点或模型版本变了 |
| 4 | 输出会量化（S8/S16） | `output type=` + `scale=` | 代码已按 `scale` 反量化；若 `scale` 打印为 0 需检查 |
| 5 | `hbDNNInfer` 需要调用方预分配输出缓冲 | 推理返回 false 会打 `hbDNNInfer failed` | 参考官方 sample 调整 `g_outputs` 的分配 |

---

## 5. 验证识别是否正确

```bash
# 1) 看调试图（带框 + 类别名 + 中心点）
#    foxglove 订阅 /model_debug/image_compressed
# 2) 看检测结果话题
ros2 topic echo /dart_debug          # x=水平误差, y=垂直误差
# 3) 看帧率
ros2 topic hz /image_raw
```

**类别映射**（下位机的 `enemy` 字段 → 模型类别，代码里是 `model_classes::classIdForTarget`）：

| `enemy` | 含义 | 模型类别 id | 类别名 |
|---|---|---|---|
| 0 | 无目标 | — | **不瞄准** |
| 1 | 前哨站 | 5 | `s0_o6`（前哨站只有小装甲） |
| 2 | 基地 | 11 | `s1_o7`（基地只有大装甲） |

调试图上的标签就是这 12 类的名字：`s0_o0 … s1_o7`（s=装甲尺寸，o=目标类型）。

---

## 6. 注意事项（坑清单）

### 6.1 必须知道的

1. **OpenCV 4.5.4 跑不了这个模型**。ROS humble 自带的就是 4.5.4，它的 ONNX importer 不支持
   `ArgMax` 节点，`cv::dnn` 加载直接失败。所以 CPU 方案是 **ONNX Runtime**，不是 cv::dnn。
2. **模型模式不产生 mask**。切到模型模式后 `/dart_debug/mask_compressed` 会**停止刷新**（不是坏了），
   foxglove 里那个 panel 卡住是正常的。
3. **模型模式的预处理与 HSV 完全独立**：模型模式不跑 HSV 阈值/灰度那条链路；反过来
   `h_*`/`s_*`/`v_*`/形态学/`max_area` 这些参数在模型模式下**不生效**。
4. **裁剪会丢画面左右各 160px**。1280×720 的裁剪图按 4:3 中心裁成 960×720 再缩到 768×576。
   如果靶子可能出现在画面左右边缘，要改成 letterbox（代码里改 `preprocess`）。
5. **`use_model=true` 打不开模型时会明确报错并退回 HSV**（不会静默用错模式）：
   - `model_path` 为空 → `ERROR: use_model=true 但 model_path 为空…`
   - 加载失败 → `ERROR: 模型加载失败: <路径>…`
   日志里看到这类 ERROR 就说明当前跑的是 HSV。
6. **`.bin` 不要提交进 git**（`.gitignore` 已忽略 `*.bin`），板子上单独放。
7. **模型是 AGPL-3.0**，本仓库是公开仓库，别把 `.onnx`/`.bin` 提交进去。

### 6.2 与其它模块的联动

8. **偏移量表要按现场重新标定**（`offset_0_*` / `offset_1_*` / `offset_2_*`）。
   注意代码已经修正过"偏移被叠加两次"的 bug，**现在的补偿量只有修复前的一半**，
   旧值一律不要用。标定方法：打一发 → 看弹着点偏多少 → 该目标该飞镖的 offset 加减对应像素。
9. **`crop_width` / `crop_height` 与偏移表绑定**（HSV 路径的图像中心与裁剪图尺寸有关）。
   模型路径的坐标已经映射回同一个裁剪图坐标系，所以改裁剪尺寸时 **HSV 的偏移要重标，模型路径不用**。
10. **串口数据校验**：下位机发来的 `enemy` 只能是 0/1/2、`cur` 只能是 0~3。
    收到越界值（比如 9、-3）时，代码**丢弃这个包并保留上一次的有效值**，同时打 WARN 日志。
    这样单帧脏数据不会让瞄准跳到一个不存在目标上。
11. **模型内置 TopK，不需要 NMS**。所以 `nms_threshold` 参数已被删除，
    后处理里也没有 NMS 环节（候选框之间不会互相抑制，靠 `conf_threshold` 过滤）。

### 6.3 模型本身的已知限制（作者原话）

12. **不识别"基地小装甲"**（RM2026 新增的目标，训练集未覆盖）。
13. **量化版本下 Outpost 偶有异常**（w8a16 时），作者判断是数据集脏数据，不影响拟合。
14. 模型覆盖：所有参赛环境的目标（轨道哨兵、5 号小装甲步兵、3/4/5 号大装甲步兵等）。

---

## 7. 排障速查

| 现象 | 原因 | 处理 |
|---|---|---|
| 编译报错找不到 onnxruntime | 忘了指定后端 | 加 `-DDART_INFER_BACKEND=BPU` |
| 启动日志 `模型加载失败` | `.bin` 路径错 / 格式不匹配 | 检查 `model_path`；确认编译时选的是 BPU |
| 日志 `input size mismatch` | 模型输入尺寸与 `model_input_width/height` 不一致 | 改 yaml 里的 `model_input_*` 与模型对齐 |
| `hbDNNInfer failed` | 输出缓冲分配方式与工具链版本不符 | 按第 4.4 节第 5 条核对 |
| 识别框位置整体偏移 | 裁剪/缩放假设与模型训练时不一致 | 确认是"中心裁剪 4:3"而不是 letterbox；两者坐标映射不同 |
| 检测不到目标 | `conf_threshold` 过高 / 量化掉点 / 光照 | 先降到 0.1 看有没有框 → 有框说明是阈值问题；没框就回去查量化 |
| 帧率太低 | CPU 后处理或 BPU 负载 | `ros2 topic hz` 量；BPU 单核 `core_num: 1` 可试 2 |
| `/dart_debug/mask_compressed` 不刷新 | 模型模式不产生 mask | 正常现象，看 `/model_debug/image_compressed` |

---

## 8. 附：本机已有的验证材料

以下不在仓库里（测试程序不纳入版本控制），需要时找维护者要：

- `~/.cache/dart_test/cut.onnx` —— 已切好的模型（(1,9072,28) 已验证）
- `~/.cache/dart_test/ref.json` / `ref_raw.bin` —— 参考检测结果与原始张量
- `~/.cache/dart_test/test_ort.cpp` —— ORT 后端端到端测试（含预处理/解码比对）
- `/tmp/ort` —— ONNX Runtime 1.20.1 C++ 包（开发机验证 ORT 后端用）

已验证过的结论（x86 + ORT 1.20.1）：
切图输出形状、CPU 后处理与模型内置后处理逐位一致、C++ 预处理零像素差、
C++ 解码与参考检测 30/30 类别一致。
