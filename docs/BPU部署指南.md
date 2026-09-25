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
docker run --rm -v /path/to/horizon_work:/work -w /work \
  openexplorer/ai_toolchain_ubuntu_20_x5_cpu:v1.2.8 bash -c "
    hb_mapper makertbin --config dart_576x768.yaml --model-type onnx
    rc=\$?
    chown -R $(id -u):$(id -g) /work
    exit \$rc
  "
```

> 也可以进容器交互调试：`docker run --rm -it -v ...:/work <镜像> bash`，
> 但**退出前记得在容器里 `chown -R <你的uid>:<你的gid> /work`**，否则产物是 root 属主。

产物：`model_output/praysky_dart_576x768.bin`

### 3.5 标定图要求（**直接决定精度**）

- **100~300 张**，覆盖实际比赛的光照/距离/角度；最好就是飞镖赛场用同一台相机拍的
- ❌ 不要用 `AT_NN_Detector/video/` 里那两个视频抽帧：一个是导播直播合成画面、一个是旧系统的 Foxglove 录屏，都不是有效样本
- 尺寸随意（工具链会自己处理），但内容要是真实的装甲板画面

### 3.5.1 采集工具用法（`tools/collect_dataset.py`）

工具**订阅已经在跑的相机节点**存图，而不是自己调 SDK——这样存下来的就是
**比赛那套曝光/增益/分辨率**下的原始帧（`camera_params.yaml` 说了算），
不裁剪、不改色，正好是 hb_mapper 要的输入。

```bash
# 1) 先起相机节点（只起相机，不要起 dart_aim_node —— 要的是原始全画幅图）
ros2 launch hik_camera hik_camera.launch.py

# 2) 采集
python3 tools/collect_dataset.py --out ~/calib --interval 0.5
```

**两种模式**（按有没有屏幕自动选择，也可用 `--no-window` 强制）：

| 场景 | 行为 |
|---|---|
| 有屏幕（桌面/本地终端） | 弹预览窗口（带张数/帧率/模式叠加），**按键控制**：<br>`s`/空格 存一张 · `a` 切换自动模式 · `d` 切换重复过滤 · `q`/ESC 退出 |
| 无屏幕（ssh 上板） | 自动模式，每 `--interval` 秒存一张，**Ctrl-C 结束**（会正常收尾写 meta） |

**参数**：

| 参数 | 默认 | 说明 |
|---|---|---|
| `--out DIR` | `./dataset_<时间戳>/` | 输出目录 |
| `--topic` | `/image_raw` | 图像话题 |
| `--interval` | 1.0 | 自动模式间隔秒；`0` = 纯手动 |
| `--max N` | 0（不限） | 存够 N 张自动退出 |
| `--format` | `jpg` | `jpg` / `png`（png 无损但大很多） |
| `--quality` | 95 | jpg 质量 |
| `--dup-threshold` | 1.5 | 与上一张的平均像素差低于此值就跳过；`0` = 不判断 |
| `--keep-dups` | — | 关掉重复过滤 |
| `--timeout` | 10 | 等第一帧的超时秒数 |

结束时会在输出目录写一份 `meta.json`：图像尺寸、编码、存入张数、跳过重复数、
收到帧数、间隔、起止时间——**方便回溯这批数据是怎么采的**。

### 3.5.2 采集注意事项

1. **必须用比赛时的相机参数采**。数据要代表现场，`exposure_time` / `gain` /
   `pixel_format` 改了，这批数据就白采了。采之前先核对 `camera_params.yaml`。
2. **要"多样性"，不要"数量多但都一样"**。100 张各不相同的图，远好过 300 张同一机位的连拍。
   工具默认会跳过和上一张几乎相同的画面（`--dup-threshold`），但最好还是**每次存之前稍微动一下机位**。
3. **覆盖实际会遇到的条件**：不同光照（白天/逆光/阴影）、不同距离（远/近）、
   不同角度（正对/斜角/仰角），以及靶子占画面大/小两种情况。
4. **目标要出现在画面里，但不必只有目标**。赛场背景、其他机器人、灯光干扰都值得采进去——
   量化标定是按实际分布算的，全是干净空背景反而不好。
5. **不建议用 `--format png`**：无压缩图单张 2~3MB，300 张就接近 1GB；JPEG q95 足够，
   压缩伪影对量化影响可忽略。
6. **数据的去向**：采完把图拷进 `~/horizon_work/calib/`（或直接在采集机上指定
   `--out ~/horizon_work/calib`），然后 `./makertbin.sh` 量化出 `.bin`。
7. **别用 `AT_NN_Detector/video/` 里那两个视频抽帧**：一个是导播直播合成画面、
   一个是旧系统 Foxglove 录屏，都不是有效样本（详见 3.5 开头）。
8. **同一批数据既能当标定集也能当校验集**：量化完用其中一部分图比对
   `.bin` 与 `.onnx` 的输出（见 3.4 的精度验证），能直接看出量化掉没掉点。

### 3.6 已知量化风险

| 风险点 | 说明 | 缓解 |
|---|---|---|
| **Softmax ×2 + MatMul ×4** | PSA 注意力里的 Softmax 对量化不友好，模型作者本人也警告过 | 精度不达标时，把含 Softmax/MatMul 的节点在 `node_info` 里单独配 `quant_type: int16` |
| **Resize ×2** | BPU 对 Resize 有模式/倍数限制 | `hb_mapper checker` 会报，若报错再单独处理 |

**精度验证**（量化后一定要做）：用同一张图，比对 `.bin` 与 `.onnx` 的输出。若类别/框明显变差，就回去调 int16 节点配置。

### 3.7 checker 实测结果（2026-09-22，本机已跑通）

环境：`openexplorer/ai_toolchain_ubuntu_20_x5_cpu:v1.2.8`（hbdk 3.49.15 / horizon_nn 1.1.0 / hb_mapper 1.24.3）

```bash
docker run --rm -v ~/horizon_work:/work -w /work \
  openexplorer/ai_toolchain_ubuntu_20_x5_cpu:v1.2.8 bash -c "
    hb_mapper checker --model-type onnx --march bayes-e --model praysky_dart_576x768_cut.onnx
    rc=\$?
    chown -R $(id -u):$(id -g) /work      # 把产物属主改回自己
    exit \$rc
  "
```

> ⚠️ **不要用 `--user $(id -u):$(id -g)`**：工具链的 torch 装在 `/root/.local`，
> 而 `/root` 是 700，非 root 用户根本读不到，会报 `No module named 'torch'`。
> 正确做法就是上面这样——**以 root 跑，跑完在容器里 chown 回来**（实测有效）。

**结果：零 error / 零 warning / 零 "不支持"**，路径走通。摘要：

| 项 | 结果 |
|---|---|
| 输入 | `images` `[1,3,576,768]` FLOAT32 ✅ 与设计一致 |
| 输出 | `[1,9072,28]` FLOAT32 ✅ 与设计一致 |
| 节点分配 | **328 个 BPU + 2 个 CPU** |
| 量化类型 | 299 个 int8、28 个 int16 |
| 算子映射 | Conv→`HzSQuantizedConv`、MatMul→`HzSQuantizedMatmul`、Resize→`HzQuantizedResizeUpsample`、MaxPool→`HzQuantizedMaxPool`、Sigmoid/激活→`HzLut`、Softmax→**CPU float** |

**两个需要注意的点**：

1. **PSA 注意力的 2 个 Softmax 落在 CPU 上（float 执行）**
   ```
   /model.10/m/m.0/attn/Softmax        CPU  Softmax  float
   /model.22/m.0/m.0.1/attn/Softmax    CPU  Softmax  float
   ```
   这正是模型作者警告过的"PSA 的 SoftMax 对量化不友好"——工具链自动把它挪到 CPU 保精度。
   转换仍然成功（生成的是 hybrid 模型），但**这两处在推理时会走 CPU + 数据在 BPU/CPU 之间来回搬**，
   板子上实测帧率时如果明显偏低，优先怀疑这里。想进一步验证可以看 `.hb_check/` 里导出的子图可视化。

2. **输出张量是 int16**（`Concat_5` / 最后的 `Transpose` 都是 int16）
   → 与 `model_backend_bpu.cpp` 里按 `scale` 反量化的实现一致（S16 分支），无需改代码。

---

## 4. 板子上：编译 + 运行

### 4.1 拷到板子

- 整个工作区源码（或者直接 `git pull`）
- **`praysky_dart_576x768.bin`**（建议放 `src/yq_dart_aim/model/`，注意 `.gitignore` 忽略了 `*.bin`，不会被提交）

### 4.2 编译

```bash
source /opt/ros/humble/setup.bash
cd /root/dart_ws_on
colcon build
```

> BPU / CPU 两个后端只要依赖在，编译时都会编进同一个程序（CMake 自动探测 `libdnn` 与
> onnxruntime），**运行期**再用 `infer_backend` 参数选哪条路径，不用为切后端重新编译。
> 板子上没装 onnxruntime 只是编不进 CPU 后端，不影响 BPU。
>
> 想裁掉某个后端：`colcon build --cmake-args -DDART_ENABLE_ORT=OFF`；
> ORT 装在非标准路径时加 `-DORT_ROOT=<onnxruntime 目录>`。

> 💡 **自编译 ORT 后端时的坑**：只给 `target_link_directories` 不够，运行时（尤其是 systemd
> 启动）会找不到 `libonnxruntime.so.1`。`CMakeLists.txt` 里已经写好了 `BUILD_RPATH` /
> `INSTALL_RPATH`（commit `247315b` 补的），改动时别删掉。

### 4.3 改参数（`src/yq_dart_aim/config/params.yaml`）

```yaml
use_model: true
infer_backend: BPU
model_path_bpu: /root/dart_ws_on/src/yq_dart_aim/model/praysky_dart_576x768.bin
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

**这段日志是设计好的自检输出**——`model_backend_bpu.cpp` 最早是在没有板子的机器上写的，
API 名称全靠推断，所以打印张量信息便于核对。

**2026-09-22 已在板子上编译对齐（commit `247315b`）**。最初推断与实际 hobot_dnn 1.24.5
的差异如下（均已改正，记录在此备查）：

| 最初推断 | 实际接口（hobot_dnn 1.24.5） |
|---|---|
| `hbDNNPackedHandle_t` | `hbPackedDNNHandle_t` |
| `hbDNNRelease(&handle)` | `hbDNNRelease(handle)` |
| `p.numDimensions` | `p.validShape.numDimensions` |
| `p.validShape[i]` / `p.alignedShape[i]` | `p.validShape.dimensionSize[i]` / `p.alignedShape.dimensionSize[i]` |
| `sysMem[i].vir_addr` | `sysMem[i].virAddr` |
| `hbDNNInfer(&task,&out,&in,model)` | 多一个控制参数：`hbDNNInferCtrlParam` + `HB_DNN_INITIALIZE_INFER_CTRL_PARAM(&ctrl)` |
| 输出缓冲字节数 = alignedShape 连乘 | 优先用 `p.alignedByteSize` |

日志里要核对的字段（现在还多打一个 `quanti=`）：

| 检查项 | 期望值 | 不符说明什么 |
|---|---|---|
| 输入类型 | `IMG_NV12*`（yaml 配 nv12 时）或 `S8/F32`（featuremap） | 两种都已实现；对不上就查 hb_mapper 的 `input_type_rt` |
| 输入形状 | `[1,3,576,768]` | 与 `model_input_*` 或 .bin 不一致 |
| 输出形状 | `[1,9072,28]` | 切点或模型版本变了 |
| 输出类型 + scale | 量化类型（如 `S16`）+ scale 非 0 | 代码按 `scale` 反量化；scale 为 0 要查 |
| 推理调用 | 不出现 `hbDNNInfer failed` | 出现了就对 `g_outputs` 的分配方式 |

> ⚠️ **当前状态**：2026-09-22 板子上实际运行的是 **ORT 后端**（`model_path` 指向
> 切好的 `.onnx`，见 `screen.output`）；BPU 后端已完成接口对齐并编译通过，
> **是否已在板上实际推理验证，请与维护者确认**。

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
5. **相机启动时那条 `Failed to set PixelFormat` 是正常的**（CU013 不支持 BGR8 输出格式）。
   代码会保留相机默认格式，再由 SDK 内部转成 BGR8 发布，话题 `encoding` 仍是 `bgr8`，
   下游不受影响。想要日志干净就把 `camera_params.yaml` 的 `pixel_format` 改回 `RGB8`。
6. **曝光时间直接决定帧率上限**：当前 `exposure_time: 50000`（= 50ms）→ 相机最多 **20fps**。
   模型推理再叠加几十~几百毫秒，实际出图率会更低。如果瞄准环需要更高帧率，先从这里下手。
7. **`use_model=true` 打不开模型时会明确报错并退回 HSV**（不会静默用错模式）：
   - `model_path` 为空 → `ERROR: use_model=true 但 model_path 为空…`
   - 加载失败 → `ERROR: 模型加载失败: <路径>…`
   日志里看到这类 ERROR 就说明当前跑的是 HSV。
8. **`.bin` 不要提交进 git**（`.gitignore` 已忽略 `*.bin`），板子上单独放。
9. **模型是 AGPL-3.0**，本仓库是公开仓库，别把 `.onnx`/`.bin` 提交进去。

### 6.2 与其它模块的联动

10. **偏移量表要按现场重新标定**（`offset_0_*` / `offset_1_*` / `offset_2_*`）。
   注意代码已经修正过"偏移被叠加两次"的 bug，**现在的补偿量只有修复前的一半**，
   旧值一律不要用。标定方法：打一发 → 看弹着点偏多少 → 该目标该飞镖的 offset 加减对应像素。
11. **`crop_width` / `crop_height` 与偏移表绑定**（HSV 路径的图像中心与裁剪图尺寸有关）。
   模型路径的坐标已经映射回同一个裁剪图坐标系，所以改裁剪尺寸时 **HSV 的偏移要重标，模型路径不用**。
12. **串口数据校验**：下位机发来的 `enemy` 只能是 0/1/2、`cur` 只能是 0~3。
    收到越界值（比如 9、-3）时，代码**丢弃这个包并保留上一次的有效值**，同时打 WARN 日志。
    这样单帧脏数据不会让瞄准跳到一个不存在目标上。
13. **模型内置 TopK，不需要 NMS**。所以 `nms_threshold` 参数已被删除，
    后处理里也没有 NMS 环节（候选框之间不会互相抑制，靠 `conf_threshold` 过滤）。

### 6.3 模型本身的已知限制（作者原话）

14. **不识别"基地小装甲"**（RM2026 新增的目标，训练集未覆盖）。
15. **量化版本下 Outpost 偶有异常**（w8a16 时），作者判断是数据集脏数据，不影响拟合。
16. 模型覆盖：所有参赛环境的目标（轨道哨兵、5 号小装甲步兵、3/4/5 号大装甲步兵等）。

---

## 7. 排障速查

| 现象 | 原因 | 处理 |
|---|---|---|
| 编译日志 `跳过 CPU(ORT) 后端` | 板子上没装 onnxruntime | 正常，不影响 BPU；要 CPU 路径就装 ORT 并加 `-DORT_ROOT=` |
| 启动日志 `推理后端 ... 未编译进本程序` | 该后端被 `-DDART_ENABLE_*=OFF` 关掉了，或缺依赖 | 去掉该开关重新编译 |
| 启动日志 `模型加载失败` | `.bin` 路径错 / 格式不匹配 / 后端选错 | 检查 `model_path_bpu` 与 `infer_backend` 是否都是 BPU |
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
