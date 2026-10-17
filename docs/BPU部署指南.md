# RDK X5 BPU 部署指南

> 适用：`dart_aim_node` 的**模型识别模式**走 RDK X5 的 BPU 加速。
> 模型来源：`AT_NN_Detector`（PraySky 的 YOLO26n-Pose 装甲板检测，AGPL-3.0）。
> 本文覆盖：**从 ONNX 到板子上跑起来**的完整流程 + 所有已知坑。
>
> 📌 **实战经验版（推荐先看）**：`docs/量化部署经验总结.md` —— 汇总了最终方案选型、
> 三层验证法、板端实测数据与完整踩坑清单；本文是工具链操作的详细版。

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

> **2026-10-16 更新（当前部署的模型）**：现用 `praysky_coord_576x768.bin`（CoordAtt 版）。
> 它由 `praysky_coord_noe2e_0331_576x768.onnx`（496 节点）切出，切点 `/Transpose_output_0`，
> 输出同样是 `(1, 9072, 28)`。与上面的 c2psa 不同：coord 完整模型内置的是 **NMS**
> （`NonMaxSuppression` / `NonZero`，地平线运行时不支持，同样必须切掉），
> 且 coord 主干里没有落 CPU 的 Softmax（全 BPU、单 subgraph）。
> 板端后处理现状见 §6.2.15（已加同类框 IoU NMS）。

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

### 3.7 checker 实测结果（2026-09-22，旧 C2PSA cut 的记录）

> 换成 CoordAtt 版后「2 个 Softmax 落 CPU」的问题已不存在：coord 版为**全 BPU、
> 单 subgraph**（详见 `model/README.md`）。下面这段作为历史记录保留。

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

2. ~~输出张量是 int16~~ **实测最终 `.bin` 输出是 F32**（工具链在输出前已插入反量化，
   scale=1，形状是 4 维 `[1,9072,28,1]`）。`model_backend_bpu.cpp` 已适配 4D 输出的解析
   （2026-10-16 修复，原先只认最后两维会解析成 n=1）；按 scale 反量化的分支保留，
   用于兼容 S16 输出的模型。

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
model_path_bpu: /root/dart_ws_on/src/yq_dart_aim/model/praysky_coord_576x768.bin
model_input_width: 768
model_input_height: 576
conf_threshold: 0.25      # 先 0.25，误检多就往上调
model_nms_threshold: 0.7  # 同类框 IoU NMS，0.7 = 与模型内置 NMS 一致
```

> ⚠️ `params.yaml` 是唯一权威参数来源。`dart_aim_node` 代码里的默认值只是兜底，
> 两者已对齐，但**改参数请改 yaml**。

### 4.4 第一次运行：**必看这段日志**

```bash
ros2 launch yq_dart_aim start.launch.py
```

BPU 后端加载时会打印（`[model_backend_bpu]`），下面是 2026-10-16 在板上的实测输出：

```
input  type=IMG_NV12 quanti=0 valid=[1,3,576,768] aligned=[1,3,576,768] stride=[0,0,0,0]  scale=1.000000
output type=F32      quanti=0 valid=[1,9072,28,1] aligned=[1,9072,28,1] stride=[1016064,112,4,4] scale=1.000000
BPU model loaded: 768x576 nv12
```

注意：输出是 **4 维 `[1,9072,28,1]` 且为 F32**（trailing 1 维要忽略、无需反量化）；
输入 NV12 的 stride 读出来是全 0（运行时按紧凑布局处理，backend 里 `y_stride<=0` 时回退到宽度）。

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
| 输出类型 + scale | 实测是 **F32**（4 维 `[1,9072,28,1]`，scale=1，无需反量化） | 形状/类型对不上就查切点或模型版本 |
| 推理调用 | 不出现 `hbDNNInfer failed` | 出现了就对 `g_outputs` 的分配方式 |

> ✅ **当前状态（2026-10-16）**：BPU 后端已上板实跑（`infer_backend: BPU`，
> `model_path_bpu` 指向 `praysky_coord_576x768.bin`）。验证结论：
>
> - **全链路一致**：把校验集图片注入 `/image_raw` 走真实节点链路（cropCenter →
>   letterbox → NV12 → BPU → 解码+NMS），类别/框/置信度与 PC 离线 ONNX 逐项对齐
>   （例：s0_o6 float 0.780 ↔ 板端画面标注 78%）。
> - **性能**：`bpuInfer` 全流程（含 BGR→NV12 转换）实测约 **22 ms/帧**（p50 21.8ms），
>   对比 ORT CPU 约 0.5 s/帧提速 20 倍以上；`dart_aim_node` CPU 占用约 3.8 核 → 约 1.5 核。
> - **量化精度**：清晰目标顶多掉 0.03~0.08 置信度、类别稳定；边界目标（0.3~0.6 级）
>   存在置信度下滑甚至类别翻转，必要时提高 `conf_threshold` 或改用 mix(int16) 重新量化。

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
4. **模型预处理 = letterbox**（2026-10-16 起；此前用的是 4:3 中心裁剪，会丢画面左右各 160px）。
   1280×720 等比缩到 768×432、上下补灰(114)到 768×576，与训练和 `.bin` 量化标定分布一致。
   实测 letterbox 在边界样本上置信度明显更高（例：outpost_c2 某帧 float 0.77 vs 裁剪 0.42；
   量化后 0.39 vs 0.22，后者已掉出 0.25 阈值）。坐标反解用 `LetterboxMap`（减 pad 除 scale），
   检测结果仍映射回裁剪图坐标系，偏移表不受影响。
5. **相机启动时那几条 `PixelFormat` 报错是正常的，且无法消除**。这台相机
   （Hikrobot MV-CU013-A0UC）**只支持 Bayer 输出，不支持 BGR8/RGB8 直出**：
   `MV_CC_GetPixelFormat` 返回的支持列表里只有 5 个 Bayer/Mono 格式；
   GenICam XML 里 BGR8 的可用条件绑定在 `ADCBitDepth != 0`，而该寄存器是只读的
   （`MV_CC_WriteMemory` 返回 `MV_E_NO_PERMISSION`），所以**相机 ISP 做不了去马赛克+上色**，
   这一步只能在主机上做。代码已用 OpenCV 双线性去马赛克替代了慢 5 倍的 SDK 转换。
   若配置 `pixel_format: RGB8` 想让日志干净，注意 OpenCV 去马赛克只能产出 **BGR**，
   此时话题 `encoding` 会被如实改成 `bgr8`（不改字节顺序），下游按 encoding 解释即可。
6. **裁剪已下放到相机（ROI）**。`camera_params.yaml` 里的 `roi_height: 720` 让相机
   只输出下游要用的那 720 行，不再是「相机吐全幅、主机再裁」。
   ⚠️ **`roi_offset_y` 固定为 76，不要改回 -1（自动居中）**：相机节点读 Height 上限
   是在复位 Offset 之前做的，上限值取决于相机上次断电前的状态，用 -1 会让 ROI 窗口
   每次重启在 @76 / @152 / 回退全幅之间跳变（2026-10-16 实锤，历史日志里有完整证据）。
   76 = 标定时代「原软裁剪」的等价窗口（sensor 行 76..796），实测逐像素一致，
   **标定偏移表和模型输入都不用重做**；改这个窗口 = 必须重新场地标定偏移。
   实测收益：相机节点 CPU 25%→19%（单核），去马赛克 4.47ms→3.06ms，帧率 113→134fps。
   改 `crop_height` 时要同步改 `roi_height`，否则会多传或少传用不到的行。
7. **曝光时间直接决定帧率上限**：当前 `exposure_time: 50000`（= 50ms）→ 相机最多 **20fps**。
   模型推理再叠加几十~几百毫秒，实际出图率会更低。如果瞄准环需要更高帧率，先从这里下手。
8. **调试图不要开、限多少 fps**。JPEG 编码 1280×720 实测约 20ms/帧，跟着相机满帧率发的话
   单这一项就要占掉两个多核，图像回调会被拖垮。现在调试图已挪到独立线程并限速
   （`debug_max_fps`，默认 10fps）；调试完可以直接 `publish_debug_image:=false` 关掉。
9. **`use_model=true` 打不开模型时会明确报错并退回 HSV**（不会静默用错模式）：
   - `model_path` 为空 → `ERROR: use_model=true 但 model_path 为空…`
   - 加载失败 → `ERROR: 模型加载失败: <路径>…`
   日志里看到这类 ERROR 就说明当前跑的是 HSV。
10. **`.bin` 不要提交进 git**（`.gitignore` 已忽略 `*.bin`），板子上单独放。
11. **模型是 AGPL-3.0**，本仓库是公开仓库，别把 `.onnx`/`.bin` 提交进去。

### 6.2 与其它模块的联动

12. **偏移量表要按现场重新标定**（`offset_0_*` / `offset_1_*` / `offset_2_*`）。
   注意代码已经修正过"偏移被叠加两次"的 bug，**现在的补偿量只有修复前的一半**，
   旧值一律不要用。标定方法：打一发 → 看弹着点偏多少 → 该目标该飞镖的 offset 加减对应像素。
13. **`crop_width` / `crop_height` 与偏移表绑定**（HSV 路径的图像中心与裁剪图尺寸有关）。
   模型路径的坐标已经映射回同一个裁剪图坐标系，所以改裁剪尺寸时 **HSV 的偏移要重标，模型路径不用**。
14. **串口数据校验**：下位机发来的 `enemy` 只能是 0/1/2、`cur` 只能是 0~3。
    收到越界值（比如 9、-3）时，代码**丢弃这个包并保留上一次的有效值**，同时打 WARN 日志。
    这样单帧脏数据不会让瞄准跳到一个不存在目标上。
15. **后处理含同类框 IoU NMS**（2026-10-16 起）：coord 完整模型内置的是 **NMS**
    （`NonMaxSuppression`，iou_threshold=0.7，输入是**框**不是关键点），板端 cut 后
    用**框**做**同类内** NMS，阈值参数 `model_nms_threshold`（默认 0.7），
    NMS 前候选池上限 300（对齐 prompt 文档的 top-k）。
    ⚠️ 别改回"用 4 关键点四边形做 IoU"：实况帧实测量化模型的关键点在邻近 anchor 之间
    抖动很大（角点 451↔464、232↔244 这样跳），同一目标的候选框 IoU 稳定在 0.78~1.0，
    但关键点四边形包围框 IoU 在 0.47~1.0 乱跳——会出现两个同类别框（实况调试图上
    真出现过：两条 s0_o6 0.853、框 IoU=1.0 却因关键点 IoU=0.6 没被压掉）。
    实测 cut+后处理的 top1 与模型内置 NMS 逐位一致。
    （旧 c2psa 模型内置的是 TopK 而非 NMS，没有这个环节，已随模型更换更新。）

### 6.3 模型本身的已知限制（作者原话）

16. **不识别"基地小装甲"**（RM2026 新增的目标，训练集未覆盖）。
17. **量化版本下 Outpost 偶有异常**（w8a16 时），作者判断是数据集脏数据，不影响拟合。
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
| 识别框位置整体偏移 | 预处理与坐标反解不配对 | 模型模式现在 = letterbox + `LetterboxMap` 反解，改预处理必须同步改反解 |
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
