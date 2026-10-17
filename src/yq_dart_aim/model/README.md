# RDK X5 装甲板检测模型量化部署指南（horizon_work）

> 记录一次从「项目搁置很久」到「成功产出可上板的 `.bin`」的完整过程：
> 分析思路、成功路线、踩过的坑、经验沉淀。
>
> 最后更新：2026-10-16　　目标硬件：**RDK X5（地平线 BPU，march = bayes-e）**

---

## 0. 结论速览（TL;DR）

- 项目：把开源装甲板检测模型（`AT_NN_Detector`，河北科大 AT 战队，AGPL-3.0）部署到 RDK X5 的 BPU 上。
- 已成功产出：**`model_output/praysky_coord_576x768.bin`**（5.05 MB，全 BPU、单 subgraph、输入 `nv12 576×768`）。
- 采用的路线不是「完整端到端模型直接编」，而是：
  **CoordAtt 主干 → 裁掉 NMS 后处理（cut）→ 量化编译成 `.bin`**。
- 后处理（decode + NMS）需在 CPU / 自研代码里完成，模型只输出检测头原始张量 `[1, 9072, 28]`。
- 最关键的教训：**带 `NonMaxSuppression` / `NonZero` 的完整模型，地平线运行时（HorizonRT）根本不支持，无法编译，必须 cut 掉后处理。**

> ## ✅ 板端部署回执（2026-10-17 更新）
>
> 本 .bin（CoordAtt 版）已正式上板、进服务实跑。与本文原始记录的差异（均已实测修正）：
>
> 1. **运行时输出不是 S16 `[1,9072,28]`，而是 F32 `[1,9072,28,1]`**（工具链在输出前已反量化，
>    scale=1）；板端张量解析已按 4D 适配。
> 2. **NMS 用「框 IoU」实现**（同类、阈值 0.7 = 模型内置 `NonMaxSuppression` 的 iou_threshold）：
>    实测量化后关键点跨 anchor 抖动大，用「4 关键点四边形」做 IoU 会漏压同类重复框。
> 3. **推理预处理 = letterbox**（与本文的标定预处理一致）；坐标反解用 `(x−pad)/scale`。
> 4. 实测板端单帧 ~22ms（≈45fps 上限），清晰目标置信度较 float 掉 0.03~0.08。
>
> 完整经验（选型对比、三层验证法、踩坑清单、实测数据）见板端仓库：
> `/root/dart_ws_on/docs/量化部署经验总结.md`。

---

## 1. 背景与目标

最初的目标是在 RDK X5 上用 BPU 做「端到端模型识别」。项目开到一半搁置，重启时环境与状态都不清楚。
本指南把重启后的完整排查与打通过程记录下来，供后续复现和排错。

核心问题链路：
**ONNX 模型 → 量化（int8）→ 编译成 RDK X5 能加载的 `.bin` → 上板推理。**

---

## 2. 先分清两条线：RDK X5 线 vs 算能线

本机同时存在两个「量化部署」项目，**芯片不同、工具链不同、产物不同，绝不能混用**：

| | 本项目（RDK X5） | 并行项目（算能） |
|---|---|---|
| 目录 | `~/horizon_work` | `~/tpu-workspace/dart_quant` |
| 芯片 | 地平线 **BPU**（Sunrise X5） | 算能 **TPU** |
| 工具链 | **hb_mapper / hbdk**（OpenExplorer） | **TPU-MLIR**（sophgo） |
| 容器镜像 | `openexplorer/ai_toolchain_ubuntu_20_x5_cpu:v1.2.8` | `sophgo/tpuc_dev:v3.4` |
| 产物 | `.bin` | `.bmodel` |

> 类比：同一个模型，要分别「翻译」成两种芯片各自读得懂的格式。
> X5 **只认 `.bin`**，算能那边就算全部跑通，X5 也不能直接用，必须走自己的工具链重来一遍。
>
> 容易混淆的地方：两边的**原始素材可共享**（ONNX 模型、真实图片），
> 但**各自的产物（`.bin` / `.bmodel`）专属，不能互换**。

---

## 3. 环境现状

| 项 | 值 |
|---|---|
| 系统 | Ubuntu 22.04 / x86_64（编译在 x86 容器里做，不是在 X5 上） |
| Docker | 正常，当前用户免 sudo |
| 工具链镜像 | `openexplorer/ai_toolchain_ubuntu_20_x5_cpu:v1.2.8`（7.37 GB） |
| hb_mapper | 1.24.3 |
| hbdk | 3.49.15（runtime 3.15.55.0） |
| horizon_nn | 1.1.0 |
| march | `bayes-e`（= RDK X5） |
| RDK X5 板子 | 编译阶段用不到；上板验证时才需要（当时**未连接**） |

**重要：容器里必须以 root 跑**（工具链的 torch 装在 `/root/.local`，非 root 读不到）。
脚本里跑完会 `chown` 回当前用户，避免产物变成 root 属主删不掉。

---

## 4. 分析与决策过程（思路复盘）

这一节是「为什么这么做」，把每一步的推理记下来，比命令本身更重要。

### 4.1 第一步：先搞清楚「到底在哪个项目、哪块板」

- `~/horizon_work` 时间戳停在 9/22，卡在标定图（`calib/` 为空）；
- `~/tpu-workspace/dart_quant` 是**今天刚动过**的，还带「端到端(e2e)」模型。
- 一度以为「端到端」在算能那边、和 RDK X5 矛盾。**结论：以 RDK X5 线（`horizon_work`）为准**，
  算能线是并行的另一条部署，互不干扰。

### 4.2 模型选型：C2PSA 还是 CoordAtt？

开源包 `AT_NN_Detector` 提供 4 个模型（C2PSA / CoordAtt × 640×640 / 576×768）。
关键情报来自其 README：

> 「C2PSA 中的 **SoftMax 在 uint16/uint8 量化时会有精度损失**」

这正好对应 `horizon_work` 里旧模型 `praysky_dart_576x768_cut.onnx` 的现象：
checker 显示有 **2 个注意力 `Softmax` 落在 CPU**，模型被切成 **3 段 subgraph**（既慢又伤精度）。

作者自己在实际部署时，就是**把 P5 的 PSA 换成 CoordAtt**（全卷积 + Sigmoid，量化友好），
代价是 mAP 从 0.89 降到 0.87。

→ **决策：改用 CoordAtt 版**（`praysky_coord_noe2e_0331_576x768.onnx`）。

### 4.3 完整模型能不能直接编？——不能，必须 cut

直接对**完整** CoordAtt 模型跑 checker，**失败（退出码 255）**：

```
ERROR *** ERROR-OCCUR-DURING {runtime.runtime_model_generation} ***
error message: HorizonRT not support these cpu operators: NonMaxSuppression NonZero
```

要点：完整模型内置了 NMS 后处理（`NonMaxSuppression`、`NonZero`、`GatherND`、`ScatterND`…）。
其中 **`NonMaxSuppression` 和 `NonZero` 连「丢 CPU」都不行**——HorizonRT 直接不支持，编译直接报错。
（其余 40+ 个后处理算子可以落 CPU，但这两个是致命的。）

→ **必须把后处理切掉（cut），模型只保留 BPU 主干。**

> 这也解释了旧模型为什么叫 `..._cut.onnx`：原作者/前人就是这么处理的。

### 4.4 怎么确定 cut 点？

对比两个模型的检测头结构，发现**输出位置完全一致**：

| | C2PSA cut（旧） | CoordAtt 完整 |
|---|---|---|
| 检测头输出张量 | `/model.23/Transpose_output_0` | `/Transpose_output_0` |
| 形状 | `[1, 9072, 28]` | `[1, 9072, 28]` |

- `9072` = 三个尺度 anchor 数之和（72×96 + 36×48 + 18×24）。
- `28` = 每个 anchor 的原始输出（4 box + 12 类 + 4 颜色 + 8 关键点），为检测头**未解码**输出。

→ **在 `/Transpose_output_0` 处切断**，丢掉它后面的所有 NMS 节点。
用 `onnx.utils.extract_model()` 按「输入 images、输出 /Transpose_output_0」截取即可，
产出的 cut 模型**接口与旧 c2psa cut 完全一致**，后处理代码可以原样复用。

### 4.5 标定数据的「格式坑」

`hb_mapper makertbin` 第一次直接报：

```
ERROR load cal data for input 'images' error
ERROR cannot reshape array of size 134317 into shape (1,3,576,768)
```

排查发现 `134317 × 4 ≈ 537271`（正好是那张 PNG 的字节数）——**工具链把 PNG 当原始 float32 二进制在读**。
翻源码（`horizon_tc_ui/data/loader.py`）确认，两条路径：

- `RawImagesDirLoader`（默认路径）：`np.fromfile(file, dtype).reshape(shape)` → 只认**纯二进制 float32**。
- `ImageDirLoader`（`preprocess_on: True` 时）：用 skimage 解码图片 → 自动 resize + `/255`。

→ **决策：图片走 `preprocess_on: True`**（改动最小）。
（另一条等价路线是把自己预处理好的 `[0,1] float32 NCHW` 用 `.tofile()` 存成 `.bin` 喂给默认路径；
两条路最终都归到 `[0,1] RGB`，效果一致。）

---

## 5. 成功路线（可复现步骤）

假设已在 `~/horizon_work`，镜像已存在。依次执行：

### 步骤 1　生成 cut 模型（裁掉 NMS 后处理）

```bash
cd ~/horizon_work
# 先把开源完整模型拷进来（若还没有）
cp ~/下载/开源/AT_NN_Detector/common/praysky_coord_noe2e_0331_576x768.onnx .
# 裁出 cut 版：输出张量固定为 /Transpose_output_0
./make_coord_cut.sh
# 产出 praysky_coord_576x768_cut.onnx（输入 images [1,3,576,768]，输出 [1,9072,28]）
```

### 步骤 2　checker 验证算子支持

```bash
./checker.sh praysky_coord_576x768_cut.onnx
# 期望：退出码 0；grep CPU 无输出（全 BPU）；无 Softmax
# 产物：.hb_check/（含各 subgraph、量化模型）
```

### 步骤 3　准备标定图（letterbox 到 576×768）

```bash
# 校验集：~/下载/校验集（base/outpost × c1/c2 各 100 张，1280x872 实拍）
python3 prepare_calib.py
# 产出 calib/*.png（400 张，768x576）
# ⚠️ calib/ 里只能放图片，别混入 .md / 其它文件
```

### 步骤 4　量化编译成 .bin

```bash
./makertbin.sh          # 内部用 dart_576x768.yaml
# 约 40 分钟后产出 model_output/praysky_coord_576x768.bin
```

### 步骤 5　校验产物

```bash
docker run --rm -v "$PWD:/work" -w /work \
  openexplorer/ai_toolchain_ubuntu_20_x5_cpu:v1.2.8 \
  hb_model_info model_output/praysky_coord_576x768.bin
# 期望：BPU march = bayes-e；input images nv12 1x3x576x768；output FLOAT
# subgraph 数应为 1（只有 subgraph_0），说明没有 CPU 段被拆出来
```

---

## 6. 踩过的坑（清单）

| # | 现象 | 原因 | 解决 |
|---|---|---|---|
| 1 | checker 报 `HorizonRT not support ... NonMaxSuppression NonZero` | 完整模型内置 NMS 后处理，这两个算子不支持 | 用 `extract_model` 裁掉后处理（cut） |
| 2 | C2PSA 模型 2 个 `Softmax` 落 CPU，模型被切成 3 段 | PSA 注意力里的 Softmax 量化不友好 | 换 **CoordAtt** 版 |
| 3 | `cannot reshape array of size 134317 into shape (1,3,576,768)` | 默认 loader 把 PNG 当 float32 二进制读 | 标定喂**图片**时设 `preprocess_on: True` |
| 4 | `Could not find a backend to open '.../放置标定图.md'` | `calib/` 里混了非图片文件，skimage 打不开 | 把说明文件移出 `calib/` |
| 5 | `docker run ... python3 - <<'PY'` 无任何输出 | 忘了加 `-i`，heredoc 没进容器 stdin | 加 `-i`，或把脚本挂载进容器 |
| 6 | 产物属主变 root，删不掉 | 容器内以 root 运行 | 脚本跑完 `chown -R $(id -u):$(id -g)` |
| 7 | 标定图分布不对 | 模型训练用 letterbox，工具链只会直接 resize | 预先把图 letterbox 到 576×768 |
| 8 | `preprocess_on` 有 deprecation 警告 | 官方建议改用 `calibration_type: skip`（随机数据，不准） | 忽略警告；或走 raw `.bin` 路径 |

---

## 7. 产物清单

```
horizon_work/
├── README.md                              # 本文档
├── 标定图说明.md                          # 标定图要求（从 calib/ 移出）
├── dart_576x768.yaml                      # 量化编译配置（已指向 coord cut）
├── checker.sh                             # 算子检查脚本
├── makertbin.sh                           # 量化编译脚本
├── make_coord_cut.sh                      # 生成 cut 模型（切 NMS）
├── prepare_calib.py                       # 生成 letterbox 标定图
├── praysky_coord_noe2e_0331_576x768.onnx  # 开源完整 coord 模型（含 NMS）
├── praysky_coord_576x768_cut.onnx         # ★ cut 版（BPU 可编译，输出 [1,9072,28]）
├── praysky_dart_576x768_cut.onnx          # 旧 c2psa cut（已弃用，保留对比）
├── calib/                                 # 400 张 letterbox 后的标定图
└── model_output/
    ├── praysky_coord_576x768.bin          # ★ 最终产物：RDK X5 可加载
    ├── *_original_float_model.onnx        # 中间产物
    ├── *_optimized_float_model.onnx
    ├── *_calibrated_model.onnx
    ├── *_quantized_model.onnx
    ├── *_quant_info.json                  # 逐层量化信息（调精度用）
    └── Extracted from {main_graph}_subgraph_0.*   # 计算图可视化（html/json）
```

---

## 8. 遗留事项与下一步

1. **后处理要自己写**：cut 掉了 NMS，模型输出 `[1,9072,28]` 需在 CPU / 自研代码里做
   **解码 + NMS**。组成（从计算图推断）：`4 box + 12 类 + 4 颜色 + 8 关键点 = 28`。
   可参考算能线（`dart_quant`）或开源 `AT_NN_Detector/axera` 里的后处理。
2. **上板验证**：RDK X5 当时未连接（无串口、网络不通）。需要用 `hb_perf` 看性能，
   用板端 runtime 加载 `.bin` 推理。
3. **精度验证**：用校验集对比 float 模型与量化模型的输出；若精度不够，
   把 `calibration_type` 从 `default` 调成 `mix`（对精度敏感节点配 int16）。
4. **可选优化**：`input_source` 目前默认 `pyramid`；如需 resizer 路径可显式配置。

---

## 9. 经验与最佳实践

1. **先确认「哪块板、哪条工具链」再动手**。RDK X5 = hb_mapper/.bin；算能 = TPU-MLIR/.bmodel，产物不通用。
2. **用 `hb_mapper checker` 做第一道门**：它不消耗标定图、几十秒出结果，能提前暴露
   不支持的算子（尤其是 `NonMaxSuppression` / `NonZero` 这类「不可落 CPU」的）。
3. **量化优先选「量化友好」的主干**：全卷积 + Sigmoid（CoordAtt）远好于带 Softmax 的注意力（C2PSA）。
4. **检测模型的后处理几乎都要 cut**：BPU 生态里，NMS 这类后处理通常不放芯片上，
   而是「主干出 BPU、后处理回 CPU」。cut 点选在检测头原始输出处，接口保持稳定便于复用。
5. **标定数据格式要吃透**：默认 loader 只认纯二进制 float32；图片必须开 `preprocess_on`。
   两条路殊途同归（都归到 `[0,1] RGB`），选一条即可。
6. **标定预处理必须与推理预处理一致**：训练用 letterbox，标定图就要预 letterbox，否则分布错位。
7. **`subgraph` 数量是健康指标**：只有 `subgraph_0` 说明全 BPU；出现 `_1`/`_2` 意味着有 CPU 段被拆出，要警惕性能/精度。
8. **容器以 root 跑、跑完 chown 回来**：避免产物属主混乱。
9. **调 hb_mapper 出问题就翻源码**：`/usr/local/lib/python3.10/dist-packages/horizon_tc_ui/`
   下的 `data/loader.py`、`helper.py`、`config/mapper_conf_parser.py` 能直接解释报错。

---

## 10. 附录

### 10.1 关键命令

```bash
# 检查算子
./checker.sh <model.onnx>

# 量化编译
./makertbin.sh [config.yaml]

# 查看 bin 信息
docker run --rm -v "$PWD:/work" -w /work \
  openexplorer/ai_toolchain_ubuntu_20_x5_cpu:v1.2.8 \
  hb_model_info model_output/praysky_coord_576x768.bin

# 生成 cut 模型
./make_coord_cut.sh

# 生成标定图
python3 prepare_calib.py
```

### 10.2 输入输出规格（最终模型）

| 项 | 值 |
|---|---|
| 输入名 | `images` |
| 输入格式 | `nv12`，layout `NCHW`，形状 `1×3×576×768` |
| 训练域 | RGB，`data_scale`，scale = 1/255 |
| 输出 | `[1, 9072, 28]`，FLOAT |
| 预处理 | Letterbox（等比缩放 + pad=114），BGR→RGB |
| march | `bayes-e` |

### 10.3 相关路径

| 用途 | 路径 |
|---|---|
| 本项目 | `~/horizon_work` |
| 开源模型包 | `~/下载/开源/AT_NN_Detector`（模型在 `common/`） |
| 校验集（标定素材） | `~/下载/校验集` |
| 并行算能项目 | `~/tpu-workspace/dart_quant` |
| 工具链源码 | 容器内 `/usr/local/lib/python3.10/dist-packages/horizon_tc_ui/` |

### 10.4 参考

- `AT_NN_Detector/README.md`（模型结构、类别、输入输出、量化经验）
- horizon_tc_ui 源码（`data/loader.py` / `helper.py` / `config/mapper_conf_parser.py`）

---

*本文档由项目重启过程中的实际排查整理而成，命令与结论均在本机验证过。*
