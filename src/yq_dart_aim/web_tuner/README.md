# 飞镖调参系统 (Web Tuner)

## 简介

浏览器打开即用的零门槛调参工具，机械组/视觉组都能用。
支持手机和笔记本同时访问，参数修改实时生效。
无后端时自动降级到演示模式，可离线预览全部 UI。

---

## 依赖安装

```bash
pip3 install flask flask-socketio pyyaml
```

> RDKX5 上执行一次即可，已安装过则跳过。

---

## 使用方式

### 方式1：随主程序一起启动（推荐比赛用）

```bash
ros2 launch yq_dart_aim start.launch.py web_tuner:=true
```

可选参数：
```bash
# 修改端口（默认 8080）
ros2 launch yq_dart_aim start.launch.py web_tuner:=true web_tuner_port:=9090

# 同时关闭 foxglove
ros2 launch yq_dart_aim start.launch.py web_tuner:=true debug:=false
```

### 方式2：单独启动（主程序已在运行时）

```bash
# 确保 ROS2 环境已 source
source /opt/ros/humble/setup.bash
source install/setup.bash

# 启动 web tuner
python3 src/yq_dart_aim/web_tuner/server.py --port 8080
```

### 方式3：通过 systemd watchdog 启用

编辑 `/etc/systemd/system/rm.service` 或设置环境变量：

```bash
export WEB_TUNER=true
export WEB_TUNER_PORT=8080
```

watchdog 会自动拉起 web_tuner 并监控其存活。

### 方式4：本地预览 UI（无需 ROS2）

只需看界面效果、不需要调参功能时：

```bash
cd src/yq_dart_aim/web_tuner/static
python3 -m http.server 8080
```

浏览器打开 `http://localhost:8080`，页面自动加载内置演示数据，所有控件可交互。

### 访问

浏览器打开：`http://<RDKX5的IP>:8080`

手机和笔记本可同时访问，修改参数会双向同步。

---

## 参数分组说明

| 分组 | 参数 | 说明 |
|------|------|------|
| HSV 颜色阈值 | h_min/h_max, s_min/s_max, v_min/v_max | 目标颜色筛选 |
| 形态学处理 | morph_kernel_size, morph_dilate_kernel_size | 去噪、连接断裂区域 |
| 目标筛选 | max_area, circle_mask | 面积和形状过滤 |
| 坐标偏移 | p_err, offset_y, offset_X_Y (12个) | 瞄准点偏移补偿 |
| 相机参数 | exposure_time, gain | 曝光和增益（hik_camera 节点） |
| 模型检测 | use_model, conf_threshold, nms_threshold | ONNX 模型推理 |
| 系统设置 | crop_width/height, use_serial, publish_* | 裁剪、串口、调试输出 |

---

## 预设配置说明

### 内置预设

| 预设名 | 用途 | 包含的参数 |
|--------|------|-----------|
| `match_day`（比赛模式） | 正式比赛 | HSV阈值 + 形态学 + 偏移 + 串口开启 |
| `debug`（调试模式） | 开发调试 | 关闭串口，开启全部调试输出 |

### 什么时候用哪个预设？

**比赛模式 (`match_day`)：**
- 正式比赛上电后
- 串口已连接电控MCU
- 需要稳定输出瞄准误差
- 参数已调好，只需微调

**调试模式 (`debug`)：**
- 赛前调试阶段
- 电控还没接好，或者不需要串口
- 需要看完整的调试图像和mask
- 开发新功能、测试新算法时

### 自定义预设

1. 在 Web 界面调好所有参数
2. 点击「预设」→「保存当前配置为预设」
3. 输入名称（如 `outpost_far`）和显示名称（如 `远距离前哨站`）
4. 下次直接从预设列表加载

---

## params.yaml 加载时机

### 启动时加载

`params.yaml` 在 **dart_aim_node 启动时** 被读取一次：

```
ros2 launch yq_dart_aim start.launch.py
  → 启动 dart_aim_node
    → 读取 params.yaml 中的所有参数作为初始值
    → 参数生效，开始运行
```

**这是唯一一次自动加载 params.yaml 的时机。**

### 运行时修改

运行期间通过 Web 界面或 `ros2 param set` 修改的参数：
- **立即生效**（通过 ROS2 参数回调）
- **不会自动写回** params.yaml
- **节点重启后丢失**，恢复为 params.yaml 中的值

### 保存到 params.yaml

只有在 Web 界面点击「保存配置」按钮时，当前参数值才会写回 `params.yaml`。

**建议的比赛流程：**
1. 上电，系统自动启动，加载 params.yaml 中的上次保存的参数
2. 打开 Web 界面，微调参数（实时生效，但不持久化）
3. 调好后点「保存配置」，参数写入 params.yaml
4. 如果需要重启，参数不会丢失

---

## 典型使用场景

### 场景1：比赛前三分钟

```
1. 上电 → 系统自动启动（params.yaml 加载上次的参数）
2. 手机扫码打开 Web 界面
3. 切换到「比赛模式」预设
4. 微调 HSV 阈值（看实时预览窗口确认效果）
5. 调整偏移量（根据实际弹着点修正）
6. 点「保存配置」持久化
```

### 场景2：赛前调试

```
1. 启动：ros2 launch yq_dart_aim start.launch.py web_tuner:=true
2. 笔记本打开 Web 界面
3. 切换到「调试模式」预设
4. 调整 HSV 参数，观察 mask 效果
5. 测试不同偏移量
6. 保存为自定义预设（如 `practice_day1`）
```

### 场景3：换目标（前哨站 → 基地）

```
1. 电控发送目标切换信号（通过串口）
2. Web 界面自动显示当前目标对应的偏移量
3. 微调 offset_2_0 ~ offset_2_3（基地的4个飞镖偏移）
```

---

## 故障排查

| 问题 | 原因 | 解决 |
|------|------|------|
| 打不开网页 | web_tuner 没启动 | 检查 `python3 server.py` 是否在运行 |
| 参数改了没效果 | 节点没启动 | 确认 `dart_aim_node` 在运行 |
| 图像预览黑屏 | 没有图像话题 | 检查 `publish_compressed` 是否开启 |
| 手机连不上 | 网络不通 | 确认手机和 RDKX5 在同一局域网 |
| 保存失败 | 文件权限 | 检查 `config/params.yaml` 是否可写 |
| 页面显示但无数据 | 后端未启动 | 页面自动进入演示模式，用内置数据预览 |
| 白屏/加载慢 | 无网络 | 已去掉 CDN 依赖，不需要联网 |

---

## 端口说明

| 服务 | 端口 | 说明 |
|------|------|------|
| Web 调参 | 8080 | 本系统 |
| Foxglove | 8765 | ROS2 可视化（已有的） |

两个服务互不影响，可同时使用。
