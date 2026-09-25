# RM 飞镖镖架视觉识别系统

## 快速启动

```bash
source /opt/ros/humble/setup.bash
source /root/dart_ws_on/install/setup.bash
ros2 launch yq_dart_aim start.launch.py
```

禁用 foxglove 可视化：
```bash
ros2 launch yq_dart_aim start.launch.py debug:=false
```

## 编译

```bash
cd /root/dart_ws_on
source /opt/ros/humble/setup.bash
colcon build
```

### 模型推理后端（运行期切换）

模型识别有两条路径，由 `params.yaml` 的 `infer_backend` 在**运行期**切换，不用重新编译：

| 取值 | 说明 | 模型文件 |
|---|---|---|
| `BPU` | RDK X5 BPU（hobot_dnn），板子部署用，速度最快 | `model_path_bpu` 指向量化 `.bin` |
| `CPU` | ONNX Runtime，纯 CPU 推理，用于没量化模型时验证 | `model_path_cpu` 指向切好后的 `.onnx` |
| `DNN` | OpenCV DNN，备选（比 ONNX Runtime 慢） | 同 CPU |

```yaml
use_model: true          # 一级开关：模型模式（false = 传统 HSV）
infer_backend: CPU       # 二级开关：选 BPU 还是 CPU
model_path_cpu: /root/dart_ws_on/src/yq_dart_aim/model/praysky_dart_576x768_cut.onnx
model_path_bpu: ''       # 量化模型放好后填 .bin 路径
```

运行时切换（会自动卸载旧后端、热加载新模型，失败明确报错并退回 HSV）：

```bash
ros2 param set /dart_aim_node infer_backend BPU
```

编译期只决定"哪些后端编进程序"（默认自动探测依赖）：

```bash
# 显式指定 ONNX Runtime 路径（找不到系统安装时）
colcon build --cmake-args -DORT_ROOT=/root/onnxruntime/onnxruntime-linux-aarch64-1.19.2

# 关掉某个后端（减小体积/避免缺依赖）
colcon build --cmake-args -DDART_ENABLE_ORT=OFF

# params.yaml 没写 infer_backend 时的默认值
colcon build --cmake-args -DDART_INFER_BACKEND=BPU
```

> BPU 需要量化好的 `.bin`（生成流程见 `docs/BPU部署指南.md`）；CPU 需要切掉后处理的 `.onnx`（同文档第 2 步）。
> CPU 路径在 RDK X5 上单帧约 0.5 秒，只能用来验证流程，不能实时运行。

## 生产部署（systemd 开机自启）

```bash
sudo bash /root/dart_ws_on/src/rm_start/register_service.sh
```

## 调试命令

### 节点与话题

```bash
# 查看运行中的节点
ros2 node list

# 查看所有话题
ros2 topic list

# 查看话题信息（类型、发布者、订阅者）
ros2 topic info /dart_debug
ros2 topic info /image_raw

# 查看话题频率
ros2 topic hz /image_raw
ros2 topic hz /dart_debug

# 查看话题带宽
ros2 topic bw /image_raw
```

### 实时数据查看

```bash
# 查看瞄准误差（x=水平误差, y=垂直误差, z=0）
ros2 topic echo /dart_debug

# 查看串口接收数据（target, dart_id, encoder_angle）
ros2 topic echo /serial

# 查看串口原始 hex 数据
ros2 topic echo /serial_debug

# 查看调试图像（会刷屏，建议用 foxglove）
ros2 topic echo /dart_debug/image --no-arr
```

### 串口调试

```bash
# 查看串口设备
ls -la /dev/ttyACM*

# 查看串口权限
ls -la /dev/ttyACM0

# 临时赋予串口权限
sudo chmod 666 /dev/ttyACM0

# 监听串口原始数据（需安装 screen）
screen /dev/ttyACM0 115200

# 查看串口 hex 调试输出
ros2 topic echo /serial_debug
```

### 图像调试

```bash
# 保存一帧调试图像到文件
ros2 topic echo /dart_debug/image --no-arr | head -20
```

### Foxglove 远程可视化

```bash
# foxglove_bridge 默认监听 8765 端口
# 在 Foxglove Studio 中连接 ws://<机器IP>:8765
# 可查看：实时图像、调试图像、mask、串口数据、瞄准误差
```

### systemd 服务管理

```bash
# 查看服务状态
systemctl status rm.service

# 启动/停止/重启服务
sudo systemctl start rm.service
sudo systemctl stop rm.service
sudo systemctl restart rm.service

# 查看服务日志
journalctl -u rm.service -f          # 实时跟踪
journalctl -u rm.service --since today  # 今天的日志
journalctl -u rm.service -n 100      # 最近 100 行

# 禁用/启用开机自启
sudo systemctl disable rm.service
sudo systemctl enable rm.service

# 手动清理所有 ROS 进程
sudo bash /usr/sbin/rm_clean_up.sh

# 查看 watchdog 日志
cat /tmp/rm_watchdog.log
tail -f /tmp/rm_watchdog.log
```

### ROS2 通用调试

```bash
# 查看 ROS2 daemon 状态
ros2 daemon status

# 重启 ROS2 daemon
ros2 daemon stop && ros2 daemon start

# 查看节点的连接关系
ros2 node info /dart_aim_node

# 查看 ROS2 日志目录
ls /root/.ros/log/

# 查看 launch 输出
cat /root/dart_ws_on/screen.output
```

### 常见问题排查

```bash
# 相机未找到
# 检查 USB 连接
lsusb | grep -i hikvision
# 检查相机节点日志
journalctl -u rm.service | grep -i camera

# 串口连接失败
# 检查设备是否存在
ls /dev/ttyACM*
# 检查权限
ls -la /dev/ttyACM0
# 检查串口是否被占用
sudo fuser /dev/ttyACM0

# 编译失败
# 清除旧产物重新编译
rm -rf build/ install/ log/
source /opt/ros/humble/setup.bash
colcon build
```

### 局域网代理

```bash
export HTTP_PROXY=http://10.35.11.222:7897
export HTTPS_PROXY=http://10.35.11.222:7897
export ALL_PROXY=socks5://10.35.11.222:7897
# 内网地址不走代理，避免ssh、局域网访问卡住
export NO_PROXY=localhost,127.0.0.1,192.168.*,10.*
```

```bash
# 查看出口IP，验证代理是否生效
curl https://api.ipify.org
# 访问github测试
curl -I https://github.com
```

