#!/bin/bash
set -e

# 获取当前用户的用户名和家目录
USER_NAME=${SUDO_USER:-$(whoami)}
HOME_DIR=$(eval echo ~$USER_NAME)
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="${WORKSPACE_DIR:-$(cd "$SCRIPT_DIR/../.." && pwd)}"
SERVICE_NAME="rm.service"
SERVICE_PATH="/etc/systemd/system/$SERVICE_NAME"

# 检查是否为 root 用户，因为修改 systemd 配置需要 root 权限
if [ "$(id -u)" -ne 0 ]; then
    echo "This script must be run as root."
    exit 1
fi

if [ ! -f "$WORKSPACE_DIR/install/setup.bash" ]; then
    echo "Workspace setup not found: $WORKSPACE_DIR/install/setup.bash"
    exit 1
fi

# 复制脚本到目标路径
install -m 755 "$SCRIPT_DIR/rm_clean_up.sh" /usr/sbin/rm_clean_up.sh
install -m 755 "$SCRIPT_DIR/rm_watch_dog.sh" /usr/sbin/rm_watch_dog.sh

# USB 网络调参通道（RNDIS/ECM over Type-C）只适用于 RDK X5 板端，
# x86（NUC/开发机）上跳过；板端首次安装时用 ENABLE_USB_NETWORK=1 显式打开。
if [ -z "${ENABLE_USB_NETWORK:-}" ]; then
  if [ "$(uname -m)" = "aarch64" ]; then
    ENABLE_USB_NETWORK=1
  else
    ENABLE_USB_NETWORK=0
  fi
fi
if [ "$ENABLE_USB_NETWORK" = "1" ] && [ -f "$SCRIPT_DIR/dart-usb-network.service" ]; then
  install -m 644 "$SCRIPT_DIR/dart-usb-network.service" /etc/systemd/system/dart-usb-network.service
  # 服务文件里的板端路径替换成当前工作区（原生支持任意安装位置）
  sed -i "s|/root/dart_ws_on|$WORKSPACE_DIR|g" /etc/systemd/system/dart-usb-network.service
  systemctl daemon-reload
  systemctl enable dart-usb-network.service >/dev/null 2>&1 || true
  systemctl restart dart-usb-network.service >/dev/null 2>&1 || true
  echo "USB 网络调参通道已注册 (dart-usb-network.service)"
else
  echo "跳过 USB 网络调参通道（非板端或 ENABLE_USB_NETWORK=0）"
fi

# 动态创建 systemd service 文件内容
SERVICE_CONTENT=$(cat << EOF
[Unit]
Description=FYT2024 Vision Project
After=network-online.target
Wants=network-online.target

[Service]
User=$USER_NAME
Type=simple
Environment=WORKSPACE_DIR=$WORKSPACE_DIR
ExecStart=/usr/sbin/rm_watch_dog.sh
ExecStop=/usr/sbin/rm_clean_up.sh
Restart=always
TimeoutStopSec=10
RestartSec=10s
WorkingDirectory=$HOME_DIR

[Install]
WantedBy=multi-user.target
EOF
)

# 创建/覆写 systemd service 文件
echo "Creating systemd service file at $SERVICE_PATH..."
echo "$SERVICE_CONTENT" > "$SERVICE_PATH"

# 重新加载 systemd 配置
echo "Reloading systemd daemon..."
systemctl daemon-reload

# 启用服务
echo "Enabling service $SERVICE_NAME..."
systemctl enable "$SERVICE_NAME"

# 启动服务
echo "Starting service $SERVICE_NAME..."
systemctl start "$SERVICE_NAME"

echo "Service $SERVICE_NAME has been registered and started."
