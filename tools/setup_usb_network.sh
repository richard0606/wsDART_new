#!/bin/bash
# USB 网络（USB RNDIS/ECM）配置 —— 让 X5 用一根 Type-C 线连 PC 联网
#
# 原理：Type-C/OTG 口接上 PC 后，PC 会多出一个网卡（RNDIS 或 ECM），
#       板子和 PC 之间就有了一条网络通路，可以在上面开网页做调参界面。
#
# 关键事实（这块 RDK 内核实测）：
#   - libcomposite 用旧式 g_comp 布局，厂商已经预置并绑好了 ecm.0 / rndis.0 /
#     mass_storage.0 三个功能，UDC 已绑到 35300000.usb（Type-C/OTG 口）
#   - 所以这里不需要、也不能新建 functions（configfs 不允许新增函数链接）
#   - 我们只负责"确保配置在 + 确保板子侧的网卡/RNDIS 主机起来"
#
# 用法：
#   sudo tools/setup_usb_network.sh up       启用（开机自动跑）
#   sudo tools/setup_usb_network.sh down     关闭
#   sudo tools/setup_usb_network.sh status   状态
#
# 注意：
#   - 必须用板子的 Type-C / OTG 口，不能用接相机的 USB-A 口
#   - 别用 Type-C 口给板子供电（会和 gadget 冲突）
#   - 插上 PC 之后，PC 上会多一个网卡；Windows 认 RNDIS，Linux 认 CDC-ECM
set -e

GC=/sys/kernel/config/usb_gadget/g_comp
IFACE_POOL="usb0 usb1"

is_udc_bound() {
  [[ -f "$GC/UDC" ]] && [[ -n "$(cat "$GC/UDC" 2>/dev/null)" ]]
}

ensure_composite() {
  modprobe libcomposite 2>/dev/null || true
  modprobe usb_f_rndis   2>/dev/null || true
  modprobe usb_f_ecm     2>/dev/null || true
  modprobe rndis_host    2>/dev/null || true

  if ! is_udc_bound; then
    # 少数情况下 UDC 没绑上，重绑一次（针对已存在的 configs/c.1）
    local udc
    udc=$(ls /sys/class/udc 2>/dev/null | head -1)
    if [[ -n "$udc" && -d "$GC/configs/c.1" ]]; then
      echo "$udc" > "$GC/UDC" 2>/dev/null || true
    fi
  fi
}

bring_up_ifaces() {
  # 板子侧 gadget 网卡要在 PC 插上线、主机枚举出网卡后才会真正 UP（carrier）。
  # 枚举前接口存在但 carrier=0，此时配地址没意义，所以只在 UP 且有 carrier 时才配。
  # 结果放进全局 CONNECTED（不用返回值，避免 set -e 下非零退出把 systemd 服务搞挂）
  CONNECTED=0
  for i in $IFACE_POOL; do
    ip link show "$i" >/dev/null 2>&1 || continue
    ip link set "$i" up 2>/dev/null || true
    # 等 carrier（PC 插上线才有）
    local waited=0 carrier=0
    while [[ $waited -lt 20 ]]; do
      carrier=$(cat /sys/class/net/$i/carrier 2>/dev/null || echo 0)
      [[ "$carrier" == "1" ]] && break
      sleep 0.5; waited=$((waited+1))
    done
    if [[ "${carrier:-0}" == "1" ]]; then
      ip addr replace 192.168.55.2/24 dev "$i" 2>/dev/null || true
      echo "  $i 已连接并配置 192.168.55.2/24"
      CONNECTED=1
    else
      echo "  $i 存在但无连接（Type-C 没插线），跳过配置"
    fi
  done
}

do_up() {
  ensure_composite
  if ! is_udc_bound; then
    echo "警告：gadget 未绑定 UDC，插上 Type-C 线后可能需要再跑一次 up" >&2
  fi
  echo "USB 网络已启用，等待 Type-C 连接（最多 10 秒）..."
  bring_up_ifaces
  if [[ $CONNECTED -eq 1 ]]; then
    echo "✓ 已连接，板子地址 192.168.55.2 ，PC 网卡填 192.168.55.1"
    # 顺便让板子侧的调参 TCP 通道被允许（ufw 若启用）
    (command -v ufw >/dev/null && ufw allow from 192.168.55.0/24 to any port 8766 proto tcp >/dev/null 2>&1) || true
  else
    echo "  还没检测到 Type-C 连接。插上线后重跑：$0 up"
  fi
}

do_down() {
  for i in $IFACE_POOL; do
    if ip link show "$i" >/dev/null 2>&1; then
      ip link set "$i" down 2>/dev/null || true
      echo "  $i 已关闭"
    fi
  done
  echo "✓ 已关闭（Type-C 线的网卡会消失）"
}

do_status() {
  echo -n "gadget 绑定 UDC: "
  is_udc_bound && cat "$GC/UDC" || echo "未绑定"
  echo "已绑功能: $(ls "$GC/configs/c.1" 2>/dev/null | grep -vE 'bmAttributes|MaxPower|strings' | tr '\n' ' ')"
  echo "板子侧 gadget 网卡（carrier=1 才有连接）:"
  found=0
  for i in $IFACE_POOL; do
    if ip link show "$i" >/dev/null 2>&1; then
      car=
      car=$(cat /sys/class/net/$i/carrier 2>/dev/null || echo 0)
      echo "  $i: $(ip -br addr show $i 2>/dev/null | awk '{print $3}') carrier=$car"
      found=1
    fi
  done
  [[ $found -eq 0 ]] && echo "  （无接口）"
  echo "PC 端请看：Windows 设备管理器有无 RNDIS；Linux 有无 usb0/cdc_ether 新网卡"
}

case "${1:-up}" in
  up) do_up ;;
  down) do_down ;;
  status) do_status ;;
  *) echo "用法: $0 {up|down|status}" >&2; exit 1 ;;
esac
