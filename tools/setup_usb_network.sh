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
#   - 地址沿用板子 netplan 的既有配置：板子 192.168.128.10，PC 配 192.168.128.1
set -e

GC=/sys/kernel/config/usb_gadget/g_comp
IFACE_POOL="usb0 usb1"
# 这台板子的 netplan 已经配好了这两个地址，别改（见 bring_up_ifaces 注释）
BOARD_IP=192.168.128.10
PC_IP=192.168.128.1

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

find_active_iface() {
  # 板子上有两张 gadget 网卡：usb0(ECM) 和 usb1(RNDIS)。
  # PC 端只会认其中一种（Windows 认 RNDIS，Linux 认 ECM），
  # 所以实际"哪张卡连着 PC"取决于对端系统，不能写死。
  #
  # 判据：向 PC_IP 发一次探测，再看该网卡上 ARP 是否解析成功。
  # 只看 carrier=1 是不够的——两张卡 carrier 都是 1，但只有一张真的对端在应答。
  # （ICMP 可能被防火墙挡，所以以 ARP 解析成功为准，不以 ping 回包为准。）
  local dev=$1
  arp_resolved_on() {
    # 三个坑都踩过一遍，这里写清楚：
    # 1) 不能靠 `ip neigh show dev $dev` 过滤，它会把两个同名网段的条目混在一起；
    #    改用 /proc/net/arp 按设备名精确匹配。
    # 2) 标志在第 3 列（$3）：0x2=已解析，0x0=未解析(INCOMPLETE)。
    # 3) 设备名在最后一列，用 $(NF) 取，别硬编码列号（列数会随内核版本变）。
    #    示例行：192.168.128.1  0x1  0x2  76:80:50:ad:b1:a0  *  usb0
    awk -v d="$dev" -v ip="$PC_IP" \
      '$1==ip && $3=="0x2" && $(NF)==d {found=1} END{exit !found}' /proc/net/arp 2>/dev/null
  }
  arp_resolved_on "$dev" && return 0
  ping -I "$dev" -c1 -W1 "$PC_IP" >/dev/null 2>&1 || true
  arp_resolved_on "$dev"
}

fix_routes() {
  # 让 192.168.128.0/24 优先走真正连着 PC 的那张卡。
  # netplan 给 usb1 的默认 metric 是 101，会盖过 usb0 的 800，
  # 导致包走错网卡（实测：PC 在 usb0 上，路由却指向 usb1，表现为 ping 不通）。
  local active=$1
  ip route replace 192.168.128.0/24 dev "$active" metric 10 src "$BOARD_IP" 2>/dev/null || true
  for i in $IFACE_POOL; do
    [[ "$i" == "$active" ]] && continue
    # 同一网段挂在两张卡上会让路由选择变随机，把不通的那张摘掉
    while ip route show 192.168.128.0/24 dev "$i" 2>/dev/null | grep -q .; do
      ip route del 192.168.128.0/24 dev "$i" 2>/dev/null || break
    done
    # 清掉那张卡上的残留 ARP（否则下次判断会被旧条目误导）
    ip neigh flush dev "$i" 2>/dev/null || true
  done
  echo "  路由已指向 $active"
}

bring_up_ifaces() {
  # 板子侧 gadget 网卡要在 PC 插上线、主机枚举出网卡后才会真正 UP（carrier）。
  #
  # 地址不要在这里配：这台板子的 netplan 已经配好了（实测 /etc/netplan/*.yaml）：
  #   usb0: 192.168.128.10/24 + 网关 192.168.128.1
  #   usb1: 192.168.128.10/24
  # 还有一个 NM 调度脚本会在 usb0 起来时把默认路由切到 USB 链路上。
  # 所以这里只负责 link up + 等 carrier + 兜底补地址，绝不覆盖既有配置。
  #
  # 结果放进全局 CONNECTED / ACTIVE_IF（不用返回值，避免 set -e 下非零退出把 systemd 服务搞挂）
  CONNECTED=0
  ACTIVE_IF=""
  local i waited carrier
  for i in $IFACE_POOL; do
    ip link show "$i" >/dev/null 2>&1 || continue
    ip link set "$i" up 2>/dev/null || true

    # 等 carrier（PC 插上线才有）
    waited=0; carrier=0
    while [[ $waited -lt 20 ]]; do
      carrier=$(cat /sys/class/net/$i/carrier 2>/dev/null || echo 0)
      [[ "$carrier" == "1" ]] && break
      sleep 0.5; waited=$((waited+1))
    done
    [[ "${carrier:-0}" != "1" ]] && { echo "  $i 存在但无连接（Type-C 没插线）"; continue; }

    # 兜底：万一 netplan 没生效（接口连一个 IPv4 地址都没有），补上板子侧地址。
    # 注意：ip -br 的简洁输出里没有 "inet" 关键字，不能拿它来判断有没有地址。
    if ! ip -4 addr show dev "$i" | grep -qE "^[[:space:]]+inet "; then
      echo "  $i 没有地址，补配 ${BOARD_IP}"
      ip addr replace "${BOARD_IP}/24" dev "$i" 2>/dev/null || true
    fi

    if find_active_iface "$i"; then
      echo "  $i 已连接，PC（$PC_IP）在这一侧"
      ACTIVE_IF="$i"
    else
      echo "  $i 已连接，但 PC 没应答（对端大概在另一张卡上）"
    fi
  done

  if [[ -n "$ACTIVE_IF" ]]; then
    CONNECTED=1
    fix_routes "$ACTIVE_IF"
  fi
}

do_up() {
  ensure_composite
  if ! is_udc_bound; then
    echo "警告：gadget 未绑定 UDC，插上 Type-C 线后可能需要再跑一次 up" >&2
  fi
  echo "USB 网络已启用，等待 Type-C 连接（最多 10 秒）..."
  bring_up_ifaces
  if [[ $CONNECTED -eq 1 ]]; then
    echo "✓ 板子侧 ${BOARD_IP} 已就绪（走 $ACTIVE_IF）"
    echo "  PC 端把新网卡的 IP 配成 ${PC_IP}（掩码 255.255.255.0，网关留空即可）"
    (command -v ufw >/dev/null && ufw allow from 192.168.128.0/24 to any port 8766 proto tcp >/dev/null 2>&1) || true
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
  echo "板子侧地址: ${BOARD_IP}（netplan 既有配置）"
  echo "PC 端请把新网卡配成: ${PC_IP} / 255.255.255.0（网关留空）"
}

case "${1:-up}" in
  up) do_up ;;
  down) do_down ;;
  status) do_status ;;
  *) echo "用法: $0 {up|down|status}" >&2; exit 1 ;;
esac
