#!/usr/bin/env bash
# 让 Orin 经有线口直接收 MID360 数据（做法与 PC2 相同）
#   - 有线口 enP7p1s0 额外挂一个 /32 地址 192.168.1.51
#   - 只为雷达 192.168.1.194 加一条主机路由，源地址用 .51
# 雷达在 192.168.1.x，和 Wi-Fi 网段重叠；用 /32 + 主机路由可以只把这一台设备引到有线口，
# Wi-Fi 上 192.168.1.0/24 的其他流量不受影响。只修改有线连接 go2w-wired，不碰 Wi-Fi。
#
# 用法：sudo bash setup-lidar-net.sh          # 应用
#       sudo bash setup-lidar-net.sh --undo   # 撤销
set -euo pipefail

CON=go2w-wired
DEV=enP7p1s0
HOST_IP=192.168.1.51
LIDAR_IP=192.168.1.194

if [[ $EUID -ne 0 ]]; then
  echo "请用 sudo 运行" >&2
  exit 1
fi

if [[ "${1:-}" == "--undo" ]]; then
  nmcli connection modify "$CON" -ipv4.addresses "$HOST_IP/32" -ipv4.routes "$LIDAR_IP/32 src=$HOST_IP" || true
else
  nmcli -g ipv4.addresses connection show "$CON" | grep -q "$HOST_IP/32" \
    || nmcli connection modify "$CON" +ipv4.addresses "$HOST_IP/32"
  nmcli -g ipv4.routes connection show "$CON" | grep -q "$LIDAR_IP/32" \
    || nmcli connection modify "$CON" +ipv4.routes "$LIDAR_IP/32 src=$HOST_IP"
fi

# reapply 不会断开有线连接；失败时退回到重新激活
nmcli device reapply "$DEV" || nmcli connection up "$CON"

echo "== $CON"
nmcli -g ipv4.addresses,ipv4.routes connection show "$CON"
echo "== 地址与路由"
ip -4 -br addr show "$DEV"
ip route get "$LIDAR_IP"
