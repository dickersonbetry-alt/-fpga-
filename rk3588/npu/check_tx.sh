#!/usr/bin/env bash
# 检查 RK3588 -> FPGA 的回传包是否真的从有线网口发出(无需 root)
# 用法: bash npu/check_tx.sh [网口] [端口] [目的IP] [包数]
set -e

IFACE="${1:-enP3p49s0}"
PORT="${2:-6103}"
HOST="${3:-192.168.1.255}"
COUNT="${4:-60}"
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(dirname "$HERE")"

echo "=== 路由 ==="
ip route get "$HOST" || true
echo
echo "=== 链路 ==="
ethtool "$IFACE" 2>/dev/null | grep -E 'Speed|Duplex|Link detected' || true
echo
echo "=== 发送前 ==="
tx0=$(cat "/sys/class/net/$IFACE/statistics/tx_packets")
bx0=$(cat "/sys/class/net/$IFACE/statistics/tx_bytes")
echo "tx_packets=$tx0 tx_bytes=$bx0"

python3 "$ROOT/box_protocol.py" --host "$HOST" --port "$PORT" \
    --frames "$COUNT" --fps 20 >/dev/null 2>&1 &
PID=$!
sleep 1
tx1=$(cat "/sys/class/net/$IFACE/statistics/tx_packets")
bx1=$(cat "/sys/class/net/$IFACE/statistics/tx_bytes")
wait $PID
tx2=$(cat "/sys/class/net/$IFACE/statistics/tx_packets")
bx2=$(cat "/sys/class/net/$IFACE/statistics/tx_bytes")

echo
echo "=== 结果 ==="
echo "发送 ${COUNT} 包(box_protocol) 期间:"
echo "  tx_packets 增量: $((tx2 - tx1))"
echo "  tx_bytes   增量: $((bx2 - bx1))"
echo "错误计数:"
for f in tx_errors tx_dropped tx_carrier_errors collisions; do
    printf "  %-18s %s\n" "$f" "$(cat /sys/class/net/$IFACE/statistics/$f)"
done
echo
if [ "$((tx2 - tx0))" -gt 0 ]; then
    echo "结论: RK3588 已从 $IFACE 发出回传包; 若 FPGA 仍收不到, 问题在 FPGA RX 侧。"
else
    echo "结论: $IFACE 没有发出包, 检查路由/网口/权限。"
fi
