#!/usr/bin/env bash
# 一次性网络配置: 让 RK3588 有线网口能接收 FPGA 发往 192.168.1.255:6102 的广播包
# 用法: sudo bash setup_net.sh [网口名]
set -e

IFACE="${1:-enP3p49s0}"
PORT=6102

run() {
    if [ "$(id -u)" -eq 0 ]; then "$@"; else sudo "$@"; fi
}

echo "== 配置 $IFACE =="
run ip addr replace 192.168.1.100/24 dev "$IFACE"
run ip link set "$IFACE" up

echo "== 放大 socket 接收缓冲(应对突发/调度抖动) =="
run sysctl -w net.core.rmem_max=8388608 >/dev/null
run sysctl -w net.core.rmem_default=1048576 >/dev/null

echo "== 放行 UDP $PORT =="
if command -v iptables >/dev/null 2>&1; then
    run iptables -C INPUT -p udp --dport $PORT -j ACCEPT 2>/dev/null || \
        run iptables -I INPUT -p udp --dport $PORT -j ACCEPT || \
        echo "  (iptables 不可用, 跳过, 若收不到包请检查防火墙)"
fi

echo "== 结果 =="
ip -brief addr show "$IFACE"
if command -v ethtool >/dev/null 2>&1; then
    ethtool "$IFACE" 2>/dev/null | grep -E 'Speed|Duplex|Link detected' || true
fi
echo "完成。现在可以运行: python3 recv_fpga.py --duration 10"
