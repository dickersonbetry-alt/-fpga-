#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
模拟 FPGA 发送端(仅用于本机验证接收脚本), 按接口文档协议发送:
  每包 1602B = 行号(2B 小端) + 800 像素 RGB565(小端)
"""
import argparse
import socket
import time

import numpy as np

W, H = 800, 480


def make_frame(t: int) -> np.ndarray:
    """生成 800x480 RGB565 测试图: 左侧彩色竖条, 右侧渐变, 带移动白块"""
    x = np.arange(W, dtype=np.uint16)[None, :].repeat(H, axis=0)
    y = np.arange(H, dtype=np.uint16)[:, None].repeat(W, axis=1)
    r = (x * 31 // (W - 1)).astype(np.uint16)
    g = (y * 63 // (H - 1)).astype(np.uint16)
    b = np.full((H, W), 31, dtype=np.uint16)
    # 顶部 60 行画 8 个彩色竖条
    bars = [(31, 0, 0), (63, 0, 0), (0, 63, 0), (0, 0, 31),
            (31, 63, 0), (0, 63, 31), (31, 0, 31), (31, 63, 31)]
    bw = W // len(bars)
    for i, (gr, gg, gb) in enumerate(bars):
        r[0:60, i * bw:(i + 1) * bw] = gr
        g[0:60, i * bw:(i + 1) * bw] = gg
        b[0:60, i * bw:(i + 1) * bw] = gb
    # 移动白块, 便于确认帧在更新
    cx = 100 + (t * 40) % (W - 200)
    r[200:280, cx:cx + 80] = 31
    g[200:280, cx:cx + 80] = 63
    b[200:280, cx:cx + 80] = 31
    return ((r << 11) | (g << 5) | b).astype(np.uint16)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dst", default="127.255.255.255")
    ap.add_argument("--port", type=int, default=6102)
    ap.add_argument("--frames", type=int, default=10)
    ap.add_argument("--fps", type=float, default=30)
    args = ap.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    interval = 1.0 / (args.fps * H)  # 按真实行速率发送, 避免瞬间突发把接收缓冲打爆
    for t in range(args.frames):
        frame = make_frame(t)
        payload = frame.astype("<u2").tobytes()  # 小端
        t0 = time.perf_counter()
        for row in range(H):
            pkt = row.to_bytes(2, "little") + payload[row * W * 2:(row + 1) * W * 2]
            sock.sendto(pkt, (args.dst, args.port))
            deadline = t0 + (row + 1) * interval
            while time.perf_counter() < deadline:
                pass
        print(f"[send] 第 {t} 帧 {H} 包已发送", flush=True)
        time.sleep(0.0)
    sock.close()


if __name__ == "__main__":
    main()
