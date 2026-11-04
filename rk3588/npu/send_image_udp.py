#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""把一张 800x480 图片按上行协议逐行打包成 UDP 发送(离线测试/回放用)

  python3 npu/send_image_udp.py recordings/xxx.png --host 127.0.0.1 --port 6104 --frames 60
"""
import argparse
import socket
import sys
import time

import cv2
import numpy as np

W, H = 800, 480


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("image")
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=6104)
    ap.add_argument("--frames", type=int, default=60)
    ap.add_argument("--fps", type=float, default=30)
    args = ap.parse_args()

    img = cv2.imread(args.image)
    if img is None:
        print(f"无法读取图片: {args.image}", file=sys.stderr)
        return 2
    if img.shape[1] != W or img.shape[0] != H:
        img = cv2.resize(img, (W, H))

    b = img[:, :, 0].astype(np.uint16)
    g = img[:, :, 1].astype(np.uint16)
    r = img[:, :, 2].astype(np.uint16)
    rgb565 = (((r >> 3) << 11) | ((g >> 2) << 5) | (b >> 3)).astype("<u2")
    payload = rgb565.tobytes()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    interval = 1.0 / (args.fps * H)
    print(f"[send] {args.image} -> {args.host}:{args.port}  {args.frames} 帧 @ {args.fps:g}fps")
    for f in range(args.frames):
        t0 = time.perf_counter()
        for row in range(H):
            pkt = row.to_bytes(2, "little") + payload[row * W * 2:(row + 1) * W * 2]
            sock.sendto(pkt, (args.host, args.port))
            deadline = t0 + (row + 1) * interval
            while time.perf_counter() < deadline:
                pass
    sock.close()
    print("[send] 完成")
    return 0


if __name__ == "__main__":
    sys.exit(main())
