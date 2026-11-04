#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""反向通道接收测试工具(可在 PC/板端运行, 用于确认 FPGA 侧应收到的数据)

  python3 npu/box_recv_test.py --port 6103 --count 20
"""
import argparse
import os
import socket
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import box_protocol as bp  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bind", default="0.0.0.0")
    ap.add_argument("--port", type=int, default=6103)
    ap.add_argument("--count", type=int, default=0, help="收够多少包退出, 0=直到 Ctrl-C")
    ap.add_argument("--quiet", action="store_true")
    args = ap.parse_args()

    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    s.bind((args.bind, args.port))
    s.settimeout(1.0)
    print(f"[recv] 监听 udp {args.bind}:{args.port}")

    n = 0
    bad = 0
    total_boxes = 0
    last_fid = None
    t0 = time.time()
    try:
        while args.count == 0 or n < args.count:
            try:
                data, addr = s.recvfrom(2048)
            except socket.timeout:
                continue
            n += 1
            try:
                d = bp.unpack(data)
            except ValueError as e:
                bad += 1
                print(f"[bad] {len(data)}B 来自 {addr}: {e}")
                continue
            total_boxes += len(d["boxes"])
            gap = "" if last_fid is None else f" gap={ (d['frame_id'] - last_fid) & 0xFFFFFFFF }"
            last_fid = d["frame_id"]
            if not args.quiet and (n <= 10 or d["boxes"]):
                print(f"[{n:04d}] {addr[0]}:{addr[1]} {len(data)}B fid={d['frame_id']}"
                      f" ts={d['timestamp_ms']}ms count={len(d['boxes'])}{gap}")
                for b in d["boxes"]:
                    print(f"       box=({b[0]},{b[1]})-({b[2]},{b[3]}) "
                          f"score={b[4]:.2f} cls={b[5]}")
    except KeyboardInterrupt:
        print("\n[recv] 停止")
    finally:
        s.close()

    dt = time.time() - t0
    print(f"[recv] 收包 {n} ({n/max(dt,1e-9):.1f} pkt/s), 坏包 {bad}, "
          f"总框数 {total_boxes}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
