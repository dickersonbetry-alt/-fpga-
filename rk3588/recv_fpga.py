#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
FPGA(ACG720 / OV5640 800x480 RGB565) UDP 图像接收与组帧

协议(见《以太网图像传输接口说明_RK3588.docx》):
  UDP 目的端口 6102, 每包负载 1602 字节:
     [0:2]   行号 row, 小端
     [2:1602] 800 像素 RGB565, 每像素小端(低字节先发)
  行号 0..479, 回绕到 0 表示新一帧

用法:
  python3 recv_fpga.py                 # 只接收并统计, 保存完整帧到 ./frames
  python3 recv_fpga.py --duration 10   # 接收 10 秒后退出
  python3 recv_fpga.py --save-every 30 # 每 30 帧存一张 PNG
"""
import argparse
import os
import queue
import socket
import sys
import threading
import time

import numpy as np

try:
    import cv2
except ImportError:  # 没有 opencv 也能统计/存图(numpy 直接写 PPM)
    cv2 = None

W, H = 800, 480
PKT_LEN = 2 + W * 2  # 1602


def rgb565_to_bgr(frame565: np.ndarray) -> np.ndarray:
    """RGB565(小端已解析为 uint16) -> BGR uint8"""
    if cv2 is not None and hasattr(cv2, "COLOR_RGB5652BGR"):
        return cv2.cvtColor(frame565, cv2.COLOR_RGB5652BGR)
    v = frame565.astype(np.uint32)
    r = (v >> 11) & 0x1F
    g = (v >> 5) & 0x3F
    b = v & 0x1F
    r = (r * 255 // 31).astype(np.uint8)
    g = (g * 255 // 63).astype(np.uint8)
    b = (b * 255 // 31).astype(np.uint8)
    return np.dstack([b, g, r])


def save_image(bgr: np.ndarray, path: str):
    os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
    if cv2 is not None:
        cv2.imwrite(path, bgr)
    else:  # 退化为 PPM
        path = os.path.splitext(path)[0] + ".ppm"
        with open(path, "wb") as f:
            f.write(b"P6\n%d %d\n255\n" % (bgr.shape[1], bgr.shape[0]))
            f.write(bgr.tobytes())


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bind", default="0.0.0.0", help="监听地址")
    ap.add_argument("--port", type=int, default=6102, help="UDP 端口")
    ap.add_argument("--out", default="frames", help="保存完整帧的目录")
    ap.add_argument("--save-every", type=int, default=0,
                    help="每 N 帧保存一张, 0 表示全部保存")
    ap.add_argument("--max-frames", type=int, default=3, help="最多保存几张")
    ap.add_argument("--duration", type=float, default=0, help="运行秒数, 0=一直运行")
    ap.add_argument("--bufsize", type=int, default=8 << 20, help="socket 接收缓冲区")
    args = ap.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    try:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, args.bufsize)
    except OSError as e:
        print(f"[warn] SO_RCVBUF: {e}")
    sock.bind((args.bind, args.port))
    sock.settimeout(1.0)

    real_buf = sock.getsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF)
    print(f"[info] 监听 udp {args.bind}:{args.port}  rcvbuf={real_buf/1024:.0f} KiB")

    frame = np.zeros((H, W), dtype=np.uint16)
    seen = np.zeros(H, dtype=bool)

    # 统计
    n_pkt = 0
    n_bad_len = 0
    n_bad_row = 0
    n_ends = 0          # 检测到的帧结束次数
    n_complete = 0      # 完整帧数
    partial_rows = 0
    srcs = set()
    start = time.time()
    last_stat = start
    last_pkt_time = None
    last_row_seen = H - 1
    first_packet = True

    # 收包与存图分离: 收包线程只做拷贝, PNG 编码/写盘放到后台线程
    save_q = queue.Queue(maxsize=2)
    saved = {"n": 0, "bytes": 0}
    n_scheduled = 0     # 已交给后台线程的帧数(避免队列/计数竞态导致超存)

    def saver():
        while True:
            item = save_q.get()
            if item is None:
                break
            idx, arr = item
            bgr = rgb565_to_bgr(arr)
            ts = time.strftime("%Y%m%d_%H%M%S")
            path = os.path.join(args.out, f"frame_{idx:06d}_{ts}.png")
            save_image(bgr, path)
            size = os.path.getsize(path) if os.path.exists(path) else 0
            saved["n"] += 1
            saved["bytes"] += size
            print(f"[save] 完整帧 #{idx} -> {path}  均值={bgr.mean():.1f} "
                  f"首像素BGR={bgr[0, 0].tolist()}", flush=True)

    saver_th = threading.Thread(target=saver, daemon=True)
    saver_th.start()

    def stat(now):
        nonlocal last_stat
        dt = now - last_stat
        if dt >= 1.0:
            print(f"[stat] t={now-start:6.1f}s  pkt={n_pkt} ({n_pkt/dt:7.0f} pkt/s) "
                  f"帧结束={n_ends} 完整={n_complete} 保存={saved['n']} "
                  f"坏长度={n_bad_len} 坏行号={n_bad_row} 残帧={partial_rows} "
                  f"src={sorted(srcs)}", flush=True)
            last_stat = now

    def finish_frame():
        """把当前 seen 标记的一帧收尾: 完整则交给后台保存, 否则计为残帧"""
        nonlocal n_ends, n_complete, partial_rows, n_scheduled
        n_ends += 1
        if seen.all():
            n_complete += 1
            want = args.save_every == 0 or n_complete % args.save_every == 0
            if want and n_scheduled < args.max_frames:
                try:
                    save_q.put_nowait((n_complete, frame.copy()))
                    n_scheduled += 1
                except queue.Full:
                    print("[warn] 保存队列已满, 跳过该帧")
        else:
            partial_rows += 1
            if partial_rows <= 5 or partial_rows % 30 == 0:
                print(f"[frame] 残帧(第 {n_ends} 次回绕) 缺 {H - int(seen.sum())} 行, 丢弃")
        seen[:] = False

    buf = bytearray(4096)
    mv = memoryview(buf)

    try:
        while True:
            now = time.time()
            if args.duration and now - start >= args.duration:
                break
            try:
                n, addr = sock.recvfrom_into(buf)
            except socket.timeout:
                if last_pkt_time is None:
                    print("[warn] 1 秒内没有收到任何 UDP 包")
                else:
                    print(f"[warn] {now - last_pkt_time:.1f}s 未收到包")
                if args.duration and now - start >= args.duration:
                    break
                continue

            now = time.time()
            last_pkt_time = now
            srcs.add(addr[0])
            n_pkt += 1
            if first_packet:
                first_packet = False
                print(f"[first] 收到首个包 {addr[0]}:{addr[1]} 长度={n}")

            if n != PKT_LEN:
                n_bad_len += 1
                if n_bad_len <= 5:
                    print(f"[bad] 包长度 {n} (期望 {PKT_LEN}) 来自 {addr}")
                stat(now)
                continue

            pkt = mv[:n]
            row = pkt[0] | (pkt[1] << 8)  # 小端行号
            if row >= H:
                n_bad_row += 1
                if n_bad_row <= 5:
                    print(f"[bad] 行号 {row} 越界 来自 {addr}")
                stat(now)
                continue

            # 帧边界: 官方是行号 479->0; 若 479 那包丢了, 退化为"行号大幅回退"
            soft_wrap = (seen.any() and row != H - 1
                         and last_row_seen >= 3 * H // 4 and row <= H // 4)
            if soft_wrap:
                finish_frame()

            frame[row] = np.frombuffer(pkt[2:], dtype="<u2")  # 小端 RGB565
            seen[row] = True

            if row == H - 1:
                finish_frame()

            last_row_seen = row
            stat(now)

    except KeyboardInterrupt:
        print("\n[info] 用户中断")
    finally:
        sock.close()

    # 等待后台存图线程把队列写完
    while True:
        try:
            save_q.put(None, timeout=0.5)
            break
        except queue.Full:
            continue
    saver_th.join(timeout=20)

    total = time.time() - start
    print("=" * 60)
    print(f"运行 {total:.1f}s, 收包 {n_pkt}, 平均 {n_pkt/max(total,1e-9):.0f} pkt/s")
    print(f"帧结束 {n_ends}, 完整帧 {n_complete}, 残帧 {partial_rows}, "
          f"坏长度 {n_bad_len}, 坏行号 {n_bad_row}")
    print(f"保存图片 {saved['n']} 张, 共 {saved['bytes']/1024:.0f} KiB, "
          f"目录: {os.path.abspath(args.out)}")
    if n_pkt == 0:
        print("未收到任何包: 检查网线/LINK 灯、RK3588 网口是否配置 192.168.1.x、"
              "sudo iptables -I INPUT -p udp --dport 6102 -j ACCEPT")
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
