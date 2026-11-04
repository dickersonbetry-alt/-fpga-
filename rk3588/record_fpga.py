#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
录制 FPGA(OV5640 800x480 RGB565) UDP 图像流为视频, 并在录制结束后自动抽帧。

示例:
  python3 record_fpga.py                       # 一直录到 Ctrl-C, 录完默认按每秒 3 帧抽帧
  python3 record_fpga.py --extract-fps 3       # 同上(显式指定每秒 3 张)
  python3 record_fpga.py --extract-every 15    # 改成每 15 帧抽一张
  python3 record_fpga.py --extract-fps 0       # 只要视频, 不抽帧
  python3 record_fpga.py --duration 30         # 只录 30 秒(默认 0 = 直到 Ctrl-C)
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
except ImportError:
    print("需要 opencv-python: pip install opencv-python", file=sys.stderr)
    raise

W, H = 800, 480
PKT_LEN = 2 + W * 2  # 1602


def rgb565_to_bgr(frame565: np.ndarray) -> np.ndarray:
    """RGB565(小端已解析为 uint16) -> BGR uint8"""
    if hasattr(cv2, "COLOR_RGB5652BGR"):
        return cv2.cvtColor(frame565, cv2.COLOR_RGB5652BGR)
    v = frame565.astype(np.uint32)
    r = ((v >> 11) & 0x1F) * 255 // 31
    g = ((v >> 5) & 0x3F) * 255 // 63
    b = (v & 0x1F) * 255 // 31
    return np.dstack([b, g, r]).astype(np.uint8)


class VideoSink(threading.Thread):
    """后台线程: 把收到的帧转 BGR 并写入视频, 避免编码拖慢收包"""

    def __init__(self, path, fourcc, fps, size, qsize=8):
        super().__init__(daemon=True)
        self.path = path
        self.fps = fps
        self.q = queue.Queue(maxsize=qsize)
        self.n_written = 0
        self.n_dropped = 0
        os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
        self.writer = cv2.VideoWriter(
            path, cv2.VideoWriter_fourcc(*fourcc), fps, size)
        if not self.writer.isOpened():
            raise RuntimeError(
                f"无法创建视频文件 {path} (fourcc={fourcc}), 试试 --codec XVID 且输出 .avi")

    def run(self):
        while True:
            item = self.q.get()
            if item is None:
                break
            self.writer.write(rgb565_to_bgr(item))
            self.n_written += 1
        self.writer.release()

    def submit(self, frame565: np.ndarray):
        try:
            self.q.put_nowait(frame565)
            return True
        except queue.Full:
            self.n_dropped += 1
            return False

    def close(self):
        while True:
            try:
                self.q.put(None, timeout=0.5)
                break
            except queue.Full:
                continue
        self.join(timeout=60)


def extract_frames(video_path, out_dir, every=0, extract_fps=1.0):
    """从视频中抽帧保存为 PNG, 返回 (抽帧数, 视频总帧数, 步长)"""
    cap = cv2.VideoCapture(video_path)
    if not cap.isOpened():
        raise RuntimeError(f"无法打开视频 {video_path} (缺解码器?)")
    src_fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
    if every > 0:
        step = int(every)
    else:
        step = max(1, int(round(src_fps / max(extract_fps, 1e-6))))
    total = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))

    os.makedirs(out_dir, exist_ok=True)
    stem = os.path.splitext(os.path.basename(video_path))[0]
    idx = 0
    saved = 0
    while True:
        ok, img = cap.read()
        if not ok:
            break
        if idx % step == 0:
            name = os.path.join(out_dir, f"{stem}_{idx:06d}.png")
            if not cv2.imwrite(name, img):
                print(f"[warn] 写入失败: {name}")
            else:
                saved += 1
        idx += 1
    cap.release()
    return saved, idx, step, src_fps, total


def main():
    ts = time.strftime("%Y%m%d_%H%M%S")
    ap = argparse.ArgumentParser()
    ap.add_argument("--bind", default="0.0.0.0")
    ap.add_argument("--port", type=int, default=6102)
    ap.add_argument("--out", default=f"recordings/fpga_{ts}.mp4", help="输出视频路径")
    ap.add_argument("--fps", type=float, default=30.0, help="写入视频的帧率")
    ap.add_argument("--codec", default="mp4v", help="fourcc, 如 mp4v / XVID / MJPG")
    ap.add_argument("--duration", type=float, default=0, help="录制秒数, 0=直到 Ctrl-C")
    ap.add_argument("--max-frames", type=int, default=0, help="最多录多少帧, 0=不限")
    ap.add_argument("--bufsize", type=int, default=8 << 20, help="socket 接收缓冲区")
    ap.add_argument("--extract-fps", type=float, default=3.0,
                    help="录完自动抽帧的频率(帧/秒, 默认 3), 0=不抽帧")
    ap.add_argument("--extract-every", type=int, default=0,
                    help="按帧间隔抽帧(优先于 --extract-fps), 如 15 = 每 15 帧一张")
    ap.add_argument("--extract-dir", default="", help="抽帧输出目录, 默认 <视频名>_frames")
    args = ap.parse_args()

    if not args.out.lower().endswith((".mp4", ".avi", ".mkv")):
        print("[warn] 建议输出扩展名用 .mp4/.avi/.mkv")

    # socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    try:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, args.bufsize)
    except OSError as e:
        print(f"[warn] SO_RCVBUF: {e}")
    sock.bind((args.bind, args.port))
    sock.settimeout(1.0)
    rcvbuf = sock.getsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF)
    print(f"[info] 监听 udp {args.bind}:{args.port}  rcvbuf={rcvbuf/1024:.0f} KiB")
    print(f"[info] 录制到 {args.out}  {W}x{H} @ {args.fps:g}fps  codec={args.codec}")

    try:
        sink = VideoSink(args.out, args.codec, args.fps, (W, H))
    except RuntimeError as e:
        print(f"[error] {e}")
        return 2
    sink.start()

    frame = np.zeros((H, W), dtype=np.uint16)
    seen = np.zeros(H, dtype=bool)
    buf = bytearray(4096)
    mv = memoryview(buf)

    n_pkt = n_bad_len = n_bad_row = 0
    n_ends = n_complete = n_partial = 0
    t_first_frame = t_last_frame = 0.0
    last_row_seen = H - 1
    last_pkt_time = None
    first_packet = True
    start = last_stat = last_warn = time.time()
    srcs = set()

    def finish_frame():
        nonlocal n_ends, n_complete, n_partial, t_first_frame, t_last_frame
        n_ends += 1
        if seen.all():
            n_complete += 1
            now_t = time.time()
            if n_complete == 1:
                t_first_frame = now_t
            t_last_frame = now_t
            sink.submit(frame.copy())  # 拷贝后交给后台线程, 收包线程立刻返回
        else:
            n_partial += 1
            if n_partial <= 5:
                print(f"[frame] 残帧(第 {n_ends} 次回绕) 缺 {H - int(seen.sum())} 行, 丢弃")
        seen[:] = False

    try:
        while True:
            now = time.time()
            if args.duration and now - start >= args.duration:
                break
            if args.max_frames and n_complete >= args.max_frames:
                break
            try:
                n, addr = sock.recvfrom_into(buf)
            except socket.timeout:
                if now - last_warn >= 5.0:  # 长时间无包时节流提示, 避免刷屏
                    if last_pkt_time is None:
                        print("[warn] 还没收到任何 UDP 包 (检查 setup_net.sh / 网线 / 板子)", flush=True)
                    else:
                        print(f"[warn] 已 {now - last_pkt_time:.1f}s 没有新包, 继续等待...", flush=True)
                    last_warn = now
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
                continue

            pkt = mv[:n]
            row = pkt[0] | (pkt[1] << 8)
            if row >= H:
                n_bad_row += 1
                if n_bad_row <= 5:
                    print(f"[bad] 行号 {row} 越界 来自 {addr}")
                continue

            soft_wrap = (seen.any() and row != H - 1
                         and last_row_seen >= 3 * H // 4 and row <= H // 4)
            if soft_wrap:
                finish_frame()

            frame[row] = np.frombuffer(pkt[2:], dtype="<u2")
            seen[row] = True
            if row == H - 1:
                finish_frame()
            last_row_seen = row

            if now - last_stat >= 1.0:
                dt = now - last_stat
                print(f"[stat] t={now-start:6.1f}s  pkt={n_pkt} ({n_pkt/dt:6.0f} pkt/s)  "
                      f"完整帧={n_complete} 已写入={sink.n_written} 丢帧={sink.n_dropped} "
                      f"残帧={n_partial} src={sorted(srcs)}", flush=True)
                last_stat = now

    except KeyboardInterrupt:
        print("\n[info] 用户中断, 收尾中...")
    finally:
        sock.close()
        sink.close()

    elapsed = time.time() - start
    size = os.path.getsize(args.out) if os.path.exists(args.out) else 0
    span = t_last_frame - t_first_frame
    actual_fps = (n_complete - 1) / span if span > 0 else 0
    print("=" * 64)
    print(f"录制结束: {elapsed:.1f}s  收包 {n_pkt} ({n_pkt/max(elapsed,1e-9):.0f} pkt/s)")
    print(f"完整帧 {n_complete}, 写入视频 {sink.n_written} 帧, "
          f"编码队列丢帧 {sink.n_dropped}, 残帧 {n_partial}")
    print(f"源帧率 {actual_fps:.2f} fps, 视频 {args.out} ({size/1024/1024:.1f} MiB)")

    if n_pkt == 0:
        print("未收到任何包: 检查 setup_net.sh 是否执行、网线/LINK 灯")
        return 1

    if args.extract_fps > 0 or args.extract_every > 0:
        out_dir = args.extract_dir or (os.path.splitext(args.out)[0] + "_frames")
        print(f"[extract] 从 {args.out} 抽帧 -> {out_dir}")
        try:
            saved, total, step, src_fps, declared = extract_frames(
                args.out, out_dir, every=args.extract_every,
                extract_fps=args.extract_fps)
            print(f"[extract] 完成: {saved} 张 PNG (每 {step} 帧取 1 张, "
                  f"视频读到 {total} 帧, 容器帧率 {src_fps:.2f}, 声明总帧数 {declared})")
            print(f"[extract] 目录: {os.path.abspath(out_dir)}")
        except RuntimeError as e:
            print(f"[error] 抽帧失败: {e}")
            return 3

    return 0


if __name__ == "__main__":
    sys.exit(main())
