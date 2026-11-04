#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
RK3588 -> FPGA 目标框反向通道 (UDP 广播)

协议 (小端, 详见 docs/box_return_protocol.md):
  包头 16 字节: magic 'YBOX' | version | flags | count | frame_id | timestamp_ms
  每个框 12 字节: x1,y1,x2,y2 (uint16) | score (uint8, 0-100) | class_id (uint8) | reserved (uint16)

单独联调(发移动框给 FPGA):
  python3 box_protocol.py --host 192.168.1.255 --port 6103 --frames 300 --fps 20
  python3 box_protocol.py --host 127.0.0.1 --port 6103 --frames 3 --dump
"""
import argparse
import socket
import struct
import sys
import time

MAGIC = b"YBOX"
VERSION = 1
HEADER = struct.Struct("<4sBBHII")   # magic, ver, flags, count, frame_id, ts_ms
BOX = struct.Struct("<HHHHBBH")      # x1,y1,x2,y2, score, cls, reserved
HEADER_LEN = HEADER.size             # 16
BOX_LEN = BOX.size                   # 12
MAX_BOXES = 16


def pack(frame_id, boxes, timestamp_ms, flags=0, max_boxes=MAX_BOXES):
    """boxes: [(x1,y1,x2,y2,score0_1,class_id), ...] -> bytes"""
    boxes = list(boxes)[:max_boxes]
    out = [HEADER.pack(MAGIC, VERSION, flags & 0xFF, len(boxes),
                       frame_id & 0xFFFFFFFF, timestamp_ms & 0xFFFFFFFF)]
    for x1, y1, x2, y2, score, cls in boxes:
        x1, y1, x2, y2 = (int(round(v)) for v in (x1, y1, x2, y2))
        if x2 < x1:
            x1, x2 = x2, x1
        if y2 < y1:
            y1, y2 = y2, y1
        x1 = max(0, min(65535, x1))
        y1 = max(0, min(65535, y1))
        x2 = max(0, min(65535, x2))
        y2 = max(0, min(65535, y2))
        s8 = max(0, min(100, int(round(float(score) * 100))))
        out.append(BOX.pack(x1, y1, x2, y2, s8, int(cls) & 0xFF, 0))
    return b"".join(out)


def unpack(data):
    """解析一包, 返回 dict; 格式不对抛 ValueError"""
    if len(data) < HEADER_LEN:
        raise ValueError(f"包太短: {len(data)}")
    magic, ver, flags, count, frame_id, ts_ms = HEADER.unpack_from(data, 0)
    if magic != MAGIC:
        raise ValueError(f"magic 错误: {magic!r}")
    if ver != VERSION:
        raise ValueError(f"版本不支持: {ver}")
    if len(data) < HEADER_LEN + BOX_LEN * count:
        raise ValueError(f"长度不足: {len(data)} < {HEADER_LEN + BOX_LEN * count}")
    boxes = []
    off = HEADER_LEN
    for _ in range(count):
        x1, y1, x2, y2, s8, cls, _res = BOX.unpack_from(data, off)
        boxes.append((x1, y1, x2, y2, s8 / 100.0, cls))
        off += BOX_LEN
    return {"version": ver, "flags": flags, "frame_id": frame_id,
            "timestamp_ms": ts_ms, "boxes": boxes, "length": len(data)}


class BoxSender:
    """UDP 广播发送器; 默认发到 192.168.1.255:6103 (FPGA 未实现 ARP, 必须广播)"""

    def __init__(self, host="192.168.1.255", port=6103, max_boxes=MAX_BOXES,
                 verbose=False):
        self.host = host
        self.port = port
        self.max_boxes = max_boxes
        self.verbose = verbose
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        self.t0 = time.monotonic()
        self.n_sent = 0
        self.n_err = 0
        self.n_bytes = 0
        self.last_error = ""

    def send(self, boxes, frame_id, timestamp_ms=None):
        ts = timestamp_ms if timestamp_ms is not None else \
            int((time.monotonic() - self.t0) * 1000)
        data = pack(frame_id, boxes, ts, max_boxes=self.max_boxes)
        try:
            self.sock.sendto(data, (self.host, self.port))
            self.n_sent += 1
            self.n_bytes += len(data)
        except OSError as e:  # 网卡未配置/无路由等
            self.n_err += 1
            self.last_error = str(e)
            if self.verbose and self.n_err <= 5:
                print(f"[box] 发送失败: {e}")
        return data

    def close(self):
        self.sock.close()


def _cli():
    ap = argparse.ArgumentParser(description="目标框反向通道测试发送器")
    ap.add_argument("--host", default="192.168.1.255")
    ap.add_argument("--port", type=int, default=6103)
    ap.add_argument("--frames", type=int, default=100, help="发送包数")
    ap.add_argument("--fps", type=float, default=20)
    ap.add_argument("--dump", action="store_true", help="打印每包十六进制")
    args = ap.parse_args()

    tx = BoxSender(args.host, args.port)
    print(f"[box] -> {args.host}:{args.port}  {args.frames} 包 @ {args.fps:g}fps")
    for i in range(args.frames):
        # 一个从左到右移动的框(800x480), 分数 0.90, 类别 0(person)
        x = 20 + (i * 12) % 700
        y = 150
        boxes = [(x, y, x + 120, y + 180, 0.90, 0)]
        data = tx.send(boxes, i)
        if args.dump:
            print(f"  #{i}: {data.hex(' ')}")
        time.sleep(1.0 / args.fps)
    tx.close()
    print(f"[box] 完成: 发出 {tx.n_sent} 包, {tx.n_bytes} 字节, 失败 {tx.n_err}")
    return 0


if __name__ == "__main__":
    sys.exit(_cli())
