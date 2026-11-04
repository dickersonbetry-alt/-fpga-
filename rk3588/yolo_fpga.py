#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
FPGA 摄像头 UDP 流 -> RKNN NPU 实时 YOLO 检测

需在 NPU 运行时环境下启动(npu_run.sh 会把新版 librknnrt 挂到 /usr/lib):
  cd ~/Desktop/fpga
  ./npu/npu_run.sh python3 yolo_fpga.py

默认模型: models/ball_v2_320_fp16.rknn (YOLOv5s 单类别 ball, 320 输入, 约 19ms/帧)

常用参数:
  --model models/ball_v2_640_fp16.rknn --imgsz 640   换高精度模型(约 70ms/帧)
  --conf 0.7 --iou 0.45         置信度/IOU 阈值(默认 0.7)
  --duration 10                 运行 10 秒后退出(默认 0 = 直到 Ctrl-C)
  --save-video out.mp4          保存画框图视频
  --save-det det_dir            有检测时存图(默认最多 20 张)
  --names ball                  类别名, 默认 ball; COCO 模型传 coco
  --print-det / --no-print-det  终端打印目标坐标(默认开, 0.5s 限频)
  --print-interval 0.5          坐标打印最小间隔(秒), 0=每帧都打印
  --print-max 10                每条最多打印几个目标
  --send-boxes                  把检测框 UDP 广播回传给 FPGA
"""
import argparse
import os
import queue
import socket
import sys
import threading
import time

import cv2
import numpy as np

from rknnlite.api import RKNNLite

from box_protocol import BoxSender

W, H = 800, 480
PKT_LEN = 2 + W * 2
IN = 640

COCO_NAMES = [
    "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck",
    "boat", "traffic light", "fire hydrant", "stop sign", "parking meter", "bench",
    "bird", "cat", "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra",
    "giraffe", "backpack", "umbrella", "handbag", "tie", "suitcase", "frisbee",
    "skis", "snowboard", "sports ball", "kite", "baseball bat", "baseball glove",
    "skateboard", "surfboard", "tennis racket", "bottle", "wine glass", "cup",
    "fork", "knife", "spoon", "bowl", "banana", "apple", "sandwich", "orange",
    "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair", "couch",
    "potted plant", "bed", "dining table", "toilet", "tv", "laptop", "mouse",
    "remote", "keyboard", "cell phone", "microwave", "oven", "toaster", "sink",
    "refrigerator", "book", "clock", "vase", "scissors", "teddy bear", "hair drier",
    "toothbrush",
]


def letterbox(img, new=IN, color=(114, 114, 114)):
    h, w = img.shape[:2]
    r = min(new / w, new / h)
    nw, nh = int(round(w * r)), int(round(h * r))
    canvas = np.full((new, new, 3), color, dtype=np.uint8)
    dx, dy = (new - nw) // 2, (new - nh) // 2
    canvas[dy:dy + nh, dx:dx + nw] = cv2.resize(img, (nw, nh), interpolation=cv2.INTER_LINEAR)
    return canvas, r, dx, dy


class Detector(threading.Thread):
    """后台推理线程: 只处理最新帧, 避免积压"""

    def __init__(self, model_path, names, conf, iou, imgsz=640, video_fps=20,
                 save_video="", save_det="", max_det=20, box_sender=None,
                 print_det=True, print_interval=0.5, print_max=10):
        super().__init__(daemon=True)
        self.names = names
        self.conf_thres = conf
        self.iou_thres = iou
        self.imgsz = imgsz
        self.video_fps = video_fps
        self.box_sender = box_sender
        self.print_det = print_det
        self.print_interval = print_interval
        self.print_max = print_max
        self._last_print = 0.0
        self.q = queue.Queue(maxsize=1)
        self.stop_evt = threading.Event()
        self.n_infer = 0
        self.n_det_frames = 0
        self.n_det_objs = 0
        self.n_saved = 0
        self.save_video = save_video
        self.save_det = save_det
        self.max_det = max_det
        self.dropped = 0
        self.writer = None
        if save_video:
            os.makedirs(os.path.dirname(save_video) or ".", exist_ok=True)
            self.writer = cv2.VideoWriter(
                save_video, cv2.VideoWriter_fourcc(*"mp4v"), video_fps, (W, H))
            if not self.writer.isOpened():
                print(f"[warn] 无法创建视频 {save_video}")
                self.writer = None
        if save_det:
            os.makedirs(save_det, exist_ok=True)

        self.rknn = RKNNLite()
        if self.rknn.load_rknn(model_path) != 0:
            raise RuntimeError(f"加载模型失败: {model_path}")
        if self.rknn.init_runtime(core_mask=RKNNLite.NPU_CORE_0_1_2) != 0:
            raise RuntimeError("NPU 初始化失败")

    def submit(self, frame_bgr):
        try:
            self.q.put_nowait(frame_bgr.copy())
        except queue.Full:
            self.dropped += 1

    def run(self):
        last_t = time.time()
        fps_ema = 0.0
        while not self.stop_evt.is_set():
            try:
                frame = self.q.get(timeout=0.2)
            except queue.Empty:
                continue
            lb, scale, dx, dy = letterbox(frame, self.imgsz)
            inp = np.ascontiguousarray(lb[:, :, ::-1])[None]  # BGR->RGB NHWC uint8

            out = self.rknn.inference(inputs=[inp])
            if out is None:
                continue
            pred = out[0][0]  # (anchors, 5+nc)
            nc = pred.shape[1] - 5
            conf = pred[:, 4] * pred[:, 5:5 + nc].max(1) if nc > 0 else pred[:, 4]
            keep = conf >= self.conf_thres

            boxes_xywh = pred[keep, :4]
            scores = conf[keep]
            classes = pred[keep, 5:5 + nc].argmax(1) if nc > 0 else np.zeros(len(scores), int)
            dets = []
            if len(scores):
                idx = cv2.dnn.NMSBoxes(
                    boxes_xywh.tolist(), scores.tolist(), self.conf_thres, self.iou_thres)
                idx = np.array(idx).reshape(-1)
                for i in idx[:300]:
                    cx, cy, bw, bh = boxes_xywh[i]
                    x1 = max(0, (cx - bw / 2 - dx) / scale)
                    y1 = max(0, (cy - bh / 2 - dy) / scale)
                    x2 = min(W - 1, (cx + bw / 2 - dx) / scale)
                    y2 = min(H - 1, (cy + bh / 2 - dy) / scale)
                    dets.append((int(x1), int(y1), int(x2), int(y2),
                                 float(scores[i]), int(classes[i])))

            # 终端打印目标坐标(限频, 避免 20fps 刷屏)
            if dets and self.print_det:
                nowp = time.monotonic()
                if self.print_interval <= 0 or nowp - self._last_print >= self.print_interval:
                    self._last_print = nowp
                    print(f"[det] {time.strftime('%H:%M:%S')} fid={self.n_infer + 1} "
                          f"目标 {len(dets)} 个:", flush=True)
                    for k, (x1, y1, x2, y2, s, c) in enumerate(dets[:self.print_max], 1):
                        name = self.names[c] if c < len(self.names) else f"cls{c}"
                        print(f"       #{k} {name:<12s} conf={s:.3f} "
                              f"box=({x1},{y1})-({x2},{y2}) "
                              f"尺寸={x2 - x1 + 1}x{y2 - y1 + 1} "
                              f"中心=({(x1 + x2) // 2},{(y1 + y2) // 2})", flush=True)
                    if len(dets) > self.print_max:
                        print(f"       ... 其余 {len(dets) - self.print_max} 个略", flush=True)

            drawn = frame
            for x1, y1, x2, y2, s, c in dets:
                name = self.names[c] if c < len(self.names) else f"cls{c}"
                color = (0, 255, 0)
                cv2.rectangle(drawn, (x1, y1), (x2, y2), color, 2)
                cv2.putText(drawn, f"{name} {s:.2f}", (x1, max(12, y1 - 6)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

            self.n_infer += 1
            if self.box_sender is not None:
                # 每帧都发(无目标时 count=0), FPGA 可据此清除旧框
                self.box_sender.send(dets, self.n_infer)
            if dets:
                self.n_det_frames += 1
                self.n_det_objs += len(dets)
                if self.save_det and self.n_saved < self.max_det:
                    p = os.path.join(self.save_det, f"det_{self.n_infer:06d}.jpg")
                    cv2.imwrite(p, drawn)
                    self.n_saved += 1
            if self.writer is not None:
                self.writer.write(drawn)

            now = time.time()
            dt = now - last_t
            if dt > 0:
                fps_ema = 0.9 * fps_ema + 0.1 * (1.0 / dt) if fps_ema else 1.0 / dt
            last_t = now

        if self.writer is not None:
            self.writer.release()
        if self.box_sender is not None:
            self.box_sender.close()
        self.rknn.release()

    def stop(self):
        self.stop_evt.set()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--model", default="models/ball_v2_320_fp16.rknn",
                    help="RKNN 模型路径(默认 ball_v2 320 输入)")
    ap.add_argument("--bind", default="0.0.0.0")
    ap.add_argument("--port", type=int, default=6102)
    ap.add_argument("--conf", type=float, default=0.7, help="置信度阈值, 默认 0.7")
    ap.add_argument("--iou", type=float, default=0.45)
    ap.add_argument("--imgsz", type=int, default=320, help="模型输入尺寸, 需与导出时一致")
    ap.add_argument("--video-fps", type=float, default=20, help="画框视频的帧率(接近推理帧率即可)")
    ap.add_argument("--duration", type=float, default=0, help="0=直到 Ctrl-C")
    ap.add_argument("--save-video", default="", help="画框结果保存为视频")
    ap.add_argument("--save-det", default="", help="检测到目标时存图目录")
    ap.add_argument("--max-det-images", type=int, default=20)
    ap.add_argument("--names", default="ball",
                    help="类别名, 默认 ball; 逗号分隔; COCO 80 类模型传 coco")
    ap.add_argument("--bufsize", type=int, default=8 << 20)
    ap.add_argument("--send-boxes", action="store_true",
                    help="把检测框通过 UDP 广播回传给 FPGA")
    ap.add_argument("--box-host", default="192.168.1.255",
                    help="框接收地址(默认广播; FPGA 未实现 ARP)")
    ap.add_argument("--box-port", type=int, default=6103)
    ap.add_argument("--box-max", type=int, default=16, help="每包最多几个框")
    ap.add_argument("--print-det", dest="print_det", action="store_true", default=True,
                    help="终端打印目标坐标(默认开启)")
    ap.add_argument("--no-print-det", dest="print_det", action="store_false",
                    help="关闭终端坐标打印")
    ap.add_argument("--print-interval", type=float, default=0.5,
                    help="坐标打印最小间隔(秒), 0=每帧都打印")
    ap.add_argument("--print-max", type=int, default=10, help="每条最多打印几个目标")
    args = ap.parse_args()

    # 类别名: COCO 80 类 / 自定义逗号分隔
    if args.names == "coco":
        names = COCO_NAMES
    else:
        names = args.names.split(",")

    if not os.path.exists(args.model):
        print(f"[error] 模型文件不存在: {args.model}")
        print(f"        当前目录: {os.getcwd()}")
        print("        可用模型: models/ball_v2_320_fp16.rknn / models/ball_v2_640_fp16.rknn")
        return 2

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    try:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, args.bufsize)
    except OSError:
        pass
    sock.bind((args.bind, args.port))
    sock.settimeout(0.5)
    print(f"[info] 监听 udp {args.bind}:{args.port}  模型 {args.model}  "
          f"conf>={args.conf:g}  iou={args.iou:g}")

    box_sender = None
    if args.send_boxes:
        box_sender = BoxSender(args.box_host, args.box_port,
                               max_boxes=args.box_max, verbose=True)
        print(f"[info] 目标框回传 -> {args.box_host}:{args.box_port} "
              f"(每帧 1 包, 无目标时 count=0)")

    det = Detector(args.model, names, args.conf, args.iou, imgsz=args.imgsz,
                   video_fps=args.video_fps, save_video=args.save_video, save_det=args.save_det,
                   max_det=args.max_det_images, box_sender=box_sender,
                   print_det=args.print_det, print_interval=args.print_interval,
                   print_max=args.print_max)
    det.start()

    frame = np.zeros((H, W), dtype=np.uint16)
    seen = np.zeros(H, dtype=bool)
    buf = bytearray(4096)
    mv = memoryview(buf)

    n_pkt = n_bad = n_ends = n_complete = n_partial = 0
    last_row = H - 1
    start = last_stat = time.time()
    last_pkt = None

    def finish():
        nonlocal n_ends, n_complete, n_partial
        n_ends += 1
        if seen.all():
            n_complete += 1
            det.submit(cv2.cvtColor(frame, cv2.COLOR_RGB5652BGR)
                       if hasattr(cv2, "COLOR_RGB5652BGR") else _rgb565_to_bgr(frame))
        else:
            n_partial += 1
        seen[:] = False

    def _rgb565_to_bgr(f):
        v = f.astype(np.uint32)
        r = ((v >> 11) & 0x1F) * 255 // 31
        g = ((v >> 5) & 0x3F) * 255 // 63
        b = (v & 0x1F) * 255 // 31
        return np.dstack([b, g, r]).astype(np.uint8)

    try:
        while True:
            now = time.time()
            if args.duration and now - start >= args.duration:
                break
            try:
                n, addr = sock.recvfrom_into(buf)
            except socket.timeout:
                continue
            now = time.time()
            last_pkt = now
            n_pkt += 1
            if n != PKT_LEN:
                n_bad += 1
                continue
            pkt = mv[:n]
            row = pkt[0] | (pkt[1] << 8)
            if row >= H:
                n_bad += 1
                continue
            if seen.any() and row != H - 1 and last_row >= 3 * H // 4 and row <= H // 4:
                finish()
            frame[row] = np.frombuffer(pkt[2:], dtype="<u2")
            seen[row] = True
            if row == H - 1:
                finish()
            last_row = row

            if now - last_stat >= 1.0:
                print(f"[stat] t={now-start:5.1f}s 收包={n_pkt} 完整帧={n_complete} "
                      f"推理={det.n_infer} 有目标帧={det.n_det_frames} 目标={det.n_det_objs} "
                      f"发框={box_sender.n_sent if box_sender else 0} 跳帧={det.dropped}",
                      flush=True)
                last_stat = now
    except KeyboardInterrupt:
        print("\n[info] 停止中...")
    finally:
        sock.close()
        det.stop()
        det.join(timeout=10)

    dt = time.time() - start
    print("=" * 60)
    print(f"运行 {dt:.1f}s  收包 {n_pkt}  完整帧 {n_complete}")
    print(f"推理 {det.n_infer} 帧 ({det.n_infer/max(dt,1e-9):.1f} fps), "
          f"有目标帧 {det.n_det_frames}, 目标总数 {det.n_det_objs}, 保存检测图 {det.n_saved}")
    if det.box_sender is not None:
        bs = det.box_sender
        print(f"目标框回传: {bs.n_sent} 包, {bs.n_bytes} 字节, 失败 {bs.n_err}"
              + (f" ({bs.last_error})" if bs.n_err else ""))
    if args.save_video:
        print(f"视频: {args.save_video}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
