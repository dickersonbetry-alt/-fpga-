#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""单图 RKNN YOLOv5 检测测试(通用, 自动识别类别数)"""
import sys

import cv2
import numpy as np

from rknnlite.api import RKNNLite

COCO = ["person", "bicycle", "car", "motorcycle", "airplane", "bus", "train",
        "truck", "boat", "traffic light", "fire hydrant", "stop sign",
        "parking meter", "bench", "bird", "cat", "dog", "horse", "sheep", "cow",
        "elephant", "bear", "zebra", "giraffe", "backpack", "umbrella", "handbag",
        "tie", "suitcase", "frisbee", "skis", "snowboard", "sports ball", "kite",
        "baseball bat", "baseball glove", "skateboard", "surfboard", "tennis racket",
        "bottle", "wine glass", "cup", "fork", "knife", "spoon", "bowl", "banana",
        "apple", "sandwich", "orange", "broccoli", "carrot", "hot dog", "pizza",
        "donut", "cake", "chair", "couch", "potted plant", "bed", "dining table",
        "toilet", "tv", "laptop", "mouse", "remote", "keyboard", "cell phone",
        "microwave", "oven", "toaster", "sink", "refrigerator", "book", "clock",
        "vase", "scissors", "teddy bear", "hair drier", "toothbrush"]


def main():
    model = sys.argv[1]
    img_path = sys.argv[2]
    conf_thres, iou_thres = 0.7, 0.45
    if len(sys.argv) > 3:
        conf_thres = float(sys.argv[3])
    IN = int(sys.argv[4]) if len(sys.argv) > 4 else 640

    img = cv2.imread(img_path)
    assert img is not None, img_path
    h, w = img.shape[:2]
    s = min(IN / w, IN / h)
    nw, nh = int(round(w * s)), int(round(h * s))
    canvas = np.full((IN, IN, 3), 114, np.uint8)
    dx, dy = (IN - nw) // 2, (IN - nh) // 2
    canvas[dy:dy + nh, dx:dx + nw] = cv2.resize(img, (nw, nh))
    inp = np.ascontiguousarray(canvas[:, :, ::-1])[None]

    r = RKNNLite()
    print("load_rknn:", r.load_rknn(model))
    print("init_runtime:", r.init_runtime(core_mask=RKNNLite.NPU_CORE_0_1_2))
    print(r.get_sdk_version())
    import time
    t = time.perf_counter()
    out = r.inference(inputs=[inp])
    dt = (time.perf_counter() - t) * 1000
    r.release()

    pred = out[0][0]
    nc = pred.shape[1] - 5
    conf = pred[:, 4] * pred[:, 5:5 + nc].max(1) if nc else pred[:, 4]
    keep = conf >= conf_thres
    boxes = pred[keep, :4]
    scores = conf[keep]
    classes = pred[keep, 5:5 + nc].argmax(1) if nc else np.zeros(len(scores), int)
    print(f"inference {dt:.1f} ms, 输出 {pred.shape}, 候选 {int(keep.sum())}")

    idx = np.array(cv2.dnn.NMSBoxes(boxes.tolist(), scores.tolist(),
                                    conf_thres, iou_thres)).reshape(-1)
    print(f"NMS 后 {len(idx)} 个目标")
    for i in idx:
        cx, cy, bw, bh = boxes[i]
        x1 = max(0, int((cx - bw / 2 - dx) / s))
        y1 = max(0, int((cy - bh / 2 - dy) / s))
        x2 = min(w - 1, int((cx + bw / 2 - dx) / s))
        y2 = min(h - 1, int((cy + bh / 2 - dy) / s))
        name = COCO[int(classes[i])] if int(classes[i]) < len(COCO) else f"cls{int(classes[i])}"
        if nc != 80:
            name = f"cls{int(classes[i])}"
        print(f"  {name:12s} {scores[i]:.3f} ({x1},{y1})-({x2},{y2})")
        cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.putText(img, f"{name} {scores[i]:.2f}", (x1, max(12, y1 - 6)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

    out_path = "/tmp/rknn_yolo_test.jpg"
    cv2.imwrite(out_path, img)
    print("保存:", out_path)


if __name__ == "__main__":
    main()
