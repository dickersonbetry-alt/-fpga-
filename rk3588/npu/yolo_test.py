#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""单张图片 RKNN YOLOv5 推理验证"""
import sys

import cv2
import numpy as np

from rknnlite.api import RKNNLite

MODEL = "/home/orangepi/Desktop/ros2_yolo_text/new_best.rknn"
CONF = 0.25
IOU = 0.45
IN = 640


def letterbox(img, new=IN, color=(114, 114, 114)):
    h, w = img.shape[:2]
    r = min(new / w, new / h)
    nw, nh = int(round(w * r)), int(round(h * r))
    resized = cv2.resize(img, (nw, nh), interpolation=cv2.INTER_LINEAR)
    canvas = np.full((new, new, 3), color, dtype=np.uint8)
    dx, dy = (new - nw) // 2, (new - nh) // 2
    canvas[dy:dy + nh, dx:dx + nw] = resized
    return canvas, r, dx, dy


def main():
    path = sys.argv[1]
    img = cv2.imread(path)
    assert img is not None, path
    lb, scale, dx, dy = letterbox(img)
    inp = np.ascontiguousarray(lb[:, :, ::-1])[None]  # BGR->RGB, NHWC uint8

    print("加载模型...")
    r = RKNNLite()
    assert r.load_rknn(MODEL) == 0
    assert r.init_runtime(core_mask=RKNNLite.NPU_CORE_0_1_2) == 0
    print("推理中...")
    out = r.inference(inputs=[inp])[0][0]  # (25200,6)
    r.release()
    print("推理完成, 输出", out.shape)

    conf = out[:, 4] * out[:, 5]
    keep = conf >= CONF
    boxes = out[keep, :4]
    scores = conf[keep]
    print(f"候选框 {int(keep.sum())} 个, 最高置信度 {scores.max() if len(scores) else 0:.3f}")
    if len(scores):
        idx = cv2.dnn.NMSBoxes(boxes.tolist(), scores.tolist(), CONF, IOU)
        idx = np.array(idx).reshape(-1)
        print(f"NMS 后 {len(idx)} 个")
        for i in idx[:10]:
            cx, cy, w, h = boxes[i]
            x1 = (cx - w / 2 - dx) / scale
            y1 = (cy - h / 2 - dy) / scale
            x2 = (cx + w / 2 - dx) / scale
            y2 = (cy + h / 2 - dy) / scale
            print(f"  cls0 conf={scores[i]:.3f} box=({x1:.0f},{y1:.0f},{x2:.0f},{y2:.0f})")
            cv2.rectangle(img, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
            cv2.putText(img, f"cls0 {scores[i]:.2f}", (int(x1), max(0, int(y1) - 6)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
    out_path = "/tmp/yolo_test.jpg"
    cv2.imwrite(out_path, img)
    print("保存:", out_path)


if __name__ == "__main__":
    main()
