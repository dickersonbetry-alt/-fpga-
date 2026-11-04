#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""RKNN 运行时自检: 加载模型 + 随机输入推理"""
import sys
import time

import numpy as np

from rknnlite.api import RKNNLite

model = sys.argv[1] if len(sys.argv) > 1 else \
    "/home/orangepi/Desktop/ros2_yolo_text/new_best.rknn"

r = RKNNLite()
print("load_rknn:", r.load_rknn(model))
print("init_runtime:", r.init_runtime(core_mask=RKNNLite.NPU_CORE_0))
print("sdk version:", r.get_sdk_version())

img = np.random.randint(0, 255, (1, 640, 640, 3), dtype=np.uint8)
for i in range(10):
    t = time.perf_counter()
    out = r.inference(inputs=[img])
    dt = (time.perf_counter() - t) * 1000
    print(f"infer {i}: {dt:6.2f} ms  shapes={[o.shape for o in out]}")
r.release()
