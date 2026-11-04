#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ONNX -> RKNN 转换 (YOLOv5, RK3588)

在 rknn_env 虚拟环境里运行:
  ./rknn_env/bin/python npu/convert_onnx2rknn.py \
      --onnx models/ball_v2_320.onnx \
      --out models/ball_v2_320_fp16.rknn --no-quant

需要 INT8 时去掉 --no-quant 并指定 --dataset 校准集 txt(每行一张图片绝对路径)。
"""
import argparse
import sys

from rknn.api import RKNN


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--onnx", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--dataset", default="", help="量化校准集 txt (每行一个图片路径)")
    ap.add_argument("--target", default="rk3588")
    ap.add_argument("--no-quant", action="store_true", help="不量化(FP16, 慢但精度稳)")
    ap.add_argument("--mean", default="0,0,0")
    ap.add_argument("--std", default="255,255,255")
    ap.add_argument("--rgb2bgr", type=int, default=1,
                    help="校准图片是否 BGR->RGB (YOLOv5 训练用 RGB, 默认 1)")
    ap.add_argument("--algo", default="mmse",
                    choices=["normal", "mmse", "kl_divergence"],
                    help="量化算法, mmse 通常精度更好")
    ap.add_argument("--opt-level", type=int, default=3)
    args = ap.parse_args()

    mean = [[float(x) for x in args.mean.split(",")]]
    std = [[float(x) for x in args.std.split(",")]]

    rknn = RKNN(verbose=True)
    print(f"[config] target={args.target} mean={mean} std={std} "
          f"rgb2bgr={args.rgb2bgr} algo={args.algo} opt={args.opt_level}")
    rknn.config(
        mean_values=mean,
        std_values=std,
        target_platform=args.target,
        quant_img_RGB2BGR=bool(args.rgb2bgr),
        quantized_algorithm=args.algo,
        optimization_level=args.opt_level,
    )

    print(f"[load_onnx] {args.onnx}")
    if rknn.load_onnx(model=args.onnx) != 0:
        print("加载 ONNX 失败")
        return 1

    do_quant = not args.no_quant
    print(f"[build] quantization={do_quant} dataset={args.dataset or '(无)'}")
    if rknn.build(do_quantization=do_quant, dataset=args.dataset if do_quant else None) != 0:
        print("构建失败")
        return 1

    print(f"[export] {args.out}")
    if rknn.export_rknn(args.out) != 0:
        print("导出失败")
        return 1
    print("完成")
    return 0


if __name__ == "__main__":
    sys.exit(main())
