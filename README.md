# 基于 FPGA 的具身智能实时感知与自主控制系统

本仓库包含项目的 FPGA 端与 RK3588 端实现。原始压缩包未包含在仓库中，这里存放的是解压后的项目文件。

## 目录结构

- `fpga/`
  - `fpga_project/`：Gowin FPGA 主工程（摄像头采集、DDR3、以太网、电机控制、VGA 显示等）
  - `acg720_138_piano/`：钢琴/触摸相关工程
  - `sw/`：SolidWorks 机械模型
  - `box_return_protocol.md`：目标框回传协议
- `rk3588/`
  - `yolo_fpga.py`、`recv_fpga.py`：目标检测与 FPGA 通信脚本
  - `ball_v2/`：YOLOv5 训练结果与权重
  - `models/`：ONNX / RKNN 模型
  - `npu/`：RKNN 运行库与 NPU 测试脚本
  - `yolov5_v70/`：YOLOv5 源码

## 快速入口

- FPGA 协议文档：[`fpga/box_return_protocol.md`](fpga/box_return_protocol.md)
- NPU 说明：[`rk3588/npu/README.md`](rk3588/npu/README.md)
- FPGA 顶层：[`fpga/fpga_project/src/fpga_top.v`](fpga/fpga_project/src/fpga_top.v)
- RK3588 主脚本：[`rk3588/yolo_fpga.py`](rk3588/yolo_fpga.py)

## 说明

仓库包含较大的模型权重和厂商运行库，克隆时请耐心等待。`rk3588/npu/` 下的 `.so` 运行库需与目标板上的 RKNN Runtime 版本匹配。
