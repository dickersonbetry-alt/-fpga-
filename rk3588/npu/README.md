# NPU (RK3588) 环境说明

## 硬件/驱动状态
- NPU 通过 DRM 节点 `/dev/dri/renderD129` 暴露（`DRIVER=RKNPU`），用户 `orangepi` 有 ACL 读写权限
- 驱动版本：`0.9.6`；NPU 频率 `1.0 GHz`（`/sys/class/devfreq/fdab0000.npu`）
- 系统自带运行时 `/usr/lib/librknnrt.so` 是 **1.4.0**，无法加载新模型（v6）

## 免 root 使用新运行时
本目录放了两份官方 aarch64 运行时：

| 文件 | 版本 | 说明 |
|---|---|---|
| `librknnrt_160.so` | 1.6.0 | 旧模型兼容 |
| `librknnrt_250.so` | 2.5.0 | **当前默认**（ball_v2 模型由 toolkit 2.5.0 转换） |

`npu_run.sh` 会在用户命名空间里把它绑定挂载到 `/usr/lib/librknnrt.so`，不改动系统：

```bash
cd ~/Desktop/fpga/npu
./npu_run.sh python3 npu_test.py                  # 默认 2.5.0
RKNN_LIB=$PWD/librknnrt_160.so ./npu_run.sh python3 npu_test.py   # 指定 1.6.0
```

## Python 环境
- 已安装 `rknn-toolkit-lite2 2.5.0`（用户目录 `~/.local`）
- 已安装 `torch 2.2.2 + torchvision 0.17.2`（用于导出 ONNX / CPU 验证）
- `rknn_env` 虚拟环境：`rknn-toolkit2 2.5.0` + onnx/onnxruntime，用于板端 ONNX→RKNN 转换

## 实测
- 新模型 `ball_v2`：YOLOv5s 单类别 `ball`，640 训练，FP16 RKNN
- 320 模型 **19 ms/帧（≈52 fps）**，640 模型 **70 ms/帧（≈14 fps）**
- 实测帧上置信度约 **0.98**；整条实时管线（含收图/NMS/画框）可达 **29 fps**

## 相关脚本
- `npu_run.sh`：免 root 挂载新版 RKNN 运行时
- `npu_test.py`：加载模型 + 随机输入测速
- `test_rknn_yolo.py`：通用单图检测（自动识别类别数，支持 `imgsz` 参数）
- `convert_onnx2rknn.py`：ONNX → RKNN 转换（在 `rknn_env` 中运行）
- `box_recv_test.py`：目标框回传包接收解析
- `send_image_udp.py`：把一张 800×480 图片按上行协议回放为 UDP 流

## YOLOv5 模型转换（板端完成）

```bash
# 1) 导出 ONNX（使用 v7.0 代码，新版仓库会强依赖 ultralytics）
cd ~/Desktop/fpga/yolov5_v70
../rknn_env/bin/python export.py --weights ../ball_v2/weights/best.pt \
    --include onnx --opset 12 --imgsz 320 320     # 640 版本改 --imgsz 640 640
mv ../ball_v2/weights/best.onnx ../models/ball_v2_320.onnx

# 2) 转 RKNN（venv 内）
cd ~/Desktop/fpga
./rknn_env/bin/python npu/convert_onnx2rknn.py \
    --onnx models/ball_v2_320.onnx --out models/ball_v2_320_fp16.rknn --no-quant
```

量化需要校准集，但本系列模型 INT8 量化后 objectness 会塌缩为 0，
因此统一使用 **FP16**。如需 INT8 精度，可尝试 hybrid quantization 或更多样化的校准集。

## 已构建模型

| 文件 | 输入 | 精度 | 单帧耗时 | 说明 |
|---|---|---|---|---|
| `ball_v2_320_fp16.rknn` | 320×320 | FP16 | **~19 ms（≈52fps）** | **默认模型**，实时首选 |
| `ball_v2_640_fp16.rknn` | 640×640 | FP16 | ~70 ms（≈14fps） | 高精度版本 |

> 旧的 `yolov5s_*` 模型与 ONNX、校准集已删除。

## 实时检测

```bash
cd ~/Desktop/fpga
./npu/npu_run.sh python3 yolo_fpga.py --conf 0.7 \
    --save-video recordings/yolo_320.mp4 --save-det recordings/yolo_dets
```

- 默认模型 `models/ball_v2_320_fp16.rknn`、`--imgsz 320`，类别名 `ball`
- 收包线程与 NPU 推理线程分离，只保留最新帧
- 实测：源 30fps，整条管线约 **29 fps**；终端会打印球坐标

## 其他
- 已安装 `torch 2.2.2 + torchvision 0.17.2`（仅用于导出 ONNX 和 CPU 验证）
- CPU 直接跑 yolov5s 只有 **~1 fps**，不可用于实时

## 目标框回传（RK3588 → FPGA）
- 协议文档（给 FPGA 侧）：`docs/box_return_protocol.md`
- 发送模块：`box_protocol.py`，单独联调：
  `python3 box_protocol.py --host 192.168.1.255 --port 6103 --frames 300 --fps 20`
- 接收测试：`python3 npu/box_recv_test.py --port 6103`
- 正式使用：
  `./npu/npu_run.sh python3 yolo_fpga.py --send-boxes`
- 默认发往 `192.168.1.255:6103`（广播，避免 FPGA 实现 ARP）；每帧 1 包，无目标时 `count=0`
- 当前模型单类别 `ball`，回传包中 `class_id = 0`


