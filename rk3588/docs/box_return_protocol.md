# 目标框回传协议说明（RK3588 → FPGA）

版本：**v1.1**（2026-11-04）｜ 适用：ACG720 / GW5AT-138C（RK3588 侧 YOLO 检测结果下发）

---

## 1. 概述

RK3588 在 NPU 上完成 YOLO 推理后，把检测到的目标框通过 **UDP 广播**回传给 FPGA，
供 FPGA 做框叠加、云台跟踪或运动决策。

- **每完成一帧推理就发送 1 个 UDP 包**；没有目标时也发（`count = 0`），FPGA 可据此清除旧框。
- 只回传置信度 **≥ 0.7** 的目标（RK3588 端 `--conf` 可调），低置信度目标不会出现在包里。
- FPGA 未实现 ARP/ICMP，因此 RK3588 使用**广播地址**发送，FPGA 不需要回包，也不需要 ARP。
- 数据全部为**小端字节序**，与上行图像协议一致。

```
FPGA ──UDP 广播 192.168.1.255:6102──▶ RK3588（图像，每行一包）
FPGA ◀─UDP 广播 192.168.1.255:6103── RK3588（目标框，每帧一包）
```

---

## 2. 网络参数

| 项目 | 值 |
|---|---|
| 源 IP | 192.168.1.100（RK3588 `enP3p49s0`） |
| 源端口 | 随机临时端口（**不要按源端口过滤**） |
| 目的 IP | **192.168.1.255**（广播） |
| 目的 MAC | **FF:FF:FF:FF:FF:FF** |
| 目的端口 | **UDP 6103** |
| 包长 | **16 + 12 × count** 字节（count 0~16，最大 208 字节） |
| 发送频率 | 约 15~30 包/秒（等于 NPU 推理帧率，低于摄像头 30fps） |
| UDP 校验和 | 由 RK3588 内核正常计算（非 0）；FPGA 可校验也可直接忽略 |
| IP 校验和 | 正常计算，校验可选 |

> 广播包不经过 ARP，FPGA 只需在 RX 路径放行目的 MAC 为广播、目的 IP 为 192.168.1.255 的帧。

---

## 3. 数据包结构

### 3.1 包头（16 字节）

| 偏移 | 长度 | 字段 | 说明 |
|---:|---:|---|---|
| 0 | 4 | `magic` | ASCII `"YBOX"`，字节序列 `59 42 4F 58` |
| 4 | 1 | `version` | 协议版本，当前 **0x01**，不符则丢弃 |
| 5 | 1 | `flags` | 保留，当前恒为 0（接收端忽略） |
| 6 | 2 | `count` | 目标框数量，`0 ~ 16`（uint16，小端） |
| 8 | 4 | `frame_id` | 推理帧序号，uint32 递增，溢出回绕（可据此统计丢包） |
| 12 | 4 | `timestamp_ms` | 发送端启动后的毫秒数，uint32，约 49.7 天回绕（仅用于超时判断，可忽略） |

### 3.2 目标框（每个 12 字节，紧跟包头，连续 count 个）

| 偏移(框内) | 长度 | 字段 | 说明 |
|---:|---:|---|---|
| 0 | 2 | `x1` | 左上角 X，uint16 |
| 2 | 2 | `y1` | 左上角 Y，uint16 |
| 4 | 2 | `x2` | 右下角 X，uint16 |
| 6 | 2 | `y2` | 右下角 Y，uint16 |
| 8 | 1 | `score` | 置信度 ×100，范围 **0~100**（91 表示 0.91） |
| 9 | 1 | `class_id` | 类别索引，当前 ball_v2 模型固定为 **0（ball）**；换模型时按训练类别 |
| 10 | 2 | `reserved` | 保留，当前恒为 0 |

### 3.3 坐标约定

- 坐标系为**原图 800×480 像素坐标**，左上角为 `(0, 0)`，右下角为 `(799, 479)`。
- `x2/y2` 为右下角（**含边界**），发送前已保证：
  - `x1 <= x2`，`y1 <= y2`
  - 全部裁剪在画面内（0 ≤ x ≤ 799，0 ≤ y ≤ 479）
- 与上行图像的像素坐标完全一致，FPGA 可直接用于叠加显示。
- 宽高 = `x2 - x1 + 1`、`y2 - y1 + 1`，最小为 1。

---

## 4. 示例数据包

一帧包含 2 个目标，`frame_id = 12345`，`timestamp_ms = 6789`：

```text
59 42 4f 58  01 00  02 00  39 30 00 00  85 1a 00 00   <- 包头(16B)
0a 00 14 00  6e 00 c8 00  5b 00 00 00                  <- 框1(12B)
f4 01 2c 01  bc 02 d6 01  2e 00 00 00                  <- 框2(12B)
```

解析：

| 字段 | 原始字节 | 值 |
|---|---|---|
| magic | `59 42 4f 58` | `"YBOX"` |
| version | `01` | 1 |
| flags | `00` | 0 |
| count | `02 00` | 2 |
| frame_id | `39 30 00 00` | 12345 |
| timestamp_ms | `85 1a 00 00` | 6789 |
| 框1 | `0a 00 14 00 6e 00 c8 00 5b 00 00 00` | (10,20)-(110,200)，score=91%，cls=0 |
| 框2 | `f4 01 2c 01 bc 02 d6 01 2e 00 00 00` | (500,300)-(700,470)，score=46%，cls=0 (ball) |

---

## 5. 发送行为与容错

| 场景 | 发送端行为 | 建议 FPGA 处理 |
|---|---|---|
| 有目标 | `count = N`，正常框 | 更新内部框列表 |
| 无目标 | `count = 0`（仍发送） | 清空框列表 |
| 丢包 / 跳帧 | `frame_id` 可能跳号 | 直接使用新包，无需重传 |
| 长时间无包 | 超过 200 ms 未收到 | 清空框列表（目标已丢失） |
| 包长不对 | — | 丢弃：要求 `payload_len == 16 + 12*count` |
| magic/version 错 | — | 丢弃 |
| 超过 16 个框 | 发送端只发前 16 个 | 按 count 读取，缓冲按 16 设计 |

> `score` 分辨率为 1%（0.01），已足够叠加显示/跟踪使用。

---

## 6. FPGA 接收实现建议

### 6.1 RX 过滤条件

```text
MAC 目的地址 = FF:FF:FF:FF:FF:FF （或本机 MAC）
EtherType    = 0x0800 (IPv4)
IP 协议号    = 17 (UDP)
IP 目的地址  = 192.168.1.255 （或本机 192.168.1.10）
UDP 目的端口 = 6103
UDP 长度     = 8 + payload_len，payload_len = 16 + 12*count
```

注意：
- 必须放行**广播帧**（部分 MAC RX 过滤默认丢弃广播，需要在初始化时打开）。
- UDP 校验和由 RK3588 内核正常计算；FPGA 若不校验直接忽略即可。

### 6.2 解析状态机

```text
IDLE
  └─ 收到 UDP 头且目的端口=6103 ──▶ HEADER
HEADER（收 16 字节）
  ├─ magic != "YBOX" 或 version != 1 ──▶ 丢弃回 IDLE
  └─ 读取 count，检查 count <= 16 ──▶ BOXES
BOXES（循环收 12 字节 × count）
  └─ 写入 shadow 缓冲（x1,y1,x2,y2,score,class_id）
DONE
  └─ 校验总长 == 16 + 12*count 且无超短包 ──▶ 整包切换生效 / 输出更新脉冲
```

### 6.3 双缓冲（重要）

建议使用两块 16×12B 的 shadow/active 缓冲：

- 收包过程中只写 shadow；
- 整包校验通过后，一个时钟周期内切换 active 指针，并输出 `box_update` 脉冲；
- 这样叠加/跟踪模块永远看到完整的一帧，不会出现“半个包”的框。

### 6.4 输出接口建议

```verilog
output reg        box_valid;        // 收到新的一包后置 1 个周期
output reg [4:0]  box_count;        // 0~16
output reg [15:0] box_x1 [0:15];    // 组合成 BRAM 也可以
output reg [15:0] box_y1 [0:15];
output reg [15:0] box_x2 [0:15];
output reg [15:0] box_y2 [0:15];
output reg [7:0]  box_score [0:15]; // 0~100
output reg [7:0]  box_class [0:15];
output reg [31:0] box_frame_id;
```

资源开销：16 × 12B ≈ **192 字节**，可用 BRAM 或寄存器实现。

### 6.5 Verilog 解析伪代码

```verilog
localparam S_IDLE = 0, S_HDR = 1, S_BOX = 2, S_DONE = 3;
reg [1:0] st;
reg [4:0] idx;
reg [15:0] payload_len;
reg [15:0] byte_cnt;

always @(posedge clk) begin
  box_valid <= 0;
  case (st)
    S_IDLE: if (udp_hit && udp_dport == 16'd6103) begin
        byte_cnt <= 0; st <= S_HDR;
    end
    S_HDR: if (udp_byte_valid) begin
        hdr_buf[byte_cnt] <= udp_byte;
        if (byte_cnt == 15) begin
            // 检查 magic/version，取 count
            if (hdr_buf[0..3] != "YBOX" || hdr_buf[4] != 1 || count > 16) st <= S_IDLE;
            else begin idx <= 0; byte_cnt <= 0; st <= S_BOX; end
        end else byte_cnt <= byte_cnt + 1;
    end
    S_BOX: if (udp_byte_valid) begin
        shadow[idx][byte_cnt] <= udp_byte;
        if (byte_cnt == 11) begin
            byte_cnt <= 0;
            if (idx == count-1) st <= S_DONE;
            else idx <= idx + 1;
        end else byte_cnt <= byte_cnt + 1;
    end
    S_DONE: begin
        // 整包完成：切换 active/shadow，输出 box_valid
        swap <= ~swap; box_valid <= 1; box_frame_id <= hdr_frame_id; st <= S_IDLE;
    end
  endcase
end
```

> 实际实现时建议在 `S_HDR` 里额外校验 `udp_len - 8 == 16 + 12*count`；不匹配直接丢弃。

---

## 7. RK3588 侧启动与联调

### 7.1 启动 YOLO 检测并回传（前台）

```bash
cd ~/Desktop/fpga

# 每次开机执行一次：配置 192.168.1.100 + 放大接收缓冲（需要输入密码）
sudo bash setup_net.sh

# 启动 NPU 推理 + 终端坐标打印 + UDP 回传
# (默认模型就是 ball_v2 320，可省略 --model/--imgsz)
./npu/npu_run.sh python3 -u yolo_fpga.py \
    --model models/ball_v2_320_fp16.rknn --imgsz 320 --conf 0.7 \
    --send-boxes --box-host 192.168.1.255 --box-port 6103
```

注意：

- 必须用 `./npu/npu_run.sh` 启动（它会把新版 RKNN 运行时挂到 `/usr/lib`），
  直接用 `python3` 会报 `Invalid RKNN model version 6`。
- `--model` 必须写**完整模型路径**，不要保留文档中的 `...` 占位符。
- 按 `Ctrl-C` 停止；也可加 `--duration 60` 录制 60 秒后自动退出。
- 默认只回传 **置信度 ≥ 0.7** 的目标（可用 `--conf` 调整）。

正常启动输出：

```text
[info] 监听 udp 0.0.0.0:6102  模型 models/ball_v2_320_fp16.rknn  conf>=0.7  iou=0.45
[info] 目标框回传 -> 192.168.1.255:6103 (每帧 1 包, 无目标时 count=0)
[det] 10:21:33 fid=25 目标 1 个:
       #1 ball  conf=0.983 box=(238,305)-(369,437) 尺寸=132x133 中心=(303,371)
[stat] t=  1.0s 收包=18200 完整帧=37 推理=29 有目标帧=1 目标=1 发框=29 跳帧=7
```

- `[det]` 为终端坐标打印（默认每 0.5 s 最多一条）：
  `--print-interval 0` 每帧都打印，`--no-print-det` 关闭。
- `发框=N` 表示已回传的包数；结束时打印
  `目标框回传: N 包, M 字节, 失败 0`。
- 需要更高精度时可换 640 模型：
  `--model models/ball_v2_640_fp16.rknn --imgsz 640`（约 70 ms/帧，14 fps）。

### 7.2 后台运行（长时间采集）

```bash
cd ~/Desktop/fpga
nohup ./npu/npu_run.sh python3 -u yolo_fpga.py \
    --model models/ball_v2_320_fp16.rknn --imgsz 320 --conf 0.7 \
    --send-boxes --box-host 192.168.1.255 --box-port 6103 \
    --save-video recordings/yolo_run.mp4 > yolo.log 2>&1 &

tail -f yolo.log          # Ctrl-C 只退出 tail，不影响后台进程
```

停止后台进程：`pkill -f yolo_fpga.py`。

### 7.3 先发测试框（不跑 YOLO）

```bash
cd ~/Desktop/fpga
python3 box_protocol.py --host 192.168.1.255 --port 6103 --frames 300 --fps 20
```

会发送一个从左到右移动的框（(x,150)-(x+120,330)，score 0.90，class 0）。

### 7.4 板端抓包确认

```bash
sudo tcpdump -i enP3p49s0 -nn udp port 6103 -c 10 -XX
```

### 7.5 本机接收自测

```bash
python3 npu/box_recv_test.py --port 6103
```

正常输出示例：

```text
[0001] 127.0.0.1:51269 28B fid=1 ts=339ms count=1
       box=(247,281)-(431,475) score=0.50 cls=56
```

### 7.6 可选：单播 + 静态 ARP

若 FPGA 侧过滤广播不方便，可在 RK3588 上写死 ARP 后使用单播：

```bash
sudo ip neigh replace 192.168.1.10 lladdr 00:0A:35:01:FE:C1 dev enP3p49s0

./npu/npu_run.sh python3 -u yolo_fpga.py \
    --model models/ball_v2_320_fp16.rknn --imgsz 320 --conf 0.7 \
    --send-boxes --box-host 192.168.1.10 --box-port 6103
```

### 7.7 常见问题

| 现象 | 原因 / 处理 |
|---|---|
| 启动报 `Invalid RKNN model version 6` | 没有用 `npu_run.sh`，系统自带运行时 1.4.0 太旧 |
| `unshare` 报权限错误 | 内核未开启非特权用户命名空间，需用 root 直接替换 `/usr/lib/librknnrt.so` |
| `发框=0` 或没有 `[stat]` | 没收到图像流：检查 `setup_net.sh`、网线、`rx_packets` 是否增长 |
| 终端没有 `[det]` | 当前画面无目标或置信度低于 0.7；确认没有加 `--no-print-det` |
| FPGA 收不到框 | 检查广播帧是否放行、UDP 6103、IP 目的地址；用 `tcpdump` 确认包已发出 |
| 出现整帧 `auto 0.xx` 误检 | 摄像头被近物遮挡，移开镜头或提高 `--conf` |
| `--model ...` 报错 | 路径中不要带 `...`，用仓库里的实际文件路径 |


---

## 8. class_id 说明（ball_v2）

当前默认模型 `ball_v2`（YOLOv5s，单类别）只有一个类别：

| class_id | 名称 | 说明 |
|---:|---|---|
| 0 | ball | 球（默认模型） |

- 协议本身与类别数无关，FPGA 侧按 `class_id` 区分即可，不需要解析名称。
- 如果换成 COCO 80 类模型，`class_id` 对应 COCO 索引，名称表见 RK3588 侧
  `yolo_fpga.py` 中的 `COCO_NAMES`。
- 如果使用自训练多类别模型，`class_id` 按训练集 `names` 顺序对应，需双方约定。

---

## 9. 版本记录

| 版本 | 日期 | 变更 |
|---|---|---|
| v1.2 | 2026-11-04 | 默认模型切换为 ball_v2（单类别 ball，320/640 FP16）；更新示例与 class_id 说明 |
| v1.1 | 2026-11-04 | 默认置信度改为 0.7；补充 RK3588 启动/后台运行/排查步骤；修正带 `...` 的示例命令 |
| v1 | 2026-11-03 | 初版：16B 头 + 12B/框，UDP 广播 6103 |
