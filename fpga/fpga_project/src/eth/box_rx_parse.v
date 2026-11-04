// ============================================================
// box_rx_parse.v : 解析 RK3588 回传的 YOLO 目标框（YBOX v1）
//
// 输入：UDP 负载字节流（eth_udp_rx_gmii 的 payload_valid/dat）
// 输出：box_count + 16 组框坐标/置信度/类别，toggle 脉冲供显示域同步
//
// 包格式：16B 头（"YBOX",ver,flags,count,frame_id,ts）+ 12B × count
// 超时 200ms 无新包则清空框
// ============================================================
module box_rx_parse(
    input  wire        clk,
    input  wire        rst_n,
    input  wire [7:0]  data,
    input  wire        valid,

    output reg  [4:0]  box_count = 5'd0,
    output reg         box_toggle = 1'b0,
    output reg  [255:0] box_x1 = 256'd0,
    output reg  [255:0] box_y1 = 256'd0,
    output reg  [255:0] box_x2 = 256'd0,
    output reg  [255:0] box_y2 = 256'd0,
    output reg  [127:0] box_score = 128'd0,
    output reg  [127:0] box_cls = 128'd0
);

    localparam [1:0] S_IDLE = 2'd0, S_HDR = 2'd1, S_BOX = 2'd2, S_DONE = 2'd3;

    reg [1:0]  st = S_IDLE;
    reg [7:0]  hdr [0:15];
    reg [3:0]  hcnt = 4'd0;
    reg [4:0]  idx = 5'd0;
    reg [3:0]  bcnt = 4'd0;
    reg [4:0]  cnt = 5'd0;
    reg [7:0]  b [0:11];
    reg [24:0] tmo = 25'd0;

    wire [15:0] h_count = {hdr[7], hdr[6]};

    integer i;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            st <= S_IDLE;
            hcnt <= 4'd0;
            idx  <= 5'd0;
            bcnt <= 4'd0;
            cnt  <= 5'd0;
            tmo  <= 25'd0;
            box_count <= 5'd0;
            box_toggle <= 1'b0;
            box_x1 <= 256'd0; box_y1 <= 256'd0;
            box_x2 <= 256'd0; box_y2 <= 256'd0;
            box_score <= 128'd0; box_cls <= 128'd0;
        end else begin
            // 200ms 超时清空（125MHz）
            if (valid)
                tmo <= 25'd0;
            else if (tmo < 25'd25_000_000)
                tmo <= tmo + 25'd1;
            else if (box_count != 5'd0) begin
                box_count <= 5'd0;
                box_toggle <= ~box_toggle;
            end

            case (st)
            S_IDLE: begin
                if (valid && data == 8'h59) begin   // 'Y'
                    hdr[0] <= data;
                    hcnt <= 4'd1;
                    st   <= S_HDR;
                end
            end
            S_HDR: begin
                if (valid) begin
                    hdr[hcnt] <= data;
                    if (hcnt == 4'd15) begin
                        // magic "YBOX" + version 1 + count ≤ 16
                        if (hdr[0] == 8'h59 && hdr[1] == 8'h42 &&
                            hdr[2] == 8'h4F && hdr[3] == 8'h58 &&
                            hdr[4] == 8'h01 && h_count <= 16) begin
                            cnt  <= h_count;
                            idx  <= 5'd0;
                            bcnt <= 4'd0;
                            st   <= (h_count == 0) ? S_DONE : S_BOX;
                        end else
                            st <= S_IDLE;
                    end else
                        hcnt <= hcnt + 4'd1;
                end
            end
            S_BOX: begin
                if (valid) begin
                    b[bcnt] <= data;
                    if (bcnt == 4'd11) begin
                        bcnt <= 4'd0;
                        // 写入该框
                        box_x1[idx*16 +: 16] <= {b[1], b[0]};
                        box_y1[idx*16 +: 16] <= {b[3], b[2]};
                        box_x2[idx*16 +: 16] <= {b[5], b[4]};
                        box_y2[idx*16 +: 16] <= {b[7], b[6]};
                        box_score[idx*8 +: 8] <= b[8];
                        box_cls[idx*8 +: 8]   <= b[9];
                        if (idx + 5'd1 >= cnt) begin
                            idx <= 5'd0;
                            st  <= S_DONE;
                        end else
                            idx <= idx + 5'd1;
                    end else
                        bcnt <= bcnt + 4'd1;
                end
            end
            S_DONE: begin
                box_count  <= cnt;
                box_toggle <= ~box_toggle;
                st <= S_IDLE;
            end
            default: st <= S_IDLE;
            endcase
        end
    end

endmodule
