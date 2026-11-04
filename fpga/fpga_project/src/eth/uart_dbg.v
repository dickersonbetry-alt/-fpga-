// uart_dbg.v : 115200 8N1 调试发送，周期 10ms 发 1 帧：
// 0xAA + 12 x uint16（小端），共 25 字节
//   v0  = 以太网 GMII 总帧数
//   v1  = 以太网通过过滤的帧数（= 屏上 E）
//   v2  = IMU 串口收到的字节总数
//   v3  = IMU 校验通过的有效帧数
//   v4  = IMU 0x26 欧拉角帧数
//   v5  = IMU 0x04 原始数据帧数
//   v6  = IMU 校验失败次数
//   v7  = {最近功能字, 最近长度字节}
//   v8  = roll  (0.1°，有符号)
//   v9  = pitch (0.1°，有符号)
//   v10 = yaw   (0.1°，有符号)
//   v11 = {bit15 = cal_done, bit6:0 = cal_cnt}
module uart_dbg(
    input  wire        clk,        // 50MHz
    input  wire        rst_n,
    input  wire [15:0] v0, v1, v2, v3, v4, v5, v6, v7, v8, v9, v10, v11,
    output reg         tx = 1'b1
);
    localparam BAUD = 434;         // 50e6/115200
    localparam NFRM = 25;          // 0xAA + 12*2

    reg [9:0] frm [0:NFRM-1];
    always @(*) begin
        frm[0]  = 10'h354;                        // 起始位0 + 0xAA + 停止位1
        frm[1]  = {1'b1, v0[7:0],   1'b0};
        frm[2]  = {1'b1, v0[15:8],  1'b0};
        frm[3]  = {1'b1, v1[7:0],   1'b0};
        frm[4]  = {1'b1, v1[15:8],  1'b0};
        frm[5]  = {1'b1, v2[7:0],   1'b0};
        frm[6]  = {1'b1, v2[15:8],  1'b0};
        frm[7]  = {1'b1, v3[7:0],   1'b0};
        frm[8]  = {1'b1, v3[15:8],  1'b0};
        frm[9]  = {1'b1, v4[7:0],   1'b0};
        frm[10] = {1'b1, v4[15:8],  1'b0};
        frm[11] = {1'b1, v5[7:0],   1'b0};
        frm[12] = {1'b1, v5[15:8],  1'b0};
        frm[13] = {1'b1, v6[7:0],   1'b0};
        frm[14] = {1'b1, v6[15:8],  1'b0};
        frm[15] = {1'b1, v7[7:0],   1'b0};
        frm[16] = {1'b1, v7[15:8],  1'b0};
        frm[17] = {1'b1, v8[7:0],   1'b0};
        frm[18] = {1'b1, v8[15:8],  1'b0};
        frm[19] = {1'b1, v9[7:0],   1'b0};
        frm[20] = {1'b1, v9[15:8],  1'b0};
        frm[21] = {1'b1, v10[7:0],  1'b0};
        frm[22] = {1'b1, v10[15:8], 1'b0};
        frm[23] = {1'b1, v11[7:0],  1'b0};
        frm[24] = {1'b1, v11[15:8], 1'b0};
    end

    reg [4:0]  fidx    = 5'd0;
    reg [15:0] bit_cnt = 16'd0;
    reg [3:0]  bcnt    = 4'd0;
    reg [9:0]  sh      = 10'h3FF;
    reg        busy    = 1'b0;
    reg [19:0] per     = 20'd0;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            tx <= 1'b1; busy <= 1'b0; bit_cnt <= 16'd0;
            bcnt <= 4'd0; fidx <= 5'd0; per <= 20'd0; sh <= 10'h3FF;
        end else begin
            per <= per + 20'd1;

            if (!busy) begin
                tx <= 1'b1;
                if (per == 20'd500_000) begin    // 10ms
                    per     <= 20'd0;
                    busy    <= 1'b1;
                    fidx    <= 5'd0;
                    bcnt    <= 4'd0;
                    bit_cnt <= 16'd0;
                    sh      <= frm[0];
                end
            end else begin
                if (bit_cnt >= BAUD - 1) begin
                    bit_cnt <= 16'd0;
                    sh <= {1'b1, sh[9:1]};
                    if (bcnt == 4'd9) begin
                        bcnt <= 4'd0;
                        if (fidx == NFRM-1) begin
                            busy <= 1'b0;
                        end else begin
                            fidx <= fidx + 5'd1;
                            sh   <= frm[fidx + 5'd1];
                        end
                    end else
                        bcnt <= bcnt + 4'd1;
                end else
                    bit_cnt <= bit_cnt + 16'd1;
                tx <= sh[0];
            end
        end
    end
endmodule
