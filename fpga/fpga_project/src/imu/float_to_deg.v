// ============================================================
// float_to_deg.v : float32（弧度） -> 有符号整数（度 ×10）
//
// value = (-1)^s * 1.m * 2^(e-127)
// deg_x10 = value * 180/pi * 10 = value * 572.9578
//
// 定点做法：M = {1, m}（24 位），P = M * 573，
//           deg_x10 ≈ P >> (150 - e)
// 例：1.0 rad -> e=127, M=0x800000, P=0x800000*573, >>23 = 573 = 57.3°
// ============================================================
module float_to_deg(
    input  wire               clk,
    input  wire               rst_n,
    input  wire               in_valid,
    input  wire signed [31:0] fin,        // float32，弧度
    output reg                out_valid,
    output reg  signed [15:0] deg_x10     // 度 ×10（123 = 12.3°）
);

    // ---- stage 1：拆符号/阶码/尾数 ----
    reg               s1_sign;
    reg  signed [9:0] s1_exp;        // 无符号阶码转有符号
    reg  [23:0]       s1_m;
    reg               s1_v;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            s1_sign <= 1'b0;
            s1_exp  <= 10'sd0;
            s1_m    <= 24'd0;
            s1_v    <= 1'b0;
        end else begin
            s1_sign <= fin[31];
            s1_exp  <= {2'b00, fin[30:23]};
            s1_m    <= {1'b1, fin[22:0]};
            s1_v    <= in_valid;
        end
    end

    // ---- stage 2：乘 573 ----
    reg  [33:0]       s2_p;
    reg  signed [9:0] s2_shift;
    reg               s2_sign;
    reg               s2_v;

    wire [33:0] p_full = s1_m * 34'd573;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            s2_p     <= 34'd0;
            s2_shift <= 10'sd0;
            s2_sign  <= 1'b0;
            s2_v     <= 1'b0;
        end else begin
            s2_p     <= p_full;
            s2_shift <= 10'sd150 - s1_exp;   // >0 右移，<0 左移
            s2_sign  <= s1_sign;
            s2_v     <= s1_v;
        end
    end

    // ---- stage 3：移位 + 符号 ----
    reg signed [39:0] shifted;
    reg signed [15:0] mag;

    always @(*) begin
        if (s2_shift >= 10'sd39)
            shifted = 40'sd0;
        else if (s2_shift >= 10'sd0)
            shifted = $signed({6'd0, s2_p}) >>> s2_shift;
        else
            shifted = $signed({6'd0, s2_p}) <<< (-s2_shift);
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            deg_x10   <= 16'sd0;
            out_valid <= 1'b0;
        end else begin
            out_valid <= s2_v;
            if (shifted > 40'sd32767)
                mag = 16'sd32767;
            else if (shifted < -40'sd32768)
                mag = -16'sd32768;
            else
                mag = shifted[15:0];

            deg_x10 <= s2_sign ? -mag : mag;
        end
    end

endmodule
