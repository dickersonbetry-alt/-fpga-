// ============================================================
// quadrature.v : 增量式正交编码器 4 倍频解码（带输入去抖）
//
// 对 A/B 两相正交信号做 4 倍频计数，输出带符号的位置计数 pos。
// 输入先经过两级同步（消亚稳态）和去抖滤波（连续 DEBOUNCE 个时钟
// 稳定才认），用于滤除电机/PWM 走线耦合进来的高频毛刺。
//
// 方向约定：A 超前 B 90° 记为正转。若某一路实际方向相反，
// 交换该路 A/B 接线或在调用处取反即可。
// ============================================================
module quadrature #(
    parameter [31:0] DEBOUNCE = 1500  // 输入需稳定的时钟数（50MHz 下 1500 = 30us）
)(
    input  wire               clk,
    input  wire               rst_n,
    input  wire               a,
    input  wire               b,
    output reg  signed [31:0] pos
);

    // 1) 两级同步，消除亚稳态
    reg [1:0] a_sync, b_sync;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            a_sync <= 2'b00;
            b_sync <= 2'b00;
        end else begin
            a_sync <= {a_sync[0], a};
            b_sync <= {b_sync[0], b};
        end
    end

    // 2) 去抖：输入需连续 DEBOUNCE 个时钟保持不同，才更新滤波值
    reg       a_f, b_f;
    reg [15:0] a_cnt, b_cnt;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            a_f   <= 1'b0;
            b_f   <= 1'b0;
            a_cnt <= 16'd0;
            b_cnt <= 16'd0;
        end else begin
            // A 相
            if (a_sync[1] != a_f) begin
                if (a_cnt >= DEBOUNCE - 1) begin
                    a_f   <= a_sync[1];
                    a_cnt <= 16'd0;
                end else
                    a_cnt <= a_cnt + 16'd1;
            end else
                a_cnt <= 16'd0;
            // B 相
            if (b_sync[1] != b_f) begin
                if (b_cnt >= DEBOUNCE - 1) begin
                    b_f   <= b_sync[1];
                    b_cnt <= 16'd0;
                end else
                    b_cnt <= b_cnt + 16'd1;
            end else
                b_cnt <= 16'd0;
        end
    end

    // 3) 4 倍频解码（用去抖后的 a_f / b_f）
    reg [1:0] prev_ab;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pos     <= 32'sd0;
            prev_ab <= 2'b00;
        end else begin
            prev_ab <= {a_f, b_f};
            case ({prev_ab, a_f, b_f})
                // 一个方向: 00 -> 01 -> 11 -> 10 -> 00
                4'b0001, 4'b0111, 4'b1110, 4'b1000: pos <= pos + 32'sd1;
                // 另一个方向: 00 -> 10 -> 11 -> 01 -> 00
                4'b0010, 4'b1011, 4'b1101, 4'b0100: pos <= pos - 32'sd1;
                // 其余（状态不变 / 非法跳变）保持
                default: pos <= pos;
            endcase
        end
    end

endmodule
