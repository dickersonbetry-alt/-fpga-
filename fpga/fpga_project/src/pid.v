// ============================================================
// pid.v : 定点整数位置式 PID 控制器（两级流水）
//
// tick 为控制周期脉冲，tick 为高时启动一次计算。
// 输出已限幅；积分项带限幅，实现抗积分饱和。
//
// 增益 KP/KI/KD 均为定点数，实际增益 = 参数值 / 2^GAIN_SHIFT。
// 公式：
//   u = (KP*e + KI*Σe + KD*(上一次反馈 - 本次反馈)) >> GAIN_SHIFT
//
// 说明：对反馈量做微分（微分先行），避免目标突变造成微分冲击。
//
// 流水线：
//   第 1 拍(tick)：算 err、积分累加(带限幅)、三项乘积 -> 寄存
//   第 2 拍     ：三项求和 + 输出限幅 -> out
// 控制周期 100ms（50MHz 下 500 万拍），多 1 拍延迟完全无影响，
// 但把 64 位乘加的 24ns 长路径拆开了。
// ============================================================
module pid #(
    parameter signed [31:0] KP           = 32'sd256,   // 比例增益 (定点)
    parameter signed [31:0] KI           = 32'sd16,    // 积分增益 (定点)
    parameter signed [31:0] KD           = 32'sd0,     // 微分增益 (定点)
    parameter        [31:0] GAIN_SHIFT   = 8,          // 增益定点右移位数
    parameter signed [31:0] OUT_MAX      = 32'sd100,   // 输出上限
    parameter signed [31:0] OUT_MIN      = -32'sd100,  // 输出下限
    parameter signed [63:0] INTEGRAL_MAX = 64'sd1600  // 积分限幅：×KI/2^8 ≈ 80% 输出，积分更快饱和、抗饱和更早介入
)(
    input  wire               clk,
    input  wire               rst_n,
    input  wire               tick,
    input  wire               clr,        // 1 = 清空积分/微分状态
    input  wire signed [31:0] setpoint,   // 目标值
    input  wire signed [31:0] feedback,   // 实测反馈
    output reg  signed [31:0] out = 32'sd0
);

    // 流水第 1 拍的结果
    reg signed [63:0] p_term        = 64'sd0;
    reg signed [63:0] i_term        = 64'sd0;
    reg signed [63:0] d_term        = 64'sd0;
    reg signed [63:0] integral_next_r = 64'sd0;
    reg signed [31:0] fb_r          = 32'sd0;
    reg               stage2        = 1'b0;

    // 状态
    reg signed [63:0] integral = 64'sd0;
    reg signed [31:0] last_fb  = 32'sd0;

    // 组合中间量（仅在 always 块内先赋值后使用）
    reg signed [31:0] err;
    reg signed [63:0] integral_next;
    reg signed [63:0] sum;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            out             <= 32'sd0;
            integral        <= 64'sd0;
            last_fb         <= 32'sd0;
            p_term          <= 64'sd0;
            i_term          <= 64'sd0;
            d_term          <= 64'sd0;
            integral_next_r <= 64'sd0;
            fb_r            <= 32'sd0;
            stage2          <= 1'b0;
            err             = 32'sd0;
            integral_next   = 64'sd0;
            sum             = 64'sd0;
        end else if (clr) begin
            // 清积分项：每个动作开始/切换时调用，避免上一次动作的积分残留
            out             <= 32'sd0;
            integral        <= 64'sd0;
            last_fb         <= feedback;
            p_term          <= 64'sd0;
            i_term          <= 64'sd0;
            d_term          <= 64'sd0;
            integral_next_r <= 64'sd0;
            fb_r            <= feedback;
            stage2          <= 1'b0;
        end else begin
            if (tick) begin
                // ---------- 第 1 拍：误差、积分、三项乘积 ----------
                err = setpoint - feedback;

                integral_next = integral + err;
                if (integral_next > INTEGRAL_MAX)
                    integral_next = INTEGRAL_MAX;
                else if (integral_next < -INTEGRAL_MAX)
                    integral_next = -INTEGRAL_MAX;

                p_term <= ($signed(err) * KP) >>> GAIN_SHIFT;
                i_term <= (integral_next * KI) >>> GAIN_SHIFT;
                d_term <= (($signed(last_fb) - $signed(feedback)) * KD) >>> GAIN_SHIFT;

                integral_next_r <= integral_next;
                fb_r            <= feedback;
                stage2          <= 1'b1;
            end else if (stage2) begin
                // ---------- 第 2 拍：求和 + 限幅 + 输出 ----------
                sum = p_term + i_term + d_term;
                if (sum > OUT_MAX) begin
                    // 输出饱和：积分保持不变（抗积分饱和），
                    // 否则负载大时会一直积分，松负载后猛冲/超速
                    out <= OUT_MAX;
                end else if (sum < OUT_MIN) begin
                    out <= OUT_MIN;
                end else begin
                    out      <= sum[31:0];
                    integral <= integral_next_r;
                end
                last_fb <= fb_r;
                stage2  <= 1'b0;
            end
        end
    end

endmodule
